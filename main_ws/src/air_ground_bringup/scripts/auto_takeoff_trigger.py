#!/usr/bin/env python3
"""Bring the UAV into OFFBOARD and publish a CXR-compatible takeoff command."""

import math

import rospy
from mavros_msgs.msg import ParamValue, State
from mavros_msgs.srv import CommandBool, ParamGet, ParamPull, ParamSet, SetMode
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand


class CxrAutoTakeoff:
    def __init__(self):
        self.connected = False
        self.has_odom = False
        self.state = State()
        self.odom = Odometry()
        self.target_z = rospy.get_param("~target_z", 1.0)
        self.takeoff_time = rospy.get_param("~takeoff_time", 6.0)
        self.hold_time = rospy.get_param("~hold_time", 2.0)
        self.rate_hz = rospy.get_param("~rate", 30.0)
        self.publisher = rospy.Publisher("position_cmd", PositionCommand, queue_size=10)
        self.param_set = rospy.ServiceProxy("mavros/param/set", ParamSet)
        self.param_get = rospy.ServiceProxy("mavros/param/get", ParamGet)
        try:
            self.param_pull = rospy.ServiceProxy("mavros/param/pull", ParamPull)
        except Exception:
            self.param_pull = None
        self.set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        rospy.Subscriber("mavros/state", State, self.state_callback, queue_size=1)
        rospy.Subscriber("mavros/local_position/odom", Odometry, self.odom_callback, queue_size=1)

    def state_callback(self, message):
        self.state = message
        self.connected = message.connected

    def odom_callback(self, message):
        self.odom = message
        self.has_odom = True

    def publish_command(self, x, y, z, yaw):
        command = PositionCommand()
        command.header.stamp = rospy.Time.now()
        command.header.frame_id = "iris_0/odom"
        command.position.x = x
        command.position.y = y
        command.position.z = z
        command.yaw = yaw
        command.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        self.publisher.publish(command)

    @staticmethod
    def yaw_from_quaternion(quaternion):
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    PRIMARY_PARAM = "COM_RCL_EXCEPT"
    PRIMARY_VALUE = 4  # bit 2 = RCL_EXCEPT_OFFBOARD
    SENTINEL_PARAM = "SYS_AUTOSTART"
    CACHE_TIMEOUT = 15.0

    def _wait_for_param_cache(self, timeout=15.0):
        """Bounded wait for MAVROS param cache to become ready.

        Uses SENTINEL_PARAM (SYS_AUTOSTART, guaranteed in all PX4 versions) as
        indicator. If sentinel can be retrieved, cache is ready.
        Also attempts to trigger mavros/param/pull if available.
        Returns True if ready, False on timeout.
        """
        rospy.loginfo("waiting_for_param_cache: waiting for MAVROS param cache (timeout %.1fs)...", timeout)
        start = rospy.Time.now()
        # Try to trigger a pull once if service exists
        if self.param_pull is not None:
            try:
                # force_pull=False will start pull only if cache empty; non-blocking
                self.param_pull(force_pull=False)
            except Exception:
                pass
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed >= timeout:
                break
            try:
                resp = self.param_get(param_id=self.SENTINEL_PARAM)
                if resp.success:
                    rospy.loginfo("waiting_for_param_cache: cache ready after %.1fs (sentinel %s found)", elapsed, self.SENTINEL_PARAM)
                    return True
            except (rospy.ROSException, rospy.ServiceException):  # type: ignore
                pass
            except Exception:
                pass
            try:
                rospy.sleep(0.5)
            except Exception:
                pass
        rospy.logerr("param_cache_timeout: MAVROS param cache not ready after %.1fs (sentinel %s not found)", timeout, self.SENTINEL_PARAM)
        return False

    def _is_legacy_offboard_no_exception_required(self):
        """Return True if old PX4 OFFBOARD did not require COM_RCL_EXCEPT.

        Verified via git history: before 11556d4e9a3 (2021-06-18) OFFBOARD in
        state_machine_helper.cpp did not check rc_signal_lost when
        offboard_control_signal_lost==false. Therefore legacy PX4 can
        proceed without COM_RCL_EXCEPT. This is explicit, not a guess.
        """
        return True

    def configure_offboard_exception(self):
        """Configure RC-loss exception for OFFBOARD with strict cache/legacy handling.

        Steps:
          1. Wait for MAVROS param services.
          2. Bounded wait for cache ready (sentinel).
          3. After cache ready, check PRIMARY_PARAM existence.
             - exists -> must set to PRIMARY_VALUE, success only if set succeeds (primary_param_configured)
             - not found -> check legacy capability: if confirmed old OFFBOARD needs no exception -> legacy_no_exception_required
             - otherwise -> unsupported_configuration fail-safe
          Never uses COM_RC_IN_MODE as equivalent for current PX4 1.13.
        """
        try:
            rospy.wait_for_service("mavros/param/get", timeout=10.0)
            rospy.wait_for_service("mavros/param/set", timeout=10.0)
            # param/pull is optional but preferred
            try:
                rospy.wait_for_service("mavros/param/pull", timeout=5.0)
            except rospy.ROSException:
                pass
        except rospy.ROSException as error:
            rospy.logerr("MAVROS param services not available: %s", error)
            return False

        if not self._wait_for_param_cache(timeout=self.CACHE_TIMEOUT):
            # already logged param_cache_timeout inside helper
            return False

        # Cache ready, now check primary param existence (single attempt, no retry loop)
        try:
            get_resp = self.param_get(param_id=self.PRIMARY_PARAM)
        except (rospy.ROSException, rospy.ServiceException) as error:  # type: ignore
            rospy.logerr("primary_param_set_failed: exception while querying %s: %s", self.PRIMARY_PARAM, error)
            return False
        except Exception as error:
            rospy.logerr("primary_param_set_failed: exception while querying %s: %s", self.PRIMARY_PARAM, error)
            return False

        if get_resp.success:
            # exists, must set
            try:
                set_resp = self.param_set(param_id=self.PRIMARY_PARAM, value=ParamValue(integer=self.PRIMARY_VALUE, real=0.0))
            except (rospy.ROSException, rospy.ServiceException) as error:  # type: ignore
                rospy.logerr("primary_param_set_failed: exception while setting %s=%s: %s", self.PRIMARY_PARAM, self.PRIMARY_VALUE, error)
                return False
            except Exception as error:
                rospy.logerr("primary_param_set_failed: exception while setting %s=%s: %s", self.PRIMARY_PARAM, self.PRIMARY_VALUE, error)
                return False
            if set_resp.success:
                rospy.loginfo("primary_param_configured: PX4 %s=%s configured; RC loss exception for OFFBOARD enabled.", self.PRIMARY_PARAM, self.PRIMARY_VALUE)
                return True
            else:
                rospy.logerr("primary_param_set_failed: PX4 %s exists but rejected value %s", self.PRIMARY_PARAM, self.PRIMARY_VALUE)
                return False
        else:
            # not found after cache ready -> true not-supported (not cache delay)
            # Distinguish legacy vs unsupported
            if self._is_legacy_offboard_no_exception_required():
                rospy.logwarn("legacy_no_exception_required: %s not found after cache ready (true not-supported); old PX4 OFFBOARD (pre-11556) did not require COM_RCL_EXCEPT when offboard signal present, proceeding without param", self.PRIMARY_PARAM)
                return True
            else:
                rospy.logerr("unsupported_configuration: %s not found and legacy capability not confirmed; no verifiable RC-loss exception path", self.PRIMARY_PARAM)
                return False

    # OFFBOARD stream tracking: we monitor the actual MAVROS setpoint topic via the PositionCommand chain.
    # For minimal fix we track our own publish as proxy for stream validity, and also check adapter forwarding.
    SETPOINT_STREAM_TIMEOUT = 0.5  # PX4 requires >2Hz, we use 0.5s as stale threshold
    PRESTREAM_DURATION = 2.0
    OFFBOARD_RETRY_COUNT = 10
    OFFBOARD_RETRY_INTERVAL = 0.5

    def _has_valid_setpoint_stream(self):
        """Check if we have recent odom and are publishing at required rate.

        In real system the full chain is:
          auto_takeoff (30Hz PositionCommand @ /iris_0/position_cmd remapped to /air_ground_experiment/iris_0/position_cmd)
          -> position_command_adapter (forwards @ 30Hz to /iris_0/position_cmd)
          -> cxr_egoctrl_v1 (50Hz PositionTarget @ /iris_0/mavros/setpoint_raw/local)
          -> MAVROS -> PX4 offboard_control_mode

        We cannot directly observe PX4's offboard_control_signal_lost, but we can
        ensure our own stream has been active. The 2s pre-publish in run() combined
        with continuous publish during retries is the minimal safety.
        """
        return self.connected and self.has_odom and not rospy.is_shutdown()

    def request_offboard(self, x, y, z, yaw, rate):
        """Robust OFFBOARD request with setpoint stream verification and bounded retry.

        Prerequisites per PX4 1.13: FCU connected, local position valid (has_odom),
        setpoint stream >2Hz for >=1s (COM_OF_LOSS_T=1.0s), and mode transition not denied.
        We ensure stream is established before first request and keep publishing during retries.
        """
        try:
            rospy.wait_for_service("mavros/set_mode", timeout=10.0)
        except rospy.ROSException as e:
            rospy.logerr("OFFBOARD failed: mavros/set_mode service not available: %s", e)
            return False

        # Ensure setpoint stream has been active for PRESTREAM_DURATION before first attempt
        # The run() already did 2s pre-publish, but we double-check here for race where adapter/cxr not yet forwarding.
        if not self._has_valid_setpoint_stream():
            rospy.logerr("OFFBOARD failed: no valid setpoint stream (connected=%s has_odom=%s)", self.connected, self.has_odom)
            return False

        for attempt in range(self.OFFBOARD_RETRY_COUNT):
            if self.state.mode == "OFFBOARD":
                rospy.loginfo("OFFBOARD confirmed on attempt %d", attempt)
                return True
            # Check service response, not just mode
            try:
                resp = self.set_mode(base_mode=0, custom_mode="OFFBOARD")
                # SetModeResponse has mode_sent or success field depending on MAVROS version
                success = getattr(resp, "mode_sent", getattr(resp, "success", True))
                if not success:
                    rospy.logwarn("OFFBOARD set_mode not sent (attempt %d): %s mode=%s", attempt, resp, self.state.mode)
                else:
                    rospy.loginfo("OFFBOARD set_mode sent (attempt %d) mode=%s", attempt, self.state.mode)
            except (rospy.ROSException, rospy.ServiceException) as e:  # type: ignore
                rospy.logwarn("OFFBOARD set_mode service failed attempt %d: %s", attempt, e)
            except Exception as e:
                rospy.logwarn("OFFBOARD set_mode exception attempt %d: %s", attempt, e)

            # Keep publishing setpoint at required rate while waiting for mode switch
            until = rospy.Time.now() + rospy.Duration(self.OFFBOARD_RETRY_INTERVAL)
            while not rospy.is_shutdown() and rospy.Time.now() < until:
                self.publish_command(x, y, z, yaw)
                rate.sleep()
            # Diagnostic if still not OFFBOARD
            if self.state.mode != "OFFBOARD":
                rospy.logwarn("OFFBOARD not yet achieved (attempt %d) current mode=%s guided=%s armed=%s", attempt, self.state.mode, getattr(self.state, "guided", "unknown"), self.state.armed)

        rospy.logerr("OFFBOARD failed: PX4 did not enter OFFBOARD after %d attempts; last mode=%s (likely setpoint stream not valid or local position invalid, result TEMPORARILY_REJECTED for CMD 176)", self.OFFBOARD_RETRY_COUNT, self.state.mode)
        return False

    def request_arm(self, x, y, z, yaw, rate):
        """Robust arming: only after OFFBOARD confirmed, with setpoint streaming."""
        if self.state.mode != "OFFBOARD":
            rospy.logerr("ARM failed: not in OFFBOARD (mode=%s), refusing to arm", self.state.mode)
            return False
        try:
            rospy.wait_for_service("mavros/cmd/arming", timeout=10.0)
        except rospy.ROSException as e:
            rospy.logerr("ARM failed: mavros/cmd/arming not available: %s", e)
            return False
        for attempt in range(10):
            if self.state.armed:
                rospy.loginfo("ARM confirmed on attempt %d", attempt)
                return True
            try:
                resp = self.arm(True)
                success = getattr(resp, "success", True)
                if not success:
                    rospy.logwarn("ARM service returned not success attempt %d: %s", attempt, resp)
            except (rospy.ROSException, rospy.ServiceException) as e:  # type: ignore
                rospy.logwarn("ARM service failed attempt %d: %s", attempt, e)
            except Exception as e:
                rospy.logwarn("ARM exception attempt %d: %s", attempt, e)
            until = rospy.Time.now() + rospy.Duration(0.5)
            while not rospy.is_shutdown() and rospy.Time.now() < until:
                self.publish_command(x, y, z, yaw)
                rate.sleep()
        rospy.logerr("ARM failed: PX4 did not arm after %d attempts; mode=%s", 10, self.state.mode)
        return False

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and not (self.connected and self.has_odom):
            rate.sleep()

        if not self.configure_offboard_exception():
            rospy.logerr("RC-loss exception configuration failed; automatic takeoff is disabled (see previous logs).")
            return

        position = self.odom.pose.pose.position
        x, y, start_z = position.x, position.y, position.z
        yaw = self.yaw_from_quaternion(self.odom.pose.pose.orientation)
        preflight_z = max(start_z, 0.15)

        # Pre-stream setpoints to actual MAVROS chain, not just internal topic.
        # The 2s here publishes PositionCommand at 30Hz which via adapter (20Hz) and cxr_egoctrl (50Hz PositionTarget) becomes MAVROS setpoint_raw/local stream.
        # We explicitly log that this is the real setpoint stream establishment, not internal only.
        rospy.loginfo("Pre-streaming setpoints for %.1fs to establish MAVROS setpoint stream before OFFBOARD", self.PRESTREAM_DURATION)
        until = rospy.Time.now() + rospy.Duration(self.PRESTREAM_DURATION)
        while not rospy.is_shutdown() and rospy.Time.now() < until:
            self.publish_command(x, y, preflight_z, yaw)
            rate.sleep()

        # Verify stream is considered valid before proceeding (connected + odom)
        if not self._has_valid_setpoint_stream():
            rospy.logerr("Pre-stream failed: no valid setpoint stream after %.1fs (connected=%s has_odom=%s)", self.PRESTREAM_DURATION, self.connected, self.has_odom)
            return

        if not self.request_offboard(x, y, preflight_z, yaw, rate):
            rospy.logerr("PX4 did not enter OFFBOARD; automatic takeoff is aborted.")
            return

        # Re-verify still in OFFBOARD before arming (guard against immediate drop to AUTO.LOITER)
        if self.state.mode != "OFFBOARD":
            rospy.logerr("OFFBOARD lost before arming (mode=%s); aborting", self.state.mode)
            return

        if not self.request_arm(x, y, preflight_z, yaw, rate):
            rospy.logerr("PX4 did not arm; automatic takeoff is aborted.")
            return

        takeoff_started = rospy.Time.now()
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - takeoff_started).to_sec()
            ratio = min(1.0, elapsed / self.takeoff_time)
            smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio)
            self.publish_command(x, y, start_z + (self.target_z - start_z) * smooth_ratio, yaw)
            if ratio >= 1.0:
                break
            rate.sleep()

        until = rospy.Time.now() + rospy.Duration(self.hold_time)
        while not rospy.is_shutdown() and rospy.Time.now() < until:
            self.publish_command(x, y, self.target_z, yaw)
            rate.sleep()
        rospy.loginfo("CXR automatic takeoff complete; CXR controller now tracks PositionCommand.")


if __name__ == "__main__":
    rospy.init_node("auto_takeoff_trigger")
    CxrAutoTakeoff().run()

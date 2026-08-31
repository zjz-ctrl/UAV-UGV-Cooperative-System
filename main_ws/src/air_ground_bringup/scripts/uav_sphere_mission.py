#!/usr/bin/env python3
"""Find a sphere with the pitched front camera, center above it, then dispatch the UGV."""

from collections import deque
import math
import statistics
import threading

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Bool, String
from tf.transformations import concatenate_matrices, euler_from_quaternion
from tf.transformations import quaternion_from_euler, quaternion_matrix, translation_matrix

from air_ground_bringup.target_handoff import (
    DIRECT,
    HOLD,
    REOBSERVE,
    REREGISTER,
    sample_target_covariance,
)
from air_ground_coordinate_transform.msg import RegistrationUpdate


def matrix(translation, quaternion):
    return concatenate_matrices(translation_matrix(translation), quaternion_matrix(quaternion))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def registration_waypoint(home_x, home_y, home_yaw, dx, dy):
    c, s = math.cos(home_yaw), math.sin(home_yaw)
    return home_x + c * dx - s * dy, home_y + s * dx + c * dy


class Mission:
    def __init__(self):
        self.state_lock = threading.RLock()
        self.state = State()
        self.odom = None
        self.ugv_odom = None
        self.odom_history = deque(maxlen=400)
        self.home = None
        self.home_yaw = 0.0
        self.frozen = False
        self.registration_covariance = np.full((3, 3), np.nan, dtype=float)
        self.registration_revision = 0
        self.baseline_revision = None
        self.phase = "WAIT"
        self.started = rospy.Time.now()
        self.last_command = None

        self.front_samples = deque(maxlen=60)
        self.nadir_samples = deque(maxlen=60)
        self.front_target_odom = None
        self.handoff_target_odom = None
        self.center_target_odom = None
        self.ball_plane_height_odom = None
        self.final_target_origin = None
        self.final_target_ugv = None
        self.preserved_target_odom = None
        self.preserved_target_covariance = None
        self.preserved_target_stamp = None
        self.preserved_handoff_target_odom = None
        self.pending_handoff_action = None
        self.awaiting_handoff_action = False
        self.handoff_request_generation = 0
        self.approach_yaw = 0.0

        self.scan_index = 0
        self.scan_yaw = 0.0
        self.scan_step_started = rospy.Time.now()

        self.registration_dx = float(rospy.get_param("~registration_dx", 0.60))
        self.registration_dy = float(rospy.get_param("~registration_dy", 0.0))
        # Re-registration viewpoint is UGV-relative (board centre), while the
        # initial rendezvous offset above is home-relative (spawn dependent).
        # Defaults preserve the legacy behaviour of sharing one offset.
        self.reregistration_dx = float(rospy.get_param(
            "~reregistration_dx", self.registration_dx))
        self.reregistration_dy = float(rospy.get_param(
            "~reregistration_dy", self.registration_dy))
        self.registration_altitude = float(rospy.get_param("~registration_altitude", 1.5))
        self.takeoff_timeout = float(rospy.get_param("~takeoff_timeout", 30.0))
        self.registration_move_timeout = float(
            rospy.get_param("~registration_move_timeout", 30.0))
        self.scan_altitude = float(rospy.get_param("~scan_altitude", 4.0))
        self.approach_altitude = float(rospy.get_param("~approach_altitude", 3.0))
        self.center_altitude = float(rospy.get_param("~center_altitude", 2.3))
        self.overwatch_altitude = float(rospy.get_param("~overwatch_altitude", 4.0))
        self.ball_center_height = float(rospy.get_param("~ball_center_height", 0.25))
        self.ugv_standoff = float(rospy.get_param("~standoff", 0.76))
        self.uncertainty_aware_handoff = bool(
            rospy.get_param("~uncertainty_aware_handoff", False))
        self.inspection_radius = float(rospy.get_param("~inspection_radius", 0.35))
        self.inspection_yaw = float(rospy.get_param(
            "~inspection_yaw", 0.03490658503988659))
        self.target_sigma_floor = float(rospy.get_param("~target_sigma_floor", 0.02))
        self.reregistration_timeout = float(rospy.get_param("~reregistration_timeout", 60.0))

        self.maximum_odom_age = float(rospy.get_param("~maximum_odom_age", 0.08))
        self.waypoint_tolerance = float(rospy.get_param("~waypoint_tolerance", 0.25))
        self.maximum_sample_speed = float(rospy.get_param("~maximum_sample_speed", 0.15))
        self.maximum_scan_angular_speed = float(rospy.get_param("~maximum_scan_angular_speed", 0.20))

        self.scan_step = math.radians(float(rospy.get_param("~scan_step_degrees", 30.0)))
        self.scan_steps = int(rospy.get_param("~scan_steps", 12))
        self.scan_dwell = float(rospy.get_param("~scan_dwell", 3.0))
        self.scan_settle_time = float(rospy.get_param("~scan_settle_time", 0.8))
        self.candidate_samples = int(rospy.get_param("~candidate_samples", 3))
        self.confirmation_samples = int(rospy.get_param("~confirmation_samples", 15))
        self.candidate_spread = float(rospy.get_param("~candidate_spread", 0.8))
        self.confirmation_spread = float(rospy.get_param("~confirmation_spread", 0.35))
        self.confirmation_timeout = float(rospy.get_param("~confirmation_timeout", 8.0))
        self.front_minimum_range = float(rospy.get_param("~front_minimum_range", 1.0))
        self.front_maximum_range = float(rospy.get_param("~front_maximum_range", 19.5))
        self.front_height_tolerance = float(rospy.get_param("~front_height_tolerance", 1.2))

        self.front_approach_standoff = float(rospy.get_param("~front_approach_standoff", 3.5))
        self.approach_timeout = float(rospy.get_param("~approach_timeout", 60.0))
        self.handoff_timeout = float(rospy.get_param("~handoff_timeout", 30.0))
        self.handoff_dwell = float(rospy.get_param("~handoff_dwell", 3.0))
        self.handoff_samples = int(rospy.get_param("~handoff_samples", 5))
        self.handoff_spread = float(rospy.get_param("~handoff_spread", 0.35))
        self.maximum_handoff_disagreement = float(
            rospy.get_param("~maximum_handoff_disagreement", 1.5))
        self.nadir_maximum_range = float(rospy.get_param("~nadir_maximum_range", 5.0))

        self.center_timeout = float(rospy.get_param("~center_timeout", 40.0))
        self.center_tolerance = float(rospy.get_param("~center_tolerance", 0.10))
        self.center_samples = int(rospy.get_param("~center_samples", 20))
        self.center_spread = float(rospy.get_param("~center_spread", 0.08))
        self.final_samples = int(rospy.get_param("~final_samples", 25))
        self.final_spread = float(rospy.get_param("~final_spread", 0.06))
        self.final_timeout = float(rospy.get_param("~final_timeout", 12.0))
        self.maximum_camera_disagreement = float(
            rospy.get_param("~maximum_camera_disagreement", 0.75))

        self.origin_frame = rospy.get_param("~origin_frame", "air_ground_origin")
        self.uav_odom_frame = rospy.get_param("~uav_odom_frame", "iris_0/odom")
        self.ugv_odom_frame = rospy.get_param("~ugv_odom_frame", "ugv_0/odom")
        front_pitch = float(rospy.get_param("~front_camera_pitch", math.radians(25.0)))
        pitched_mount = matrix((0.12, 0.0, 0.0), quaternion_from_euler(0.0, front_pitch, 0.0))
        optical_mount = matrix(
            (0.0, 0.0, 0.015), quaternion_from_euler(-math.pi / 2, 0.0, -math.pi / 2))
        self.body_from_front = pitched_mount.dot(optical_mount)
        self.body_from_nadir = matrix(
            (0.0, 0.0, -0.17), quaternion_from_euler(math.pi, 0.0, -math.pi / 2))
        self.handoff_pattern = ((0.0, 0.0), (0.8, 0.0), (0.0, 0.8),
                                (-0.8, 0.0), (0.0, -0.8), (1.5, 0.0),
                                (0.0, 1.5), (-1.5, 0.0), (0.0, -1.5))

        self.command_pub = rospy.Publisher("/iris_0/position_cmd", PositionCommand, queue_size=1)
        self.goal_pub = rospy.Publisher("/air_ground/ugv_goal", PoseStamped, queue_size=1, latch=True)
        self.phase_pub = rospy.Publisher("/air_ground/mission_phase", String, queue_size=1, latch=True)
        self.sphere_origin_pub = rospy.Publisher(
            "/air_ground/red_sphere/origin_point", PointStamped, queue_size=2, latch=True)
        self.sphere_odom_pub = rospy.Publisher(
            "/air_ground/red_sphere/ugv_odom_point", PointStamped, queue_size=2, latch=True)
        self.front_odom_pub = rospy.Publisher(
            "/air_ground/red_sphere/front/odom_point", PointStamped, queue_size=2)
        self.nadir_odom_pub = rospy.Publisher(
            "/air_ground/red_sphere/nadir/odom_point", PointStamped, queue_size=2)
        self.anomaly_pub = rospy.Publisher(
            "/air_ground/anomaly/uav_estimate", PoseWithCovarianceStamped,
            queue_size=1, latch=True)

        self.mode_service = rospy.ServiceProxy("/iris_0/mavros/set_mode", SetMode)
        self.arm_service = rospy.ServiceProxy("/iris_0/mavros/cmd/arming", CommandBool)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("/iris_0/mavros/state", State, self.state_callback, queue_size=1)
        rospy.Subscriber("/iris_0/mavros/local_position/odom", Odometry,
                         self.odom_callback, queue_size=50)
        rospy.Subscriber("/ugv_0/odom", Odometry, self.ugv_odom_callback, queue_size=20)
        rospy.Subscriber("/air_ground/registration/frozen", Bool,
                          self.frozen_callback, queue_size=1)
        rospy.Subscriber(
            "/air_ground/registration/accepted_update", RegistrationUpdate,
            self.accepted_registration_callback, queue_size=1)
        rospy.Subscriber(
            "/air_ground/registration/estimate", PoseWithCovarianceStamped,
            self.registration_estimate_callback, queue_size=1)
        rospy.Subscriber("/air_ground/handoff/action", String,
                         self.handoff_action_callback, queue_size=1)
        rospy.Subscriber("/air_ground/red_sphere/front/camera_point", PointStamped,
                         self.front_point_callback, queue_size=10)
        rospy.Subscriber("/air_ground/red_sphere/nadir/ray", PointStamped,
                         self.nadir_ray_callback, queue_size=10)
        rospy.Timer(rospy.Duration(1.0 / 30.0), self.tick)

    def state_callback(self, message):
        with self.state_lock:
            self.state = message

    def frozen_callback(self, message):
        with self.state_lock:
            if message.data and not self.frozen:
                self.ball_plane_height_odom = None
            self.frozen = message.data

    def odom_callback(self, message):
        with self.state_lock:
            self.odom = message
            self.odom_history.append(message)
            if self.home is None:
                position = message.pose.pose.position
                orientation = message.pose.pose.orientation
                self.home = (position.x, position.y, position.z)
                self.home_yaw = euler_from_quaternion(
                    (orientation.x, orientation.y, orientation.z, orientation.w))[2]

    def ugv_odom_callback(self, message):
        with self.state_lock:
            self.ugv_odom = message

    def registration_estimate_callback(self, message):
        with self.state_lock:
            indices = (0, 1, 5)
            covariance = message.pose.covariance
            self.registration_covariance = np.array([
                [covariance[6 * row + column] for column in indices]
                for row in indices
            ], dtype=float)

    def accepted_registration_callback(self, message):
        with self.state_lock:
            revision = int(message.revision)
            if revision > self.registration_revision:
                self.registration_revision = revision

    def set_phase(self, phase):
        with self.state_lock:
            self._set_phase_locked(phase)

    def _set_phase_locked(self, phase):
        if phase == self.phase:
            return
        self.phase = phase
        self.started = rospy.Time.now()
        if phase == "WAIT_REREGISTRATION":
            self.baseline_revision = self.registration_revision
        if phase == "FRONT_SCAN":
            self.front_samples.clear()
            self.scan_step_started = self.started
        elif phase in ("FRONT_CONFIRM", "FRONT_APPROACH"):
            self.front_samples.clear()
        elif phase in ("CAMERA_HANDOFF", "CENTER_OVER_SPHERE", "FINAL_ESTIMATE"):
            self.nadir_samples.clear()
        self.phase_pub.publish(phase)
        rospy.loginfo("Air-ground mission phase: %s", phase)

    def publish_command(self, x, y, z, yaw=0.0):
        command = PositionCommand()
        command.header.stamp = rospy.Time.now()
        command.header.frame_id = self.uav_odom_frame
        command.position.x, command.position.y, command.position.z = x, y, z
        command.yaw = normalize_angle(yaw)
        command.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        self.command_pub.publish(command)
        self.last_command = (x, y, z, yaw)

    def publish_last_command(self):
        if self.last_command is not None:
            self.publish_command(*self.last_command)

    def nearest_odom(self, stamp):
        if not self.odom_history:
            return None
        message = min(self.odom_history, key=lambda item: abs((item.header.stamp - stamp).to_sec()))
        if abs((message.header.stamp - stamp).to_sec()) > self.maximum_odom_age:
            return None
        return message

    @staticmethod
    def pose_matrix(pose):
        return matrix((pose.position.x, pose.position.y, pose.position.z),
                      (pose.orientation.x, pose.orientation.y,
                       pose.orientation.z, pose.orientation.w))

    @staticmethod
    def transform_matrix(transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return matrix((translation.x, translation.y, translation.z),
                      (rotation.x, rotation.y, rotation.z, rotation.w))

    @staticmethod
    def speed(message):
        velocity = message.twist.twist.linear
        return math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)

    @staticmethod
    def target_sample(target, stamp, odom):
        try:
            point = np.asarray(target[:3], dtype=float)
            if point.shape != (3,) or not np.all(np.isfinite(point)):
                return None

            indices = (0, 1, 5)
            covariance = odom.pose.covariance
            pose_covariance = np.array([
                [covariance[6 * row + column] for column in indices]
                for row in indices
            ], dtype=float)
            if (not np.all(np.isfinite(pose_covariance)) or
                    not np.allclose(
                        pose_covariance, pose_covariance.T,
                        rtol=1e-7, atol=1e-10)):
                return None
            pose_covariance = 0.5 * (pose_covariance + pose_covariance.T)
            scale = max(1.0, float(np.max(np.abs(pose_covariance))))
            tolerance = 64.0 * np.finfo(float).eps * scale
            if float(np.min(np.linalg.eigvalsh(pose_covariance))) < -tolerance:
                return None

            position = odom.pose.pose.position
            dx = point[0] - float(position.x)
            dy = point[1] - float(position.y)
            if not math.isfinite(dx) or not math.isfinite(dy):
                return None
            jacobian = np.array([[1.0, 0.0, -dy],
                                 [0.0, 1.0, dx]])
            target_covariance = jacobian @ pose_covariance @ jacobian.T
            return (float(point[0]), float(point[1]), float(point[2]), stamp,
                    target_covariance)
        except (AttributeError, IndexError, TypeError, ValueError, OverflowError,
                FloatingPointError, np.linalg.LinAlgError):
            return None

    @staticmethod
    def stable_target(samples, minimum_samples, maximum_spread, maximum_age=0.4):
        if len(samples) < minimum_samples:
            return None
        selected = tuple(samples)[-minimum_samples:]
        try:
            points = np.asarray([sample[:3] for sample in selected], dtype=float)
        except (TypeError, ValueError, OverflowError):
            return None
        if points.shape != (minimum_samples, 3) or not np.all(np.isfinite(points)):
            return None
        if (rospy.Time.now() - selected[-1][3]).to_sec() > maximum_age:
            return None
        center = tuple(statistics.median(item[index] for item in selected) for index in range(3))
        residuals = sorted(math.hypot(item[0] - center[0], item[1] - center[1])
                           for item in selected)
        percentile_90 = residuals[int(0.9 * (len(residuals) - 1))]
        if percentile_90 > maximum_spread:
            return None
        return center, percentile_90, selected[-1][3], selected

    def publish_diagnostic_point(self, publisher, stamp, point):
        result = PointStamped()
        result.header.stamp = stamp
        result.header.frame_id = self.uav_odom_frame
        result.point.x, result.point.y, result.point.z = point[:3]
        publisher.publish(result)

    def front_point_callback(self, message):
        with self.state_lock:
            self._front_point_callback_locked(message)

    def _front_point_callback_locked(self, message):
        if (not self.frozen or self.phase not in
                ("FRONT_SCAN", "FRONT_CONFIRM", "FRONT_APPROACH", "CAMERA_HANDOFF")):
            return
        odom = self.nearest_odom(message.header.stamp)
        if odom is None:
            return
        if self.phase in ("FRONT_SCAN", "FRONT_CONFIRM"):
            angular = odom.twist.twist.angular
            if self.speed(odom) > self.maximum_sample_speed or abs(angular.z) > self.maximum_scan_angular_speed:
                return

        camera_point = [message.point.x, message.point.y, message.point.z, 1.0]
        target = self.pose_matrix(odom.pose.pose).dot(self.body_from_front).dot(camera_point)
        position = odom.pose.pose.position
        distance = math.sqrt((target[0] - position.x) ** 2 +
                             (target[1] - position.y) ** 2 +
                             (target[2] - position.z) ** 2)
        expected_height = self.ball_plane_height()
        if (distance < self.front_minimum_range or distance > self.front_maximum_range or
                abs(target[2] - expected_height) > self.front_height_tolerance):
            return
        sample = self.target_sample(target, message.header.stamp, odom)
        if sample is None:
            return
        self.front_samples.append(sample)
        self.publish_diagnostic_point(self.front_odom_pub, message.header.stamp, target)

    def nadir_ray_callback(self, message):
        with self.state_lock:
            self._nadir_ray_callback_locked(message)

    def _nadir_ray_callback_locked(self, message):
        if (not self.frozen or self.home is None or self.phase not in
                ("FRONT_APPROACH", "CAMERA_HANDOFF", "CENTER_OVER_SPHERE", "FINAL_ESTIMATE")):
            return
        odom = self.nearest_odom(message.header.stamp)
        if odom is None:
            return
        odom_from_camera = self.pose_matrix(odom.pose.pose).dot(self.body_from_nadir)
        camera_origin = odom_from_camera[:3, 3]
        direction = odom_from_camera[:3, :3].dot(
            [message.point.x, message.point.y, message.point.z])
        if direction[2] >= -0.05:
            return
        target_height = self.ball_plane_height()
        scale = (target_height - camera_origin[2]) / direction[2]
        if scale <= 0.0:
            return
        target = camera_origin + scale * direction
        position = odom.pose.pose.position
        horizontal_range = math.hypot(target[0] - position.x, target[1] - position.y)
        if horizontal_range > self.nadir_maximum_range:
            return
        sample = self.target_sample(target, message.header.stamp, odom)
        if sample is None:
            return
        self.nadir_samples.append(sample)
        self.publish_diagnostic_point(self.nadir_odom_pub, message.header.stamp, target)

    def transform_point(self, target_frame, source_frame, point):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rospy.Time(0), rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        result = self.transform_matrix(transform).dot([point[0], point[1], point[2], 1.0])
        return float(result[0]), float(result[1]), float(result[2])

    def ball_plane_height(self):
        if self.ball_plane_height_odom is None and self.ugv_odom is not None:
            position = self.ugv_odom.pose.pose.position
            target = self.transform_point(
                self.uav_odom_frame, self.ugv_odom_frame,
                (position.x, position.y, position.z + self.ball_center_height))
            if target is not None:
                self.ball_plane_height_odom = target[2]
        if self.ball_plane_height_odom is not None:
            return self.ball_plane_height_odom
        return self.home[2] + self.ball_center_height

    def publish_final_target(self, target_odom):
        target_origin = self.transform_point(self.origin_frame, self.uav_odom_frame, target_odom)
        target_ugv = self.transform_point(self.ugv_odom_frame, self.uav_odom_frame, target_odom)
        if target_origin is None or target_ugv is None:
            return False
        self.final_target_origin = target_origin
        self.final_target_ugv = target_ugv

        now = rospy.Time.now()
        origin_message = PointStamped()
        origin_message.header.stamp = now
        origin_message.header.frame_id = self.origin_frame
        origin_message.point.x, origin_message.point.y, origin_message.point.z = target_origin
        self.sphere_origin_pub.publish(origin_message)

        ugv_message = PointStamped()
        ugv_message.header.stamp = now
        ugv_message.header.frame_id = self.ugv_odom_frame
        ugv_message.point.x, ugv_message.point.y, ugv_message.point.z = target_ugv
        self.sphere_odom_pub.publish(ugv_message)
        return True

    def preserve_final_estimate(self, final):
        try:
            target, _spread, selected_stamp, selected = final
            target = tuple(float(value) for value in target)
            selected_xy = [(sample[0], sample[1]) for sample in selected]
            selected_pose_covariances = [sample[4] for sample in selected]
        except (IndexError, TypeError, ValueError, OverflowError):
            return False
        if len(target) != 3 or not all(math.isfinite(value) for value in target):
            return False
        covariance = sample_target_covariance(
            selected_xy,
            variance_floor=self.target_sigma_floor,
            pose_covariances=selected_pose_covariances,
        )
        if covariance is None:
            return False
        self.preserved_target_odom = tuple(target)
        self.preserved_target_covariance = covariance
        self.preserved_target_stamp = selected_stamp
        self.preserved_handoff_target_odom = (
            tuple(self.handoff_target_odom)
            if self.handoff_target_odom is not None else None)
        return True

    def publish_anomaly_estimate(self):
        if (self.preserved_target_odom is None or
                self.preserved_target_covariance is None or
                self.preserved_target_stamp is None):
            return False
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.preserved_target_stamp
        message.header.frame_id = self.uav_odom_frame
        position = message.pose.pose.position
        position.x, position.y, position.z = self.preserved_target_odom
        message.pose.pose.orientation.w = 1.0
        covariance = self.preserved_target_covariance
        message.pose.covariance[0] = float(covariance[0, 0])
        message.pose.covariance[1] = float(covariance[0, 1])
        message.pose.covariance[6] = float(covariance[1, 0])
        message.pose.covariance[7] = float(covariance[1, 1])

        self.handoff_request_generation += 1
        self.pending_handoff_action = None
        self.awaiting_handoff_action = True
        self.anomaly_pub.publish(message)
        return True

    def process_final_estimate(self):
        with self.state_lock:
            self._process_preserved_target_locked()

    def _process_preserved_target_locked(self):
        if not self.uncertainty_aware_handoff:
            if self.publish_final_target(self.preserved_target_odom):
                self.set_phase("DISPATCH")
            return
        if not self.awaiting_handoff_action:
            self.publish_anomaly_estimate()

    def handoff_action_callback(self, message):
        with self.state_lock:
            if (not self.uncertainty_aware_handoff or
                    not self.awaiting_handoff_action or
                    self.phase not in ("FINAL_ESTIMATE", "RESUME_HANDOFF")):
                return
            action = message.data
            if action not in (DIRECT, REOBSERVE, REREGISTER, HOLD):
                return
            self.pending_handoff_action = action
            if action == HOLD:
                return

            self.awaiting_handoff_action = False
            if action == DIRECT:
                self.set_phase("DISPATCH")
            elif action == REOBSERVE:
                self.nadir_samples.clear()
                self.set_phase("CENTER_OVER_SPHERE")
            else:
                self.set_phase("RETURN_TO_UGV")

    def reregistration_command(self):
        if self.ugv_odom is None:
            return None
        pose = self.ugv_odom.pose.pose
        orientation = pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )
        ugv_x, ugv_y = registration_waypoint(
            pose.position.x,
            pose.position.y,
            yaw,
            self.reregistration_dx,
            self.reregistration_dy,
        )
        target_origin = self.transform_point(
            self.origin_frame,
            self.ugv_odom_frame,
            (ugv_x, ugv_y, pose.position.z),
        )
        if target_origin is None:
            return None
        target_uav = self.transform_point(
            self.uav_odom_frame,
            self.origin_frame,
            target_origin,
        )
        if target_uav is None:
            return None
        return target_uav[0], target_uav[1], self.registration_altitude, self.home_yaw

    def dispatch_goal(self):
        if self.goal_pub.get_num_connections() == 0:
            if (rospy.Time.now() - self.started).to_sec() > 2.0:
                rospy.logerr("UGV controller is not connected to the goal topic")
                self.set_phase("ERROR_CONTROLLER")
            return
        if self.final_target_ugv is None or self.ugv_odom is None:
            return

        target_x, target_y = self.final_target_ugv[:2]
        ugv_position = self.ugv_odom.pose.pose.position
        dx, dy = target_x - ugv_position.x, target_y - ugv_position.y
        distance = math.hypot(dx, dy)
        if distance < self.ugv_standoff:
            rospy.logerr("UGV is already too close to the sphere for a safe dispatch")
            self.set_phase("ERROR_TARGET")
            return

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.ugv_odom_frame
        goal.pose.position.x = target_x - self.ugv_standoff * dx / distance
        goal.pose.position.y = target_y - self.ugv_standoff * dy / distance
        yaw = math.atan2(dy, dx)
        quaternion = quaternion_from_euler(0.0, 0.0, yaw)
        goal.pose.orientation.x, goal.pose.orientation.y = quaternion[:2]
        goal.pose.orientation.z, goal.pose.orientation.w = quaternion[2:]
        self.goal_pub.publish(goal)
        rospy.loginfo("Dispatching UGV to (%.3f, %.3f) from nadir sphere estimate (%.3f, %.3f)",
                      goal.pose.position.x, goal.pose.position.y, target_x, target_y)
        self.set_phase("OVERWATCH")

    def registration_command(self):
        x, y = registration_waypoint(
            self.home[0], self.home[1], self.home_yaw,
            self.registration_dx, self.registration_dy)
        return x, y, self.registration_altitude, self.home_yaw

    def tick(self, _event):
        with self.state_lock:
            self._tick_locked(_event)

    def _tick_locked(self, _event):
        if self.odom is None or self.home is None or not self.state.connected:
            return
        now = rospy.Time.now()
        elapsed = (now - self.started).to_sec()
        position = self.odom.pose.pose.position
        speed = self.speed(self.odom)
        registration_x, registration_y = self.registration_command()[:2]

        if self.phase == "WAIT":
            self.publish_command(self.home[0], self.home[1], max(0.15, self.home[2]), self.home_yaw)
            self.set_phase("PRESTREAM")
        elif self.phase == "PRESTREAM":
            self.publish_command(self.home[0], self.home[1], max(0.15, self.home[2]), self.home_yaw)
            if elapsed > 2.0:
                try:
                    if self.mode_service(0, "OFFBOARD").mode_sent:
                        self.set_phase("OFFBOARD")
                except rospy.ServiceException as error:
                    rospy.logwarn_throttle(2.0, "OFFBOARD request failed: %s", error)
        elif self.phase == "OFFBOARD":
            self.publish_command(self.home[0], self.home[1], 0.15, self.home_yaw)
            if self.state.mode == "OFFBOARD":
                try:
                    if self.arm_service(True).success:
                        self.set_phase("ARM")
                except rospy.ServiceException as error:
                    rospy.logwarn_throttle(2.0, "Arm request failed: %s", error)
        elif self.phase == "ARM":
            self.publish_command(self.home[0], self.home[1], 0.15, self.home_yaw)
            if self.state.armed:
                self.set_phase("TAKEOFF")
        elif self.phase == "TAKEOFF":
            self.publish_command(self.home[0], self.home[1],
                                 self.registration_altitude, self.home_yaw)
            if (math.hypot(position.x - self.home[0], position.y - self.home[1]) <=
                    self.waypoint_tolerance and
                    abs(position.z - self.registration_altitude) <= self.waypoint_tolerance and
                    speed <= 0.20):
                self.set_phase("MOVE_TO_REGISTRATION")
            elif elapsed > self.takeoff_timeout:
                rospy.logerr("UAV did not complete the vertical takeoff")
                self.set_phase("ERROR_TAKEOFF")
        elif self.phase == "MOVE_TO_REGISTRATION":
            self.publish_command(*self.registration_command())
            if (math.hypot(position.x - registration_x, position.y - registration_y) <=
                    self.waypoint_tolerance and
                    abs(position.z - self.registration_altitude) <= self.waypoint_tolerance and
                    speed <= 0.15):
                self.set_phase("WAIT_REGISTRATION")
            elif elapsed > self.registration_move_timeout:
                rospy.logerr("UAV did not reach the visual registration point")
                self.set_phase("ERROR_REGISTRATION")
        elif self.phase == "WAIT_REGISTRATION":
            self.publish_command(*self.registration_command())
            if self.frozen:
                self.set_phase("CLIMB_FOR_SCAN")
            elif elapsed > 60.0:
                rospy.logerr("Visual registration did not freeze within 60 seconds")
                self.set_phase("ERROR_REGISTRATION")
        elif self.phase == "CLIMB_FOR_SCAN":
            self.publish_command(registration_x, registration_y, self.scan_altitude, self.home_yaw)
            if (math.hypot(position.x - registration_x, position.y - registration_y) <= self.waypoint_tolerance and
                    abs(position.z - self.scan_altitude) <= self.waypoint_tolerance and speed <= 0.20):
                self.scan_index = 0
                self.scan_yaw = self.home_yaw
                self.set_phase("FRONT_SCAN")
            elif elapsed > 30.0:
                rospy.logerr("UAV did not reach the front-camera scan altitude")
                self.set_phase("ERROR_APPROACH")
        elif self.phase == "FRONT_SCAN":
            self.scan_yaw = self.home_yaw + self.scan_index * self.scan_step
            self.publish_command(registration_x, registration_y, self.scan_altitude, self.scan_yaw)
            step_elapsed = (now - self.scan_step_started).to_sec()
            candidate = self.stable_target(
                self.front_samples, self.candidate_samples, self.candidate_spread)
            if candidate is not None and step_elapsed >= self.scan_settle_time:
                rospy.loginfo("Front camera found a sphere candidate at yaw %.1f deg",
                              math.degrees(normalize_angle(self.scan_yaw)))
                self.set_phase("FRONT_CONFIRM")
            elif step_elapsed >= self.scan_dwell:
                self.scan_index += 1
                if self.scan_index >= self.scan_steps:
                    rospy.logerr("Front camera completed a full scan without a confirmed sphere")
                    self.set_phase("ERROR_TARGET")
                else:
                    self.front_samples.clear()
                    self.scan_step_started = now
        elif self.phase == "FRONT_CONFIRM":
            self.publish_command(registration_x, registration_y, self.scan_altitude, self.scan_yaw)
            confirmed = self.stable_target(
                self.front_samples, self.confirmation_samples, self.confirmation_spread)
            if confirmed is not None:
                self.front_target_odom = confirmed[0]
                distance = math.hypot(self.front_target_odom[0] - position.x,
                                      self.front_target_odom[1] - position.y)
                rospy.loginfo("Confirmed front-camera sphere target %.2f m away", distance)
                self.set_phase("FRONT_APPROACH")
            elif elapsed > self.confirmation_timeout:
                rospy.logwarn("Front-camera candidate did not pass stable confirmation")
                self.scan_index += 1
                if self.scan_index >= self.scan_steps:
                    self.set_phase("ERROR_TARGET")
                else:
                    self.set_phase("FRONT_SCAN")
        elif self.phase == "FRONT_APPROACH":
            updated = self.stable_target(self.front_samples, 5, 0.6)
            if updated is not None:
                self.front_target_odom = updated[0]
            dx = self.front_target_odom[0] - position.x
            dy = self.front_target_odom[1] - position.y
            distance = max(0.01, math.hypot(dx, dy))
            standoff = min(self.front_approach_standoff, 0.5 * distance)
            approach_x = self.front_target_odom[0] - standoff * dx / distance
            approach_y = self.front_target_odom[1] - standoff * dy / distance
            self.approach_yaw = math.atan2(dy, dx)
            self.publish_command(approach_x, approach_y, self.approach_altitude, self.approach_yaw)
            if distance <= self.front_approach_standoff + 0.4 and speed <= 0.30:
                self.set_phase("CAMERA_HANDOFF")
            elif elapsed > self.approach_timeout:
                rospy.logerr("UAV did not reach the camera handoff area")
                self.set_phase("ERROR_APPROACH")
        elif self.phase == "CAMERA_HANDOFF":
            pattern_index = min(int(elapsed / self.handoff_dwell), len(self.handoff_pattern) - 1)
            offset_x, offset_y = self.handoff_pattern[pattern_index]
            self.publish_command(self.front_target_odom[0] + offset_x,
                                 self.front_target_odom[1] + offset_y,
                                 self.center_altitude, self.approach_yaw)
            nadir = self.stable_target(
                self.nadir_samples, self.handoff_samples, self.handoff_spread)
            if nadir is not None:
                disagreement = math.hypot(nadir[0][0] - self.front_target_odom[0],
                                          nadir[0][1] - self.front_target_odom[1])
                if disagreement > self.maximum_handoff_disagreement:
                    rospy.logerr("Front/nadir target disagreement is %.2f m", disagreement)
                    self.set_phase("ERROR_COORDINATE")
                else:
                    self.handoff_target_odom = nadir[0]
                    self.center_target_odom = nadir[0]
                    rospy.loginfo("Nadir camera acquired the sphere; handoff disagreement %.2f m",
                                  disagreement)
                    self.set_phase("CENTER_OVER_SPHERE")
            elif elapsed > self.handoff_timeout:
                rospy.logerr("Nadir camera did not acquire the front-camera target")
                self.set_phase("ERROR_TARGET")
        elif self.phase == "CENTER_OVER_SPHERE":
            nadir = self.stable_target(self.nadir_samples, 5, 0.25)
            if nadir is not None:
                self.center_target_odom = nadir[0]
            self.publish_command(self.center_target_odom[0], self.center_target_odom[1],
                                 self.center_altitude, self.approach_yaw)
            final_candidate = self.stable_target(
                self.nadir_samples, self.center_samples, self.center_spread)
            position_error = math.hypot(position.x - self.center_target_odom[0],
                                        position.y - self.center_target_odom[1])
            if (final_candidate is not None and position_error <= self.center_tolerance and
                    abs(position.z - self.center_altitude) <= self.waypoint_tolerance and
                    speed <= self.maximum_sample_speed):
                self.center_target_odom = final_candidate[0]
                self.set_phase("FINAL_ESTIMATE")
            elif elapsed > self.center_timeout:
                rospy.logerr("UAV could not center above the sphere")
                self.set_phase("ERROR_TARGET")
        elif self.phase == "FINAL_ESTIMATE":
            self.publish_command(self.center_target_odom[0], self.center_target_odom[1],
                                 self.center_altitude, self.approach_yaw)
            final = self.stable_target(self.nadir_samples, self.final_samples, self.final_spread)
            if final is not None:
                self.preserve_final_estimate(final)
                if self.handoff_target_odom is not None:
                    disagreement = math.hypot(
                        self.handoff_target_odom[0] - final[0][0],
                        self.handoff_target_odom[1] - final[0][1])
                    if disagreement > self.maximum_camera_disagreement:
                        rospy.logerr("Handoff and final nadir coordinates differ by %.2f m",
                                     disagreement)
                        self.set_phase("ERROR_COORDINATE")
                        return
                    rospy.loginfo("Nadir target drift after handoff: %.2f m", disagreement)
                self._process_preserved_target_locked()
            elif (self.uncertainty_aware_handoff and
                  self.preserved_target_odom is not None and
                  self.pending_handoff_action is not None):
                self._evaluate_preserved_handoff_locked()
            elif elapsed > self.final_timeout:
                rospy.logerr("No stable final nadir estimate")
                self.set_phase("ERROR_TARGET")
        elif self.phase == "RETURN_TO_UGV":
            if elapsed > self.registration_move_timeout:
                rospy.logerr("UAV did not return to the visual registration point")
                self.set_phase("ERROR_REGISTRATION")
                return
            command = self.reregistration_command()
            if command is None:
                self.publish_last_command()
            else:
                self.publish_command(*command)
                if (math.hypot(position.x - command[0], position.y - command[1]) <=
                        self.waypoint_tolerance and
                        abs(position.z - self.registration_altitude) <=
                        self.waypoint_tolerance and speed <= 0.15):
                    self.set_phase("WAIT_REREGISTRATION")
                    return
        elif self.phase == "WAIT_REREGISTRATION":
            if elapsed > self.reregistration_timeout:
                rospy.logerr("Visual re-registration did not produce a newer revision")
                self.set_phase("ERROR_REGISTRATION")
                return
            command = self.reregistration_command()
            if command is None:
                self.publish_last_command()
                return
            self.publish_command(*command)
            if self.registration_revision > self.baseline_revision:
                self.set_phase("RESUME_HANDOFF")
        elif self.phase == "RESUME_HANDOFF":
            self.publish_last_command()
            if (self.preserved_target_odom is not None and
                    self.publish_final_target(self.preserved_target_odom)):
                self.set_phase("DISPATCH")
        elif self.phase == "DISPATCH":
            self.publish_command(self.center_target_odom[0], self.center_target_odom[1],
                                 self.center_altitude, self.approach_yaw)
            self.dispatch_goal()
        elif self.phase == "OVERWATCH":
            self.publish_command(self.center_target_odom[0], self.center_target_odom[1],
                                 self.overwatch_altitude, self.approach_yaw)
        else:
            self.publish_last_command()


if __name__ == "__main__":
    rospy.init_node("uav_sphere_mission")
    Mission()
    rospy.spin()

#!/usr/bin/env python3
"""Register UAV and UGV frames from robust, uncertainty-aware visual windows."""

from collections import deque
import math

import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String, UInt32
from tf.transformations import concatenate_matrices, euler_from_quaternion
from tf.transformations import quaternion_from_euler, quaternion_from_matrix, quaternion_matrix, translation_matrix
from tf2_ros import TransformBroadcaster

from air_ground_coordinate_transform.msg import RegistrationUpdate
from air_ground_coordinate_transform.acquisition_diagnostics import (
    AcquisitionDiagnostics,
    REASON_BELOW_HEIGHT,
    REASON_INTERPOLATION,
    REASON_NO_ODOM,
    REASON_ODOM_BRACKET,
    REASON_STAMP_ZERO,
    REASON_UGV_FAST,
    REASON_UAV_FAST,
    REASON_UAV_YAW_FAST,
)
from air_ground_coordinate_transform.odom_buffer import OdomBuffer
from air_ground_coordinate_transform.registration_coordinator import RegistrationCoordinator
from air_ground_coordinate_transform.registration_estimator import (
    RegistrationFilter,
    RobustBatchEstimator,
    fixed_yaw_estimate,
    registration_sample_from_observation,
    resolve_observation_input_frame,
    valid_observation_frame,
    valid_odom_frames,
)
from air_ground_coordinate_transform.se2 import (
    compose,
    matrix_from_xyyaw,
    xyyaw_from_matrix,
)


def transform_matrix(translation, rpy):
    return concatenate_matrices(translation_matrix(translation), quaternion_matrix(quaternion_from_euler(*rpy)))


def finite_odometry(message):
    pose = message.pose.pose
    twist = message.twist.twist
    values = (
        message.header.stamp.to_sec(),
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
        twist.linear.x,
        twist.linear.y,
        twist.linear.z,
        twist.angular.x,
        twist.angular.y,
        twist.angular.z,
    )
    return (
        all(math.isfinite(float(value)) for value in values)
        and values[0] >= 0.0
        and sum(float(value) ** 2 for value in values[4:8]) > 0.0
    )


class Registration:
    def __init__(self):
        self.min_origin = int(rospy.get_param("~minimum_origin_samples", 30))
        self.min_samples = int(rospy.get_param("~minimum_samples", 30))
        self.period = float(rospy.get_param("~sample_period", 0.10))
        self.max_bracket = float(rospy.get_param("~max_odom_bracket", 0.08))
        self.max_translation = float(rospy.get_param("~max_translation_residual", 0.12))
        self.max_yaw = float(rospy.get_param("~max_yaw_residual", 0.07))
        self.minimum_translation_sigma = float(
            rospy.get_param("~minimum_translation_sigma", 0.01)
        )
        self.minimum_yaw_sigma = float(rospy.get_param("~minimum_yaw_sigma", 0.005))
        self.align_origin_to_uav_heading = bool(
            rospy.get_param("~align_origin_to_uav_heading", True))
        self.fixed_origin_yaw = float(rospy.get_param("~fixed_origin_yaw", 0.0))
        self.use_visual_frame_yaw = bool(rospy.get_param("~use_visual_frame_yaw", True))
        self.fixed_frame_yaw = float(rospy.get_param("~fixed_frame_yaw", 0.0))
        self.minimum_uav_height = float(rospy.get_param("~minimum_uav_height", 1.2))
        self.maximum_uav_speed = float(rospy.get_param("~maximum_uav_speed", 0.10))
        self.maximum_uav_angular_speed = float(rospy.get_param("~maximum_uav_angular_speed", 0.10))
        self.maximum_ugv_speed = float(rospy.get_param("~maximum_ugv_speed", 0.03))
        self.registration_mode = rospy.get_param("~registration_mode", "one_shot")
        self.registration_window_seconds = float(
            rospy.get_param("~registration_window_seconds", 3.0)
        )
        self.registration_window_max_samples = int(
            rospy.get_param("~registration_window_max_samples", 60)
        )
        self.periodic_update_seconds = float(
            rospy.get_param("~periodic_update_seconds", 30.0)
        )
        self.degraded_covariance_trace_threshold = float(
            rospy.get_param("~degraded_covariance_trace_threshold", 0.25)
        )
        self.innovation_mahalanobis_threshold = float(
            rospy.get_param("~innovation_mahalanobis_threshold", 11.344866730144373)
        )
        self.origin_frame = rospy.get_param("~origin_frame", "air_ground_origin")
        self.uav_odom_frame = rospy.get_param("~uav_odom_frame", "iris_0/odom")
        self.uav_base_frame = rospy.get_param("~uav_base_frame", "iris_0/base_link")
        self.ugv_odom_frame = rospy.get_param("~ugv_odom_frame", "ugv_0/odom")
        self.ugv_base_frame = rospy.get_param("~ugv_base_frame", "ugv_0/base_link")
        self.camera_frame = rospy.get_param("~nadir_camera_frame", "iris_0/nadir_camera_optical_frame")
        self.uav_input_parent = rospy.get_param("~uav_odom_input_parent_frame", "map")
        self.uav_input_child = rospy.get_param("~uav_odom_input_child_frame", "base_link")
        self.ugv_input_parent = rospy.get_param("~ugv_odom_input_parent_frame", "ugv_0/odom")
        self.ugv_input_child = rospy.get_param("~ugv_odom_input_child_frame", "ugv_0/base_link")
        self.observation_input_frame = resolve_observation_input_frame(
            rospy.get_param, self.camera_frame
        )
        self.base_camera = transform_matrix(rospy.get_param("~uav_base_to_camera_translation"), rospy.get_param("~uav_base_to_camera_rpy"))
        self.base_board = transform_matrix(rospy.get_param("~ugv_base_to_board_translation"), rospy.get_param("~ugv_base_to_board_rpy"))
        self.uav = deque(maxlen=300)
        self.ugv = deque(maxlen=300)
        self.uav_buffer = OdomBuffer(maxlen=300, max_bracket=self.max_bracket)
        self.ugv_buffer = OdomBuffer(maxlen=300, max_bracket=self.max_bracket)
        self.origin_samples = []
        self.origin_to_uav_odom = None
        self.estimator = RobustBatchEstimator(
            self.min_samples,
            self.max_translation,
            self.max_yaw,
            self.minimum_translation_sigma,
            self.minimum_yaw_sigma,
        )
        process_noise = {
            name: float(rospy.get_param("~" + name))
            for name in RegistrationFilter._PROCESS_NOISE_NAMES
        }
        self.registration_filter = RegistrationFilter(None, None, process_noise)
        self.coordinator = RegistrationCoordinator(
            mode=self.registration_mode,
            registration_filter=self.registration_filter,
            estimator=self.estimator,
            registration_window_seconds=self.registration_window_seconds,
            registration_window_max_samples=self.registration_window_max_samples,
            sample_period=self.period,
            periodic_update_seconds=self.periodic_update_seconds,
            degraded_covariance_trace_threshold=self.degraded_covariance_trace_threshold,
            innovation_mahalanobis_threshold=self.innovation_mahalanobis_threshold,
            max_batch_coalesce_age=self.max_bracket,
            batch_postprocessor=self.apply_yaw_policy,
        )
        self.tf = TransformBroadcaster()
        # Observability only: tallies why observations fail the pre-existing
        # gates so a starving pipeline reports its reason instead of silence.
        self.acquisition_diagnostics = AcquisitionDiagnostics(throttle_seconds=5.0)
        self._last_idle_diagnostics = None
        self.valid_pub = rospy.Publisher("/air_ground/registration/valid", Bool, queue_size=1, latch=True)
        self.frozen_pub = rospy.Publisher("/air_ground/registration/frozen", Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("/air_ground/registration/status", String, queue_size=1, latch=True)
        self.count_pub = rospy.Publisher("/air_ground/registration/inlier_count", UInt32, queue_size=1, latch=True)
        self.estimate_pub = rospy.Publisher("/air_ground/registration/estimate", PoseWithCovarianceStamped, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher(
            "/air_ground/registration/state",
            RegistrationUpdate,
            queue_size=1,
            latch=True,
        )
        self.accepted_update_pub = rospy.Publisher(
            "/air_ground/registration/accepted_update",
            RegistrationUpdate,
            queue_size=1,
            latch=True,
        )
        self.revision_pub = rospy.Publisher("/air_ground/registration/revision", UInt32, queue_size=1, latch=True)
        self.innovation_pub = rospy.Publisher("/air_ground/registration/innovation", Float64, queue_size=1, latch=True)
        self.pose_pub = rospy.Publisher("/air_ground/ugv/pose_takeoff", PoseWithCovarianceStamped, queue_size=2)
        rospy.Subscriber(rospy.get_param("~uav_odom_topic"), Odometry, self.uav_callback, queue_size=100)
        rospy.Subscriber(rospy.get_param("~ugv_odom_topic"), Odometry, self.ugv_callback, queue_size=100)
        rospy.Subscriber(rospy.get_param("~observation_topic"), PoseWithCovarianceStamped, self.observation_callback, queue_size=20)
        rospy.Timer(rospy.Duration(0.05), self.publish)
        self.valid_pub.publish(False)
        self.frozen_pub.publish(False)
        self.revision_pub.publish(0)
        self.status_pub.publish("CAPTURING_ORIGIN")

    def uav_callback(self, message):
        if not valid_odom_frames(
            message.header.frame_id,
            message.child_frame_id,
            self.uav_input_parent,
            self.uav_input_child,
        ):
            rospy.logwarn_throttle(
                2.0,
                "Ignoring UAV odometry frame '%s' -> '%s'; expected '%s' -> '%s'",
                message.header.frame_id,
                message.child_frame_id,
                self.uav_input_parent,
                self.uav_input_child,
            )
            return
        if not finite_odometry(message):
            rospy.logwarn_throttle(2.0, "Ignoring nonfinite UAV odometry")
            return
        with self.coordinator.lock:
            position = message.pose.pose.position
            acceptance = self.coordinator.observe_odometry(
                "uav", message.header.stamp.to_sec(), position.x, position.y
            )
            if not acceptance.accepted:
                rospy.logwarn_throttle(
                    2.0, "Ignoring UAV odometry: %s", acceptance.reason
                )
                return
            self.uav.append(message)
            self.uav_buffer.append_odometry(message)
            if self.origin_to_uav_odom is None:
                self.origin_samples.append(message)
                if len(self.origin_samples) >= self.min_origin:
                    positions = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for m in self.origin_samples])
                    yaws = np.array([euler_from_quaternion((m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w))[2] for m in self.origin_samples])
                    center = np.mean(positions, axis=0)
                    yaw = math.atan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws)))
                    origin_yaw = -yaw if self.align_origin_to_uav_heading else self.fixed_origin_yaw
                    rotation = np.array([[math.cos(origin_yaw), -math.sin(origin_yaw), 0],
                                         [math.sin(origin_yaw), math.cos(origin_yaw), 0],
                                         [0, 0, 1]])
                    translation = -np.dot(rotation, center)
                    self.origin_to_uav_odom = transform_matrix(
                        translation, (0.0, 0.0, origin_yaw)
                    )
                    status = (
                        "ACQUIRING_REGISTRATION"
                        if self.registration_mode == "one_shot"
                        else "ACQUIRING_INITIAL"
                    )
                    self.status_pub.publish(status)

    def ugv_callback(self, message):
        if not valid_odom_frames(
            message.header.frame_id,
            message.child_frame_id,
            self.ugv_input_parent,
            self.ugv_input_child,
        ):
            rospy.logwarn_throttle(
                2.0,
                "Ignoring UGV odometry frame '%s' -> '%s'; expected '%s' -> '%s'",
                message.header.frame_id,
                message.child_frame_id,
                self.ugv_input_parent,
                self.ugv_input_child,
            )
            return
        if not finite_odometry(message):
            rospy.logwarn_throttle(2.0, "Ignoring nonfinite UGV odometry")
            return
        with self.coordinator.lock:
            position = message.pose.pose.position
            acceptance = self.coordinator.observe_odometry(
                "ugv", message.header.stamp.to_sec(), position.x, position.y
            )
            if not acceptance.accepted:
                rospy.logwarn_throttle(
                    2.0, "Ignoring UGV odometry: %s", acceptance.reason
                )
                return
            self.ugv.append(message)
            self.ugv_buffer.append_odometry(message)

    def observation_callback(self, observation):
        with self.coordinator.lock:
            snapshot = self.coordinator.snapshot()
            if (
                self.origin_to_uav_odom is None
                or (self.registration_mode == "one_shot" and snapshot.state.initialized)
            ):
                return
            if not valid_observation_frame(
                observation.header.frame_id, self.observation_input_frame
            ):
                rospy.logwarn_throttle(
                    2.0,
                    "Ignoring board observation frame '%s'; expected '%s'",
                    observation.header.frame_id,
                    self.observation_input_frame,
                )
                return
            stamp = observation.header.stamp
            if stamp.is_zero():
                self.acquisition_diagnostics.observe()
                self.acquisition_diagnostics.drop(REASON_STAMP_ZERO)
                return
            uav_message = min(self.uav, key=lambda message: abs((message.header.stamp - stamp).to_sec())) if self.uav else None
            ugv_message = min(self.ugv, key=lambda message: abs((message.header.stamp - stamp).to_sec())) if self.ugv else None
            self.acquisition_diagnostics.observe()
            if uav_message is None or ugv_message is None:
                self.acquisition_diagnostics.drop(REASON_NO_ODOM)
                return
            bracket_uav = abs((uav_message.header.stamp - stamp).to_sec())
            bracket_ugv = abs((ugv_message.header.stamp - stamp).to_sec())
            if bracket_uav > self.max_bracket or bracket_ugv > self.max_bracket:
                self.acquisition_diagnostics.drop(REASON_ODOM_BRACKET)
                return
            uav_linear, uav_angular = uav_message.twist.twist.linear, uav_message.twist.twist.angular
            ugv_linear = ugv_message.twist.twist.linear
            if uav_message.pose.pose.position.z < self.minimum_uav_height:
                self.acquisition_diagnostics.drop(REASON_BELOW_HEIGHT)
                return
            if math.sqrt(uav_linear.x ** 2 + uav_linear.y ** 2 + uav_linear.z ** 2) > self.maximum_uav_speed:
                self.acquisition_diagnostics.drop(REASON_UAV_FAST)
                return
            if math.sqrt(uav_angular.x ** 2 + uav_angular.y ** 2 + uav_angular.z ** 2) > self.maximum_uav_angular_speed:
                self.acquisition_diagnostics.drop(REASON_UAV_YAW_FAST)
                return
            if math.hypot(ugv_linear.x, ugv_linear.y) > self.maximum_ugv_speed:
                self.acquisition_diagnostics.drop(REASON_UGV_FAST)
                return
            uav = self.uav_buffer.interpolate_full(stamp)
            ugv = self.ugv_buffer.interpolate_full(stamp)
            if uav is None or ugv is None:
                self.acquisition_diagnostics.drop(REASON_INTERPOLATION)
                return
            self.acquisition_diagnostics.accept()
            pose = observation.pose.pose
            quaternion = pose.orientation
            observation_mean = np.array(
                [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    *euler_from_quaternion(
                        (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
                    ),
                ]
            )
            covariance = np.asarray(observation.pose.covariance, dtype=float).reshape(6, 6)
            sample = registration_sample_from_observation(
                origin_to_uav_odom=self.origin_to_uav_odom,
                uav_pose=uav,
                base_camera=self.base_camera,
                observation_mean=observation_mean,
                observation_covariance=covariance,
                ugv_pose=ugv,
                base_board=self.base_board,
                anchor=ugv[:2],
                stamp=stamp.to_sec(),
            )
            decision = self.coordinator.add_sample(
                sample, now=rospy.Time.now().to_sec()
            )
            if decision is not None:
                self.publish_decision(decision)

    def apply_yaw_policy(self, samples, inlier_indices, estimate):
        if self.use_visual_frame_yaw:
            return estimate
        return fixed_yaw_estimate(
            samples,
            inlier_indices,
            self.fixed_frame_yaw,
            self.minimum_translation_sigma,
            self.minimum_yaw_sigma,
        )

    def publish_decision(self, decision):
        if decision.accepted:
            self.count_pub.publish(decision.inlier_count)
            self.valid_pub.publish(True)
            self.frozen_pub.publish(True)
            if math.isfinite(decision.mahalanobis):
                self.innovation_pub.publish(decision.mahalanobis)
            frozen = matrix_from_xyyaw(*decision.state.mean)
            self.send_tf(
                self.origin_frame,
                self.ugv_odom_frame,
                frozen,
                rospy.Time.now(),
            )
            estimate_message = self.publish_estimate(decision.state, decision.revision)
            accepted_update = RegistrationUpdate()
            accepted_update.header.stamp = estimate_message.header.stamp
            accepted_update.header.frame_id = estimate_message.header.frame_id
            accepted_update.revision = decision.revision
            accepted_update.pose = self._serialize_pose(decision.state)
            self.accepted_update_pub.publish(accepted_update)
            self.revision_pub.publish(decision.revision)
            self.status_pub.publish(decision.status)
            rospy.loginfo(
                "Accepted registration revision %d with %d inliers, NIS %.6g",
                decision.revision,
                decision.inlier_count,
                decision.mahalanobis,
            )
            return
        self.count_pub.publish(decision.inlier_count)
        if math.isfinite(decision.mahalanobis):
            self.innovation_pub.publish(decision.mahalanobis)
        self.status_pub.publish("REJECTED")
        rospy.logwarn_throttle(
            2.0,
            "Rejected registration window: %s (NIS=%s)",
            decision.reason,
            "{:.6g}".format(decision.mahalanobis)
            if math.isfinite(decision.mahalanobis)
            else "not-computed",
        )

    @staticmethod
    def _serialize_pose(estimate):
        pose = PoseWithCovarianceStamped().pose
        mean = estimate.mean
        pose.pose.position.x = mean[0]
        pose.pose.position.y = mean[1]
        quaternion = quaternion_from_euler(0.0, 0.0, mean[2])
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = quaternion
        covariance_indices = (0, 1, 5)
        for row, target_row in enumerate(covariance_indices):
            for column, target_column in enumerate(covariance_indices):
                pose.covariance[target_row * 6 + target_column] = (
                    estimate.covariance[row, column]
                )
        return pose

    def publish_estimate(self, estimate, revision=None):
        message = PoseWithCovarianceStamped()
        message.header.stamp = rospy.Time.from_sec(estimate.stamp)
        message.header.frame_id = self.origin_frame
        message.pose = self._serialize_pose(estimate)
        self.estimate_pub.publish(message)

        state = RegistrationUpdate()
        state.header.stamp = message.header.stamp
        state.header.frame_id = message.header.frame_id
        state.revision = estimate.revision if revision is None else revision
        state.pose = self._serialize_pose(estimate)
        self.state_pub.publish(state)
        return message

    def send_tf(self, parent, child, matrix, stamp):
        message = TransformStamped()
        message.header.stamp = stamp
        message.header.frame_id, message.child_frame_id = parent, child
        if matrix.shape == (3, 3):
            mean = xyyaw_from_matrix(matrix)
            message.transform.translation.x = mean[0]
            message.transform.translation.y = mean[1]
            message.transform.translation.z = 0.0
            q = quaternion_from_euler(0.0, 0.0, mean[2])
        else:
            message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = matrix[:3, 3]
            q = quaternion_from_matrix(matrix)
        message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z, message.transform.rotation.w = q
        self.tf.sendTransform(message)

    def _idle_diagnostics_due(self, now):
        """Periodic line even when NO observations arrive at all."""
        if self.acquisition_diagnostics.received > 0:
            return False
        if (
            self._last_idle_diagnostics is None
            or now - self._last_idle_diagnostics >= 10.0
        ):
            self._last_idle_diagnostics = now
            return True
        return False

    def publish(self, _event):
        with self.coordinator.lock:
            now = rospy.Time.now()
            decision = self.coordinator.tick(now.to_sec())
            if decision is not None:
                self.publish_decision(decision)
                self.coordinator.complete_publication_cycle()
                return
            if self.origin_to_uav_odom is not None:
                self.send_tf(self.origin_frame, self.uav_odom_frame, self.origin_to_uav_odom, now)
            snapshot = self.coordinator.snapshot()
            state = snapshot.state
            if self.origin_to_uav_odom is not None and not state.initialized:
                diagnostics_now = now.to_sec()
                if (
                    self.acquisition_diagnostics.should_report(diagnostics_now)
                    or self._idle_diagnostics_due(diagnostics_now)
                ):
                    rospy.logwarn_throttle(
                        1.0,
                        "%s",
                        self.acquisition_diagnostics.summary(),
                    )
            if self.origin_to_uav_odom is None:
                self.status_pub.publish("CAPTURING_ORIGIN")
            elif not state.initialized:
                status = (
                    "ACQUIRING_REGISTRATION"
                    if self.registration_mode == "one_shot"
                    else "ACQUIRING_INITIAL"
                )
                self.status_pub.publish(status)
            else:
                self.status_pub.publish(snapshot.status)
            self.valid_pub.publish(state.initialized)
            self.frozen_pub.publish(state.initialized)
            if state.initialized:
                self.publish_estimate(state)
                frozen = matrix_from_xyyaw(*state.mean)
                self.send_tf(self.origin_frame, self.ugv_odom_frame, frozen, now)
                if self.ugv:
                    target = PoseWithCovarianceStamped()
                    target.header.stamp, target.header.frame_id = self.ugv[-1].header.stamp, self.origin_frame
                    pose = self.ugv[-1].pose.pose
                    quaternion = pose.orientation
                    ugv = matrix_from_xyyaw(
                        pose.position.x,
                        pose.position.y,
                        euler_from_quaternion(
                            (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
                        )[2],
                    )
                    matrix = compose(frozen, ugv)
                    mean = xyyaw_from_matrix(matrix)
                    target.pose.pose.position.x = mean[0]
                    target.pose.pose.position.y = mean[1]
                    target.pose.pose.position.z = pose.position.z
                    q = quaternion_from_euler(0.0, 0.0, mean[2])
                    target.pose.pose.orientation.x, target.pose.pose.orientation.y, target.pose.pose.orientation.z, target.pose.pose.orientation.w = q
                    self.pose_pub.publish(target)
            self.revision_pub.publish(state.revision)
            self.coordinator.complete_publication_cycle()


if __name__ == "__main__":
    rospy.init_node("takeoff_registration")
    Registration()
    rospy.spin()

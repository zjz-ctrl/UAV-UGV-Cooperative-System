#!/usr/bin/env python3
"""Release only real detector observations through a deterministic impairment queue."""

import copy

import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

from air_ground_experiments.frame_perturbation import (
    ObservationGateSchedule,
    diagnostic_json,
    domain_seed,
    inject_pose_outlier,
)


def _pose_record(message):
    pose = message.pose.pose
    return {
        "position": np.array([pose.position.x, pose.position.y, pose.position.z]),
        "orientation": np.array(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        ),
        "covariance": np.asarray(message.pose.covariance).reshape(6, 6),
    }


def _populate_pose(message, record):
    pose = message.pose.pose
    pose.position.x, pose.position.y, pose.position.z = record["position"]
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = record["orientation"]
    message.pose.covariance = record["covariance"].reshape(-1).tolist()


class ObservationGateNode:
    def __init__(self):
        self.trial_seed = int(rospy.get_param("~seed", 0))
        epoch = float(rospy.get_param("~epoch_seconds", 0.0))
        self.schedule = ObservationGateSchedule(
            visibility_windows=rospy.get_param(
                "~visibility_windows", [[0.0, 1.0e12]]
            ),
            delay_seconds=float(rospy.get_param("~delay_seconds", 0.0)),
            delay_jitter_seconds=float(
                rospy.get_param("~delay_jitter_seconds", 0.0)
            ),
            outlier_probability=float(rospy.get_param("~outlier_probability", 0.0)),
            outlier_translation_m=float(
                rospy.get_param("~outlier_translation_m", 0.0)
            ),
            outlier_yaw_rad=float(rospy.get_param("~outlier_yaw_rad", 0.0)),
            seed=domain_seed(
                self.trial_seed, rospy.get_param("~seed_domain", "gate")
            ),
            visibility_probability=float(
                rospy.get_param("~visibility_probability", 1.0)
            ),
            epoch_seconds=epoch,
        )
        # Runtime scenario control. Disabled unless ~control_topic is set;
        # commands are input-level impairments only (hide / gross outlier).
        self.control_mode = "pass"
        self.manual_outlier = (0.0, 0.0, 0.0)
        self.publisher = rospy.Publisher(
            rospy.get_param(
                "~destination_topic", "/air_ground_experiment/charuco/observation"
            ),
            PoseWithCovarianceStamped,
            queue_size=20,
        )
        self.diagnostic_publisher = rospy.Publisher(
            rospy.get_param(
                "~diagnostic_topic", "/air_ground_experiment/charuco/injected_delay"
            ),
            String,
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~source_topic", "/air_ground/charuco/observation"),
            PoseWithCovarianceStamped,
            self.callback,
            queue_size=20,
        )
        control_topic = rospy.get_param("~control_topic", "")
        if control_topic:
            rospy.Subscriber(
                control_topic,
                String,
                self.control_callback,
                queue_size=5,
            )
        rospy.Timer(rospy.Duration(0.01), self.release_ready)

    def control_callback(self, message):
        fields = str(message.data).split()
        if not fields:
            return
        command = fields[0]
        if command == "pass" and len(fields) == 1:
            self.control_mode = "pass"
            self.manual_outlier = (0.0, 0.0, 0.0)
        elif command == "hide" and len(fields) == 1:
            self.control_mode = "hide"
        elif command == "outlier" and len(fields) == 4:
            try:
                outlier = tuple(float(value) for value in fields[1:4])
            except ValueError:
                rospy.logwarn_throttle(2.0, "Invalid outlier control: %s", message.data)
                return
            self.control_mode = "outlier"
            self.manual_outlier = outlier
        else:
            rospy.logwarn_throttle(2.0, "Unknown gate control command: %s", message.data)
            return
        rospy.loginfo(
            "Observation gate control: mode=%s outlier=%s",
            self.control_mode,
            self.manual_outlier,
        )

    def callback(self, message):
        if self.control_mode == "hide":
            rospy.logwarn_throttle(
                5.0,
                "Observation at %.3f dropped by hide control",
                message.header.stamp.to_sec(),
            )
            return
        try:
            accepted = self.schedule.enqueue(
                copy.deepcopy(message),
                message.header.stamp.to_sec(),
                rospy.Time.now().to_sec(),
            )
        except ValueError as error:
            rospy.logwarn_throttle(
                2.0,
                "Dropping observation outside the trial window: %s",
                error,
            )
            return
        if not accepted:
            rospy.logwarn_throttle(
                5.0,
                "Observation at %.3f falls outside configured visibility",
                message.header.stamp.to_sec(),
            )

    def release_ready(self, _event):
        now = rospy.Time.now().to_sec()
        for item in self.schedule.release_ready(now):
            message = item.payload
            effective_outlier = item.outlier_xyyaw
            if any(item.outlier_xyyaw):
                _populate_pose(
                    message,
                    inject_pose_outlier(_pose_record(message), item.outlier_xyyaw),
                )
            if self.control_mode == "outlier" and any(self.manual_outlier):
                effective_outlier = self.manual_outlier
                _populate_pose(
                    message,
                    inject_pose_outlier(_pose_record(message), self.manual_outlier),
                )
            self.publisher.publish(message)
            self.diagnostic_publisher.publish(
                String(
                    data=diagnostic_json(
                        item.image_stamp,
                        item.release_time,
                        item.injected_delay,
                        effective_outlier,
                        item.seed,
                        receipt_time=item.receipt_time,
                        actual_release=now,
                        trial_seed=self.trial_seed,
                    )
                )
            )


if __name__ == "__main__":
    rospy.init_node("observation_gate")
    ObservationGateNode()
    rospy.spin()

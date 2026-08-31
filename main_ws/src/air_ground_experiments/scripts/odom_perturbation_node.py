#!/usr/bin/env python3
"""Thin ROS adapter for deterministic independent-frame odometry."""

import copy

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from air_ground_experiments.frame_perturbation import (
    FramePerturbation,
    domain_seed,
    odometry_record,
    populate_odometry,
    truth_json,
)


class OdomPerturbationNode:
    def __init__(self):
        self.source_topic = rospy.get_param("~source_topic")
        self.destination_topic = rospy.get_param("~destination_topic")
        self.source_frame = rospy.get_param("~source_frame")
        self.destination_frame = rospy.get_param("~destination_frame")
        self.twist_convention = rospy.get_param("~twist_convention", "body")
        initial = rospy.get_param("~initial_xyyaw", [0.0, 0.0, 0.0])
        translation_rate = float(rospy.get_param("~translational_drift_rate", 0.0))
        yaw_rate = float(rospy.get_param("~yaw_drift_rate", 0.0))
        drift_step = float(rospy.get_param("~drift_step_seconds", 1.0))
        seed = int(rospy.get_param("~seed", 0))
        domain = rospy.get_param("~seed_domain", "default")
        effective_seed = domain_seed(seed, domain)
        epoch = float(rospy.get_param("~epoch_seconds", 0.0))
        maximum_elapsed = rospy.get_param("~maximum_elapsed_seconds", None)
        self.trial_seed = seed
        self.perturbation = FramePerturbation(
            initial,
            [translation_rate, translation_rate, yaw_rate],
            effective_seed,
            drift_step_seconds=drift_step,
            epoch_seconds=epoch,
            maximum_elapsed_seconds=(
                None if maximum_elapsed is None else float(maximum_elapsed)
            ),
        )
        self.odom_publisher = rospy.Publisher(
            self.destination_topic, Odometry, queue_size=20
        )
        self.truth_publisher = rospy.Publisher(
            rospy.get_param("~truth_topic"), String, queue_size=20
        )
        rospy.Subscriber(self.source_topic, Odometry, self.callback, queue_size=100)

    def callback(self, message):
        if message.header.frame_id != self.source_frame:
            rospy.logwarn_throttle(
                2.0,
                "Ignoring odometry frame '%s'; expected '%s'",
                message.header.frame_id,
                self.source_frame,
            )
            return
        stamp, pose, twist = odometry_record(message)
        try:
            transformed_pose, transformed_twist, truth = (
                self.perturbation.transform_odom(
                    pose, twist, stamp,
                    twist_convention=self.twist_convention,
                )
            )
        except ValueError as error:
            rospy.logwarn_throttle(
                2.0,
                "Dropping odometry outside the perturbation window: %s",
                error,
            )
            return
        output = populate_odometry(
            copy.deepcopy(message),
            transformed_pose,
            transformed_twist,
            self.destination_frame,
            twist_convention=self.twist_convention,
        )
        self.odom_publisher.publish(output)
        self.truth_publisher.publish(
            String(
                data=truth_json(
                    truth,
                    self.perturbation.seed,
                    stamp,
                    self.source_frame,
                    self.destination_frame,
                    trial_seed=self.trial_seed,
                )
            )
        )


if __name__ == "__main__":
    rospy.init_node("odom_perturbation")
    OdomPerturbationNode()
    rospy.spin()

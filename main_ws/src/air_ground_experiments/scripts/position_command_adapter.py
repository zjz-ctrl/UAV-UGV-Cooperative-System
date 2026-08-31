#!/usr/bin/env python3
"""Convert experiment-frame UAV commands back to the CXR controller frame."""

import copy

import numpy as np
import rospy
from quadrotor_msgs.msg import PositionCommand

from air_ground_experiments.frame_perturbation import (
    FramePerturbation,
    adapt_position_command,
    domain_seed,
)


def _vector(message):
    return np.array([message.x, message.y, message.z], dtype=float)


def _set_vector(message, values):
    message.x, message.y, message.z = (float(value) for value in values)


class PositionCommandAdapter:
    def __init__(self):
        initial = rospy.get_param("~initial_xyyaw", [0.0, 0.0, 0.0])
        translation_rate = float(rospy.get_param("~translational_drift_rate", 0.0))
        yaw_rate = float(rospy.get_param("~yaw_drift_rate", 0.0))
        self.destination_frame = rospy.get_param("~destination_frame", "iris_0/odom")
        epoch = float(rospy.get_param("~epoch_seconds", 0.0))
        maximum_elapsed = rospy.get_param("~maximum_elapsed_seconds", None)
        self.perturbation = FramePerturbation(
            initial,
            [translation_rate, translation_rate, yaw_rate],
            domain_seed(
                int(rospy.get_param("~seed", 0)),
                rospy.get_param("~seed_domain", "uav"),
            ),
            drift_step_seconds=float(rospy.get_param("~drift_step_seconds", 1.0)),
            epoch_seconds=epoch,
            maximum_elapsed_seconds=(
                None if maximum_elapsed is None else float(maximum_elapsed)
            ),
        )
        self.publisher = rospy.Publisher(
            rospy.get_param("~destination_topic", "/iris_0/position_cmd"),
            PositionCommand,
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~source_topic", "/air_ground_experiment/iris_0/position_cmd"
            ),
            PositionCommand,
            self.callback,
            queue_size=20,
        )

    def callback(self, message):
        stamp = message.header.stamp.to_sec()
        try:
            transform = self.perturbation.at(stamp)
        except ValueError as error:
            rospy.logwarn_throttle(
                2.0,
                "Dropping command outside the perturbation window: %s",
                error,
            )
            return
        command = {
            "stamp": stamp,
            "frame_id": message.header.frame_id,
            "position": _vector(message.position),
            "velocity": _vector(message.velocity),
            "acceleration": _vector(message.acceleration),
            "jerk": _vector(message.jerk),
            "yaw": message.yaw,
            "yaw_dot": message.yaw_dot,
            "kx": list(message.kx),
            "kv": list(message.kv),
            "trajectory_id": message.trajectory_id,
            "trajectory_flag": message.trajectory_flag,
        }
        adapted = adapt_position_command(
            command, transform, self.destination_frame
        )
        output = copy.deepcopy(message)
        output.header.frame_id = adapted["frame_id"]
        _set_vector(output.position, adapted["position"])
        _set_vector(output.velocity, adapted["velocity"])
        _set_vector(output.acceleration, adapted["acceleration"])
        _set_vector(output.jerk, adapted["jerk"])
        output.yaw = adapted["yaw"]
        output.yaw_dot = adapted["yaw_dot"]
        self.publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("position_command_adapter")
    PositionCommandAdapter()
    rospy.spin()

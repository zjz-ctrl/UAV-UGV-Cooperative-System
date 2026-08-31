#!/usr/bin/env python3
"""Evaluation-only one-shot trial recorder; never publishes autonomy inputs."""

import json
import math
import threading

import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

from air_ground_experiments.frame_perturbation import domain_seed
from air_ground_experiments.metrics import (
    TrialResultWriter,
    TrialTruthEvaluator,
    build_trial_row,
    classify_mission_phase,
    final_inspection_distance,
    handoff_error_2d,
    wrapped_yaw_error,
)

EVALUATION_STATUS_TOPIC = "/air_ground_experiment/evaluation/status"
TRUTH_TOPIC_UAV = "/air_ground_experiment/truth/uav_frame"
TRUTH_TOPIC_UGV = "/air_ground_experiment/truth/ugv_frame"
EXPERIMENT_UAV_ODOM_TOPIC = "/air_ground_experiment/uav/odom"
MISSION_PHASE_DEFAULT = "/air_ground/mission_phase"


def _planar_xyyaw_from_pose(pose):
    yaw = euler_from_quaternion(
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    )[2]
    return np.array([pose.position.x, pose.position.y, yaw])


class ExperimentRecorder:
    def __init__(self):
        self.trial_id = str(rospy.get_param("~trial_id", "trial-000"))
        self.seed = int(rospy.get_param("~seed", 0))
        self.timeout_seconds = float(rospy.get_param("~timeout_seconds", 120.0))
        self.success_radius = float(rospy.get_param("~success_radius_m", 0.5))
        self.uav_model = rospy.get_param("~uav_model", "iris_0")
        self.ugv_model = rospy.get_param("~ugv_model", "ugv_0")
        self.anomaly_model = rospy.get_param("~anomaly_model", "red_sphere")
        self.mission_phase_topic = rospy.get_param(
            "~mission_phase_topic", MISSION_PHASE_DEFAULT
        )
        self.relation_window_seconds = float(
            rospy.get_param("~source_relation_window_seconds", 30.0)
        )
        self.started = rospy.Time.now()
        self.finalized = False
        self._finalize_lock = threading.Lock()
        self.writer = TrialResultWriter(
            rospy.get_param("~output_directory", "/tmp/air_ground_experiments")
        )
        self.evaluator = TrialTruthEvaluator(
            minimum_anchor_samples=int(rospy.get_param("~minimum_anchor_samples", 30)),
            align_origin_to_uav_heading=bool(
                rospy.get_param("~align_origin_to_uav_heading", True)
            ),
            fixed_origin_yaw=float(rospy.get_param("~fixed_origin_yaw", 0.0)),
        )
        self.expected_stream_seeds = {
            "uav": domain_seed(self.seed, "uav"),
            "ugv": domain_seed(self.seed, "ugv"),
        }
        self.model_positions = {}
        self.estimate = None
        self.registration_status = "NOT_STARTED"
        self.status_publisher = rospy.Publisher(
            EVALUATION_STATUS_TOPIC, String, queue_size=1, latch=True
        )
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback, queue_size=1)
        rospy.Subscriber(TRUTH_TOPIC_UAV, String, self.uav_truth_callback, queue_size=50)
        rospy.Subscriber(TRUTH_TOPIC_UGV, String, self.ugv_truth_callback, queue_size=50)
        rospy.Subscriber(EXPERIMENT_UAV_ODOM_TOPIC, Odometry, self.anchor_odom_callback, queue_size=50)
        rospy.Subscriber("/air_ground/registration/estimate", PoseWithCovarianceStamped, self.estimate_callback, queue_size=1)
        rospy.Subscriber("/air_ground/registration/status", String, self.registration_callback, queue_size=1)
        rospy.Subscriber(self.mission_phase_topic, String, self.phase_callback, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.tick)

    def model_callback(self, message):
        for model_name in (self.uav_model, self.ugv_model, self.anomaly_model):
            if model_name in message.name:
                pose = message.pose[message.name.index(model_name)]
                self.model_positions[model_name] = np.array(
                    [pose.position.x, pose.position.y, pose.position.z], dtype=float
                )
        if (
            self.evaluator.source_relation is None
            and (rospy.Time.now() - self.started).to_sec() <= self.relation_window_seconds
            and self.uav_model in message.name
            and self.ugv_model in message.name
        ):
            uav_pose = message.pose[message.name.index(self.uav_model)]
            ugv_pose = message.pose[message.name.index(self.ugv_model)]
            try:
                self.evaluator.record_source_relation(
                    _planar_xyyaw_from_pose(uav_pose),
                    _planar_xyyaw_from_pose(ugv_pose),
                )
            except ValueError as error:
                rospy.logwarn_throttle(2.0, "Rejected source relation: %s", error)

    def anchor_odom_callback(self, message):
        try:
            self.evaluator.record_anchor_sample(
                _planar_xyyaw_from_pose(message.pose.pose)
            )
        except ValueError as error:
            rospy.logwarn_throttle(2.0, "Rejected anchor sample: %s", error)

    def _truth_callback(self, key, message):
        try:
            document = json.loads(message.data)
            if int(document["seed"]) != self.expected_stream_seeds[key]:
                return
            self.evaluator.record_truth(key, document)
        except (KeyError, TypeError, ValueError):
            rospy.logwarn_throttle(2.0, "Ignoring malformed experiment truth message")

    def uav_truth_callback(self, message):
        self._truth_callback("uav", message)

    def ugv_truth_callback(self, message):
        self._truth_callback("ugv", message)

    def estimate_callback(self, message):
        stamp = message.header.stamp.to_sec()
        estimate = _planar_xyyaw_from_pose(message.pose.pose)
        self.estimate = (stamp, estimate)

    def registration_callback(self, message):
        self.registration_status = message.data

    def phase_callback(self, message):
        outcome, failure_code = classify_mission_phase(message.data)
        if outcome == "SUCCESS":
            self.finish_completed()
        elif outcome == "FAILED":
            self.finish("FAILED", failure_code)

    def metrics(self):
        """Return (yaw_error, handoff_error, inspection_distance) or NaNs."""
        yaw_error = handoff_error = inspection_distance = float("nan")
        if self.estimate is not None:
            truth = self.evaluator.registration_truth_at(self.estimate[0])
            if truth is not None:
                yaw_error = wrapped_yaw_error(self.estimate[1][2], truth[2])
                handoff_error = handoff_error_2d(self.estimate[1], truth)
        ugv_position = self.model_positions.get(self.ugv_model)
        anomaly_position = self.model_positions.get(self.anomaly_model)
        if ugv_position is not None and anomaly_position is not None:
            inspection_distance = final_inspection_distance(
                ugv_position, anomaly_position
            )
        return yaw_error, handoff_error, inspection_distance

    def completion_failure_code(self):
        yaw_error, handoff_error, inspection_distance = self.metrics()
        if not math.isfinite(yaw_error) or not math.isfinite(handoff_error):
            return "INCOMPLETE_TRUTH_SYNC"
        if not math.isfinite(inspection_distance):
            return "ANOMALY_TRUTH_UNAVAILABLE"
        if inspection_distance > self.success_radius:
            return "OUTSIDE_SUCCESS_RADIUS"
        return ""

    def finish_completed(self):
        failure_code = self.completion_failure_code()
        if failure_code:
            self.finish("FAILED", failure_code)
        else:
            self.finish("COMPLETED", "")

    def finish(self, status, failure_code):
        with self._finalize_lock:
            if self.finalized:
                return
            yaw_error, handoff_error, inspection_distance = self.metrics()
            duration = (rospy.Time.now() - self.started).to_sec()
            row = build_trial_row(
                self.trial_id,
                self.seed,
                status=status,
                failure_code=failure_code,
                yaw_error_rad=yaw_error,
                handoff_error_m=handoff_error,
                final_inspection_distance_m=inspection_distance,
                success_radius_m=self.success_radius,
                duration_seconds=duration,
            )
            csv_path, json_path = self.writer.write(
                row,
                {
                    "registration_status": self.registration_status,
                    "frame_definitions": (
                        "truth_registration = A @ F_uav(t) @ Delta @ F_ugv(t)^-1 "
                        "at the estimate stamp; A anchors the experiment UAV "
                        "stream takeoff using the estimator's configurable "
                        "align_origin_to_uav_heading / fixed_origin_yaw rule."
                    ),
                    "model_positions": {
                        key: value.tolist()
                        for key, value in self.model_positions.items()
                    },
                    "expected_stream_seeds": self.expected_stream_seeds,
                    "trial_seed": self.seed,
                },
            )
            self.finalized = True
        self.status_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "trial_id": self.trial_id,
                        "status": status,
                        "failure_code": failure_code,
                        "csv": str(csv_path),
                        "metadata": str(json_path),
                        "seed": self.seed,
                    },
                    sort_keys=True,
                )
            )
        )

    def tick(self, _event):
        if (
            not self.finalized
            and (rospy.Time.now() - self.started).to_sec() >= self.timeout_seconds
        ):
            self.finish("TIMEOUT", "TRIAL_TIMEOUT")


if __name__ == "__main__":
    rospy.init_node("experiment_recorder")
    ExperimentRecorder()
    rospy.spin()

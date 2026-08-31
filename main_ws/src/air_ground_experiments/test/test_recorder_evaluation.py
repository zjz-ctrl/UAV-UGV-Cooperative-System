#!/usr/bin/env python3
"""Recorder evaluation-flow tests using duck-typed ROS messages."""

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import csv
import json
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parents[1] / "src"))

import ros_stubs
from ros_stubs import (
    FakeTime,
    Header,
    ModelStates,
    Odometry,
    Point,
    Pose,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
    String,
    Vector3,
    load_script_class,
)

TRUTH_TOPIC_UAV = "/air_ground_experiment/truth/uav_frame"
TRUTH_TOPIC_UGV = "/air_ground_experiment/truth/ugv_frame"


def make_model_states():
    message = ModelStates()
    def add(name, x, y, z):
        message.name.append(name)
        pose = Pose()
        pose.position = Point(x, y, z)
        message.pose.append(pose)
    add("iris_0", 0.0, 0.0, 2.0)
    add("ugv_0", 3.0, 1.0, 0.0)
    add("red_sphere", 3.3, 1.2, 0.2)
    return message


def truth_message(stamp, transform, effective_seed):
    payload = {
        "stamp": stamp,
        "transform_xyyaw": list(transform),
        "seed": effective_seed,
        "trial_seed": 17,
    }
    return String(data=json.dumps(payload))


class RecorderHarness(unittest.TestCase):
    def build_recorder(self, **extra_params):
        self.rospy, self.saved = ros_stubs.install_fake_ros()
        self.addCleanup(ros_stubs.restore_ros, self.saved)
        recorder_class = load_script_class(
            "experiment_recorder.py", "ExperimentRecorder"
        )
        self.output_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.output_directory.cleanup)
        parameters = {
            "~trial_id": "m1b-fix",
            "~seed": 17,
            "~output_directory": self.output_directory.name,
            "~timeout_seconds": 120.0,
            "~success_radius_m": 0.5,
            "~minimum_anchor_samples": 2,
            "~source_relation_window_seconds": 30.0,
        }
        parameters.update(extra_params)
        self.rospy.parameters = parameters
        return recorder_class()

    def subscriber(self, topic):
        for record in self.rospy.subscribers:
            if record.topic == topic:
                return record.callback
        raise AssertionError("no subscription to " + topic)

    def publisher(self, topic):
        for record in self.rospy.publishers:
            if record.topic == topic:
                return record
        raise AssertionError("no publication to " + topic)

    def csv_rows(self):
        path = Path(self.output_directory.name) / "trials.csv"
        if not path.exists():
            return []
        with path.open(newline="") as stream:
            return list(csv.DictReader(stream))


class RecorderWiringTest(RecorderHarness):
    def test_subscribes_to_the_real_mission_phase_topic(self):
        self.build_recorder()

        topics = [record.topic for record in self.rospy.subscribers]
        self.assertIn("/air_ground/mission_phase", topics)
        self.assertNotIn("/air_ground/demo_phase", topics)

    def test_feeds_takeoff_anchor_from_experiment_uav_odom(self):
        from air_ground_experiments.frame_perturbation import domain_seed

        recorder = self.build_recorder()
        odom_topic = "/air_ground_experiment/uav/odom"
        self.assertIn(odom_topic, [r.topic for r in self.rospy.subscribers])

        anchor_message = Odometry()
        anchor_message.header = Header(stamp=FakeTime(1.0), frame_id="experiment")
        anchor_message.pose.pose.position.x = 4.0
        anchor_message.pose.pose.position.y = -2.0
        self.subscriber(odom_topic)(anchor_message)
        self.subscriber(odom_topic)(anchor_message)

        np.testing.assert_allclose(recorder.evaluator.anchor[:2, 2], [-4.0, 2.0])

    def test_recorder_mirrors_estimator_anchor_parameters(self):
        recorder = self.build_recorder(
            **{
                "~align_origin_to_uav_heading": False,
                "~fixed_origin_yaw": 0.4,
            }
        )

        self.assertFalse(recorder.evaluator.align_origin_to_uav_heading)
        self.assertEqual(recorder.evaluator.fixed_origin_yaw, 0.4)

    def test_truth_messages_are_filtered_by_expected_stream_seeds(self):
        from air_ground_experiments.frame_perturbation import domain_seed

        recorder = self.build_recorder()
        good = {"stamp": 12.0, "transform_xyyaw": [1.0, 0.0, 0.0],
                "seed": domain_seed(17, "uav"), "trial_seed": 17}
        wrong_seed = {"stamp": 13.0, "transform_xyyaw": [9.0, 9.0, 0.0],
                      "seed": 999999, "trial_seed": 17}

        self.subscriber(TRUTH_TOPIC_UAV)(String(data=json.dumps(good)))
        self.subscriber(TRUTH_TOPIC_UAV)(String(data=json.dumps(wrong_seed)))

        stamps = [entry[0] for entry in recorder.evaluator._truth_history["uav"]]
        self.assertEqual(stamps, [12.0])


class RecorderCompletionTest(RecorderHarness):
    def prepare_complete_state(self, recorder):
        from air_ground_experiments.frame_perturbation import domain_seed

        odom_topic = "/air_ground_experiment/uav/odom"
        sample = Odometry()
        sample.header = Header(stamp=FakeTime(1.0), frame_id="experiment")
        for _ in range(2):
            self.subscriber(odom_topic)(sample)

        self.subscriber(TRUTH_TOPIC_UAV)(truth_message(
            100.0, [0.0, 0.0, 0.0], domain_seed(17, "uav")))
        self.subscriber(TRUTH_TOPIC_UAV)(truth_message(
            110.0, [0.0, 0.0, 0.0], domain_seed(17, "uav")))
        self.subscriber(TRUTH_TOPIC_UGV)(truth_message(
            100.0, [0.0, 0.0, 0.0], domain_seed(17, "ugv")))
        self.subscriber(TRUTH_TOPIC_UGV)(truth_message(
            110.0, [0.0, 0.0, 0.0], domain_seed(17, "ugv")))

        self.subscriber("/gazebo/model_states")(make_model_states())

        estimate = PoseWithCovarianceStamped()
        estimate.header = Header(stamp=FakeTime(105.0), frame_id="air_ground_origin")
        estimate.pose.pose.position.x = 3.05
        estimate.pose.pose.position.y = 1.02
        estimate.pose.pose.orientation.w = 1.0
        self.subscriber("/air_ground/registration/estimate")(estimate)

    def test_inspection_confirmed_records_success_with_ugv_to_anomaly_distance(self):
        recorder = self.build_recorder()
        self.prepare_complete_state(recorder)

        self.subscriber("/air_ground/mission_phase")(
            String(data="INSPECTION_CONFIRMED"))

        rows = self.csv_rows()
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["status"], "COMPLETED")
        self.assertEqual(rows[0]["success"], "True")
        # Final UGV (3,1,0) to red_sphere (3.3,1.2,0.2): hand-computed.
        expected = (0.3 ** 2 + 0.2 ** 2 + 0.2 ** 2) ** 0.5
        self.assertAlmostEqual(float(rows[0]["final_inspection_distance_m"]),
                               expected, places=6)
        status = json.loads(self.publisher(
            "/air_ground_experiment/evaluation/status").published[0].data)
        self.assertEqual(status["status"], "COMPLETED")

    def test_error_phase_maps_to_stable_failure_code_without_timeout(self):
        recorder = self.build_recorder(timeout_seconds=120.0)
        self.prepare_complete_state(recorder)

        self.subscriber("/air_ground/mission_phase")(
            String(data="ERROR_REGISTRATION"))

        rows = self.csv_rows()
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["status"], "FAILED")
        self.assertEqual(rows[0]["failure_code"], "MISSION_REGISTRATION")

    def test_missing_truth_sync_fails_instead_of_publishing_pseudo_metrics(self):
        recorder = self.build_recorder()
        odom_topic = "/air_ground_experiment/uav/odom"
        sample = Odometry()
        for _ in range(2):
            self.subscriber(odom_topic)(sample)
        self.subscriber("/gazebo/model_states")(make_model_states())
        estimate = PoseWithCovarianceStamped()
        estimate.header = Header(stamp=FakeTime(105.0))
        self.subscriber("/air_ground/registration/estimate")(estimate)

        self.subscriber("/air_ground/mission_phase")(
            String(data="INSPECTION_CONFIRMED"))

        rows = self.csv_rows()
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["status"], "FAILED")
        self.assertEqual(rows[0]["failure_code"], "INCOMPLETE_TRUTH_SYNC")

    def test_timeout_and_error_race_finalize_exactly_once(self):
        import threading as threading_module

        recorder = self.build_recorder(**{"~timeout_seconds": 0.05})
        self.prepare_complete_state(recorder)

        # Advance the fake clock beyond started + timeout so tick() is live.
        self.rospy.now_seconds = self.rospy.now_seconds + 1.0

        phase_callback = self.subscriber("/air_ground/mission_phase")
        start_barrier = threading_module.Barrier(2)
        errors = []

        def timeout_path():
            try:
                start_barrier.wait()
                recorder.tick(None)
            except Exception as error:  # pragma: no cover - surfaced below
                errors.append(error)

        def error_path():
            try:
                start_barrier.wait()
                phase_callback(String(data="ERROR_TARGET"))
            except Exception as error:  # pragma: no cover - surfaced below
                errors.append(error)

        with ThreadPoolExecutor(max_workers=2) as pool:
            futures = [pool.submit(timeout_path), pool.submit(error_path)]
            for future in futures:
                future.result()
        self.assertEqual(errors, [])

        rows = self.csv_rows()
        self.assertEqual(len(rows), 1)
        self.assertIn(rows[0]["status"], ("TIMEOUT", "FAILED"))


if __name__ == "__main__":
    unittest.main()

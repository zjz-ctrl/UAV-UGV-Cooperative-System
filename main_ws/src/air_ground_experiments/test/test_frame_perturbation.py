#!/usr/bin/env python3

import json
import math
from pathlib import Path
import sys
import unittest

import numpy as np


PACKAGE_SOURCE = Path(__file__).parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SOURCE))

from air_ground_experiments.frame_perturbation import (
    FramePerturbation,
    ObservationGateSchedule,
    adapt_position_command,
    diagnostic_json,
    inject_pose_outlier,
    truth_json,
)


class FramePerturbationTest(unittest.TestCase):
    def test_same_seed_produces_same_fixed_step_walk(self):
        first = FramePerturbation(
            [2.0, -1.0, 0.4], [0.01, 0.0, 0.001], seed=17,
            drift_step_seconds=0.5,
        )
        second = FramePerturbation(
            [2.0, -1.0, 0.4], [0.01, 0.0, 0.001], seed=17,
            drift_step_seconds=0.5,
        )

        np.testing.assert_array_equal(first.at(30.0), second.at(30.0))
        np.testing.assert_array_equal(first.at(30.1), first.at(30.4))

    def test_different_seeds_produce_different_walks(self):
        first = FramePerturbation([0.0, 0.0, 0.0], [0.1, 0.1, 0.01], 17)
        second = FramePerturbation([0.0, 0.0, 0.0], [0.1, 0.1, 0.01], 18)

        self.assertFalse(np.array_equal(first.at(20.0), second.at(20.0)))

    def test_repeated_and_out_of_order_queries_do_not_change_a_timestamp(self):
        chronological = FramePerturbation(
            [1.0, 2.0, -0.3], [0.02, 0.03, 0.004], 91,
            drift_step_seconds=1.0,
        )
        shuffled = FramePerturbation(
            [1.0, 2.0, -0.3], [0.02, 0.03, 0.004], 91,
            drift_step_seconds=1.0,
        )
        expected = {stamp: chronological.at(stamp) for stamp in (3.0, 8.0, 15.0)}

        observed = [shuffled.at(stamp) for stamp in (15.0, 3.0, 8.0, 15.0)]

        np.testing.assert_array_equal(observed[0], expected[15.0])
        np.testing.assert_array_equal(observed[1], expected[3.0])
        np.testing.assert_array_equal(observed[2], expected[8.0])
        np.testing.assert_array_equal(observed[3], expected[15.0])

    def test_walk_does_not_consume_numpy_global_rng(self):
        np.random.seed(1234)
        expected = np.random.random(3)
        np.random.seed(1234)

        perturbation = FramePerturbation([0.0] * 3, [0.1] * 3, 4)
        perturbation.at(10.0)

        np.testing.assert_array_equal(np.random.random(3), expected)

    def test_transform_odom_uses_one_3d_jacobian_for_pose_twist_and_covariance(self):
        perturbation = FramePerturbation(
            [10.0, -2.0, math.pi / 2.0], [0.0, 0.0, 0.0], 7
        )
        pose = {
            "position": np.array([1.0, 2.0, 3.0]),
            "orientation": np.array([0.5, 0.0, 0.0, math.sqrt(3.0) / 2.0]),
            "covariance": np.diag([1.0, 4.0, 9.0, 16.0, 25.0, 36.0]),
        }
        twist = {
            "linear": np.array([1.0, 2.0, 3.0]),
            "angular": np.array([4.0, 5.0, 6.0]),
            "covariance": np.diag([2.0, 5.0, 8.0, 11.0, 14.0, 17.0]),
        }

        transformed_pose, transformed_twist, truth = perturbation.transform_odom(
            pose, twist, 0.0
        )

        np.testing.assert_allclose(transformed_pose["position"], [8.0, -1.0, 3.0])
        np.testing.assert_allclose(
            transformed_pose["orientation"],
            [math.sqrt(2.0) / 4.0, math.sqrt(2.0) / 4.0,
             math.sqrt(6.0) / 4.0, math.sqrt(6.0) / 4.0],
            atol=1e-12,
        )
        np.testing.assert_allclose(transformed_twist["linear"], [-2.0, 1.0, 3.0])
        np.testing.assert_allclose(transformed_twist["angular"], [-5.0, 4.0, 6.0])
        np.testing.assert_allclose(
            np.diag(transformed_pose["covariance"]), [4.0, 1.0, 9.0, 25.0, 16.0, 36.0]
        )
        np.testing.assert_allclose(
            np.diag(transformed_twist["covariance"]), [5.0, 2.0, 8.0, 14.0, 11.0, 17.0]
        )
        np.testing.assert_allclose(truth, [10.0, -2.0, math.pi / 2.0])

    def test_transform_odom_does_not_mutate_input_records(self):
        perturbation = FramePerturbation([1.0, 2.0, 0.2], [0.0] * 3, 3)
        pose = {
            "position": np.array([3.0, 4.0, 5.0]),
            "orientation": np.array([0.0, 0.0, 0.0, 1.0]),
            "covariance": np.eye(6),
        }
        twist = {
            "linear": np.array([1.0, 0.0, 2.0]),
            "angular": np.array([0.0, 1.0, 3.0]),
            "covariance": np.eye(6),
        }

        perturbation.transform_odom(pose, twist, 1.0)

        np.testing.assert_array_equal(pose["position"], [3.0, 4.0, 5.0])
        np.testing.assert_array_equal(twist["linear"], [1.0, 0.0, 2.0])


class AdapterBehaviorTest(unittest.TestCase):
    def test_position_command_inverts_frame_and_preserves_nonplanar_semantics(self):
        command = {
            "stamp": 12.5,
            "frame_id": "uav_experiment_frame",
            "position": np.array([8.0, -1.0, 3.0]),
            "velocity": np.array([-2.0, 1.0, 4.0]),
            "acceleration": np.array([-6.0, 5.0, 7.0]),
            "jerk": np.array([-9.0, 8.0, 10.0]),
            "yaw": math.pi,
            "yaw_dot": -0.25,
            "trajectory_id": 44,
            "trajectory_flag": 3,
            "kx": [1.0, 2.0, 3.0],
            "kv": [4.0, 5.0, 6.0],
        }

        result = adapt_position_command(
            command, np.array([10.0, -2.0, math.pi / 2.0]), "iris_0/odom"
        )

        np.testing.assert_allclose(result["position"], [1.0, 2.0, 3.0])
        np.testing.assert_allclose(result["velocity"], [1.0, 2.0, 4.0])
        np.testing.assert_allclose(result["acceleration"], [5.0, 6.0, 7.0])
        np.testing.assert_allclose(result["jerk"], [8.0, 9.0, 10.0])
        self.assertAlmostEqual(result["yaw"], math.pi / 2.0)
        self.assertEqual(result["yaw_dot"], -0.25)
        self.assertEqual(result["stamp"], 12.5)
        self.assertEqual(result["frame_id"], "iris_0/odom")
        self.assertEqual(result["trajectory_id"], 44)
        self.assertEqual(result["trajectory_flag"], 3)
        self.assertEqual(result["kx"], [1.0, 2.0, 3.0])
        np.testing.assert_array_equal(command["position"], [8.0, -1.0, 3.0])

    def test_observation_queue_releases_seeded_schedule_reproducibly(self):
        settings = dict(
            visibility_windows=[(0.0, 5.0)],
            delay_seconds=0.4,
            delay_jitter_seconds=0.2,
            outlier_probability=0.5,
            outlier_translation_m=2.0,
            outlier_yaw_rad=0.5,
            seed=29,
        )
        first = ObservationGateSchedule(**settings)
        second = ObservationGateSchedule(**settings)

        for schedule in (first, second):
            self.assertTrue(schedule.enqueue("detector-message", 2.0, 2.1))
        first_item = first.release_ready(10.0)[0]
        second_item = second.release_ready(10.0)[0]

        self.assertEqual(first_item, second_item)
        self.assertEqual(first_item.payload, "detector-message")
        self.assertEqual(first_item.image_stamp, 2.0)
        self.assertAlmostEqual(
            first_item.release_time - 2.1, first_item.injected_delay
        )
        self.assertEqual(first_item.seed, 29)

    def test_observation_queue_never_synthesizes_or_releases_invisible_input(self):
        schedule = ObservationGateSchedule(
            visibility_windows=[(1.0, 2.0)],
            delay_seconds=0.0,
            delay_jitter_seconds=0.0,
            outlier_probability=0.0,
            outlier_translation_m=0.0,
            outlier_yaw_rad=0.0,
            seed=8,
        )

        self.assertEqual(schedule.release_ready(100.0), [])
        self.assertFalse(schedule.enqueue("real-but-hidden", 3.0, 3.0))
        self.assertEqual(schedule.release_ready(100.0), [])

    def test_visibility_keep_drop_schedule_is_seeded_and_intermittent(self):
        settings = dict(
            visibility_windows=[(0.0, 100.0)],
            visibility_probability=0.5,
            delay_seconds=0.0,
            delay_jitter_seconds=0.0,
            outlier_probability=0.0,
            outlier_translation_m=0.0,
            outlier_yaw_rad=0.0,
            seed=101,
        )
        first = ObservationGateSchedule(**settings)
        second = ObservationGateSchedule(**settings)

        first_pattern = [first.enqueue(index, index, index) for index in range(20)]
        second_pattern = [second.enqueue(index, index, index) for index in range(20)]

        self.assertEqual(first_pattern, second_pattern)
        self.assertIn(True, first_pattern)
        self.assertIn(False, first_pattern)

    def test_pose_outlier_applies_only_se2_and_preserves_z_roll_pitch_covariance(self):
        covariance = np.arange(36, dtype=float).reshape(6, 6)
        pose = {
            "position": np.array([1.0, 2.0, 3.0]),
            "orientation": np.array([0.5, 0.0, 0.0, math.sqrt(3.0) / 2.0]),
            "covariance": covariance,
        }

        result = inject_pose_outlier(pose, [10.0, -2.0, math.pi / 2.0])

        np.testing.assert_allclose(result["position"], [8.0, -1.0, 3.0])
        np.testing.assert_allclose(
            result["orientation"],
            [math.sqrt(2.0) / 4.0, math.sqrt(2.0) / 4.0,
             math.sqrt(6.0) / 4.0, math.sqrt(6.0) / 4.0],
        )
        np.testing.assert_array_equal(result["covariance"], covariance)




class PerturbationEpochTest(unittest.TestCase):
    def test_shared_epoch_indexes_steps_from_epoch_not_zero(self):
        anchored = FramePerturbation(
            [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 5,
            epoch_seconds=1000.0, drift_step_seconds=1.0,
        )
        baseline = FramePerturbation(
            [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 5,
            epoch_seconds=0.0, drift_step_seconds=1.0,
        )

        np.testing.assert_array_equal(anchored.at(1007.4), baseline.at(7.4))

    def test_stamp_before_epoch_fails_fast_without_allocating(self):
        perturbation = FramePerturbation(
            [0.0] * 3, [0.01] * 3, 5, epoch_seconds=10.0
        )

        with self.assertRaises(ValueError):
            perturbation.at(9.999)

    def test_elapsed_beyond_maximum_fails_fast_in_bounded_time(self):
        perturbation = FramePerturbation(
            [0.0] * 3, [0.01] * 3, 5,
            maximum_elapsed_seconds=60.0,
        )

        with self.assertRaises(ValueError):
            perturbation.at(1.8e9)

class DomainSeedTest(unittest.TestCase):
    def test_uav_and_ugv_streams_differ_and_reproduce(self):
        from air_ground_experiments.frame_perturbation import domain_seed

        uav_first = domain_seed(17, "uav")
        ugv_first = domain_seed(17, "ugv")
        uav_again = domain_seed(17, "uav")

        self.assertNotEqual(uav_first, ugv_first)
        self.assertEqual(uav_first, uav_again)
        self.assertNotEqual(uav_first, 17)

    def test_truth_json_reports_trial_and_effective_stream_seeds(self):
        payload = json.loads(
            truth_json([1.0, -2.0, 0.3], 4242, 4.5, "map", "independent",
                       trial_seed=17)
        )

        self.assertEqual(payload["trial_seed"], 17)
        self.assertEqual(payload["seed"], 4242)


class GateEpochVisibilityTest(unittest.TestCase):
    BASE_SETTINGS = dict(
        delay_seconds=0.4,
        delay_jitter_seconds=0.0,
        outlier_probability=0.0,
        outlier_translation_m=0.0,
        outlier_yaw_rad=0.0,
        seed=11,
    )

    def test_visibility_follows_image_occurrence_relative_to_shared_epoch(self):
        schedule = ObservationGateSchedule(
            visibility_windows=[(0.0, 5.0)], epoch_seconds=1000.0,
            **self.BASE_SETTINGS
        )

        self.assertTrue(schedule.enqueue("msg", 1003.5, 1004.9))
        self.assertFalse(schedule.enqueue("msg", 1006.0, 1006.0))

    def test_image_stamp_before_epoch_fails_fast(self):
        schedule = ObservationGateSchedule(
            visibility_windows=[(0.0, 1.0e9)], epoch_seconds=100.0,
            **self.BASE_SETTINGS
        )

        with self.assertRaises(ValueError):
            schedule.enqueue("msg", 99.0, 101.0)

    def test_scheduled_item_and_diagnostic_carry_full_timing_trace(self):
        schedule = ObservationGateSchedule(
            visibility_windows=[(0.0, 10.0)], epoch_seconds=0.0,
            **self.BASE_SETTINGS
        )
        schedule.enqueue("msg", 2.0, 2.7)
        item = schedule.release_ready(99.0)[0]

        self.assertEqual(item.image_stamp, 2.0)
        self.assertEqual(item.receipt_time, 2.7)
        self.assertAlmostEqual(item.release_time, 3.1)

        payload = json.loads(
            diagnostic_json(
                item.image_stamp, item.release_time, item.injected_delay,
                item.outlier_xyyaw, item.seed,
                receipt_time=item.receipt_time, actual_release=99.0,
                trial_seed=17,
            )
        )

        self.assertEqual(payload["image_stamp"], 2.0)
        self.assertEqual(payload["receipt_time"], 2.7)
        self.assertAlmostEqual(payload["scheduled_release"], 3.1)
        self.assertEqual(payload["actual_release"], 99.0)
        self.assertEqual(payload["trial_seed"], 17)


if __name__ == "__main__":
    unittest.main()

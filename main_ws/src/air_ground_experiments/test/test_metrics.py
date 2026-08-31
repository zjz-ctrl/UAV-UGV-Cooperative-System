#!/usr/bin/env python3

import math
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import sys
import tempfile
import unittest
import csv
import json

import numpy as np


PACKAGE_SOURCE = Path(__file__).parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SOURCE))

from air_ground_experiments.metrics import (
    TRIAL_COLUMNS,
    TrialResultWriter,
    TrialTruthEvaluator,
    build_trial_row,
    classify_success,
    classify_mission_phase,
    final_inspection_distance,
    handoff_error_2d,
    wrapped_yaw_error,
)


class MetricsTest(unittest.TestCase):
    def test_yaw_error_wraps_across_pi(self):
        error = wrapped_yaw_error(math.radians(-179.0), math.radians(179.0))

        self.assertAlmostEqual(error, math.radians(2.0))

    def test_handoff_error_uses_only_planar_coordinates(self):
        error = handoff_error_2d([4.0, 6.0, 100.0], [1.0, 2.0, -100.0])

        self.assertEqual(error, 5.0)

    def test_final_inspection_distance_preserves_three_dimensions(self):
        distance = final_inspection_distance([1.0, 2.0, 5.0], [1.0, -2.0, 2.0])

        self.assertEqual(distance, 5.0)

    def test_success_radius_is_inclusive(self):
        self.assertTrue(classify_success(0.5, 0.5))
        self.assertFalse(classify_success(0.500001, 0.5))
        self.assertFalse(classify_success(float("nan"), 0.5))

    def test_failed_trial_row_has_complete_schema_seed_and_failure_code(self):
        row = build_trial_row(
            trial_id="m1b-004",
            seed=37,
            status="FAILED",
            failure_code="REGISTRATION_TIMEOUT",
            yaw_error_rad=0.2,
            handoff_error_m=1.4,
            final_inspection_distance_m=2.1,
            success_radius_m=0.5,
            duration_seconds=30.0,
        )

        self.assertEqual(tuple(row.keys()), TRIAL_COLUMNS)
        self.assertEqual(row["seed"], 37)
        self.assertEqual(row["failure_code"], "REGISTRATION_TIMEOUT")
        self.assertEqual(row["status"], "FAILED")
        self.assertFalse(row["success"])
        self.assertFalse(row["timeout"])

    def test_failed_or_timeout_row_rejects_empty_failure_code(self):
        with self.assertRaises(ValueError):
            build_trial_row("trial", 1, status="FAILED")
        with self.assertRaises(ValueError):
            build_trial_row("trial", 1, status="TIMEOUT")

    def test_result_writer_appends_csv_and_writes_json_for_failed_timeout(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = TrialResultWriter(directory)
            row = build_trial_row(
                "timeout-7",
                71,
                status="TIMEOUT",
                failure_code="TRIAL_TIMEOUT",
                final_inspection_distance_m=4.2,
            )

            csv_path, json_path = writer.write(row, {"reason": "no frozen estimate"})

            with csv_path.open(newline="") as stream:
                records = list(csv.DictReader(stream))
            with json_path.open() as stream:
                metadata = json.load(stream)
            self.assertEqual(len(records), 1)
            self.assertEqual(records[0]["trial_id"], "timeout-7")
            self.assertEqual(records[0]["failure_code"], "TRIAL_TIMEOUT")
            self.assertEqual(metadata["result"]["seed"], 71)
            self.assertEqual(metadata["metadata"]["reason"], "no frozen estimate")

    def test_noncanonical_status_values_are_rejected(self):
        for bad_status in ("ERROR", "SUCCEEDED", "OK", "completed "):
            with self.assertRaises(ValueError):
                build_trial_row("trial", 1, status=bad_status)

    def test_completed_row_rejects_failure_codes_and_missing_distance(self):
        with self.assertRaises(ValueError):
            build_trial_row("trial", 1, status="COMPLETED", failure_code="X")
        with self.assertRaises(ValueError):
            build_trial_row(
                "trial", 1, status="COMPLETED",
                final_inspection_distance_m=float("nan"),
            )

    def test_timeout_flag_is_derived_from_canonical_status(self):
        failed = build_trial_row(
            "trial", 1, status="FAILED", failure_code="MISSION_ERROR",
            final_inspection_distance_m=9.9,
        )
        timeout = build_trial_row(
            "trial", 2, status="TIMEOUT", failure_code="TRIAL_TIMEOUT",
            final_inspection_distance_m=9.9,
        )
        completed = build_trial_row(
            "trial", 3, status="COMPLETED",
            final_inspection_distance_m=0.4,
        )

        self.assertFalse(failed["timeout"])
        self.assertTrue(timeout["timeout"])
        self.assertFalse(failed["success"])
        self.assertTrue(completed["success"])


def completed_row(trial_id):
    return build_trial_row(
        trial_id, 5, status="COMPLETED",
        yaw_error_rad=0.01, handoff_error_m=0.02,
        final_inspection_distance_m=0.3,
    )


class WriterExactlyOnceTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.writer = TrialResultWriter(self.directory.name)

    def csv_ids(self):
        if not self.writer.csv_path.exists():
            return []
        with self.writer.csv_path.open(newline="") as stream:
            return [record["trial_id"] for record in csv.DictReader(stream)]

    def test_duplicate_trial_id_is_rejected_without_new_rows(self):
        self.writer.write(completed_row("trial-A"), {})

        with self.assertRaises(ValueError):
            self.writer.write(completed_row("trial-A"), {})

        self.assertEqual(self.csv_ids(), ["trial-A"])

    def test_failed_json_write_rolls_back_the_csv_row_for_retry(self):
        from unittest import mock
        import air_ground_experiments.metrics as metrics_module

        before = []
        with mock.patch.object(
            metrics_module.json, "dump", side_effect=IOError("disk full")
        ):
            try:
                self.writer.write(completed_row("trial-B"), {})
            except IOError:
                pass
            else:
                self.fail("expected the simulated I/O failure to raise")
            before = self.csv_ids()

        csv_path, json_path = self.writer.write(completed_row("trial-B"), {})

        self.assertEqual(before, [])
        self.assertEqual(self.csv_ids(), ["trial-B"])
        self.assertTrue(json_path.exists())

    def test_concurrent_writes_persist_each_trial_exactly_once(self):
        def write(index):
            self.writer.write(completed_row("parallel-{}".format(index)), {})

        with ThreadPoolExecutor(max_workers=8) as pool:
            list(pool.map(write, range(16)))

        ids = self.csv_ids()
        self.assertEqual(len(ids), 16)
        self.assertEqual(len(set(ids)), 16)


class TrialTruthEvaluatorTest(unittest.TestCase):
    def make_evaluator(self, **kwargs):
        kwargs.setdefault("minimum_anchor_samples", 2)
        evaluator = TrialTruthEvaluator(**kwargs)
        for sample in ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)):
            evaluator.record_anchor_sample(sample)
        evaluator.record_source_relation((2.0, 0.0, 0.0), (5.0, 0.0, 0.0))
        return evaluator

    def test_zero_drift_truth_equals_true_source_relation(self):
        evaluator = self.make_evaluator()
        for stamp in (10.0, 20.0):
            evaluator.record_truth("uav", {"stamp": stamp, "transform_xyyaw": [0, 0, 0]})
            evaluator.record_truth("ugv", {"stamp": stamp, "transform_xyyaw": [0, 0, 0]})

        truth = evaluator.registration_truth_at(15.0)

        np.testing.assert_allclose(truth, [3.0, 0.0, 0.0], atol=1e-12)

    def test_interpolates_both_injected_transforms_at_estimate_stamp(self):
        evaluator = self.make_evaluator()
        # UAV injected offset grows linearly 0 -> 1 m in x across 10..20 s;
        # UGV stays identity.
        for stamp in (10.0, 20.0):
            evaluator.record_truth(
                "uav", {"stamp": stamp, "transform_xyyaw": [(stamp - 10.0) / 10.0, 0, 0]}
            )
            evaluator.record_truth(
                "ugv", {"stamp": stamp, "transform_xyyaw": [0, 0, 0]}
            )

        truth_at_mid = evaluator.registration_truth_at(15.0)
        truth_at_start = evaluator.registration_truth_at(10.0)

        # p_O(G)= A * F_u * Delta * inv(F_g): at t=10 F_u=0 -> (3,0);
        # at t=15 F_u x-offset=+0.5 shifts the observed origin by +0.5.
        np.testing.assert_allclose(truth_at_start[:2], [3.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(truth_at_mid[:2], [3.5, 0.0], atol=1e-12)

    def test_unsynchronized_or_incomplete_inputs_return_none(self):
        evaluator = self.make_evaluator()
        self.assertIsNone(evaluator.registration_truth_at(15.0))

        evaluator.record_truth("uav", {"stamp": 10.0, "transform_xyyaw": [0, 0, 0]})
        evaluator.record_truth("uav", {"stamp": 20.0, "transform_xyyaw": [0, 0, 0]})
        self.assertIsNone(evaluator.registration_truth_at(15.0))

    def test_takeoff_anchor_absorbs_constant_injected_offset(self):
        evaluator = TrialTruthEvaluator(minimum_anchor_samples=2)
        for sample in ((4.0, -2.0, 0.0), (4.0, -2.0, 0.0)):
            evaluator.record_anchor_sample(sample)
        evaluator.record_source_relation((0.0, 0.0, 0.0), (3.0, 0.0, 0.0))
        for stamp in (1.0, 2.0):
            evaluator.record_truth(
                "uav", {"stamp": stamp, "transform_xyyaw": [4.0, -2.0, 0.0]}
            )
            evaluator.record_truth(
                "ugv", {"stamp": stamp, "transform_xyyaw": [0.0, 0.0, 0.0]}
            )

        truth = evaluator.registration_truth_at(1.5)

        np.testing.assert_allclose(truth[:2], [3.0, 0.0], atol=1e-9)

    def test_truth_history_retains_early_freeze_across_high_rate_ingest(self):
        evaluator = TrialTruthEvaluator(minimum_anchor_samples=2)
        for sample in ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)):
            evaluator.record_anchor_sample(sample)
        evaluator.record_source_relation((0.0, 0.0, 0.0), (3.0, 0.0, 0.0))

        # Registration freezes early: both streams bracketed around t=15.
        for stamp in (10.0, 14.0, 16.0, 20.0):
            offset = max(0.0, (stamp - 10.0) / 10.0)
            evaluator.record_truth(
                "uav", {"stamp": stamp, "transform_xyyaw": [offset, 0.0, 0.0]}
            )
            evaluator.record_truth(
                "ugv", {"stamp": stamp, "transform_xyyaw": [0.0, 0.0, 0.0]}
            )

        # The trial runs on: more than the old 600-entry deque capacity of
        # later truth pairs arrives before finalization.
        for index in range(700):
            stamp = 1000.0 + index
            evaluator.record_truth(
                "uav", {"stamp": stamp, "transform_xyyaw": [0.0, 0.0, 0.0]}
            )
            evaluator.record_truth(
                "ugv", {"stamp": stamp, "transform_xyyaw": [0.0, 0.0, 0.0]}
            )

        truth_at_freeze = evaluator.registration_truth_at(15.0)

        # t=15 sits between offsets 0.4 and 0.6 -> interpolated 0.5 shift,
        # plus the Delta translation of +3 m.
        np.testing.assert_allclose(truth_at_freeze[:2], [3.5, 0.0], atol=1e-9)

    def test_fixed_yaw_anchor_matches_estimator_branch(self):
        # Estimator rule (takeoff_registration.py): origin_yaw =
        # -mean_yaw when aligning, else fixed_origin_yaw; translation
        # = -R(origin_yaw) @ center. Hand-derived literals below.
        mean_yaw = math.pi / 2.0
        fixed = 0.25
        evaluator = TrialTruthEvaluator(
            minimum_anchor_samples=2,
            align_origin_to_uav_heading=False,
            fixed_origin_yaw=fixed,
        )
        for _ in range(2):
            evaluator.record_anchor_sample((1.0, 0.0, mean_yaw))

        expected_translation_x = -math.cos(fixed)
        expected_translation_y = -math.sin(fixed)

        np.testing.assert_allclose(
            evaluator.anchor[:2, 2],
            [expected_translation_x, expected_translation_y], atol=1e-12,
        )
        self.assertAlmostEqual(evaluator.anchor[0, 0], math.cos(fixed))
        self.assertAlmostEqual(evaluator.anchor[1, 0], math.sin(fixed))


class MissionPhaseClassifierTest(unittest.TestCase):
    def test_planned_success_terminal_is_recognized(self):
        self.assertEqual(classify_mission_phase("INSPECTION_CONFIRMED"), ("SUCCESS", ""))

    def test_every_error_prefix_maps_to_a_stable_code(self):
        for phase, expected in (
            ("ERROR_TAKEOFF", "MISSION_TAKEOFF"),
            ("ERROR_REGISTRATION", "MISSION_REGISTRATION"),
            ("ERROR_APPROACH", "MISSION_APPROACH"),
            ("ERROR_TARGET", "MISSION_TARGET"),
            ("ERROR_COORDINATE", "MISSION_COORDINATE"),
            ("ERROR_CONTROLLER", "MISSION_CONTROLLER"),
            ("ERROR_INSPECTION", "MISSION_INSPECTION"),
        ):
            self.assertEqual(classify_mission_phase(phase), ("FAILED", expected))

    def test_transient_phases_stay_pending(self):
        for phase in ("WAIT", "TAKEOFF", "OVERWATCH", "DISPATCH"):
            self.assertEqual(classify_mission_phase(phase), ("PENDING", ""))


if __name__ == "__main__":
    unittest.main()

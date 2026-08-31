#!/usr/bin/env python3

from pathlib import Path
import inspect
import math
import sys
import unittest
from unittest.mock import patch

import numpy as np


PACKAGE_SRC = Path(__file__).parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SRC))

try:
    from air_ground_bringup.target_handoff import (
        DIRECT,
        HOLD,
        REOBSERVE,
        REREGISTER,
        UncertaintyBudget,
    )
except ImportError:
    DIRECT = HOLD = REOBSERVE = REREGISTER = UncertaintyBudget = None

try:
    from air_ground_bringup.target_handoff import sample_target_covariance
except ImportError:
    sample_target_covariance = None

try:
    from air_ground_bringup.target_handoff import (
        HandoffResult,
        evaluate_handoff,
        registration_execution_covariance,
        standoff_goal,
    )
except ImportError:
    HandoffResult = evaluate_handoff = None
    registration_execution_covariance = standoff_goal = None


class TargetHandoffPolicyTest(unittest.TestCase):
    def budget(self, registration, target, radius=1.0, yaw=0.1):
        self.assertIsNotNone(
            UncertaintyBudget,
            "air_ground_bringup.target_handoff policy is missing",
        )
        return UncertaintyBudget(registration, target, radius, yaw)

    def test_correlated_planar_covariance_uses_largest_eigenvalue(self):
        budget = self.budget(
            [[0.04, 0.03, 0.0], [0.03, 0.04, 0.0], [0.0, 0.0, 0.0001]],
            [[0.0, 0.0], [0.0, 0.0]],
        )

        self.assertAlmostEqual(budget.registration_radius, 0.6476129386, places=9)
        self.assertAlmostEqual(budget.confidence_radius, 0.6476129386, places=9)
        self.assertAlmostEqual(budget.yaw_confidence, 0.01959964, places=9)

    def test_inside_both_budgets_is_direct_including_exact_equality(self):
        budget = self.budget(
            [[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.0001]],
            [[0.0, 0.0], [0.0, 0.0]],
            radius=0.2447746830658759,
            yaw=0.01959964,
        )

        self.assertEqual(budget.choose_action(), DIRECT)

    def test_combined_radius_uses_sum_before_largest_eigenvalue(self):
        budget = self.budget(
            [[0.04, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.0001]],
            [[0.01, 0.0], [0.0, 0.04]],
        )

        self.assertAlmostEqual(budget.registration_radius, 0.4895493661, places=9)
        self.assertAlmostEqual(budget.target_radius, 0.4895493661, places=9)
        self.assertAlmostEqual(budget.confidence_radius, 0.5473328305, places=9)

    def test_target_dominated_planar_excess_reobserves(self):
        budget = self.budget(
            [[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.0001]],
            [[0.09, 0.0], [0.0, 0.09]],
            radius=0.5,
        )

        self.assertEqual(budget.choose_action(), REOBSERVE)

    def test_registration_dominated_or_equal_planar_excess_reregisters(self):
        for target_variance in (0.01, 0.04):
            with self.subTest(target_variance=target_variance):
                budget = self.budget(
                    [[0.04, 0.0, 0.0], [0.0, 0.04, 0.0], [0.0, 0.0, 0.0001]],
                    [[target_variance, 0.0], [0.0, target_variance]],
                    radius=0.4,
                )
                self.assertEqual(budget.choose_action(), REREGISTER)

    def test_yaw_excess_precedes_target_dominated_planar_excess(self):
        budget = self.budget(
            [[0.001, 0.0, 0.0], [0.0, 0.001, 0.0], [0.0, 0.0, 0.01]],
            [[0.2, 0.0], [0.0, 0.2]],
            radius=0.2,
            yaw=0.1,
        )

        self.assertEqual(budget.choose_action(), REREGISTER)

    def test_invalid_covariances_hold_with_nonfinite_confidences(self):
        invalid_pairs = (
            ([[1.0, 0.0], [0.0, 1.0]], np.eye(2)),
            (np.eye(3), [[1.0, np.nan], [np.nan, 1.0]]),
            ([[1.0, 0.2, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], np.eye(2)),
            ([[1.0, 0.0, 0.0], [0.0, -0.01, 0.0], [0.0, 0.0, 1.0]], np.eye(2)),
        )
        for registration, target in invalid_pairs:
            with self.subTest(registration=registration, target=target):
                budget = self.budget(registration, target)
                self.assertEqual(budget.choose_action(), HOLD)
                self.assertFalse(np.isfinite(budget.registration_radius))
                self.assertFalse(np.isfinite(budget.target_radius))
                self.assertFalse(np.isfinite(budget.confidence_radius))
                self.assertFalse(np.isfinite(budget.yaw_confidence))

    def test_invalid_thresholds_hold_instead_of_raising(self):
        for radius, yaw in (
            (0.0, 0.1),
            (-1.0, 0.1),
            (np.inf, 0.1),
            (np.nan, 0.1),
            (1.0, 0.0),
            (1.0, -0.1),
            (1.0, np.inf),
            (1.0, np.nan),
        ):
            with self.subTest(radius=radius, yaw=yaw):
                budget = self.budget(np.eye(3), np.eye(2), radius, yaw)
                self.assertEqual(budget.choose_action(), HOLD)
                self.assertFalse(np.isfinite(budget.confidence_radius))
                self.assertFalse(np.isfinite(budget.yaw_confidence))

    def test_rank_one_psd_roundoff_is_clamped_instead_of_held(self):
        budget = self.budget(
            np.zeros((3, 3)),
            [[1.0, 0.1], [0.1, 0.01]],
            radius=3.0,
        )

        self.assertEqual(budget.choose_action(), DIRECT)
        self.assertAlmostEqual(budget.target_radius, 2.4599551200, places=9)
        self.assertTrue(np.isfinite(budget.confidence_radius))

    def test_accepted_near_symmetric_covariance_is_symmetrized(self):
        budget = self.budget(
            [
                [1.0, 0.100000005, 0.0],
                [0.099999995, 0.01, 0.0],
                [0.0, 0.0, 0.0001],
            ],
            np.zeros((2, 2)),
            radius=3.0,
        )

        self.assertAlmostEqual(budget.registration_radius, 2.459955120011, places=11)

    def test_linear_algebra_failure_becomes_invalid_hold(self):
        with patch(
            "air_ground_bringup.target_handoff.np.linalg.eigvalsh",
            side_effect=np.linalg.LinAlgError("did not converge"),
        ):
            budget = self.budget(np.eye(3), np.eye(2))

        self.assertEqual(budget.choose_action(), HOLD)
        self.assertFalse(np.isfinite(budget.confidence_radius))

    def test_failure_at_any_numeric_stage_resets_all_public_properties(self):
        real_eigvalsh = np.linalg.eigvalsh
        for stage, fail_at in (
            ("registration_validation", 1),
            ("target_validation", 2),
            ("registration_radius", 3),
            ("target_radius", 4),
            ("combined_radius", 5),
        ):
            with self.subTest(stage=stage):
                calls = {"count": 0}

                def staged_eigvalsh(covariance):
                    calls["count"] += 1
                    if calls["count"] == fail_at:
                        raise np.linalg.LinAlgError(stage)
                    return real_eigvalsh(covariance)

                with patch(
                    "air_ground_bringup.target_handoff.np.linalg.eigvalsh",
                    side_effect=staged_eigvalsh,
                ):
                    budget = self.budget(np.eye(3), np.eye(2))

                self.assertEqual(budget.choose_action(), HOLD)
                self.assertTrue(all(not np.isfinite(value) for value in (
                    budget.registration_radius,
                    budget.target_radius,
                    budget.confidence_radius,
                    budget.yaw_confidence,
                )))

        real_sqrt = __import__("math").sqrt
        sqrt_calls = {"count": 0}

        def fail_at_yaw(value):
            sqrt_calls["count"] += 1
            if sqrt_calls["count"] == 4:
                raise FloatingPointError("yaw_confidence")
            return real_sqrt(value)

        with patch(
            "air_ground_bringup.target_handoff.math.sqrt",
            side_effect=fail_at_yaw,
        ):
            budget = self.budget(np.eye(3), np.eye(2))

        self.assertEqual(budget.choose_action(), HOLD)
        self.assertTrue(all(not np.isfinite(value) for value in (
            budget.registration_radius,
            budget.target_radius,
            budget.confidence_radius,
            budget.yaw_confidence,
        )))
    def test_finite_gate_exception_and_nonfinite_result_remain_invalid(self):
        real_isfinite = __import__("math").isfinite
        finite_calls = {"count": 0}

        def fail_at_first_local(value):
            finite_calls["count"] += 1
            if finite_calls["count"] == 3:
                raise FloatingPointError("local_result_finite_check")
            return real_isfinite(value)

        with patch(
            "air_ground_bringup.target_handoff.math.isfinite",
            side_effect=fail_at_first_local,
        ):
            budget = self.budget(np.eye(3), np.eye(2))

        self.assertEqual(budget.choose_action(), HOLD)
        self.assertTrue(all(not np.isfinite(value) for value in (
            budget.registration_radius,
            budget.target_radius,
            budget.confidence_radius,
            budget.yaw_confidence,
        )))

        with patch.object(
            UncertaintyBudget,
            "_r95",
            side_effect=(1.0, 2.0, np.inf),
        ):
            budget = self.budget(np.eye(3), np.eye(2))

        self.assertEqual(budget.choose_action(), HOLD)
        self.assertTrue(all(not np.isfinite(value) for value in (
            budget.registration_radius,
            budget.target_radius,
            budget.confidence_radius,
            budget.yaw_confidence,
        )))


class TargetSampleCovarianceTest(unittest.TestCase):
    def covariance(self, *args, **kwargs):
        self.assertIsNotNone(
            sample_target_covariance,
            "sample_target_covariance is missing",
        )
        return sample_target_covariance(*args, **kwargs)

    def test_combines_each_sensing_term_once(self):
        covariance = self.covariance(
            [[0.0, 0.0], [2.0, 0.0], [0.0, 2.0]],
            variance_floor=0.5,
            pose_covariances=[
                [[0.1, 0.02], [0.02, 0.2]],
                [[0.3, 0.04], [0.04, 0.4]],
                [[0.5, 0.06], [0.06, 0.6]],
            ],
            range_axes=[[2.0, 0.0], [0.0, 3.0], [4.0, 4.0]],
            range_variance=0.7,
        )

        np.testing.assert_allclose(
            covariance,
            [[2.233333333333333, -0.51],
             [-0.51, 2.333333333333333]],
            rtol=0.0,
            atol=1e-12,
        )

    def test_default_optional_terms_leave_unbiased_scatter_and_floor(self):
        covariance = self.covariance(
            [[-1.0, 0.0], [1.0, 0.0]],
            variance_floor=0.2,
        )

        np.testing.assert_allclose(
            covariance,
            [[2.04, 0.0], [0.0, 0.04]],
            rtol=0.0,
            atol=1e-12,
        )

    def test_rejects_malformed_samples_and_scalar_parameters(self):
        invalid_calls = (
            (([[0.0, 0.0]], 0.1), {}),
            (([0.0, 1.0], 0.1), {}),
            (([[0.0, 0.0], [np.nan, 1.0]], 0.1), {}),
            (([[0.0, 0.0], [1.0, 1.0]], -0.1), {}),
            (([[0.0, 0.0], [1.0, 1.0]], np.inf), {}),
            (([[0.0, 0.0], [1.0, 1.0]], 0.1), {"range_variance": -0.1}),
            (([[0.0, 0.0], [1.0, 1.0]], 0.1), {"range_variance": np.nan}),
        )
        for args, kwargs in invalid_calls:
            with self.subTest(args=args, kwargs=kwargs):
                self.assertIsNone(self.covariance(*args, **kwargs))

    def test_rejects_malformed_per_sample_covariances(self):
        samples = [[0.0, 0.0], [1.0, 1.0]]
        invalid_covariances = (
            [np.eye(2)],
            np.zeros((2, 3, 3)),
            [[[1.0, np.nan], [np.nan, 1.0]], np.eye(2)],
            [[[1.0, 0.2], [0.0, 1.0]], np.eye(2)],
            [[[1.0, 0.0], [0.0, -0.01]], np.eye(2)],
        )
        for pose_covariances in invalid_covariances:
            with self.subTest(pose_covariances=pose_covariances):
                self.assertIsNone(self.covariance(
                    samples,
                    0.1,
                    pose_covariances=pose_covariances,
                ))

    def test_rejects_malformed_or_degenerate_range_axes(self):
        samples = [[0.0, 0.0], [1.0, 1.0]]
        invalid_axes = (
            [[1.0, 0.0]],
            [[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            [[1.0, 0.0], [np.nan, 1.0]],
            [[1.0, 0.0], [0.0, 0.0]],
        )
        for range_axes in invalid_axes:
            with self.subTest(range_axes=range_axes):
                self.assertIsNone(self.covariance(
                    samples,
                    0.1,
                    range_axes=range_axes,
                    range_variance=0.2,
                ))


class HandoffCovarianceTest(unittest.TestCase):
    def assert_interfaces_exist(self):
        for interface in (
            HandoffResult,
            evaluate_handoff,
            registration_execution_covariance,
            standoff_goal,
        ):
            self.assertIsNotNone(interface, "pure handoff interface is missing")

    def evaluate_with_registration(
        self,
        target_xy,
        target_covariance,
        origin_from_uav,
        origin_from_registration,
        registration_covariance,
        standoff,
        inspection_radius,
        inspection_yaw,
    ):
        self.assertIn(
            "origin_from_registration",
            inspect.signature(evaluate_handoff).parameters,
            "evaluate_handoff needs a distinct registration mean transform",
        )
        return evaluate_handoff(
            target_xy,
            target_covariance,
            origin_from_uav,
            origin_from_registration,
            registration_covariance,
            standoff,
            inspection_radius,
            inspection_yaw,
        )

    def test_standoff_mean_and_target_jacobian_match_geometry(self):
        self.assert_interfaces_exist()

        mean, jacobian = standoff_goal([13.0, 14.0], [1.0, 5.0], 3.0)

        np.testing.assert_allclose(
            mean,
            [10.6, 12.2, 0.6435011087932844],
            rtol=0.0,
            atol=1e-12,
        )
        np.testing.assert_allclose(
            jacobian,
            [[0.928, 0.096], [0.096, 0.872], [-0.04, 0.0533333333333333]],
            rtol=0.0,
            atol=1e-12,
        )

    def test_standoff_degeneracy_check_is_translation_invariant(self):
        self.assert_interfaces_exist()
        anchor = np.array([1e10, -1e10])

        result = standoff_goal(anchor + [12.0, 9.0], anchor, 3.0)
        self.assertIsNotNone(
            result,
            "standoff degeneracy must depend on local target-anchor geometry",
        )
        mean, jacobian = result

        np.testing.assert_allclose(
            mean[:2] - anchor,
            [9.6, 7.2],
            rtol=0.0,
            atol=1e-6,
        )
        np.testing.assert_allclose(
            jacobian,
            [[0.928, 0.096], [0.096, 0.872], [-0.04, 0.0533333333333333]],
            rtol=0.0,
            atol=1e-12,
        )

    def test_registration_covariance_includes_lever_arm_and_cross_terms(self):
        self.assert_interfaces_exist()
        yaw_variance = math.radians(1.0) ** 2

        yaw_only = registration_execution_covariance(
            [15.0, 0.0],
            0.0,
            np.diag([0.0, 0.0, yaw_variance]),
        )
        self.assertAlmostEqual(
            yaw_only[1, 1],
            (15.0 * math.radians(1.0)) ** 2,
            places=14,
        )
        np.testing.assert_allclose(
            yaw_only,
            [[0.0, 0.0, 0.0],
             [0.0, 225.0 * yaw_variance, 15.0 * yaw_variance],
             [0.0, 15.0 * yaw_variance, yaw_variance]],
            rtol=0.0,
            atol=1e-14,
        )

        with_cross_terms = registration_execution_covariance(
            [3.0, 4.0],
            0.0,
            [[0.04, 0.01, 0.002],
             [0.01, 0.09, -0.003],
             [0.002, -0.003, 0.0004]],
        )
        np.testing.assert_allclose(
            with_cross_terms,
            [[0.0304, 0.0232, 0.0004],
             [0.0232, 0.0756, -0.0018],
             [0.0004, -0.0018, 0.0004]],
            rtol=0.0,
            atol=1e-12,
        )

    def test_registration_yaw_jacobian_rotates_registration_frame_lever(self):
        self.assert_interfaces_exist()
        self.assertIn(
            "registration_yaw",
            inspect.signature(registration_execution_covariance).parameters,
            "registration propagation needs the registration mean yaw",
        )
        yaw_variance = 0.0025

        covariance = registration_execution_covariance(
            [3.0, 4.0],
            math.pi / 2.0,
            np.diag([0.0, 0.0, yaw_variance]),
        )

        np.testing.assert_allclose(
            covariance,
            [[9.0 * yaw_variance, 12.0 * yaw_variance, -3.0 * yaw_variance],
             [12.0 * yaw_variance, 16.0 * yaw_variance, -4.0 * yaw_variance],
             [-3.0 * yaw_variance, -4.0 * yaw_variance, yaw_variance]],
            rtol=0.0,
            atol=1e-14,
        )

    def test_registration_translation_changes_anchor_but_not_origin_target(self):
        self.assert_interfaces_exist()
        target_covariance = np.diag([0.04, 0.09])
        registration_covariance = np.diag([0.01, 0.04, 0.0025])
        origin_from_uav = [10.0, -2.0, math.pi / 2.0]
        base = self.evaluate_with_registration(
            [3.0, 4.0],
            target_covariance,
            origin_from_uav,
            [2.0, 1.0, 0.0],
            registration_covariance,
            1.0,
            10.0,
            1.0,
        )
        shifted = self.evaluate_with_registration(
            [3.0, 4.0],
            target_covariance,
            origin_from_uav,
            [2.0, 4.0, 0.0],
            registration_covariance,
            1.0,
            10.0,
            1.0,
        )

        self.assertTrue(base.valid)
        self.assertTrue(shifted.valid)
        np.testing.assert_allclose(base.origin_mean, [6.0, 1.0], atol=1e-12)
        np.testing.assert_allclose(shifted.origin_mean, base.origin_mean, atol=0.0)
        np.testing.assert_allclose(
            shifted.origin_covariance,
            base.origin_covariance,
            atol=0.0,
        )
        np.testing.assert_allclose(base.goal_mean, [5.0, 1.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(
            shifted.goal_mean,
            [5.2, 1.6, -0.6435011087932844],
            rtol=0.0,
            atol=1e-12,
        )
        np.testing.assert_allclose(
            base.goal_registration_covariance,
            [[0.01, 0.0, 0.0], [0.0, 0.08, 0.01], [0.0, 0.01, 0.0025]],
            rtol=0.0,
            atol=1e-14,
        )
        np.testing.assert_allclose(
            shifted.goal_registration_covariance,
            [[0.0325, 0.03, 0.0075],
             [0.03, 0.08, 0.01],
             [0.0075, 0.01, 0.0025]],
            rtol=0.0,
            atol=1e-14,
        )

    def test_uav_rotation_propagates_anisotropic_target_and_split_radii(self):
        self.assert_interfaces_exist()
        result = self.evaluate_with_registration(
            [3.0, 4.0],
            np.diag([0.04, 0.09]),
            [10.0, -2.0, math.pi / 2.0],
            [2.0, 1.0, 0.0],
            np.diag([0.01, 0.04, 0.0025]),
            1.0,
            10.0,
            1.0,
        )

        self.assertTrue(result.valid)
        np.testing.assert_allclose(
            result.origin_covariance,
            [[0.09, 0.0], [0.0, 0.04]],
            rtol=0.0,
            atol=1e-12,
        )
        np.testing.assert_allclose(
            result.goal_sensing_covariance,
            [[0.09, 0.0, 0.0], [0.0, 0.0225, 0.0075], [0.0, 0.0075, 0.0025]],
            rtol=0.0,
            atol=1e-12,
        )
        self.assertAlmostEqual(
            result.registration_radius,
            math.sqrt(5.991464547 * 0.08),
            places=12,
        )
        self.assertAlmostEqual(
            result.target_radius,
            math.sqrt(5.991464547 * 0.09),
            places=12,
        )
        self.assertAlmostEqual(
            result.confidence_radius,
            math.sqrt(5.991464547 * 0.1025),
            places=12,
        )

    def test_evaluation_splits_covariance_and_adds_registration_once(self):
        self.assert_interfaces_exist()
        yaw_variance = math.radians(1.0) ** 2
        target_covariance = np.diag([0.04, 0.09])
        no_registration = evaluate_handoff(
            [15.0, 0.0],
            target_covariance,
            [2.0, 3.0, 0.0],
            [2.0, 3.0, 0.0],
            np.zeros((3, 3)),
            3.5,
            10.0,
            1.0,
        )
        result = evaluate_handoff(
            [15.0, 0.0],
            target_covariance,
            [2.0, 3.0, 0.0],
            [2.0, 3.0, 0.0],
            np.diag([0.0, 0.0, yaw_variance]),
            3.5,
            10.0,
            1.0,
        )

        self.assertTrue(result.valid)
        self.assertEqual(result.action, DIRECT)
        np.testing.assert_allclose(result.origin_mean, [17.0, 3.0], atol=1e-12)
        np.testing.assert_allclose(result.origin_covariance, target_covariance, atol=1e-12)
        np.testing.assert_allclose(
            result.origin_covariance,
            no_registration.origin_covariance,
            atol=0.0,
        )
        np.testing.assert_allclose(result.goal_mean, [13.5, 3.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(
            result.goal_sensing_covariance,
            [[0.04, 0.0, 0.0], [0.0, 0.0529, 0.0046], [0.0, 0.0046, 0.0004]],
            rtol=0.0,
            atol=1e-12,
        )
        expected_registration = np.array([
            [0.0, 0.0, 0.0],
            [0.0, 225.0 * yaw_variance, 15.0 * yaw_variance],
            [0.0, 15.0 * yaw_variance, yaw_variance],
        ])
        np.testing.assert_allclose(
            result.goal_registration_covariance,
            expected_registration,
            rtol=0.0,
            atol=1e-14,
        )
        np.testing.assert_allclose(
            result.goal_covariance,
            result.goal_sensing_covariance + expected_registration,
            rtol=0.0,
            atol=1e-14,
        )
        self.assertAlmostEqual(
            result.goal_covariance[1, 1] - result.goal_sensing_covariance[1, 1],
            (15.0 * math.radians(1.0)) ** 2,
            places=14,
        )
        self.assertAlmostEqual(
            result.confidence_radius,
            math.sqrt(5.991464547 * (0.0529 + 225.0 * yaw_variance)),
            places=12,
        )
        self.assertAlmostEqual(
            result.yaw_confidence,
            1.959964 * math.radians(1.0),
            places=12,
        )

    def test_result_is_immutable(self):
        self.assert_interfaces_exist()
        result = evaluate_handoff(
            [5.0, 0.0],
            np.eye(2) * 0.01,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            np.eye(3) * 0.001,
            1.0,
            10.0,
            1.0,
        )

        with self.assertRaises(AttributeError):
            result.action = HOLD
        with self.assertRaises(ValueError):
            result.origin_mean[0] = 100.0

    def test_malformed_inputs_return_none_or_invalid_hold(self):
        self.assert_interfaces_exist()
        for args in (
            ([1.0], [0.0, 0.0], 1.0),
            ([1.0, np.nan], [0.0, 0.0], 1.0),
            ([1.0, 1.0], [0.0, 0.0], 0.0),
            ([1.0, 1.0], [1.0, 1.0], 1.0),
            ([1e-12, 0.0], [0.0, 0.0], 1.0),
        ):
            with self.subTest(standoff_args=args):
                self.assertIsNone(standoff_goal(*args))

        for target, covariance in (
            ([1.0], np.eye(3)),
            ([1.0, np.nan], np.eye(3)),
            ([1.0, 1.0], np.eye(2)),
            ([1.0, 1.0], [[1.0, 0.2, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
            ([1.0, 1.0], np.diag([1.0, -0.1, 1.0])),
        ):
            with self.subTest(registration_target=target, covariance=covariance):
                self.assertIsNone(registration_execution_covariance(
                    target,
                    0.0,
                    covariance,
                ))
        self.assertIsNone(registration_execution_covariance(
            [1.0, 1.0],
            np.nan,
            np.eye(3),
        ))

        invalid_evaluations = (
            ([0.0, 0.0], np.eye(2), [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], np.eye(3), 1.0, 1.0, 0.1),
            ([1.0, 0.0], np.eye(3), [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], np.eye(3), 1.0, 1.0, 0.1),
            ([1.0, 0.0], np.eye(2), [0.0, 0.0], [0.0, 0.0, 0.0], np.eye(3), 1.0, 1.0, 0.1),
            ([1.0, 0.0], np.eye(2), [0.0, 0.0, 0.0], [0.0, 0.0], np.eye(3), 1.0, 1.0, 0.1),
            ([1.0, 0.0], np.eye(2), [0.0, 0.0, 0.0], [0.0, np.nan, 0.0], np.eye(3), 1.0, 1.0, 0.1),
            ([1.0, 0.0], np.eye(2), [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], np.eye(3), -1.0, 1.0, 0.1),
        )
        for args in invalid_evaluations:
            with self.subTest(evaluation_args=args):
                result = evaluate_handoff(*args)
                self.assertIsInstance(result, HandoffResult)
                self.assertFalse(result.valid)
                self.assertEqual(result.action, HOLD)
                self.assertIsNone(result.origin_mean)
                self.assertFalse(np.isfinite(result.confidence_radius))


if __name__ == "__main__":
    unittest.main()

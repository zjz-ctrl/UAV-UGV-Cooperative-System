#!/usr/bin/env python3

import math
from concurrent.futures import ThreadPoolExecutor
import threading
import unittest

import numpy as np

import air_ground_coordinate_transform.registration_estimator as registration_estimator
from air_ground_coordinate_transform.registration_estimator import (
    RegistrationSample,
    RobustBatchEstimator,
)
from air_ground_coordinate_transform.se2 import wrap_angle


def sample(mean, stamp, covariance=None):
    if covariance is None:
        covariance = np.diag([0.002 ** 2, 0.002 ** 2, 0.001 ** 2])
    return RegistrationSample(
        mean=np.asarray(mean, dtype=float),
        anchor=np.zeros(2),
        covariance=np.asarray(covariance, dtype=float),
        stamp=float(stamp),
    )


def process_noise(**overrides):
    rates = {
        "translation_time_variance_rate": 0.1,
        "translation_uav_distance_variance_rate": 0.2,
        "translation_ugv_distance_variance_rate": 0.3,
        "yaw_time_variance_rate": 0.01,
        "yaw_uav_distance_variance_rate": 0.02,
        "yaw_ugv_distance_variance_rate": 0.03,
    }
    rates.update(overrides)
    return rates


def batch(mean, covariance, stamp=1.0):
    return registration_estimator.BatchEstimate(
        mean=np.asarray(mean, dtype=float),
        covariance=np.asarray(covariance, dtype=float),
        inlier_count=20,
        stamp=float(stamp),
    )


def state_record(state):
    return (
        None if state.mean is None else state.mean.copy(),
        None if state.covariance is None else state.covariance.copy(),
        state.revision,
        state.stamp,
        state.initialized,
    )


def assert_state_record(test_case, state, expected):
    mean, covariance, revision, stamp, initialized = expected
    if mean is None:
        test_case.assertIsNone(state.mean)
    else:
        np.testing.assert_array_equal(state.mean, mean)
    if covariance is None:
        test_case.assertIsNone(state.covariance)
    else:
        np.testing.assert_array_equal(state.covariance, covariance)
    test_case.assertEqual(state.revision, revision)
    test_case.assertEqual(state.stamp, stamp)
    test_case.assertEqual(state.initialized, initialized)


class RegistrationFilterPredictionTest(unittest.TestCase):
    def test_prediction_adds_exact_time_and_distance_variance_without_moving_mean(self):
        filter_type = getattr(registration_estimator, "RegistrationFilter", None)
        self.assertIsNotNone(filter_type)
        initial_mean = np.array([1.0, -2.0, 0.3])
        initial_covariance = np.array(
            [
                [0.4, 0.03, 0.01],
                [0.03, 0.5, -0.02],
                [0.01, -0.02, 0.2],
            ]
        )
        registration_filter = filter_type(
            initial_mean, initial_covariance, process_noise()
        )

        state = registration_filter.predict(2.0, 3.0, 4.0)

        np.testing.assert_array_equal(state.mean, initial_mean)
        np.testing.assert_allclose(
            state.covariance,
            initial_covariance + np.diag([2.0, 2.0, 0.2]),
            rtol=0.0,
            atol=1e-15,
        )
        self.assertTrue(np.all(np.diag(state.covariance) > np.diag(initial_covariance)))
        self.assertEqual(state.revision, 1)
        self.assertEqual(state.stamp, 2.0)
        self.assertTrue(state.initialized)

    def test_prediction_with_zero_time_and_travel_has_exactly_zero_growth(self):
        filter_type = getattr(registration_estimator, "RegistrationFilter", None)
        self.assertIsNotNone(filter_type)
        initial_covariance = np.array(
            [
                [0.4, 0.03, 0.01],
                [0.03, 0.5, -0.02],
                [0.01, -0.02, 0.2],
            ]
        )
        registration_filter = filter_type(
            np.array([1.0, -2.0, 0.3]), initial_covariance, process_noise()
        )

        state = registration_filter.predict(0.0, 0.0, 0.0)

        np.testing.assert_array_equal(state.covariance, initial_covariance)
        self.assertEqual(state.stamp, 0.0)

    def test_none_prior_is_uninitialized_and_prediction_does_not_advance_stamp(self):
        registration_filter = registration_estimator.RegistrationFilter(
            None, None, process_noise()
        )

        state = registration_filter.predict(2.0, 3.0, 4.0)

        self.assertFalse(registration_filter.initialized)
        self.assertFalse(state.initialized)
        self.assertIsNone(state.mean)
        self.assertIsNone(state.covariance)
        self.assertEqual(state.revision, 0)
        self.assertEqual(state.stamp, 0.0)

    def test_constructor_rejects_invalid_prior_and_process_noise(self):
        valid_mean = np.zeros(3)
        valid_covariance = np.eye(3)
        invalid_cases = [
            (valid_mean, valid_covariance, process_noise(yaw_time_variance_rate=-0.1)),
            (valid_mean, valid_covariance, process_noise(yaw_time_variance_rate=np.inf)),
            (valid_mean, valid_covariance, {}),
            (np.zeros(2), valid_covariance, process_noise()),
            (np.array([0.0, np.nan, 0.0]), valid_covariance, process_noise()),
            (valid_mean, np.eye(2), process_noise()),
            (valid_mean, np.full((3, 3), np.nan), process_noise()),
            (
                valid_mean,
                np.array([[1.0, 0.2, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
                process_noise(),
            ),
            (valid_mean, np.diag([1.0, 1.0, -0.1]), process_noise()),
            (object(), valid_covariance, process_noise()),
            (valid_mean, object(), process_noise()),
            (None, valid_covariance, process_noise()),
            (valid_mean, None, process_noise()),
        ]

        for mean, covariance, rates in invalid_cases:
            with self.subTest(mean=mean, covariance=covariance, rates=rates):
                with self.assertRaises(ValueError):
                    registration_estimator.RegistrationFilter(mean, covariance, rates)

    def test_prediction_rejects_nonfinite_or_negative_inputs(self):
        invalid_inputs = [
            (-0.1, 0.0, 0.0),
            (0.0, -0.1, 0.0),
            (0.0, 0.0, -0.1),
            (np.nan, 0.0, 0.0),
            (0.0, np.inf, 0.0),
            (0.0, 0.0, -np.inf),
        ]
        for values in invalid_inputs:
            with self.subTest(values=values):
                registration_filter = registration_estimator.RegistrationFilter(
                    np.zeros(3), np.eye(3), process_noise()
                )
                with self.assertRaises(ValueError):
                    registration_filter.predict(*values)

    def test_constructor_inputs_and_public_state_do_not_alias_internal_arrays(self):
        initial_mean = np.array([1.0, -2.0, 0.3])
        initial_covariance = np.array(
            [[0.4, 0.03, 0.01], [0.03, 0.5, -0.02], [0.01, -0.02, 0.2]]
        )
        registration_filter = registration_estimator.RegistrationFilter(
            initial_mean, initial_covariance, process_noise()
        )
        expected = state_record(registration_filter.state)

        initial_mean[:] = 99.0
        initial_covariance[:] = 88.0
        public_state = registration_filter.state
        public_state.mean[:] = 77.0
        public_state.covariance[:] = 66.0

        assert_state_record(self, registration_filter.state, expected)

    def test_public_state_cannot_replace_internal_state(self):
        registration_filter = registration_estimator.RegistrationFilter(
            np.zeros(3), np.eye(3), process_noise()
        )
        snapshot = registration_filter.state

        with self.assertRaises(AttributeError):
            registration_filter.state = snapshot

    def test_prediction_result_and_old_snapshots_do_not_alias_internal_arrays(self):
        registration_filter = registration_estimator.RegistrationFilter(
            np.array([1.0, -2.0, 0.3]), np.eye(3), process_noise()
        )
        old_snapshot = registration_filter.state
        predicted = registration_filter.predict(1.0, 0.5, 0.25)
        expected = state_record(registration_filter.state)

        old_snapshot.mean[:] = 91.0
        old_snapshot.covariance[:] = 92.0
        predicted.mean[:] = 93.0
        predicted.covariance[:] = 94.0

        assert_state_record(self, registration_filter.state, expected)

    def test_prediction_overflow_raises_and_preserves_complete_state(self):
        registration_filter = registration_estimator.RegistrationFilter(
            np.zeros(3),
            np.eye(3),
            process_noise(
                translation_time_variance_rate=1e308,
                yaw_time_variance_rate=1e308,
            ),
        )
        before = state_record(registration_filter.state)

        with np.errstate(over="ignore", invalid="ignore"):
            with self.assertRaises(ArithmeticError):
                registration_filter.predict(2.0, 0.0, 0.0)

        assert_state_record(self, registration_filter.state, before)


class RegistrationFilterUpdateTest(unittest.TestCase):
    def test_first_valid_batch_initializes_without_gating_against_fake_prior(self):
        registration_filter = registration_estimator.RegistrationFilter(
            None, None, process_noise()
        )
        update = getattr(registration_filter, "update", None)
        self.assertIsNotNone(update)
        covariance = np.array(
            [
                [0.04, 0.006, 0.002],
                [0.006, 0.09, -0.003],
                [0.002, -0.003, 0.01],
            ]
        )

        result = update(
            batch([2.0, -1.0, math.radians(200.0)], covariance, stamp=12.5),
            mahalanobis_threshold=0.0,
        )

        self.assertTrue(result.accepted)
        self.assertEqual(result.reason, "initialized")
        np.testing.assert_allclose(
            result.mean, [2.0, -1.0, math.radians(-160.0)], atol=1e-15
        )
        np.testing.assert_array_equal(result.covariance, covariance)
        np.testing.assert_array_equal(result.innovation, np.zeros(3))
        self.assertEqual(result.mahalanobis, 0.0)
        self.assertEqual(result.revision, 1)
        self.assertEqual(registration_filter.state.stamp, 12.5)
        self.assertTrue(registration_filter.initialized)

    def test_update_rejects_invalid_gate_threshold_as_programmer_error(self):
        invalid_thresholds = [-0.1, np.nan, np.inf, "not-a-number"]
        for threshold in invalid_thresholds:
            with self.subTest(threshold=threshold):
                registration_filter = registration_estimator.RegistrationFilter(
                    np.zeros(3), np.eye(3), process_noise()
                )
                before = registration_filter.state

                with self.assertRaises(ValueError):
                    registration_filter.update(
                        batch(np.zeros(3), np.eye(3)),
                        mahalanobis_threshold=threshold,
                    )

                np.testing.assert_array_equal(registration_filter.state.mean, before.mean)
                np.testing.assert_array_equal(
                    registration_filter.state.covariance, before.covariance
                )
                self.assertEqual(registration_filter.state.revision, before.revision)
                self.assertEqual(registration_filter.state.stamp, before.stamp)

    def test_yaw_innovation_wraps_from_plus_179_to_minus_179_as_plus_2_degrees(self):
        covariance = np.diag([0.01, 0.01, 0.01])
        registration_filter = registration_estimator.RegistrationFilter(
            np.array([0.0, 0.0, math.radians(179.0)]),
            covariance,
            process_noise(),
        )

        result = registration_filter.update(
            batch([0.0, 0.0, math.radians(-179.0)], covariance),
            mahalanobis_threshold=0.0,
        )

        self.assertIsNotNone(result)
        self.assertFalse(result.accepted)
        self.assertAlmostEqual(result.innovation[2], math.radians(2.0), places=14)
        self.assertAlmostEqual(result.mahalanobis, 0.06092348395734171, places=14)

    def test_gross_outlier_is_gated_without_changing_any_state_field(self):
        registration_filter = registration_estimator.RegistrationFilter(
            np.array([1.0, -2.0, 0.2]),
            np.array(
                [
                    [0.04, 0.01, 0.003],
                    [0.01, 0.09, -0.004],
                    [0.003, -0.004, 0.01],
                ]
            ),
            process_noise(),
        )
        registration_filter.predict(3.0, 0.5, 0.25)
        before = registration_filter.state

        result = registration_filter.update(
            batch([25.0, -40.0, 2.8], np.diag([0.01, 0.01, 0.002]), stamp=9.0),
            mahalanobis_threshold=11.344866730144373,
        )

        self.assertIsNotNone(result)
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "mahalanobis_gate")
        self.assertGreater(result.mahalanobis, 11.344866730144373)
        np.testing.assert_array_equal(registration_filter.state.mean, before.mean)
        np.testing.assert_array_equal(
            registration_filter.state.covariance, before.covariance
        )
        self.assertEqual(registration_filter.state.stamp, before.stamp)
        self.assertEqual(registration_filter.state.revision, before.revision)

    def test_measurement_just_inside_three_dof_chi_square_gate_is_accepted(self):
        threshold = 11.344866730144373
        target_nis = threshold - 1e-9
        innovation_x = math.sqrt(2.0 * target_nis)
        registration_filter = registration_estimator.RegistrationFilter(
            np.zeros(3), np.eye(3), process_noise()
        )

        result = registration_filter.update(
            batch([innovation_x, 0.0, 0.0], np.eye(3), stamp=3.0),
            mahalanobis_threshold=threshold,
        )

        self.assertIsNotNone(result)
        self.assertTrue(result.accepted)
        self.assertEqual(result.reason, "accepted")
        self.assertLess(result.mahalanobis, threshold)
        self.assertAlmostEqual(result.mahalanobis, target_nis, places=12)

    def test_accepted_update_matches_independent_full_joseph_result(self):
        prior_covariance = np.array(
            [
                [0.5, 0.12, 0.04],
                [0.12, 0.4, -0.03],
                [0.04, -0.03, 0.2],
            ]
        )
        measurement_covariance = np.array(
            [
                [0.2, -0.02, 0.01],
                [-0.02, 0.3, 0.015],
                [0.01, 0.015, 0.08],
            ]
        )
        registration_filter = registration_estimator.RegistrationFilter(
            np.array([1.0, -2.0, math.radians(179.0)]),
            prior_covariance,
            process_noise(),
        )

        result = registration_filter.update(
            batch(
                [1.2, -2.1, math.radians(-179.5)],
                measurement_covariance,
                stamp=8.0,
            ),
            mahalanobis_threshold=11.344866730144373,
        )

        expected_covariance = np.array(
            [
                [0.1392923663977936, 0.00772314260347205, 0.00979692842435348],
                [0.00772314260347205, 0.16317539717022123, 0.00171339516784102],
                [0.00979692842435348, 0.00171339516784102, 0.05653595783667378],
            ]
        )
        np.testing.assert_allclose(
            result.innovation, [0.2, -0.1, math.radians(1.5)], atol=1e-14
        )
        np.testing.assert_allclose(
            result.mean,
            [1.1339143935468317, -2.0382359682630398, -3.135417272473372],
            atol=1e-14,
        )
        np.testing.assert_allclose(result.covariance, expected_covariance, atol=1e-14)
        self.assertTrue(np.all(np.isfinite(result.covariance)))
        np.testing.assert_allclose(result.covariance, result.covariance.T, atol=1e-15)
        self.assertGreaterEqual(float(np.min(np.linalg.eigvalsh(result.covariance))), 0.0)
        self.assertEqual(result.revision, 2)
        self.assertEqual(registration_filter.state.revision, 2)
        self.assertEqual(registration_filter.state.stamp, 8.0)

    def test_invalid_batches_have_stable_reason_and_leave_initialized_state_unchanged(self):
        invalid_batches = [
            batch([0.0, np.nan, 0.0], np.eye(3)),
            batch([0.0, 0.0], np.eye(3)),
            batch([0.0, 0.0, 0.0], np.full((3, 3), np.inf)),
            batch([0.0, 0.0, 0.0], np.eye(2)),
            batch(
                [0.0, 0.0, 0.0],
                [[1.0, 0.2, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            ),
            batch([0.0, 0.0, 0.0], np.diag([1.0, 1.0, -0.1])),
            batch([0.0, 0.0, 0.0], np.eye(3), stamp=np.nan),
        ]
        for invalid_batch in invalid_batches:
            with self.subTest(batch=invalid_batch):
                registration_filter = registration_estimator.RegistrationFilter(
                    np.array([1.0, -2.0, 0.3]), np.eye(3), process_noise()
                )
                before = registration_filter.state

                result = registration_filter.update(
                    invalid_batch, mahalanobis_threshold=11.344866730144373
                )

                self.assertFalse(result.accepted)
                self.assertEqual(result.reason, "invalid_batch")
                np.testing.assert_array_equal(registration_filter.state.mean, before.mean)
                np.testing.assert_array_equal(
                    registration_filter.state.covariance, before.covariance
                )
                self.assertEqual(registration_filter.state.revision, before.revision)
                self.assertEqual(registration_filter.state.stamp, before.stamp)

    def test_invalid_first_batch_does_not_initialize_filter(self):
        registration_filter = registration_estimator.RegistrationFilter(
            None, None, process_noise()
        )

        result = registration_filter.update(
            batch([0.0, 0.0, 0.0], np.diag([1.0, -0.1, 1.0])),
            mahalanobis_threshold=11.344866730144373,
        )

        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "invalid_batch")
        self.assertFalse(registration_filter.initialized)
        self.assertEqual(registration_filter.state.revision, 0)

    def test_singular_innovation_covariance_has_stable_rejection_reason(self):
        registration_filter = registration_estimator.RegistrationFilter(
            np.zeros(3), np.zeros((3, 3)), process_noise()
        )
        before = registration_filter.state

        result = registration_filter.update(
            batch([0.0, 0.0, 0.0], np.zeros((3, 3))),
            mahalanobis_threshold=11.344866730144373,
        )

        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "singular_innovation_covariance")
        np.testing.assert_array_equal(registration_filter.state.mean, before.mean)
        np.testing.assert_array_equal(registration_filter.state.covariance, before.covariance)
        self.assertEqual(registration_filter.state.revision, before.revision)
        self.assertEqual(registration_filter.state.stamp, before.stamp)

    def test_finite_extreme_means_reject_overflowing_innovation_without_exception(self):
        cases = (
            ([-1e308, 0.0, 0.0], [1e308, 0.0, 0.0]),
            ([0.0, 0.0, -1e308], [0.0, 0.0, 1e308]),
        )
        for prior_mean, measurement_mean in cases:
            with self.subTest(prior_mean=prior_mean, measurement_mean=measurement_mean):
                registration_filter = registration_estimator.RegistrationFilter(
                    np.asarray(prior_mean), np.eye(3), process_noise()
                )
                before = state_record(registration_filter.state)

                try:
                    with np.errstate(all="raise"):
                        result = registration_filter.update(
                            batch(measurement_mean, np.eye(3), stamp=0.0),
                            mahalanobis_threshold=11.344866730144373,
                        )
                except Exception as error:
                    self.fail("finite extreme innovation raised {!r}".format(error))

                self.assertFalse(result.accepted)
                self.assertEqual(result.reason, "singular_innovation_covariance")
                np.testing.assert_array_equal(result.innovation, np.zeros(3))
                self.assertTrue(math.isnan(result.mahalanobis))
                np.testing.assert_array_equal(result.mean, before[0])
                np.testing.assert_array_equal(result.covariance, before[1])
                self.assertEqual(result.revision, before[2])
                assert_state_record(self, registration_filter.state, before)

    def test_each_accepted_update_increments_revision_once(self):
        registration_filter = registration_estimator.RegistrationFilter(
            np.zeros(3), np.eye(3), process_noise()
        )
        revisions = []
        for stamp in (2.0, 4.0, 7.0):
            result = registration_filter.update(
                batch(np.zeros(3), np.eye(3), stamp=stamp),
                mahalanobis_threshold=11.344866730144373,
            )
            self.assertIsNotNone(result)
            self.assertTrue(result.accepted)
            revisions.append(result.revision)

        self.assertEqual(revisions, [2, 3, 4])
        self.assertEqual(registration_filter.state.revision, 4)
        self.assertEqual(registration_filter.state.stamp, 7.0)

    def test_update_results_and_pretransition_snapshots_do_not_alias_filter(self):
        registration_filter = registration_estimator.RegistrationFilter(
            np.zeros(3), np.eye(3), process_noise()
        )
        old_snapshot = registration_filter.state
        accepted = registration_filter.update(
            batch([0.1, -0.1, 0.01], np.eye(3), stamp=1.0),
            mahalanobis_threshold=11.344866730144373,
        )
        self.assertTrue(accepted.accepted)
        expected_after_accept = state_record(registration_filter.state)

        old_snapshot.mean[:] = 20.0
        old_snapshot.covariance[:] = 21.0
        accepted.mean[:] = 22.0
        accepted.covariance[:] = 23.0
        accepted.innovation[:] = 24.0
        assert_state_record(self, registration_filter.state, expected_after_accept)

        rejected = registration_filter.update(
            batch([100.0, -100.0, 2.5], np.eye(3), stamp=1.0),
            mahalanobis_threshold=11.344866730144373,
        )
        self.assertFalse(rejected.accepted)
        self.assertEqual(rejected.reason, "mahalanobis_gate")
        expected_after_reject = state_record(registration_filter.state)
        rejected.mean[:] = 31.0
        rejected.covariance[:] = 32.0
        rejected.innovation[:] = 33.0

        assert_state_record(self, registration_filter.state, expected_after_reject)

    def test_negative_batch_stamp_is_invalid_and_does_not_initialize(self):
        registration_filter = registration_estimator.RegistrationFilter(
            None, None, process_noise()
        )
        before = state_record(registration_filter.state)

        result = registration_filter.update(
            batch(np.zeros(3), np.eye(3), stamp=-0.01),
            mahalanobis_threshold=11.344866730144373,
        )

        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "invalid_batch")
        assert_state_record(self, registration_filter.state, before)

    def test_stale_batch_is_rejected_and_equal_stamp_is_allowed(self):
        registration_filter = registration_estimator.RegistrationFilter(
            np.zeros(3), np.eye(3), process_noise()
        )
        registration_filter.predict(2.0, 0.0, 0.0)
        before = state_record(registration_filter.state)

        stale = registration_filter.update(
            batch(np.zeros(3), np.eye(3), stamp=1.999),
            mahalanobis_threshold=11.344866730144373,
        )

        self.assertFalse(stale.accepted)
        self.assertEqual(stale.reason, "stale_batch")
        assert_state_record(self, registration_filter.state, before)

        equal = registration_filter.update(
            batch(np.zeros(3), np.eye(3), stamp=2.0),
            mahalanobis_threshold=11.344866730144373,
        )
        self.assertTrue(equal.accepted)
        self.assertEqual(registration_filter.state.stamp, 2.0)

    def test_stamp_is_monotonic_across_initialization_prediction_acceptance_and_rejection(self):
        registration_filter = registration_estimator.RegistrationFilter(
            None, None, process_noise()
        )

        initialized = registration_filter.update(
            batch(np.zeros(3), np.eye(3), stamp=2.0),
            mahalanobis_threshold=0.0,
        )
        self.assertTrue(initialized.accepted)
        self.assertEqual(registration_filter.state.stamp, 2.0)

        predicted = registration_filter.predict(1.5, 0.0, 0.0)
        self.assertEqual(predicted.stamp, 3.5)

        accepted = registration_filter.update(
            batch(np.zeros(3), np.eye(3), stamp=3.5),
            mahalanobis_threshold=11.344866730144373,
        )
        self.assertTrue(accepted.accepted)
        self.assertEqual(registration_filter.state.stamp, 3.5)

        before_reject = state_record(registration_filter.state)
        rejected = registration_filter.update(
            batch([100.0, 100.0, 2.5], np.eye(3), stamp=4.0),
            mahalanobis_threshold=11.344866730144373,
        )
        self.assertFalse(rejected.accepted)
        assert_state_record(self, registration_filter.state, before_reject)

    def test_exact_mahalanobis_threshold_equality_is_accepted(self):
        registration_filter = registration_estimator.RegistrationFilter(
            np.zeros(3), np.zeros((3, 3)), process_noise()
        )

        result = registration_filter.update(
            batch([1.0, 0.0, 0.0], np.eye(3), stamp=0.0),
            mahalanobis_threshold=1.0,
        )

        self.assertEqual(result.mahalanobis, 1.0)
        self.assertTrue(result.accepted)

    def test_diagonally_scaled_tiny_spd_innovation_is_computed_without_exception(self):
        registration_filter = registration_estimator.RegistrationFilter(
            np.zeros(3), np.diag([1.0, 0.0, 1.0]), process_noise()
        )

        try:
            with np.errstate(all="raise"):
                result = registration_filter.update(
                    batch(np.zeros(3), np.diag([0.0, 1e-320, 0.0]), stamp=0.0),
                    mahalanobis_threshold=11.344866730144373,
                )
        except Exception as error:
            self.fail("scaled-safe SPD update raised {!r}".format(error))

        self.assertTrue(result.accepted)
        self.assertTrue(np.all(np.isfinite(result.mean)))
        self.assertTrue(np.all(np.isfinite(result.covariance)))
        self.assertEqual(result.revision, 2)
        np.testing.assert_array_equal(result.covariance, np.zeros((3, 3)))
        assert_state_record(
            self,
            registration_filter.state,
            (np.zeros(3), np.zeros((3, 3)), 2, 0.0, True),
        )

    def test_near_singular_scaled_innovation_is_rejected_with_complete_state_preserved(self):
        correlation = 1.0 - 1e-10
        measurement_covariance = np.array(
            [[1.0, correlation, 0.0], [correlation, 1.0, 0.0], [0.0, 0.0, 1.0]]
        )
        registration_filter = registration_estimator.RegistrationFilter(
            np.zeros(3), np.zeros((3, 3)), process_noise()
        )
        before = state_record(registration_filter.state)

        result = registration_filter.update(
            batch(np.zeros(3), measurement_covariance, stamp=0.0),
            mahalanobis_threshold=11.344866730144373,
        )

        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "singular_innovation_covariance")
        assert_state_record(self, registration_filter.state, before)


class RegistrationFilterMonteCarloTest(unittest.TestCase):
    @staticmethod
    def run_sequences():
        rates = {
            "translation_time_variance_rate": 0.0004,
            "translation_uav_distance_variance_rate": 0.0009,
            "translation_ugv_distance_variance_rate": 0.0016,
            "yaw_time_variance_rate": 0.000025,
            "yaw_uav_distance_variance_rate": 0.000049,
            "yaw_ugv_distance_variance_rate": 0.0001,
        }
        measurement_covariance = np.array(
            [
                [0.0004, 0.00008, 0.00001],
                [0.00008, 0.000625, -0.000015],
                [0.00001, -0.000015, 0.0001],
            ]
        )
        filtered_errors = []
        prediction_errors = []
        accepted_updates = 0
        rejected_updates = 0

        for sequence in range(100):
            rng = np.random.RandomState(73000 + sequence)
            truth = np.zeros(3)
            initial_covariance = np.zeros((3, 3))
            filtered = registration_estimator.RegistrationFilter(
                np.zeros(3), initial_covariance, rates
            )
            prediction_only = registration_estimator.RegistrationFilter(
                np.zeros(3), initial_covariance, rates
            )
            previous_revision = filtered.state.revision
            simulation_stamp = 0.0

            for step in range(1, 81):
                dt = 0.2
                simulation_stamp += dt
                uav_distance = 0.12 + rng.uniform(0.0, 0.06)
                ugv_distance = 0.025 + rng.uniform(0.0, 0.025)
                translation_variance = (
                    rates["translation_time_variance_rate"] * dt
                    + rates["translation_uav_distance_variance_rate"]
                    * uav_distance
                    + rates["translation_ugv_distance_variance_rate"]
                    * ugv_distance
                )
                yaw_variance = (
                    rates["yaw_time_variance_rate"] * dt
                    + rates["yaw_uav_distance_variance_rate"] * uav_distance
                    + rates["yaw_ugv_distance_variance_rate"] * ugv_distance
                )
                truth += rng.multivariate_normal(
                    np.zeros(3),
                    np.diag(
                        [translation_variance, translation_variance, yaw_variance]
                    ),
                )
                truth[2] = wrap_angle(truth[2])

                filtered.predict(dt, uav_distance, ugv_distance)
                prediction_only.predict(dt, uav_distance, ugv_distance)
                if step % 4 == 0:
                    measurement = truth + rng.multivariate_normal(
                        np.zeros(3), measurement_covariance
                    )
                    measurement[2] = wrap_angle(measurement[2])
                    result = filtered.update(
                        batch(
                            measurement,
                            measurement_covariance,
                            stamp=simulation_stamp,
                        ),
                        mahalanobis_threshold=11.344866730144373,
                    )
                    if result.accepted:
                        accepted_updates += 1
                        self_revision = previous_revision + 1
                    else:
                        rejected_updates += 1
                        self_revision = previous_revision
                    if result.revision != self_revision:
                        raise AssertionError("accepted-update revisions are not monotonic")
                    previous_revision = result.revision

                for state in (filtered.state, prediction_only.state):
                    if not np.all(np.isfinite(state.mean)):
                        raise AssertionError("nonfinite Monte Carlo state mean")
                    if not np.all(np.isfinite(state.covariance)):
                        raise AssertionError("nonfinite Monte Carlo covariance")
                    np.testing.assert_allclose(
                        state.covariance, state.covariance.T, atol=1e-12
                    )
                    if float(np.min(np.linalg.eigvalsh(state.covariance))) < -1e-12:
                        raise AssertionError("non-PSD Monte Carlo covariance")

            filtered_error = filtered.state.mean - truth
            prediction_error = prediction_only.state.mean - truth
            filtered_error[2] = wrap_angle(filtered_error[2])
            prediction_error[2] = wrap_angle(prediction_error[2])
            filtered_errors.append(filtered_error)
            prediction_errors.append(prediction_error)

        filtered_errors = np.asarray(filtered_errors)
        prediction_errors = np.asarray(prediction_errors)
        filtered_sequence_rmse = np.sqrt(np.mean(filtered_errors ** 2, axis=1))
        prediction_sequence_rmse = np.sqrt(np.mean(prediction_errors ** 2, axis=1))
        return {
            "improved_count": int(
                np.count_nonzero(filtered_sequence_rmse < prediction_sequence_rmse)
            ),
            "filtered_component_rmse": np.sqrt(
                np.mean(filtered_errors ** 2, axis=0)
            ),
            "prediction_component_rmse": np.sqrt(
                np.mean(prediction_errors ** 2, axis=0)
            ),
            "filtered_state_rmse": float(np.sqrt(np.mean(filtered_errors ** 2))),
            "prediction_state_rmse": float(np.sqrt(np.mean(prediction_errors ** 2))),
            "accepted_updates": accepted_updates,
            "rejected_updates": rejected_updates,
        }

    def test_intermittent_updates_improve_at_least_95_of_100_seeded_walks(self):
        summary = self.run_sequences()

        self.assertGreaterEqual(summary["improved_count"], 95)
        self.assertLess(summary["filtered_state_rmse"], summary["prediction_state_rmse"])


class RobustBatchEstimatorTest(unittest.TestCase):
    def test_rejects_gross_outliers_and_returns_psd_mean_covariance(self):
        rng = np.random.RandomState(1204)
        samples = [
            sample(
                [
                    2.0 + rng.normal(0.0, 0.012),
                    -1.0 + rng.normal(0.0, 0.012),
                    0.35 + rng.normal(0.0, 0.004),
                ],
                index,
            )
            for index in range(20)
        ]
        for index in range(8):
            samples.append(
                sample(
                    [8.0 + index, -7.0 + index, 0.35 + (-1) ** index * 1.2],
                    20 + index,
                )
            )

        estimate = RobustBatchEstimator(20, 0.12, 0.03).estimate(samples)

        self.assertIsNotNone(estimate)
        np.testing.assert_allclose(estimate.mean[:2], [2.0, -1.0], atol=0.03)
        self.assertLess(abs(wrap_angle(estimate.mean[2] - 0.35)), 0.01)
        self.assertGreaterEqual(estimate.inlier_count, 18)
        self.assertEqual(estimate.stamp, 19.0)
        np.testing.assert_allclose(
            estimate.covariance, estimate.covariance.T, atol=1e-12
        )
        self.assertGreaterEqual(
            float(np.min(np.linalg.eigvalsh(estimate.covariance))), -1e-12
        )

    def test_translation_outliers_do_not_bias_yaw_gate(self):
        rng = np.random.RandomState(711)
        samples = [
            sample(
                [
                    2.0 + rng.normal(0.0, 0.01),
                    -1.0 + rng.normal(0.0, 0.01),
                    0.35 + rng.normal(0.0, 0.003),
                ],
                index,
            )
            for index in range(20)
        ]
        samples.extend(
            sample([8.0 + index, -7.0, 2.8], 20 + index) for index in range(8)
        )

        estimate = RobustBatchEstimator(20, 0.12, 0.03).estimate(samples)

        self.assertIsNotNone(estimate)
        self.assertEqual(estimate.inlier_count, 20)
        self.assertLess(abs(wrap_angle(estimate.mean[2] - 0.35)), 0.01)

    def test_circular_mean_handles_samples_across_wrapped_yaw_boundary(self):
        samples = [
            sample([1.0, 2.0, math.radians(yaw_degrees)], index)
            for index, yaw_degrees in enumerate(
                [179.8, -179.7, 179.9, -179.8, 179.7, -179.9]
            )
        ]

        estimate = RobustBatchEstimator(6, 0.12, math.radians(1.0)).estimate(
            samples
        )

        self.assertIsNotNone(estimate)
        self.assertLess(abs(wrap_angle(estimate.mean[2] - math.pi)), 0.01)

    def test_returns_none_when_too_few_samples_survive_residual_gates(self):
        samples = [sample([2.0, -1.0, 0.35], index) for index in range(19)]
        samples.extend(
            sample([8.0 + index, -7.0, 0.35], 19 + index)
            for index in range(9)
        )

        estimate = RobustBatchEstimator(20, 0.12, 0.03).estimate(samples)

        self.assertIsNone(estimate)

    def test_covariance_includes_input_uncertainty_and_configured_floors(self):
        covariance = np.diag([0.2 ** 2, 0.1 ** 2, 0.05 ** 2])
        samples = [sample([1.0, 2.0, 0.3], index, covariance) for index in range(4)]
        estimator = RobustBatchEstimator(
            4,
            0.12,
            0.03,
            minimum_translation_sigma=0.01,
            minimum_yaw_sigma=0.005,
        )

        estimate = estimator.estimate(samples)

        expected = covariance / 4.0 + np.diag([0.01 ** 2, 0.01 ** 2, 0.005 ** 2])
        np.testing.assert_allclose(estimate.covariance, expected, atol=1e-12)


class RegistrationGeometryTest(unittest.TestCase):
    def test_full_uav_attitude_is_applied_before_planar_projection(self):
        build_sample = getattr(
            registration_estimator, "registration_sample_from_observation", None
        )
        self.assertIsNotNone(build_sample)
        identity_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        uav_pose = np.array(
            [
                0.0,
                0.0,
                0.0,
                math.sin(math.pi / 4.0),
                0.0,
                0.0,
                math.cos(math.pi / 4.0),
            ]
        )

        result = build_sample(
            origin_to_uav_odom=np.eye(4),
            uav_pose=uav_pose,
            base_camera=np.eye(4),
            observation_mean=np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0]),
            observation_covariance=np.zeros((6, 6)),
            ugv_pose=identity_pose,
            base_board=np.eye(4),
            anchor=np.zeros(2),
            stamp=2.5,
        )

        np.testing.assert_allclose(result.mean, [0.0, 0.0, 0.0], atol=1e-12)
        self.assertEqual(result.stamp, 2.5)

    def test_observation_covariance_propagates_rotation_cross_terms_and_lever_arm(self):
        build_sample = getattr(
            registration_estimator, "registration_sample_from_observation", None
        )
        self.assertIsNotNone(build_sample)
        camera = np.array(
            [
                [0.0, -1.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        board = np.eye(4)
        board[:3, 3] = [1.0, 2.0, 0.0]
        covariance = np.zeros((6, 6))
        covariance[np.ix_([0, 1, 5], [0, 1, 5])] = np.array(
            [
                [0.04, 0.006, 0.003],
                [0.006, 0.09, -0.004],
                [0.003, -0.004, 0.01],
            ]
        )
        identity_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

        result = build_sample(
            origin_to_uav_odom=np.eye(4),
            uav_pose=identity_pose,
            base_camera=camera,
            observation_mean=np.zeros(6),
            observation_covariance=covariance,
            ugv_pose=identity_pose,
            base_board=board,
            anchor=np.zeros(2),
            stamp=1.0,
        )

        np.testing.assert_allclose(result.mean, [2.0, -1.0, math.pi / 2.0])
        np.testing.assert_allclose(
            result.covariance,
            [
                [0.108, 0.025, 0.014],
                [0.025, 0.092, 0.023],
                [0.014, 0.023, 0.01],
            ],
            atol=1e-8,
        )
        np.testing.assert_allclose(
            result.covariance, result.covariance.T, atol=1e-12
        )
        self.assertGreaterEqual(
            float(np.min(np.linalg.eigvalsh(result.covariance))), -1e-12
        )


class FixedYawEstimateTest(unittest.TestCase):
    @staticmethod
    def samples():
        covariance = np.diag([0.0, 0.0, 0.04])
        return [
            RegistrationSample(
                mean=np.array([1.0, 2.0, 0.1]),
                anchor=np.asarray(anchor),
                covariance=covariance,
                stamp=float(index),
            )
            for index, anchor in enumerate([[1.0, 0.0], [0.0, 1.0], [1.0, 1.0]])
        ] + [sample([9.0, -8.0, 0.1], 3.0, covariance)]

    def test_estimator_exposes_the_exact_survivor_indices(self):
        estimator = RobustBatchEstimator(3, 0.12, 0.03)
        estimate_with_inliers = getattr(estimator, "estimate_with_inliers", None)
        self.assertIsNotNone(estimate_with_inliers)

        estimate, inlier_indices = estimate_with_inliers(self.samples())

        self.assertIsNotNone(estimate)
        self.assertEqual(inlier_indices, (0, 1, 2))
        self.assertEqual(estimate.inlier_count, len(inlier_indices))

    def test_fixed_yaw_reanchor_uses_survivors_and_propagates_covariance(self):
        estimator = RobustBatchEstimator(3, 0.12, 0.03)
        estimate_with_inliers = getattr(estimator, "estimate_with_inliers", None)
        fixed_yaw_estimate = getattr(
            registration_estimator, "fixed_yaw_estimate", None
        )
        self.assertIsNotNone(estimate_with_inliers)
        self.assertIsNotNone(fixed_yaw_estimate)
        samples = self.samples()
        _, inlier_indices = estimate_with_inliers(samples)

        estimate = fixed_yaw_estimate(
            samples,
            inlier_indices,
            fixed_yaw=0.0,
            minimum_translation_sigma=0.01,
            minimum_yaw_sigma=0.005,
        )

        np.testing.assert_allclose(
            estimate.mean, [0.900166583353, 2.094837581925, 0.0], atol=1e-12
        )
        np.testing.assert_allclose(
            estimate.covariance,
            [
                [0.0109266324, -0.00380353186, 0.0],
                [-0.00380353186, 0.00927151638, 0.0],
                [0.0, 0.0, 0.000025],
            ],
            atol=1e-10,
        )
        self.assertEqual(estimate.inlier_count, 3)
        self.assertEqual(estimate.stamp, 2.0)


class OneShotRegistrationStateTest(unittest.TestCase):
    def test_concurrent_updates_make_one_atomic_zero_to_one_transition(self):
        state_type = getattr(
            registration_estimator, "OneShotRegistrationState", None
        )
        self.assertIsNotNone(state_type)
        state = state_type()
        start = threading.Barrier(16)
        calls = []

        def update():
            start.wait()

            def build_value():
                calls.append(len(calls) + 1)
                return "frozen-{}".format(calls[-1])

            return state.update(build_value)

        with ThreadPoolExecutor(max_workers=16) as executor:
            results = list(executor.map(lambda _index: update(), range(16)))

        revision, value = state.snapshot()
        self.assertEqual(calls, [1])
        self.assertEqual(sum(result is not None for result in results), 1)
        self.assertEqual(revision, 1)
        self.assertEqual(value, "frozen-1")


class InputFrameValidationTest(unittest.TestCase):
    def test_observation_input_frame_defaults_to_legacy_camera_and_allows_override(self):
        resolve_frame = getattr(
            registration_estimator, "resolve_observation_input_frame", None
        )
        self.assertIsNotNone(resolve_frame)

        def absent(_name, default):
            return default

        def explicit(_name, _default):
            return "override_camera"

        self.assertEqual(resolve_frame(absent, "legacy_camera"), "legacy_camera")
        self.assertEqual(resolve_frame(explicit, "legacy_camera"), "override_camera")

    def test_odom_requires_exact_nonempty_parent_and_child_frames(self):
        valid_odom_frames = getattr(
            registration_estimator, "valid_odom_frames", None
        )
        self.assertIsNotNone(valid_odom_frames)

        self.assertTrue(valid_odom_frames("map", "base_link", "map", "base_link"))
        self.assertFalse(valid_odom_frames("", "base_link", "map", "base_link"))
        self.assertFalse(valid_odom_frames("map", "", "map", "base_link"))
        self.assertFalse(valid_odom_frames("odom", "base_link", "map", "base_link"))
        self.assertFalse(valid_odom_frames("map", "uav", "map", "base_link"))

    def test_observation_requires_exact_nonempty_input_frame(self):
        valid_observation_frame = getattr(
            registration_estimator, "valid_observation_frame", None
        )
        self.assertIsNotNone(valid_observation_frame)

        self.assertTrue(valid_observation_frame("camera_optical", "camera_optical"))
        self.assertFalse(valid_observation_frame("", "camera_optical"))
        self.assertFalse(valid_observation_frame("camera", "camera_optical"))


if __name__ == "__main__":
    unittest.main()

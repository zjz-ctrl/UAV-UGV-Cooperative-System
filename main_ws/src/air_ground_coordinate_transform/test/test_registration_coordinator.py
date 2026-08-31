#!/usr/bin/env python3

from concurrent.futures import ThreadPoolExecutor
import math
import threading
import unittest

import numpy as np

from air_ground_coordinate_transform.registration_coordinator import (
    RegistrationCoordinator,
)
from air_ground_coordinate_transform.registration_estimator import (
    RegistrationFilter,
    RegistrationSample,
    RobustBatchEstimator,
)
from air_ground_coordinate_transform.se2 import matrix_from_xyyaw


def process_noise():
    return {
        "translation_time_variance_rate": 0.1,
        "translation_uav_distance_variance_rate": 0.2,
        "translation_ugv_distance_variance_rate": 0.3,
        "yaw_time_variance_rate": 0.01,
        "yaw_uav_distance_variance_rate": 0.02,
        "yaw_ugv_distance_variance_rate": 0.03,
    }


def sample(mean, stamp):
    return RegistrationSample(
        mean=np.asarray(mean, dtype=float),
        anchor=np.zeros(2),
        covariance=np.diag([1e-6, 1e-6, 1e-7]),
        stamp=float(stamp),
    )


def coordinator(
    mode="opportunistic",
    minimum_samples=3,
    maximum_samples=6,
    periodic_seconds=10.0,
    degraded_threshold=0.01,
    max_coalesce_age=0.1,
    batch_postprocessor=None,
    window_seconds=2.0,
    sample_period=0.01,
):
    estimator = RobustBatchEstimator(
        minimum_samples,
        max_translation_residual=0.05,
        max_yaw_residual=0.03,
        minimum_translation_sigma=0.01,
        minimum_yaw_sigma=0.005,
    )
    return RegistrationCoordinator(
        mode=mode,
        registration_filter=RegistrationFilter(None, None, process_noise()),
        estimator=estimator,
        registration_window_seconds=window_seconds,
        registration_window_max_samples=maximum_samples,
        sample_period=sample_period,
        periodic_update_seconds=periodic_seconds,
        degraded_covariance_trace_threshold=degraded_threshold,
        innovation_mahalanobis_threshold=11.344866730144373,
        max_batch_coalesce_age=max_coalesce_age,
        batch_postprocessor=batch_postprocessor,
    )


def add_consistent_window(target, center, start_stamp, now=None):
    decisions = []
    for index, offset in enumerate((-0.002, 0.001, 0.003)):
        stamp = start_stamp + 0.02 * index
        decision = target.add_sample(
            sample(
                [center[0] + offset, center[1] - offset, center[2] + offset * 0.1],
                stamp,
            ),
            now=stamp if now is None else now,
        )
        decisions.append(decision)
    return decisions


class RegistrationEventTest(unittest.TestCase):
    def test_existing_fixed_yaw_batch_policy_can_be_applied_before_filter_update(self):
        def force_fixed_yaw(samples, inlier_indices, estimate):
            del samples, inlier_indices
            return type(estimate)(
                mean=np.array([estimate.mean[0], estimate.mean[1], -0.4]),
                covariance=estimate.covariance,
                inlier_count=estimate.inlier_count,
                stamp=estimate.stamp,
            )

        target = coordinator(mode="one_shot", batch_postprocessor=force_fixed_yaw)

        decision = add_consistent_window(target, [1.0, 2.0, 0.7], 1.0)[-1]

        self.assertIsNotNone(decision)
        self.assertTrue(decision.accepted)
        self.assertAlmostEqual(decision.state.mean[2], -0.4)

    def test_visual_frames_do_not_increment_revision_before_first_batch(self):
        target = coordinator(mode="one_shot")

        first = target.add_sample(sample([1.0, -2.0, 0.2], 1.0), now=1.0)
        second = target.add_sample(sample([1.001, -2.001, 0.2], 1.1), now=1.1)

        self.assertIsNone(first)
        self.assertIsNone(second)
        self.assertEqual(target.snapshot().state.revision, 0)
        decision = target.add_sample(sample([0.999, -1.999, 0.2], 1.2), now=1.2)
        self.assertIsNotNone(decision)
        self.assertTrue(decision.accepted)
        self.assertEqual(decision.revision, 1)
        self.assertEqual(target.snapshot().state.revision, 1)

    def test_one_shot_ignores_all_later_frames_and_prediction(self):
        target = coordinator(mode="one_shot")
        decision = add_consistent_window(target, [1.0, 2.0, 0.1], 1.0)[-1]
        self.assertIsNotNone(decision)
        frozen = target.snapshot()

        for index in range(12):
            target.add_sample(sample([-8.0, 9.0, -2.0], 2.0 + index * 0.02))
        target.observe_odometry("uav", 10.0, 0.0, 0.0)
        target.observe_odometry("uav", 20.0, 50.0, 0.0)

        after = target.snapshot()
        self.assertTrue(decision.accepted)
        self.assertEqual(after.status, "FROZEN")
        self.assertEqual(after.state.revision, 1)
        np.testing.assert_array_equal(after.state.mean, frozen.state.mean)
        np.testing.assert_array_equal(after.state.covariance, frozen.state.covariance)
        self.assertEqual(after.state.stamp, frozen.state.stamp)
        self.assertEqual(after.window_size, 0)

    def test_second_accepted_window_is_exactly_one_new_event_and_is_consumed(self):
        target = coordinator()
        first = add_consistent_window(target, [1.0, -2.0, 0.2], 1.0)[-1]

        decisions = add_consistent_window(target, [1.002, -2.001, 0.201], 2.0)

        self.assertIsNotNone(first)
        self.assertIsNotNone(decisions[-1])
        self.assertEqual(first.revision, 1)
        self.assertEqual([item is not None for item in decisions], [False, False, True])
        self.assertTrue(decisions[-1].accepted)
        self.assertEqual(decisions[-1].revision, 2)
        self.assertEqual(target.snapshot().state.revision, 2)
        self.assertEqual(target.snapshot().window_size, 0)

    def test_later_acceptance_keeps_updating_for_one_publication_cycle(self):
        target = coordinator()
        add_consistent_window(target, [1.0, -2.0, 0.2], 1.0)

        decision = add_consistent_window(target, [1.002, -2.001, 0.201], 2.0)[-1]

        self.assertIsNotNone(decision)
        self.assertEqual(decision.status, "UPDATING")
        self.assertEqual(target.snapshot().status, "UPDATING")
        complete = getattr(target, "complete_publication_cycle", None)
        self.assertIsNotNone(complete)
        complete()
        self.assertEqual(target.snapshot().status, "TRACKING")

    def test_gated_window_preserves_estimate_transform_stamp_and_revision(self):
        target = coordinator()
        initialized = add_consistent_window(target, [1.0, -2.0, 0.2], 1.0)[-1]
        self.assertIsNotNone(initialized)
        before = target.snapshot()
        before_transform = matrix_from_xyyaw(*before.state.mean)

        decision = add_consistent_window(target, [20.0, -30.0, 2.5], 2.0)[-1]

        self.assertIsNotNone(decision)
        after = target.snapshot()
        self.assertFalse(decision.accepted)
        self.assertEqual(decision.reason, "mahalanobis_gate")
        self.assertEqual(after.status, "REJECTED")
        self.assertEqual(after.reason, "mahalanobis_gate")
        np.testing.assert_array_equal(after.state.mean, before.state.mean)
        np.testing.assert_array_equal(after.state.covariance, before.state.covariance)
        np.testing.assert_array_equal(
            matrix_from_xyyaw(*after.state.mean), before_transform
        )
        self.assertEqual(after.state.stamp, before.state.stamp)
        self.assertEqual(after.state.revision, before.state.revision)
        self.assertEqual(after.window_size, 0)

    def test_capped_nonrobust_window_rejects_once_and_clears(self):
        target = coordinator(maximum_samples=4)
        add_consistent_window(target, [0.0, 0.0, 0.0], 1.0)

        decisions = []
        for index, x in enumerate((0.0, 10.0, 20.0, 30.0)):
            decisions.append(
                target.add_sample(sample([x, 0.0, 0.0], 2.0 + index * 0.02))
            )

        self.assertIsNotNone(decisions[-1])
        self.assertEqual([item is not None for item in decisions], [False, False, False, True])
        self.assertFalse(decisions[-1].accepted)
        self.assertEqual(decisions[-1].reason, "insufficient_inliers")
        self.assertEqual(decisions[-1].revision, 1)
        self.assertEqual(target.snapshot().window_size, 0)

    def test_production_jitter_window_expiry_rejects_insufficient_inliers_once(self):
        target = coordinator(
            minimum_samples=20,
            maximum_samples=60,
            window_seconds=3.0,
            sample_period=0.1,
        )
        for index in range(20):
            decision = target.add_sample(
                sample([0.001 * (index % 2), 0.0, 0.0], 0.11 * index)
            )
        self.assertIsNotNone(decision)
        self.assertTrue(decision.accepted)

        rejection = None
        for index in range(100):
            rejection = target.add_sample(
                sample([float(index), 0.2 * index, 0.1 * index], 3.0 + 0.11 * index)
            )
            if rejection is not None:
                break

        self.assertIsNotNone(rejection)
        self.assertFalse(rejection.accepted)
        self.assertEqual(rejection.reason, "insufficient_inliers")
        self.assertEqual(rejection.revision, 1)
        self.assertEqual(target.snapshot().window_size, 0)
        self.assertIsNone(
            target.add_sample(sample([200.0, 40.0, 2.0], 3.0 + 0.11 * (index + 1)))
        )
        self.assertEqual(target.snapshot().window_size, 1)

    def test_periodic_mode_cannot_update_before_interval(self):
        target = coordinator(mode="periodic", periodic_seconds=10.0)
        first = add_consistent_window(target, [1.0, 1.0, 0.1], 1.0)[-1]

        early = add_consistent_window(target, [1.001, 1.0, 0.1], 10.95)
        due = target.add_sample(sample([1.0, 1.001, 0.1], 11.05), now=11.05)

        self.assertIsNotNone(first)
        self.assertIsNotNone(due)
        self.assertEqual(first.revision, 1)
        self.assertTrue(all(item is None for item in early))
        self.assertEqual(due.revision, 2)
        self.assertEqual(target.snapshot().state.revision, 2)
        self.assertEqual(target.snapshot().window_size, 0)

    def test_periodic_tick_decides_ready_window_without_new_frame_at_deadline(self):
        target = coordinator(mode="periodic", periodic_seconds=10.0)
        first = add_consistent_window(target, [1.0, 1.0, 0.1], 1.0)[-1]
        early = add_consistent_window(target, [1.001, 1.0, 0.1], 10.95)
        tick = getattr(target, "tick", None)

        self.assertIsNotNone(first)
        self.assertTrue(all(item is None for item in early))
        self.assertIsNotNone(tick)
        decision = tick(11.05)
        self.assertIsNotNone(decision)
        self.assertTrue(decision.accepted)
        self.assertEqual(decision.revision, 2)
        self.assertEqual(target.snapshot().window_size, 0)
        self.assertIsNone(tick(11.05))
        self.assertEqual(target.snapshot().state.revision, 2)

    def test_periodic_tick_passes_materially_stale_ready_window_to_task6(self):
        target = coordinator(
            mode="periodic",
            periodic_seconds=10.0,
            max_coalesce_age=0.1,
            window_seconds=20.0,
        )
        add_consistent_window(target, [0.0, 0.0, 0.0], 1.0)
        retained = add_consistent_window(target, [0.001, 0.0, 0.0], 10.0)
        self.assertTrue(all(item is None for item in retained))
        target.observe_odometry("uav", 12.0, 0.0, 0.0)

        tick = getattr(target, "tick", None)
        self.assertIsNotNone(tick)
        decision = tick(12.0)

        self.assertIsNotNone(decision)
        self.assertFalse(decision.accepted)
        self.assertEqual(decision.reason, "stale_batch")
        self.assertEqual(decision.revision, 1)
        self.assertEqual(target.snapshot().window_size, 0)

    def test_same_window_concurrency_can_create_only_one_event(self):
        target = coordinator(mode="one_shot")
        target.add_sample(sample([1.0, 2.0, 0.3], 1.0))
        target.add_sample(sample([1.001, 1.999, 0.3], 1.1))
        barrier = threading.Barrier(2)

        def submit_last_frame():
            barrier.wait()
            return target.add_sample(sample([0.999, 2.001, 0.3], 1.2))

        with ThreadPoolExecutor(max_workers=2) as executor:
            decisions = list(executor.map(lambda _: submit_last_frame(), range(2)))

        self.assertEqual(sum(item is not None for item in decisions), 1)
        self.assertEqual(sum(item is not None and item.accepted for item in decisions), 1)
        self.assertEqual(target.snapshot().state.revision, 1)


class RegistrationPredictionTest(unittest.TestCase):
    def test_one_shot_first_batch_preserves_legacy_covariance_and_stamp(self):
        target = coordinator(
            mode="one_shot", minimum_samples=1, max_coalesce_age=0.2
        )
        target.observe_odometry("uav", 0.9, 0.0, 0.0)
        target.observe_odometry("ugv", 0.9, 0.0, 0.0)
        target.observe_odometry("uav", 1.1, 2.0, 0.0)
        target.observe_odometry("ugv", 1.08, 0.9, 0.0)

        decision = target.add_sample(sample([1.0, -2.0, 0.2], 1.0), now=1.1)

        self.assertIsNotNone(decision)
        self.assertTrue(decision.accepted)
        np.testing.assert_allclose(
            decision.state.mean, [1.0, -2.0, 0.2], rtol=0.0, atol=2e-16
        )
        np.testing.assert_allclose(
            decision.state.covariance,
            np.diag([0.000101, 0.000101, 0.0000251]),
            rtol=0.0,
            atol=1e-14,
        )
        self.assertEqual(decision.state.stamp, 1.0)
        self.assertEqual(decision.state.revision, 1)
        self.assertEqual(decision.status, "FROZEN")

    def test_first_batch_accounts_preinitialization_time_and_segment_distances(self):
        target = coordinator(minimum_samples=1, max_coalesce_age=0.2)
        target.observe_odometry("uav", 0.9, 0.0, 0.0)
        target.observe_odometry("ugv", 0.9, 0.0, 0.0)
        target.observe_odometry("uav", 1.1, 2.0, 0.0)
        target.observe_odometry("ugv", 1.08, 0.9, 0.0)

        decision = target.add_sample(sample([1.0, -2.0, 0.2], 1.0), now=1.1)

        self.assertIsNotNone(decision)
        self.assertTrue(decision.accepted)
        np.testing.assert_allclose(
            decision.state.mean, [1.0, -2.0, 0.2], rtol=0.0, atol=2e-16
        )
        np.testing.assert_allclose(
            decision.state.covariance,
            np.diag([0.330101, 0.330101, 0.0330251]),
            rtol=0.0,
            atol=1e-14,
        )
        self.assertEqual(decision.state.stamp, 1.1)
        self.assertEqual(decision.state.revision, 1)

    def test_first_batch_older_than_global_coalesce_age_is_task6_stale(self):
        for mode in ("one_shot", "opportunistic", "periodic"):
            with self.subTest(mode=mode):
                target = coordinator(
                    mode=mode, minimum_samples=1, max_coalesce_age=0.08
                )
                target.observe_odometry("uav", 1.15, 0.0, 0.0)

                decision = target.add_sample(
                    sample([1.0, -2.0, 0.2], 1.03), now=1.15
                )

                self.assertIsNotNone(decision)
                self.assertFalse(decision.accepted)
                self.assertEqual(decision.reason, "stale_batch")
                self.assertFalse(decision.state.initialized)
                self.assertEqual(decision.state.revision, 0)

    def test_first_batch_newer_than_global_clock_advances_clock_without_duplicate_dt(self):
        target = coordinator(minimum_samples=1, max_coalesce_age=0.08)
        target.observe_odometry("uav", 1.0, 0.0, 0.0)
        decision = target.add_sample(sample([1.0, -2.0, 0.2], 1.05), now=1.05)
        before = target.snapshot().state

        acceptance = target.observe_odometry("ugv", 1.02, 0.0, 0.0)

        self.assertTrue(decision.accepted)
        self.assertTrue(acceptance.accepted)
        after = target.snapshot().state
        self.assertEqual(after.stamp, 1.05)
        np.testing.assert_array_equal(after.covariance, before.covariance)
        self.assertEqual(after.revision, 1)

    def test_two_vehicle_callbacks_accumulate_distance_but_elapsed_time_once(self):
        target = coordinator()
        target.observe_odometry("uav", 1.0, 0.0, 0.0)
        target.observe_odometry("ugv", 1.0, 0.0, 0.0)
        initialized = add_consistent_window(target, [1.0, -2.0, 0.2], 0.96)[-1]
        self.assertIsNotNone(initialized)
        before = target.snapshot().state

        target.observe_odometry("uav", 2.0, 1.0, 0.0)
        target.observe_odometry("ugv", 2.0, 0.0, 2.0)

        after = target.snapshot().state
        np.testing.assert_array_equal(after.mean, before.mean)
        np.testing.assert_allclose(
            after.covariance,
            before.covariance + np.diag([0.9, 0.9, 0.09]),
            rtol=0.0,
            atol=1e-14,
        )
        self.assertEqual(after.revision, before.revision)
        self.assertEqual(after.stamp, 2.0)

    def test_prediction_enters_degraded_and_accepted_update_contracts_to_tracking(self):
        target = coordinator(degraded_threshold=0.01)
        add_consistent_window(target, [1.0, -2.0, 0.2], 1.0)
        self.assertEqual(target.snapshot().status, "TRACKING")
        revision = target.snapshot().state.revision

        target.observe_odometry("uav", 1.04, 0.0, 0.0)
        target.observe_odometry("uav", 2.04, 0.0, 0.0)

        degraded = target.snapshot()
        self.assertEqual(degraded.status, "DEGRADED")
        self.assertEqual(degraded.state.revision, revision)
        decision = add_consistent_window(target, [1.001, -2.001, 0.201], 2.1)[-1]
        self.assertIsNotNone(decision)
        self.assertTrue(decision.accepted)
        self.assertEqual(decision.revision, revision + 1)
        self.assertEqual(target.snapshot().status, "UPDATING")
        target.complete_publication_cycle()
        self.assertEqual(target.snapshot().status, "TRACKING")
        self.assertLess(np.trace(target.snapshot().state.covariance), 0.01)

    def test_near_ordering_delay_coalesces_but_material_delay_is_stale(self):
        target = coordinator(max_coalesce_age=0.1)
        add_consistent_window(target, [0.0, 0.0, 0.0], 1.0)
        target.observe_odometry("uav", 1.04, 0.0, 0.0)
        target.observe_odometry("uav", 2.0, 0.0, 0.0)

        near = add_consistent_window(target, [0.001, 0.0, 0.0], 1.93)[-1]
        self.assertIsNotNone(near)
        self.assertTrue(near.accepted)
        self.assertEqual(target.snapshot().state.stamp, 2.0)
        target.observe_odometry("uav", 3.0, 0.0, 0.0)
        before_stale = target.snapshot().state

        stale = add_consistent_window(target, [0.002, 0.0, 0.0], 2.5)[-1]

        self.assertIsNotNone(stale)
        self.assertFalse(stale.accepted)
        self.assertEqual(stale.reason, "stale_batch")
        after = target.snapshot().state
        np.testing.assert_array_equal(after.mean, before_stale.mean)
        np.testing.assert_array_equal(after.covariance, before_stale.covariance)
        self.assertEqual(after.stamp, before_stale.stamp)
        self.assertEqual(after.revision, before_stale.revision)


if __name__ == "__main__":
    unittest.main()

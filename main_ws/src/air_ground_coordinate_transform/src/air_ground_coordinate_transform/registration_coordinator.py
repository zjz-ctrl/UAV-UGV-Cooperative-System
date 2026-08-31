"""Serialize registration window decisions independently of ROS callbacks."""

from dataclasses import dataclass
import math
import threading

import numpy as np

from .registration_estimator import BatchEstimate


@dataclass(frozen=True)
class CoordinatorSnapshot:
    state: object
    status: str
    reason: str
    window_size: int


@dataclass(frozen=True)
class RegistrationDecision:
    accepted: bool
    revision: int
    reason: str
    status: str
    inlier_count: int
    innovation: np.ndarray
    mahalanobis: float
    state: object


@dataclass(frozen=True)
class OdometryAcceptance:
    accepted: bool
    reason: str


class RegistrationCoordinator:
    MODES = ("one_shot", "periodic", "opportunistic")

    def __init__(
        self,
        mode,
        registration_filter,
        estimator,
        registration_window_seconds,
        registration_window_max_samples,
        sample_period,
        periodic_update_seconds,
        degraded_covariance_trace_threshold,
        innovation_mahalanobis_threshold,
        max_batch_coalesce_age,
        batch_postprocessor=None,
    ):
        if mode not in self.MODES:
            raise ValueError("registration mode must be one_shot, periodic, or opportunistic")
        self.lock = threading.RLock()
        self.mode = mode
        self._filter = registration_filter
        self._estimator = estimator
        self._window_seconds = float(registration_window_seconds)
        self._window_max_samples = int(registration_window_max_samples)
        self._sample_period = float(sample_period)
        self._periodic_seconds = float(periodic_update_seconds)
        self._degraded_threshold = float(degraded_covariance_trace_threshold)
        self._mahalanobis_threshold = float(innovation_mahalanobis_threshold)
        self._max_coalesce_age = float(max_batch_coalesce_age)
        self._batch_postprocessor = batch_postprocessor
        if (
            self._window_seconds <= 0.0
            or self._window_max_samples < self._estimator.min_samples
            or self._sample_period < 0.0
            or self._periodic_seconds < 0.0
            or self._degraded_threshold < 0.0
            or self._max_coalesce_age < 0.0
        ):
            raise ValueError("registration coordinator timing and limits are invalid")
        self._samples = []
        self._last_sample_stamp = None
        self._last_revision_time = None
        self._prediction_stamp = None
        self._odometry = {"uav": None, "ugv": None}
        self._preinitialization_motion = []
        self._status = "ACQUIRING_INITIAL"
        self._reason = ""
        self._pending_status = None

    def snapshot(self):
        with self.lock:
            return CoordinatorSnapshot(
                self._filter.state,
                self._status,
                self._reason,
                len(self._samples),
            )

    def _tracking_status(self, state):
        if float(np.trace(state.covariance)) > self._degraded_threshold:
            return "DEGRADED"
        return "TRACKING"

    def _decision(self, update, inlier_count):
        state = self._filter.state
        if update.accepted:
            self._last_revision_time = self._decision_time
            if self._prediction_stamp is None or state.stamp > self._prediction_stamp:
                self._prediction_stamp = state.stamp
            final_status = (
                "FROZEN" if self.mode == "one_shot" else self._tracking_status(state)
            )
            if self.mode != "one_shot" and state.revision > 1:
                self._status = "UPDATING"
                self._pending_status = final_status
            else:
                self._status = final_status
                self._pending_status = None
        else:
            self._status = "REJECTED"
            self._pending_status = None
        self._reason = update.reason
        return RegistrationDecision(
            accepted=update.accepted,
            revision=state.revision,
            reason=update.reason,
            status=self._status,
            inlier_count=int(inlier_count),
            innovation=update.innovation.copy(),
            mahalanobis=float(update.mahalanobis),
            state=state,
        )

    def complete_publication_cycle(self):
        with self.lock:
            if self._pending_status is not None:
                self._status = self._pending_status
                self._pending_status = None

    def _coalesce_callback_ordering_delay(self, batch):
        state = self._filter.state
        if not state.initialized or batch.stamp >= state.stamp:
            return batch
        age = state.stamp - batch.stamp
        if age > self._max_coalesce_age:
            return batch
        return BatchEstimate(
            mean=batch.mean,
            covariance=batch.covariance,
            inlier_count=batch.inlier_count,
            stamp=state.stamp,
        )

    def _preinitialization_distances_since(self, stamp):
        distances = {"uav": 0.0, "ugv": 0.0}
        for vehicle, start, end, distance in self._preinitialization_motion:
            overlap_start = max(float(stamp), start)
            if end <= overlap_start:
                continue
            distances[vehicle] += distance * (end - overlap_start) / (end - start)
        return distances["uav"], distances["ugv"]

    def _attempt_window(self, decision_time, expired=False):
        if len(self._samples) < self._estimator.min_samples:
            return None
        due = (
            not self._filter.initialized
            or self.mode != "periodic"
            or self._last_revision_time is None
            or decision_time - self._last_revision_time >= self._periodic_seconds
        )
        if not due:
            return None

        batch, inlier_indices = self._estimator.estimate_with_inliers(self._samples)
        if batch is None:
            full = len(self._samples) >= self._window_max_samples
            aged = self._samples[-1].stamp - self._samples[0].stamp >= self._window_seconds
            if not full and not aged and not expired:
                return None
            self._samples = []
            self._status = "REJECTED"
            self._reason = "insufficient_inliers"
            state = self._filter.state
            return RegistrationDecision(
                accepted=False,
                revision=state.revision,
                reason=self._reason,
                status=self._status,
                inlier_count=0,
                innovation=np.zeros(3),
                mahalanobis=float("nan"),
                state=state,
            )

        if self._batch_postprocessor is not None:
            batch = self._batch_postprocessor(
                tuple(self._samples), inlier_indices, batch
            )
        self._samples = []
        self._decision_time = decision_time
        was_initialized = self._filter.initialized
        current_stamp = None
        prediction_age = None
        if (
            not was_initialized
            and self._prediction_stamp is not None
            and batch.stamp < self._prediction_stamp
        ):
            age = self._prediction_stamp - batch.stamp
            if age > self._max_coalesce_age:
                current_stamp = self._prediction_stamp
            else:
                prediction_age = age
        else:
            batch = self._coalesce_callback_ordering_delay(batch)
        update = self._filter.update(
            batch,
            self._mahalanobis_threshold,
            current_stamp=current_stamp,
        )
        if not was_initialized and update.accepted:
            if prediction_age is not None and self.mode != "one_shot":
                uav_distance, ugv_distance = self._preinitialization_distances_since(
                    batch.stamp
                )
                self._filter.predict(prediction_age, uav_distance, ugv_distance)
            self._preinitialization_motion = []
        return self._decision(update, len(inlier_indices))

    def add_sample(self, sample, now=None):
        with self.lock:
            if self.mode == "one_shot" and self._filter.initialized:
                return None
            stamp = float(sample.stamp)
            decision_time = stamp if now is None else float(now)
            if not math.isfinite(stamp) or not math.isfinite(decision_time):
                return None
            if (
                self._last_sample_stamp is not None
                and stamp - self._last_sample_stamp < self._sample_period
            ):
                return None
            self._last_sample_stamp = stamp
            oldest_fresh_stamp = stamp - self._window_seconds
            expired = any(
                retained.stamp < oldest_fresh_stamp for retained in self._samples
            )
            self._samples = [
                retained
                for retained in self._samples
                if retained.stamp >= oldest_fresh_stamp
            ]
            self._samples.append(sample)
            if len(self._samples) > self._window_max_samples:
                self._samples = self._samples[-self._window_max_samples :]
            return self._attempt_window(decision_time, expired=expired)

    def tick(self, now):
        now = float(now)
        if not math.isfinite(now):
            return None
        with self.lock:
            if self.mode != "periodic" or not self._filter.initialized:
                return None
            oldest_fresh_stamp = now - self._window_seconds
            expired = any(
                retained.stamp < oldest_fresh_stamp for retained in self._samples
            )
            self._samples = [
                retained
                for retained in self._samples
                if retained.stamp >= oldest_fresh_stamp
            ]
            return self._attempt_window(now, expired=expired)

    def observe_odometry(self, vehicle, stamp, x, y):
        if vehicle not in self._odometry:
            raise ValueError("vehicle must be uav or ugv")
        try:
            stamp, x, y = float(stamp), float(x), float(y)
        except (TypeError, ValueError, OverflowError):
            return OdometryAcceptance(False, "invalid_odometry")
        if not all(math.isfinite(value) for value in (stamp, x, y)) or stamp < 0.0:
            return OdometryAcceptance(False, "invalid_odometry")
        with self.lock:
            previous = self._odometry[vehicle]
            if previous is not None and stamp <= previous[0]:
                return OdometryAcceptance(False, "nonmonotonic_odometry")
            distance = 0.0
            if previous is not None:
                distance = math.hypot(x - previous[1], y - previous[2])
                if not self._filter.initialized:
                    self._preinitialization_motion.append(
                        (vehicle, previous[0], stamp, distance)
                    )
            self._odometry[vehicle] = (stamp, x, y)

            dt = 0.0
            if self._prediction_stamp is None:
                self._prediction_stamp = stamp
            elif stamp > self._prediction_stamp:
                dt = stamp - self._prediction_stamp
                self._prediction_stamp = stamp
            if not self._filter.initialized and self._prediction_stamp is not None:
                cutoff = self._prediction_stamp - self._max_coalesce_age
                self._preinitialization_motion = [
                    increment
                    for increment in self._preinitialization_motion
                    if increment[2] >= cutoff
                ]
            if self.mode == "one_shot" and self._filter.initialized:
                return OdometryAcceptance(True, "accepted")
            uav_distance = distance if vehicle == "uav" else 0.0
            ugv_distance = distance if vehicle == "ugv" else 0.0
            state = self._filter.predict(dt, uav_distance, ugv_distance)
            if state.initialized and np.trace(state.covariance) > self._degraded_threshold:
                if self._pending_status is None:
                    self._status = "DEGRADED"
                else:
                    self._pending_status = "DEGRADED"
            return OdometryAcceptance(True, "accepted")

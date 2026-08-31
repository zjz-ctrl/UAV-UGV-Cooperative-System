"""ROS-independent robust estimation for planar frame registration."""

from dataclasses import dataclass
import math
import threading

import numpy as np

from .se2 import wrap_angle, wrap_xyyaw


@dataclass(frozen=True)
class RegistrationSample:
    mean: np.ndarray
    anchor: np.ndarray
    covariance: np.ndarray
    stamp: float


@dataclass(frozen=True)
class BatchEstimate:
    mean: np.ndarray
    covariance: np.ndarray
    inlier_count: int
    stamp: float


@dataclass(frozen=True)
class FilterState:
    mean: np.ndarray
    covariance: np.ndarray
    revision: int
    stamp: float
    initialized: bool


@dataclass(frozen=True)
class UpdateResult:
    accepted: bool
    innovation: np.ndarray
    mahalanobis: float
    mean: np.ndarray
    covariance: np.ndarray
    revision: int
    reason: str


class RegistrationFilter:
    _PROCESS_NOISE_NAMES = (
        "translation_time_variance_rate",
        "translation_uav_distance_variance_rate",
        "translation_ugv_distance_variance_rate",
        "yaw_time_variance_rate",
        "yaw_uav_distance_variance_rate",
        "yaw_ugv_distance_variance_rate",
    )
    _SCALED_RCOND_MIN = math.sqrt(np.finfo(float).eps)

    def __init__(self, initial_mean, initial_covariance, process_noise):
        try:
            rates = {
                name: float(process_noise[name]) for name in self._PROCESS_NOISE_NAMES
            }
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            raise ValueError(
                "process_noise must provide six finite nonnegative variance rates"
            ) from error
        if any(not math.isfinite(value) or value < 0.0 for value in rates.values()):
            raise ValueError(
                "process_noise must provide six finite nonnegative variance rates"
            )
        self._process_noise = rates

        if initial_mean is None and initial_covariance is None:
            self._state = FilterState(None, None, 0, 0.0, False)
            return
        if initial_mean is None or initial_covariance is None:
            raise ValueError(
                "initial_mean and initial_covariance must both be None or both be valid"
            )
        try:
            mean = np.asarray(initial_mean, dtype=float)
            covariance = np.asarray(initial_covariance, dtype=float)
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError(
                "initial_mean and initial_covariance must contain numeric values"
            ) from error
        if mean.shape != (3,) or not np.all(np.isfinite(mean)):
            raise ValueError("initial_mean must be finite with shape (3,)")
        if not self._valid_covariance(covariance):
            raise ValueError(
                "initial_covariance must be finite, symmetric PSD with shape (3, 3)"
            )
        self._state = FilterState(mean.copy(), covariance.copy(), 1, 0.0, True)

    @staticmethod
    def _valid_covariance(covariance):
        if covariance.shape != (3, 3) or not np.all(np.isfinite(covariance)):
            return False
        if not np.allclose(covariance, covariance.T, rtol=0.0, atol=1e-12):
            return False
        try:
            return float(np.min(np.linalg.eigvalsh(covariance))) >= -1e-12
        except np.linalg.LinAlgError:
            return False

    @staticmethod
    def _snapshot(state):
        return FilterState(
            mean=None if state.mean is None else state.mean.copy(),
            covariance=None if state.covariance is None else state.covariance.copy(),
            revision=state.revision,
            stamp=state.stamp,
            initialized=state.initialized,
        )

    @staticmethod
    def _cholesky_solve(cholesky, right_hand_side):
        intermediate = np.linalg.solve(cholesky, right_hand_side)
        return np.linalg.solve(cholesky.T, intermediate)

    @property
    def state(self):
        return self._snapshot(self._state)

    @property
    def initialized(self):
        return self._state.initialized

    def predict(self, dt, uav_distance, ugv_distance):
        values = []
        for name, value in (
            ("dt", dt),
            ("uav_distance", uav_distance),
            ("ugv_distance", ugv_distance),
        ):
            try:
                value = float(value)
            except (TypeError, ValueError, OverflowError) as error:
                raise ValueError("{} must be finite and nonnegative".format(name)) from error
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("{} must be finite and nonnegative".format(name))
            values.append(value)
        dt, uav_distance, ugv_distance = values
        if not self.initialized:
            return self.state

        translation_growth = (
            self._process_noise["translation_time_variance_rate"] * dt
            + self._process_noise["translation_uav_distance_variance_rate"]
            * uav_distance
            + self._process_noise["translation_ugv_distance_variance_rate"]
            * ugv_distance
        )
        yaw_growth = (
            self._process_noise["yaw_time_variance_rate"] * dt
            + self._process_noise["yaw_uav_distance_variance_rate"] * uav_distance
            + self._process_noise["yaw_ugv_distance_variance_rate"] * ugv_distance
        )
        stamp = self._state.stamp + dt
        if not all(math.isfinite(value) for value in (translation_growth, yaw_growth, stamp)):
            raise ArithmeticError("prediction produced a nonfinite state")
        try:
            with np.errstate(over="raise", invalid="raise"):
                covariance = self._state.covariance + np.diag(
                    [translation_growth, translation_growth, yaw_growth]
                )
        except FloatingPointError as error:
            raise ArithmeticError("prediction produced a nonfinite state") from error
        if not self._valid_covariance(covariance):
            raise ArithmeticError("prediction covariance is not finite, symmetric, and PSD")
        self._state = FilterState(
            self._state.mean.copy(),
            covariance,
            self._state.revision,
            stamp,
            True,
        )
        return self.state

    def update(self, batch, mahalanobis_threshold, current_stamp=None):
        try:
            mahalanobis_threshold = float(mahalanobis_threshold)
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError(
                "mahalanobis_threshold must be finite and nonnegative"
            ) from error
        if not math.isfinite(mahalanobis_threshold) or mahalanobis_threshold < 0.0:
            raise ValueError("mahalanobis_threshold must be finite and nonnegative")
        if current_stamp is not None:
            try:
                current_stamp = float(current_stamp)
            except (TypeError, ValueError, OverflowError) as error:
                raise ValueError("current_stamp must be finite and nonnegative") from error
            if not math.isfinite(current_stamp) or current_stamp < 0.0:
                raise ValueError("current_stamp must be finite and nonnegative")

        try:
            measurement_mean = np.asarray(batch.mean, dtype=float)
            measurement_covariance = np.asarray(batch.covariance, dtype=float)
            measurement_stamp = float(batch.stamp)
        except (AttributeError, TypeError, ValueError, OverflowError):
            return self._rejection("invalid_batch")
        if (
            measurement_mean.shape != (3,)
            or not np.all(np.isfinite(measurement_mean))
            or not self._valid_covariance(measurement_covariance)
            or not math.isfinite(measurement_stamp)
            or measurement_stamp < 0.0
        ):
            return self._rejection("invalid_batch")
        causal_stamp = self._state.stamp if self.initialized else current_stamp
        if causal_stamp is not None and measurement_stamp < causal_stamp:
            return self._rejection("stale_batch")

        if not self.initialized:
            mean = wrap_xyyaw(measurement_mean)
            covariance = measurement_covariance.copy()
            self._state = FilterState(mean, covariance, 1, measurement_stamp, True)
            return UpdateResult(
                True,
                np.zeros(3),
                0.0,
                mean.copy(),
                covariance.copy(),
                1,
                "initialized",
            )

        try:
            with np.errstate(over="raise", invalid="raise"):
                innovation = measurement_mean - self._state.mean
                innovation[2] = wrap_angle(innovation[2])
        except (FloatingPointError, OverflowError):
            return self._rejection("singular_innovation_covariance")
        if not np.all(np.isfinite(innovation)):
            return self._rejection("singular_innovation_covariance")

        try:
            with np.errstate(
                over="raise", invalid="raise", divide="raise", under="ignore"
            ):
                innovation_covariance = (
                    self._state.covariance + measurement_covariance
                )
                diagonal_scale = np.sqrt(np.diag(innovation_covariance))
                scale_outer = np.outer(diagonal_scale, diagonal_scale)
                scaled_innovation_covariance = innovation_covariance / scale_outer
        except FloatingPointError:
            return self._rejection(
                "singular_innovation_covariance", innovation=innovation
            )
        if (
            not np.all(np.isfinite(innovation_covariance))
            or not np.all(np.isfinite(diagonal_scale))
            or np.any(diagonal_scale <= 0.0)
            or not np.all(np.isfinite(scaled_innovation_covariance))
        ):
            return self._rejection(
                "singular_innovation_covariance", innovation=innovation
            )

        try:
            scaled_eigenvalues = np.linalg.eigvalsh(scaled_innovation_covariance)
        except np.linalg.LinAlgError:
            return self._rejection(
                "singular_innovation_covariance", innovation=innovation
            )
        largest_eigenvalue = float(scaled_eigenvalues[-1])
        smallest_eigenvalue = float(scaled_eigenvalues[0])
        if (
            not np.all(np.isfinite(scaled_eigenvalues))
            or smallest_eigenvalue <= 0.0
            or largest_eigenvalue <= 0.0
            or smallest_eigenvalue / largest_eigenvalue < self._SCALED_RCOND_MIN
        ):
            return self._rejection(
                "singular_innovation_covariance", innovation=innovation
            )

        try:
            with np.errstate(
                over="raise", invalid="raise", divide="raise", under="ignore"
            ):
                cholesky = np.linalg.cholesky(scaled_innovation_covariance)
                scaled_innovation = innovation / diagonal_scale
                solved_innovation = self._cholesky_solve(
                    cholesky, scaled_innovation
                )
                mahalanobis = float(scaled_innovation @ solved_innovation)
        except (np.linalg.LinAlgError, FloatingPointError, OverflowError):
            return self._rejection(
                "singular_innovation_covariance", innovation=innovation
            )
        if (
            not np.all(np.isfinite(cholesky))
            or not np.all(np.isfinite(scaled_innovation))
            or not np.all(np.isfinite(solved_innovation))
            or not math.isfinite(mahalanobis)
            or mahalanobis < 0.0
        ):
            return self._rejection(
                "singular_innovation_covariance", innovation=innovation
            )
        if mahalanobis > mahalanobis_threshold:
            return self._rejection(
                "mahalanobis_gate", innovation=innovation, mahalanobis=mahalanobis
            )

        try:
            with np.errstate(
                over="raise", invalid="raise", divide="raise", under="ignore"
            ):
                scaled_prior = self._state.covariance / diagonal_scale[np.newaxis, :]
                scaled_gain = self._cholesky_solve(
                    cholesky, scaled_prior.T
                ).T
                gain = scaled_gain / diagonal_scale[np.newaxis, :]
                correction = gain @ innovation
                mean = wrap_xyyaw(self._state.mean + correction)
                identity_minus_gain = np.eye(3) - gain
                covariance = (
                    identity_minus_gain
                    @ self._state.covariance
                    @ identity_minus_gain.T
                    + gain @ measurement_covariance @ gain.T
                )
                covariance = 0.5 * (covariance + covariance.T)
        except (np.linalg.LinAlgError, FloatingPointError, OverflowError):
            return self._rejection(
                "singular_innovation_covariance",
                innovation=innovation,
                mahalanobis=mahalanobis,
            )
        if (
            not np.all(np.isfinite(scaled_prior))
            or not np.all(np.isfinite(scaled_gain))
            or not np.all(np.isfinite(gain))
            or not np.all(np.isfinite(correction))
            or not np.all(np.isfinite(mean))
            or not self._valid_covariance(covariance)
        ):
            return self._rejection(
                "singular_innovation_covariance",
                innovation=innovation,
                mahalanobis=mahalanobis,
            )

        revision = self._state.revision + 1
        self._state = FilterState(
            mean, covariance, revision, measurement_stamp, True
        )
        return UpdateResult(
            True,
            innovation.copy(),
            mahalanobis,
            mean.copy(),
            covariance.copy(),
            revision,
            "accepted",
        )

    def _rejection(self, reason, innovation=None, mahalanobis=float("nan")):
        if innovation is None:
            innovation = np.zeros(3)
        mean = None if self._state.mean is None else self._state.mean.copy()
        covariance = (
            None if self._state.covariance is None else self._state.covariance.copy()
        )
        return UpdateResult(
            False,
            np.asarray(innovation, dtype=float).copy(),
            float(mahalanobis),
            mean,
            covariance,
            self._state.revision,
            reason,
        )


class OneShotRegistrationState:
    """Serialize candidate updates and publish exactly one frozen value."""

    def __init__(self):
        self._lock = threading.Lock()
        self._revision = 0
        self._value = None

    def update(self, build_value):
        with self._lock:
            if self._revision != 0:
                return None
            value = build_value()
            if value is None:
                return None
            self._value = value
            self._revision = 1
            return value

    def snapshot(self):
        with self._lock:
            return self._revision, self._value


def valid_odom_frames(parent, child, expected_parent, expected_child):
    return (
        bool(parent)
        and bool(child)
        and parent == expected_parent
        and child == expected_child
    )


def valid_observation_frame(frame, expected_frame):
    return bool(frame) and frame == expected_frame


def resolve_observation_input_frame(get_param, camera_frame):
    return get_param("~observation_input_frame", camera_frame)


def _quaternion_matrix(quaternion):
    quaternion = np.asarray(quaternion, dtype=float)
    norm = float(np.dot(quaternion, quaternion))
    if norm == 0.0:
        return np.eye(4)
    x, y, z, w = quaternion / math.sqrt(norm)
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
                0.0,
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
                0.0,
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
                0.0,
            ],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def _pose_matrix(pose):
    pose = np.asarray(pose, dtype=float)
    matrix = _quaternion_matrix(pose[3:7])
    matrix[:3, 3] = pose[:3]
    return matrix


def _xyzrpy_matrix(value):
    x, y, z, roll, pitch, yaw = np.asarray(value, dtype=float)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    matrix = np.eye(4)
    matrix[:3, :3] = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    matrix[:3, 3] = [x, y, z]
    return matrix


def _planar_mean(matrix):
    return wrap_xyyaw(
        [matrix[0, 3], matrix[1, 3], math.atan2(matrix[1, 0], matrix[0, 0])]
    )


def _symmetric_psd(covariance):
    covariance = 0.5 * (
        np.asarray(covariance, dtype=float) + np.asarray(covariance, dtype=float).T
    )
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    return (eigenvectors * np.maximum(eigenvalues, 0.0)) @ eigenvectors.T


def registration_sample_from_observation(
    origin_to_uav_odom,
    uav_pose,
    base_camera,
    observation_mean,
    observation_covariance,
    ugv_pose,
    base_board,
    anchor,
    stamp,
):
    """Build one planar sample and propagate camera-frame pose covariance."""
    prefix = (
        np.asarray(origin_to_uav_odom, dtype=float)
        @ _pose_matrix(uav_pose)
        @ np.asarray(base_camera, dtype=float)
    )
    suffix = np.linalg.inv(
        _pose_matrix(ugv_pose) @ np.asarray(base_board, dtype=float)
    )

    def evaluate(value):
        return _planar_mean(prefix @ _xyzrpy_matrix(value) @ suffix)

    observation_mean = np.asarray(observation_mean, dtype=float)
    mean = evaluate(observation_mean)
    jacobian = np.empty((3, 6))
    step = 1e-6
    for column in range(6):
        upper = observation_mean.copy()
        lower = observation_mean.copy()
        upper[column] += step
        lower[column] -= step
        delta = evaluate(upper) - evaluate(lower)
        delta[2] = wrap_angle(delta[2])
        jacobian[:, column] = delta / (2.0 * step)
    covariance = jacobian @ np.asarray(observation_covariance, dtype=float) @ jacobian.T
    return RegistrationSample(
        mean=mean,
        anchor=np.asarray(anchor, dtype=float).copy(),
        covariance=_symmetric_psd(covariance),
        stamp=float(stamp),
    )


def _batch_estimate(samples, inlier_indices, translation_sigma, yaw_sigma):
    selected = [samples[index] for index in inlier_indices]
    means = np.asarray([sample.mean for sample in selected], dtype=float)
    mean = np.array(
        [
            np.median(means[:, 0]),
            np.median(means[:, 1]),
            math.atan2(np.mean(np.sin(means[:, 2])), np.mean(np.cos(means[:, 2]))),
        ]
    )
    residuals = means - mean
    residuals[:, 2] = [wrap_angle(value) for value in residuals[:, 2]]
    if len(selected) > 1:
        empirical_covariance = np.cov(residuals, rowvar=False, ddof=1)
    else:
        empirical_covariance = np.zeros((3, 3))
    input_covariance = np.mean(
        np.asarray([sample.covariance for sample in selected], dtype=float), axis=0
    )
    covariance = (empirical_covariance + input_covariance) / len(selected)
    covariance += np.diag(
        [translation_sigma ** 2, translation_sigma ** 2, yaw_sigma ** 2]
    )
    return BatchEstimate(
        mean=wrap_xyyaw(mean),
        covariance=_symmetric_psd(covariance),
        inlier_count=len(selected),
        stamp=float(max(sample.stamp for sample in selected)),
    )


def fixed_yaw_estimate(
    samples,
    inlier_indices,
    fixed_yaw,
    minimum_translation_sigma=0.01,
    minimum_yaw_sigma=0.005,
):
    """Re-anchor the estimator's exact inliers to a deterministic fixed yaw."""
    fixed_rotation = np.array(
        [
            [math.cos(fixed_yaw), -math.sin(fixed_yaw)],
            [math.sin(fixed_yaw), math.cos(fixed_yaw)],
        ]
    )
    transformed = []
    for index in inlier_indices:
        sample = samples[index]
        yaw = sample.mean[2]
        rotation = np.array(
            [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
        )
        rotation_derivative = np.array(
            [
                [-math.sin(yaw), -math.cos(yaw)],
                [math.cos(yaw), -math.sin(yaw)],
            ]
        )
        jacobian = np.zeros((3, 3))
        jacobian[:2, :2] = np.eye(2)
        jacobian[:2, 2] = rotation_derivative @ sample.anchor[:2]
        transformed.append(
            RegistrationSample(
                mean=np.array(
                    [
                        *(
                            sample.mean[:2]
                            + (rotation - fixed_rotation) @ sample.anchor[:2]
                        ),
                        fixed_yaw,
                    ]
                ),
                anchor=np.asarray(sample.anchor, dtype=float).copy(),
                covariance=_symmetric_psd(
                    jacobian @ sample.covariance @ jacobian.T
                ),
                stamp=sample.stamp,
            )
        )
    return _batch_estimate(
        transformed,
        tuple(range(len(transformed))),
        float(minimum_translation_sigma),
        float(minimum_yaw_sigma),
    )


class RobustBatchEstimator:
    def __init__(
        self,
        min_samples,
        max_translation_residual,
        max_yaw_residual,
        minimum_translation_sigma=0.01,
        minimum_yaw_sigma=0.005,
    ):
        self.min_samples = int(min_samples)
        self.max_translation_residual = float(max_translation_residual)
        self.max_yaw_residual = float(max_yaw_residual)
        self.minimum_translation_sigma = float(minimum_translation_sigma)
        self.minimum_yaw_sigma = float(minimum_yaw_sigma)

    def estimate_with_inliers(self, samples):
        if len(samples) < self.min_samples:
            return None, ()

        means = np.asarray([sample.mean for sample in samples], dtype=float)
        translation_center = np.median(means[:, :2], axis=0)
        translation_residual = np.linalg.norm(
            means[:, :2] - translation_center, axis=1
        )
        translation_inliers = translation_residual <= self.max_translation_residual
        if int(np.count_nonzero(translation_inliers)) < self.min_samples:
            return None, ()

        yaw_candidates = means[translation_inliers, 2]
        yaw_center = math.atan2(
            np.mean(np.sin(yaw_candidates)), np.mean(np.cos(yaw_candidates))
        )
        yaw_residual = np.abs(
            np.asarray([wrap_angle(yaw - yaw_center) for yaw in means[:, 2]])
        )
        inlier_mask = translation_inliers & (yaw_residual <= self.max_yaw_residual)
        inlier_count = int(np.count_nonzero(inlier_mask))
        if inlier_count < self.min_samples:
            return None, ()

        inlier_indices = tuple(int(index) for index in np.flatnonzero(inlier_mask))
        return _batch_estimate(
            samples,
            inlier_indices,
            self.minimum_translation_sigma,
            self.minimum_yaw_sigma,
        ), inlier_indices

    def estimate(self, samples):
        estimate, _ = self.estimate_with_inliers(samples)
        return estimate

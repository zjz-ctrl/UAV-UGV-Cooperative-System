"""Pure uncertainty policy for deciding whether an anomaly can be handed off."""

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np


DIRECT = "DIRECT"
REOBSERVE = "REOBSERVE"
REREGISTER = "REREGISTER"
HOLD = "HOLD"


@dataclass(frozen=True, eq=False)
class HandoffResult:
    valid: bool
    origin_mean: Optional[np.ndarray]
    origin_covariance: Optional[np.ndarray]
    goal_mean: Optional[np.ndarray]
    goal_sensing_covariance: Optional[np.ndarray]
    goal_registration_covariance: Optional[np.ndarray]
    goal_covariance: Optional[np.ndarray]
    action: str
    registration_radius: float
    target_radius: float
    confidence_radius: float
    yaw_confidence: float


def sample_target_covariance(
    samples_xy,
    variance_floor,
    pose_covariances=None,
    range_axes=None,
    range_variance=0.0,
):
    try:
        samples = np.asarray(samples_xy, dtype=float)
        floor = float(variance_floor)
        range_variance = float(range_variance)
        if (samples.ndim != 2 or samples.shape[0] < 2 or
                samples.shape[1] != 2 or not np.all(np.isfinite(samples)) or
                not math.isfinite(floor) or floor < 0.0 or
                not math.isfinite(range_variance) or range_variance < 0.0):
            return None

        covariance = np.cov(samples, rowvar=False, ddof=1)
        covariance += floor ** 2 * np.eye(2)

        if pose_covariances is not None:
            pose_covariances = np.asarray(pose_covariances, dtype=float)
            if pose_covariances.shape != (samples.shape[0], 2, 2):
                return None
            validated = []
            for pose_covariance in pose_covariances:
                pose_covariance = UncertaintyBudget._validated_covariance(
                    pose_covariance,
                    (2, 2),
                )
                if pose_covariance is None:
                    return None
                validated.append(pose_covariance)
            covariance += np.mean(validated, axis=0)

        if range_axes is None:
            if range_variance > 0.0:
                return None
        else:
            axes = np.asarray(range_axes, dtype=float)
            if (axes.shape != samples.shape or
                    not np.all(np.isfinite(axes))):
                return None
            norms = np.linalg.norm(axes, axis=1)
            if not np.all(np.isfinite(norms)) or np.any(norms <= 0.0):
                return None
            unit_axes = axes / norms[:, np.newaxis]
            covariance += range_variance * np.mean(
                [np.outer(axis, axis) for axis in unit_axes],
                axis=0,
            )

        return UncertaintyBudget._validated_covariance(covariance, (2, 2))
    except (TypeError, ValueError, OverflowError, FloatingPointError,
            np.linalg.LinAlgError):
        return None


class UncertaintyBudget:
    def __init__(
        self,
        registration_covariance,
        target_covariance,
        inspection_radius,
        inspection_yaw,
    ):
        self.registration_radius = math.nan
        self.target_radius = math.nan
        self.confidence_radius = math.nan
        self.yaw_confidence = math.nan
        self._valid = False

        try:
            registration = np.asarray(registration_covariance, dtype=float)
            target = np.asarray(target_covariance, dtype=float)
            radius_limit = float(inspection_radius)
            yaw_limit = float(inspection_yaw)
        except (TypeError, ValueError, OverflowError):
            return

        if (not math.isfinite(radius_limit) or radius_limit <= 0.0 or
                not math.isfinite(yaw_limit) or yaw_limit <= 0.0):
            return

        try:
            registration = self._validated_covariance(registration, (3, 3))
            target = self._validated_covariance(target, (2, 2))
            if registration is None or target is None:
                return
            registration_radius = self._r95(registration[:2, :2])
            target_radius = self._r95(target)
            confidence_radius = self._r95(registration[:2, :2] + target)
            yaw_confidence = 1.959964 * math.sqrt(
                max(0.0, registration[2, 2]))
            if not all(math.isfinite(value) for value in (
                    registration_radius,
                    target_radius,
                    confidence_radius,
                    yaw_confidence,
            )):
                return
        except (np.linalg.LinAlgError, FloatingPointError, ValueError, OverflowError):
            return
        self.registration_radius = registration_radius
        self.target_radius = target_radius
        self.confidence_radius = confidence_radius
        self.yaw_confidence = yaw_confidence
        self._inspection_radius = radius_limit
        self._inspection_yaw = yaw_limit
        self._valid = True

    @staticmethod
    def _validated_covariance(covariance, shape):
        if covariance.shape != shape or not np.all(np.isfinite(covariance)):
            return None
        if not np.allclose(covariance, covariance.T, rtol=1e-7, atol=1e-10):
            return None
        symmetric = 0.5 * (covariance + covariance.T)
        eigenvalues = np.linalg.eigvalsh(symmetric)
        scale = max(1.0, float(np.max(np.abs(symmetric))))
        tolerance = 64.0 * np.finfo(float).eps * scale
        if float(np.min(eigenvalues)) < -tolerance:
            return None
        return symmetric

    @staticmethod
    def _r95(covariance):
        largest = max(0.0, float(np.max(np.linalg.eigvalsh(covariance))))
        return math.sqrt(5.991464547 * largest)

    def choose_action(self):
        if not self._valid:
            return HOLD
        if (self.confidence_radius <= self._inspection_radius and
                self.yaw_confidence <= self._inspection_yaw):
            return DIRECT
        if self.yaw_confidence > self._inspection_yaw:
            return REREGISTER
        if self.target_radius > self.registration_radius:
            return REOBSERVE
        return REREGISTER


def standoff_goal(target_xy, anchor_xy, standoff):
    try:
        target = np.asarray(target_xy, dtype=float)
        anchor = np.asarray(anchor_xy, dtype=float)
        standoff = float(standoff)
        if (target.shape != (2,) or anchor.shape != (2,) or
                not np.all(np.isfinite(target)) or
                not np.all(np.isfinite(anchor)) or
                not math.isfinite(standoff) or standoff <= 0.0):
            return None

        relative = target - anchor
        distance = float(np.linalg.norm(relative))
        scale = max(1.0, standoff)
        if (not math.isfinite(distance) or
                distance <= math.sqrt(np.finfo(float).eps) * scale):
            return None

        unit = relative / distance
        projection = np.eye(2) - np.outer(unit, unit)
        xy_jacobian = np.eye(2) - (standoff / distance) * projection
        yaw_jacobian = np.array([
            -relative[1] / distance ** 2,
            relative[0] / distance ** 2,
        ])
        mean = np.array([
            *(target - standoff * unit),
            math.atan2(relative[1], relative[0]),
        ])
        jacobian = np.vstack((xy_jacobian, yaw_jacobian))
        if not np.all(np.isfinite(mean)) or not np.all(np.isfinite(jacobian)):
            return None
        return mean, jacobian
    except (TypeError, ValueError, OverflowError, FloatingPointError,
            np.linalg.LinAlgError):
        return None


def registration_execution_covariance(
    target_in_registration_frame,
    registration_yaw,
    registration_covariance,
):
    try:
        target = np.asarray(target_in_registration_frame, dtype=float)
        registration_yaw = float(registration_yaw)
        registration = np.asarray(registration_covariance, dtype=float)
        if (target.shape != (2,) or not np.all(np.isfinite(target)) or
                not math.isfinite(registration_yaw)):
            return None
        registration = UncertaintyBudget._validated_covariance(
            registration,
            (3, 3),
        )
        if registration is None:
            return None
        cosine = math.cos(registration_yaw)
        sine = math.sin(registration_yaw)
        yaw_derivative = np.array([
            -sine * target[0] - cosine * target[1],
            cosine * target[0] - sine * target[1],
        ])
        jacobian = np.array([
            [1.0, 0.0, yaw_derivative[0]],
            [0.0, 1.0, yaw_derivative[1]],
            [0.0, 0.0, 1.0],
        ])
        covariance = jacobian @ registration @ jacobian.T
        return UncertaintyBudget._validated_covariance(covariance, (3, 3))
    except (TypeError, ValueError, OverflowError, FloatingPointError,
            np.linalg.LinAlgError):
        return None


def _readonly(array):
    result = np.array(array, dtype=float, copy=True)
    result.setflags(write=False)
    return result


def _invalid_handoff_result():
    return HandoffResult(
        valid=False,
        origin_mean=None,
        origin_covariance=None,
        goal_mean=None,
        goal_sensing_covariance=None,
        goal_registration_covariance=None,
        goal_covariance=None,
        action=HOLD,
        registration_radius=math.nan,
        target_radius=math.nan,
        confidence_radius=math.nan,
        yaw_confidence=math.nan,
    )


def evaluate_handoff(
    target_xy,
    target_covariance,
    origin_from_uav,
    origin_from_registration,
    registration_covariance,
    standoff,
    inspection_radius,
    inspection_yaw,
):
    try:
        target = np.asarray(target_xy, dtype=float)
        target_covariance = np.asarray(target_covariance, dtype=float)
        uav_transform = np.asarray(origin_from_uav, dtype=float)
        registration_transform = np.asarray(
            origin_from_registration,
            dtype=float,
        )
        if (target.shape != (2,) or uav_transform.shape != (3,) or
                registration_transform.shape != (3,) or
                not np.all(np.isfinite(target)) or
                not np.all(np.isfinite(uav_transform)) or
                not np.all(np.isfinite(registration_transform))):
            return _invalid_handoff_result()
        target_covariance = UncertaintyBudget._validated_covariance(
            target_covariance,
            (2, 2),
        )
        registration_covariance = UncertaintyBudget._validated_covariance(
            np.asarray(registration_covariance, dtype=float),
            (3, 3),
        )
        if target_covariance is None or registration_covariance is None:
            return _invalid_handoff_result()

        cosine = math.cos(uav_transform[2])
        sine = math.sin(uav_transform[2])
        uav_rotation = np.array([[cosine, -sine], [sine, cosine]])
        origin_mean = uav_transform[:2] + uav_rotation @ target
        origin_covariance = uav_rotation @ target_covariance @ uav_rotation.T
        origin_covariance = UncertaintyBudget._validated_covariance(
            origin_covariance,
            (2, 2),
        )
        goal_geometry = standoff_goal(
            origin_mean,
            registration_transform[:2],
            standoff,
        )
        if origin_covariance is None or goal_geometry is None:
            return _invalid_handoff_result()

        goal_mean, target_jacobian = goal_geometry
        goal_sensing_covariance = (
            target_jacobian @ origin_covariance @ target_jacobian.T
        )
        goal_sensing_covariance = UncertaintyBudget._validated_covariance(
            goal_sensing_covariance,
            (3, 3),
        )
        registration_cosine = math.cos(registration_transform[2])
        registration_sine = math.sin(registration_transform[2])
        registration_rotation = np.array([
            [registration_cosine, -registration_sine],
            [registration_sine, registration_cosine],
        ])
        target_in_registration_frame = registration_rotation.T @ (
            origin_mean - registration_transform[:2]
        )
        goal_registration_covariance = registration_execution_covariance(
            target_in_registration_frame,
            registration_transform[2],
            registration_covariance,
        )
        if (goal_sensing_covariance is None or
                goal_registration_covariance is None):
            return _invalid_handoff_result()
        goal_covariance = UncertaintyBudget._validated_covariance(
            goal_sensing_covariance + goal_registration_covariance,
            (3, 3),
        )
        if goal_covariance is None:
            return _invalid_handoff_result()

        budget = UncertaintyBudget(
            goal_registration_covariance,
            goal_sensing_covariance[:2, :2],
            inspection_radius,
            inspection_yaw,
        )
        if not budget._valid:
            return _invalid_handoff_result()
        return HandoffResult(
            valid=True,
            origin_mean=_readonly(origin_mean),
            origin_covariance=_readonly(origin_covariance),
            goal_mean=_readonly(goal_mean),
            goal_sensing_covariance=_readonly(goal_sensing_covariance),
            goal_registration_covariance=_readonly(goal_registration_covariance),
            goal_covariance=_readonly(goal_covariance),
            action=budget.choose_action(),
            registration_radius=budget.registration_radius,
            target_radius=budget.target_radius,
            confidence_radius=budget.confidence_radius,
            yaw_confidence=budget.yaw_confidence,
        )
    except (TypeError, ValueError, OverflowError, FloatingPointError,
            np.linalg.LinAlgError):
        return _invalid_handoff_result()

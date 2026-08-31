"""Deterministic planar frame drift applied consistently to 3-D odometry."""

import math
import copy
import zlib
from dataclasses import dataclass
import heapq
import json

import numpy as np


def domain_seed(trial_seed, domain_label):
    """Derive a stable per-domain RNG seed from one shared trial seed."""
    label_digest = zlib.crc32(str(domain_label).encode("utf-8"))
    return int(
        np.random.SeedSequence([int(trial_seed), int(label_digest)])
        .generate_state(1, dtype=np.uint32)[0]
    )


def _wrap_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_rotation(yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]]
    )


def _quaternion_multiply(first, second):
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    result = np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]
    )
    norm = np.linalg.norm(result)
    if norm == 0.0:
        raise ValueError("orientation quaternion must be nonzero")
    return result / norm


def inject_pose_outlier(pose, outlier_xyyaw):
    """Apply a gross planar transform while preserving all non-planar data."""
    outlier = np.asarray(outlier_xyyaw, dtype=float)
    if outlier.shape != (3,):
        raise ValueError("outlier_xyyaw must have three values")
    rotation = _yaw_rotation(outlier[2])
    position = rotation @ np.asarray(pose["position"], dtype=float)
    position[:2] += outlier[:2]
    yaw_quaternion = np.array(
        [0.0, 0.0, math.sin(outlier[2] / 2.0), math.cos(outlier[2] / 2.0)]
    )
    return {
        "position": position,
        "orientation": _quaternion_multiply(
            yaw_quaternion, np.asarray(pose["orientation"], dtype=float)
        ),
        "covariance": np.array(pose["covariance"], dtype=float, copy=True),
    }


def adapt_position_command(command, frame_transform, destination_frame):
    """Map a command from an injected frame back into the controller frame."""
    transform = np.asarray(frame_transform, dtype=float)
    if transform.shape != (3,):
        raise ValueError("frame_transform must have three values")
    inverse_rotation = _yaw_rotation(transform[2]).T
    result = copy.deepcopy(command)
    position = np.asarray(command["position"], dtype=float).copy()
    position[:2] -= transform[:2]
    result["position"] = inverse_rotation @ position
    for field in ("velocity", "acceleration", "jerk"):
        result[field] = inverse_rotation @ np.asarray(command[field], dtype=float)
    result["yaw"] = _wrap_angle(float(command["yaw"]) - transform[2])
    result["frame_id"] = str(destination_frame)
    return result


@dataclass(frozen=True)
class ScheduledObservation:
    payload: object
    image_stamp: float
    receipt_time: float
    release_time: float
    injected_delay: float
    outlier_xyyaw: tuple
    seed: int


class ObservationGateSchedule:
    """Seeded visibility, delay, and gross-outlier queue for real observations."""

    def __init__(
        self,
        visibility_windows,
        delay_seconds,
        delay_jitter_seconds,
        outlier_probability,
        outlier_translation_m,
        outlier_yaw_rad,
        seed,
        visibility_probability=1.0,
        epoch_seconds=0.0,
    ):
        self.visibility_windows = tuple(
            (float(start), float(end)) for start, end in visibility_windows
        )
        if any(end < start for start, end in self.visibility_windows):
            raise ValueError("visibility window end must not precede start")
        self.delay_seconds = float(delay_seconds)
        self.delay_jitter_seconds = float(delay_jitter_seconds)
        self.outlier_probability = float(outlier_probability)
        self.outlier_translation_m = float(outlier_translation_m)
        self.outlier_yaw_rad = float(outlier_yaw_rad)
        self.visibility_probability = float(visibility_probability)
        if self.delay_seconds < 0.0 or self.delay_jitter_seconds < 0.0:
            raise ValueError("observation delays must be nonnegative")
        if not 0.0 <= self.outlier_probability <= 1.0:
            raise ValueError("outlier_probability must be in [0, 1]")
        if not 0.0 <= self.visibility_probability <= 1.0:
            raise ValueError("visibility_probability must be in [0, 1]")
        if self.outlier_translation_m < 0.0 or self.outlier_yaw_rad < 0.0:
            raise ValueError("outlier magnitudes must be nonnegative")
        self.seed = int(seed)
        self.epoch_seconds = float(epoch_seconds)
        if not math.isfinite(self.epoch_seconds):
            raise ValueError("epoch_seconds must be finite")
        self._rng = np.random.default_rng(self.seed)
        self._queue = []
        self._sequence = 0

    def enqueue(self, payload, image_stamp, received_at):
        image_occurrence = float(image_stamp) - self.epoch_seconds
        if not math.isfinite(image_occurrence) or image_occurrence < 0.0:
            raise ValueError(
                "image stamp {} precedes the shared trial epoch {}".format(
                    image_stamp, self.epoch_seconds
                )
            )
        now = float(received_at)
        if not any(start <= image_occurrence <= end for start, end in self.visibility_windows):
            return False
        if self._rng.random() >= self.visibility_probability:
            return False
        jitter = self._rng.uniform(
            -self.delay_jitter_seconds, self.delay_jitter_seconds
        )
        delay = max(0.0, self.delay_seconds + jitter)
        if self._rng.random() < self.outlier_probability:
            direction = self._rng.uniform(-math.pi, math.pi)
            outlier = (
                self.outlier_translation_m * math.cos(direction),
                self.outlier_translation_m * math.sin(direction),
                self._rng.choice((-self.outlier_yaw_rad, self.outlier_yaw_rad)),
            )
        else:
            outlier = (0.0, 0.0, 0.0)
        item = ScheduledObservation(
            payload=payload,
            image_stamp=float(image_stamp),
            receipt_time=now,
            release_time=now + delay,
            injected_delay=delay,
            outlier_xyyaw=outlier,
            seed=self.seed,
        )
        heapq.heappush(self._queue, (item.release_time, self._sequence, item))
        self._sequence += 1
        return True

    def release_ready(self, now):
        released = []
        while self._queue and self._queue[0][0] <= float(now):
            released.append(heapq.heappop(self._queue)[2])
        return released


def odometry_record(message):
    """Extract a ROS-like odometry message into NumPy-only records."""
    pose_message = message.pose.pose
    twist_message = message.twist.twist
    pose = {
        "position": np.array(
            [pose_message.position.x, pose_message.position.y, pose_message.position.z],
            dtype=float,
        ),
        "orientation": np.array(
            [
                pose_message.orientation.x,
                pose_message.orientation.y,
                pose_message.orientation.z,
                pose_message.orientation.w,
            ],
            dtype=float,
        ),
        "covariance": np.asarray(message.pose.covariance, dtype=float).reshape(6, 6),
    }
    twist = {
        "linear": np.array(
            [twist_message.linear.x, twist_message.linear.y, twist_message.linear.z],
            dtype=float,
        ),
        "angular": np.array(
            [twist_message.angular.x, twist_message.angular.y, twist_message.angular.z],
            dtype=float,
        ),
        "covariance": np.asarray(message.twist.covariance, dtype=float).reshape(6, 6),
    }
    return float(message.header.stamp.to_sec()), pose, twist


def _populate_vector(message, values):
    message.x, message.y, message.z = (float(value) for value in values)


def populate_odometry(
    message, pose, twist, destination_frame, twist_convention="body"
):
    """Populate odometry fields; label the twist frame honestly.

    ``parent`` twist is expressed in the new parent frame so child_frame_id is
    set to that frame; ``body`` twist stays body-expressed and the physical
    child frame is preserved.
    """
    if twist_convention not in ("parent", "body"):
        raise ValueError("twist_convention must be 'parent' or 'body'")
    message.header.frame_id = str(destination_frame)
    if twist_convention == "parent":
        message.child_frame_id = str(destination_frame)
    _populate_vector(message.pose.pose.position, pose["position"])
    orientation = pose["orientation"]
    (
        message.pose.pose.orientation.x,
        message.pose.pose.orientation.y,
        message.pose.pose.orientation.z,
        message.pose.pose.orientation.w,
    ) = (float(value) for value in orientation)
    message.pose.covariance = np.asarray(pose["covariance"]).reshape(-1).tolist()
    _populate_vector(message.twist.twist.linear, twist["linear"])
    _populate_vector(message.twist.twist.angular, twist["angular"])
    message.twist.covariance = np.asarray(twist["covariance"]).reshape(-1).tolist()
    return message


def truth_json(
    transform, seed, stamp, source_frame, destination_frame, trial_seed=None
):
    document = {
        "stamp": float(stamp),
        "source_frame": str(source_frame),
        "destination_frame": str(destination_frame),
        "transform_xyyaw": [float(value) for value in transform],
        "seed": int(seed),
    }
    if trial_seed is not None:
        document["trial_seed"] = int(trial_seed)
    return json.dumps(document, sort_keys=True)


def diagnostic_json(
    image_stamp, release_time, injected_delay, outlier_xyyaw, seed,
    receipt_time=None, actual_release=None, trial_seed=None,
):
    document = {
        "image_stamp": float(image_stamp),
        "scheduled_release": float(release_time),
        "injected_delay_seconds": float(injected_delay),
        "outlier_xyyaw": [float(value) for value in outlier_xyyaw],
        "seed": int(seed),
    }
    if receipt_time is not None:
        document["receipt_time"] = float(receipt_time)
    if actual_release is not None:
        document["actual_release"] = float(actual_release)
    if trial_seed is not None:
        document["trial_seed"] = int(trial_seed)
    return json.dumps(document, sort_keys=True)


class FramePerturbation:
    """A seeded, fixed-step random walk for an independent planar frame."""

    def __init__(
        self,
        initial_xyyaw,
        drift_rates,
        seed,
        drift_step_seconds=1.0,
        epoch_seconds=0.0,
        maximum_elapsed_seconds=None,
    ):
        initial = np.asarray(initial_xyyaw, dtype=float)
        rates = np.asarray(drift_rates, dtype=float)
        if initial.shape != (3,) or rates.shape != (3,):
            raise ValueError("initial_xyyaw and drift_rates must each have three values")
        if not np.all(np.isfinite(initial)) or not np.all(np.isfinite(rates)):
            raise ValueError("perturbation values must be finite")
        if np.any(rates < 0.0):
            raise ValueError("drift_rates must be nonnegative")
        if not math.isfinite(drift_step_seconds) or drift_step_seconds <= 0.0:
            raise ValueError("drift_step_seconds must be positive")
        if not math.isfinite(epoch_seconds):
            raise ValueError("epoch_seconds must be finite")
        if maximum_elapsed_seconds is not None and (
            not math.isfinite(maximum_elapsed_seconds)
            or maximum_elapsed_seconds <= 0.0
        ):
            raise ValueError("maximum_elapsed_seconds must be positive when set")

        self.seed = int(seed)
        self.drift_step_seconds = float(drift_step_seconds)
        self.epoch_seconds = float(epoch_seconds)
        self.maximum_elapsed_seconds = (
            None if maximum_elapsed_seconds is None
            else float(maximum_elapsed_seconds)
        )
        self._rates = rates.copy()
        self._rng = np.random.default_rng(self.seed)
        initial = initial.copy()
        initial[2] = _wrap_angle(initial[2])
        self._states = [initial]

    def _extend_to(self, step_index):
        increment_scale = self._rates * math.sqrt(self.drift_step_seconds)
        while len(self._states) <= step_index:
            next_state = self._states[-1] + self._rng.normal(
                loc=0.0, scale=increment_scale, size=3
            )
            next_state[2] = _wrap_angle(next_state[2])
            self._states.append(next_state)

    def at(self, stamp_seconds):
        """Return the piecewise-constant frame state at an absolute timestamp."""
        stamp = float(stamp_seconds)
        if not math.isfinite(stamp):
            raise ValueError("stamp_seconds must be finite")
        elapsed = stamp - self.epoch_seconds
        if elapsed < 0.0:
            raise ValueError(
                "stamp {} precedes the shared perturbation epoch {}".format(
                    stamp, self.epoch_seconds
                )
            )
        if (
            self.maximum_elapsed_seconds is not None
            and elapsed > self.maximum_elapsed_seconds
        ):
            raise ValueError(
                "elapsed time {:.3f}s exceeds maximum_elapsed_seconds {}".format(
                    elapsed, self.maximum_elapsed_seconds
                )
            )
        step_index = int(math.floor(elapsed / self.drift_step_seconds))
        self._extend_to(step_index)
        return self._states[step_index].copy()

    def transform_odom(self, pose, twist, stamp, twist_convention="parent"):
        """Transform odometry records under an explicit twist-frame ruling.

        ``parent`` rotates linear/angular twist and its covariance into the new
        parent frame (for producers that express twist in header.frame_id);
        ``body`` leaves twist untouched because a planar parent change does not
        alter body-expressed twist. Non-planar fields are kept in both modes.
        """
        if twist_convention not in ("parent", "body"):
            raise ValueError("twist_convention must be 'parent' or 'body'")
        transform = self.at(stamp)
        rotation = _yaw_rotation(transform[2])
        jacobian = np.zeros((6, 6))
        jacobian[:3, :3] = rotation
        jacobian[3:, 3:] = rotation

        position = rotation @ np.asarray(pose["position"], dtype=float)
        position[:2] += transform[:2]
        yaw_quaternion = np.array(
            [0.0, 0.0, math.sin(transform[2] / 2.0), math.cos(transform[2] / 2.0)]
        )
        transformed_pose = {
            "position": position,
            "orientation": _quaternion_multiply(
                yaw_quaternion, np.asarray(pose["orientation"], dtype=float)
            ),
            "covariance": jacobian
            @ np.asarray(pose["covariance"], dtype=float).reshape(6, 6)
            @ jacobian.T,
        }
        if twist_convention == "parent":
            transformed_twist = {
                "linear": rotation @ np.asarray(twist["linear"], dtype=float),
                "angular": rotation @ np.asarray(twist["angular"], dtype=float),
                "covariance": jacobian
                @ np.asarray(twist["covariance"], dtype=float).reshape(6, 6)
                @ jacobian.T,
            }
        else:
            transformed_twist = {
                "linear": np.array(twist["linear"], dtype=float, copy=True),
                "angular": np.array(twist["angular"], dtype=float, copy=True),
                "covariance": np.array(
                    twist["covariance"], dtype=float, copy=True
                ).reshape(6, 6),
            }
        return transformed_pose, transformed_twist, transform

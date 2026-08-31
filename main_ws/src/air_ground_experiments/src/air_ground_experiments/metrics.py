"""Pure one-shot registration trial metrics and stable result schema."""

import math
from collections import OrderedDict, deque
import csv
import json
import os
from pathlib import Path
import threading

import numpy as np

import numpy as np


TRIAL_COLUMNS = (
    "trial_id",
    "seed",
    "status",
    "success",
    "failure_code",
    "timeout",
    "yaw_error_rad",
    "handoff_error_m",
    "final_inspection_distance_m",
    "success_radius_m",
    "duration_seconds",
)


def wrapped_yaw_error(estimated_yaw, truth_yaw):
    """Return signed estimate-minus-truth error in [-pi, pi)."""
    return (float(estimated_yaw) - float(truth_yaw) + math.pi) % (
        2.0 * math.pi
    ) - math.pi


def handoff_error_2d(estimated_position, truth_position):
    """Return planar handoff error while ignoring non-planar coordinates."""
    estimated = np.asarray(estimated_position, dtype=float)
    truth = np.asarray(truth_position, dtype=float)
    if estimated.size < 2 or truth.size < 2:
        raise ValueError("handoff positions must contain x and y")
    return float(np.linalg.norm(estimated[:2] - truth[:2]))


def final_inspection_distance(position, inspection_target):
    """Return full 3-D distance from the final pose to the inspection target."""
    position = np.asarray(position, dtype=float)
    target = np.asarray(inspection_target, dtype=float)
    if position.shape != (3,) or target.shape != (3,):
        raise ValueError("inspection positions must each contain x, y, and z")
    return float(np.linalg.norm(position - target))


def classify_success(distance, success_radius):
    """Inclusive success-radius check over a finite distance."""
    radius = float(success_radius)
    value = float(distance)
    if not math.isfinite(radius) or radius < 0.0:
        raise ValueError("success_radius must be finite and nonnegative")
    return math.isfinite(value) and value <= radius


MISSION_SUCCESS_PHASE = "INSPECTION_CONFIRMED"
MISSION_ERROR_PREFIX = "ERROR_"


def classify_mission_phase(phase):
    """Map a mission phase to (outcome, failure_code) with stable codes."""
    text = str(phase).upper().strip()
    if text == MISSION_SUCCESS_PHASE:
        return "SUCCESS", ""
    if text.startswith(MISSION_ERROR_PREFIX):
        return "FAILED", "MISSION_" + text[len(MISSION_ERROR_PREFIX):]
    return "PENDING", ""


def _se2_matrix(xyyaw):
    x, y, yaw = (float(value) for value in xyyaw)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.array(
        [[cosine, -sine, x], [sine, cosine, y], [0.0, 0.0, 1.0]]
    )


def _se2_inverse(matrix):
    rotation = matrix[:2, :2]
    translation = matrix[:2, 2]
    result = np.eye(3)
    result[:2, :2] = rotation.T
    result[:2, 2] = -(rotation.T @ translation)
    return result


def _se2_xyyaw(matrix):
    return np.array(
        [matrix[0, 2], matrix[1, 2],
         math.atan2(matrix[1, 0], matrix[0, 0])]
    )


class TrialTruthEvaluator:
    """Ground-truth ``^O T_G`` on the estimator's exact frame chain.

    Frame definitions (all planar SE(2), matrices act as p' = R p + t):

    - ``F_uav(t)``/``F_ugv(t)``: injected source->experiment transforms from
      the truth stream, so ``p_experiment = F(t) p_source``.
    - ``Delta``: true constant relation between the two SOURCE odometry
      frames, measured evaluation-side from Gazebo world poses at rest.
    - ``A``: takeoff anchor in the experiment UAV stream computed with the
      estimator's own rule (mean position, circular mean yaw, origin_yaw=-yaw).

    Truth registration at the estimate stamp is
    ``A @ F_uav(t) @ Delta @ F_ugv(t)^-1``, i.e. the UGV experiment-odometry
    origin expressed in the takeoff-anchored frame — exactly what the
    published ``^O T_G`` estimate denotes.
    """

    def __init__(
        self,
        minimum_anchor_samples=30,
        align_origin_to_uav_heading=True,
        fixed_origin_yaw=0.0,
    ):
        self.minimum_anchor_samples = int(minimum_anchor_samples)
        self.align_origin_to_uav_heading = bool(align_origin_to_uav_heading)
        self.fixed_origin_yaw = float(fixed_origin_yaw)
        self._anchor_samples = deque(maxlen=max(2, self.minimum_anchor_samples))
        self.anchor = None
        self.source_relation = None
        self._truth_history = {
            "uav": [],
            "ugv": [],
        }

    def record_anchor_sample(self, position_yaw):
        position_yaw = np.asarray(position_yaw, dtype=float)
        if position_yaw.shape != (3,) or not np.all(np.isfinite(position_yaw)):
            raise ValueError("anchor samples must be finite [x, y, yaw]")
        if self.anchor is not None:
            return
        self._anchor_samples.append(position_yaw)
        if len(self._anchor_samples) >= self.minimum_anchor_samples:
            samples = np.array(self._anchor_samples)
            center = np.mean(samples[:, :2], axis=0)
            mean_sin = float(np.mean(np.sin(samples[:, 2])))
            mean_cos = float(np.mean(np.cos(samples[:, 2])))
            mean_yaw = math.atan2(mean_sin, mean_cos)
            # Mirror takeoff_registration's configurable rule:
            # origin_yaw = -mean_yaw when aligning to UAV heading, otherwise
            # the configured fixed origin yaw.
            if self.align_origin_to_uav_heading:
                origin_yaw = -mean_yaw
            else:
                origin_yaw = self.fixed_origin_yaw
            anchor_rotation = _se2_matrix([0.0, 0.0, origin_yaw])
            translation = -(anchor_rotation[:2, :2] @ center)
            self.anchor = _se2_matrix(
                [translation[0], translation[1], origin_yaw]
            )

    def record_source_relation(self, uav_world_pose, ugv_world_pose):
        """Freeze Delta once from a resting Gazebo world-pose snapshot."""
        if self.source_relation is not None:
            return
        uav = _se2_matrix(uav_world_pose)
        ugv = _se2_matrix(ugv_world_pose)
        self.source_relation = _se2_inverse(uav) @ ugv

    def record_truth(self, domain, document):
        transform = np.asarray(
            json.loads(json.dumps(dict(document)))["transform_xyyaw"], dtype=float
        )
        stamp = float(document["stamp"])
        history = self._truth_history[domain]
        if history and stamp < history[-1][0]:
            return
        # Full per-trial retention: the freeze stamp must stay interpolable
        # until finalization, which can be minutes after registration. At
        # typical truth rates a multi-minute trial holds well under ~10^5
        # small entries (a few MB), so no eviction or decimation is applied.
        history.append((stamp, transform))

    @staticmethod
    def _interpolate(history, stamp):
        if not history or stamp < history[0][0] or stamp > history[-1][0]:
            return None
        for index in range(len(history) - 1):
            start_stamp, start_value = history[index]
            end_stamp, end_value = history[index + 1]
            if start_stamp <= stamp <= end_stamp:
                if end_stamp == start_stamp:
                    return start_value.copy()
                alpha = (stamp - start_stamp) / (end_stamp - start_stamp)
                interpolated = start_value + alpha * (end_value - start_value)
                yaw_delta = wrapped_yaw_error(end_value[2], start_value[2])
                interpolated[2] = start_value[2] + alpha * yaw_delta
                return interpolated
        return None

    def registration_truth_at(self, stamp):
        """Return ground-truth ^O T_G at a stamp, or None when unsynchronized."""
        if self.anchor is None or self.source_relation is None:
            return None
        f_uav = self._interpolate(self._truth_history["uav"], stamp)
        f_ugv = self._interpolate(self._truth_history["ugv"], stamp)
        if f_uav is None or f_ugv is None:
            return None
        chain = self.anchor @ _se2_matrix(f_uav) @ self.source_relation
        chain = chain @ _se2_inverse(_se2_matrix(f_ugv))
        return _se2_xyyaw(chain)


SUCCESS_STATUS = "COMPLETED"
FAILURE_STATUSES = ("FAILED", "TIMEOUT")
ALLOWED_STATUSES = (SUCCESS_STATUS,) + FAILURE_STATUSES


def build_trial_row(
    trial_id,
    seed,
    status,
    failure_code="",
    yaw_error_rad=float("nan"),
    handoff_error_m=float("nan"),
    final_inspection_distance_m=float("nan"),
    success_radius_m=0.5,
    duration_seconds=0.0,
):
    """Build one schema-complete row under canonical status semantics."""
    normalized_status = str(status).upper().strip()
    if normalized_status not in ALLOWED_STATUSES:
        raise ValueError(
            "status must be one of {}, got {!r}".format(
                list(ALLOWED_STATUSES), status
            )
        )
    failure_code = str(failure_code)
    distance = float(final_inspection_distance_m)
    if normalized_status in FAILURE_STATUSES and not failure_code:
        raise ValueError("failed and timeout trials require a failure_code")
    if normalized_status == SUCCESS_STATUS:
        if failure_code:
            raise ValueError("successful trials must not carry a failure_code")
        if not math.isfinite(distance):
            raise ValueError("completed trials require a finite inspection distance")
    success = normalized_status == SUCCESS_STATUS and classify_success(
        distance, success_radius_m,
    )
    values = (
        str(trial_id),
        int(seed),
        normalized_status,
        success,
        failure_code,
        normalized_status == "TIMEOUT",
        float(yaw_error_rad),
        float(handoff_error_m),
        distance,
        float(success_radius_m),
        float(duration_seconds),
    )
    return OrderedDict(zip(TRIAL_COLUMNS, values))


class TrialResultWriter:
    """Persist one CSV row and one JSON metadata document per trial, once."""

    def __init__(self, output_directory):
        self.output_directory = Path(output_directory)
        self.output_directory.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.output_directory / "trials.csv"
        self._lock = threading.Lock()
        self._completed_ids = set()

    @staticmethod
    def _json_value(value):
        if isinstance(value, float) and not math.isfinite(value):
            return None
        return value

    @staticmethod
    def _safe_trial_id(trial_id):
        return "".join(
            character if character.isalnum() or character in "-_." else "_"
            for character in str(trial_id)
        )

    def write(self, row, metadata=None):
        if tuple(row.keys()) != TRIAL_COLUMNS:
            raise ValueError("trial row does not match TRIAL_COLUMNS")
        trial_id = str(row["trial_id"])
        safe_id = self._safe_trial_id(trial_id)
        json_path = self.output_directory / "{}.json".format(safe_id)
        document = {
            "result": {key: self._json_value(value) for key, value in row.items()},
            "metadata": dict(metadata or {}),
        }
        with self._lock:
            if trial_id in self._completed_ids or json_path.exists():
                raise ValueError(
                    "trial {!r} has already been finalized".format(trial_id)
                )
            temporary_path = json_path.with_suffix(".json.partial")
            appended_bytes = 0
            try:
                with temporary_path.open("w") as stream:
                    json.dump(document, stream, indent=2, sort_keys=True,
                              allow_nan=False)
                    stream.write("\n")
                write_header = not self.csv_path.exists()
                with self.csv_path.open("a", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=TRIAL_COLUMNS)
                    if write_header:
                        writer.writeheader()
                    before_append = stream.tell()
                    writer.writerow(row)
                    stream.flush()
                    appended_bytes = stream.tell() - before_append
                os.replace(temporary_path, json_path)
            except Exception:
                if appended_bytes and self.csv_path.exists():
                    with self.csv_path.open("r+b") as stream:
                        stream.truncate(stream.seek(0, os.SEEK_END) - appended_bytes)
                if temporary_path.exists():
                    temporary_path.unlink()
                raise
            self._completed_ids.add(trial_id)
        return self.csv_path, json_path

"""Pure parsing and grading for the Dynamic M2-C A/B/C/D validation.

Everything in this module is ROS-free so the runner script stays importable
and unit-testable without a ROS environment. `rostopic echo -p` CSV layout is
the single input contract.
"""

from dataclasses import dataclass, field
import math


PASS = "PASS"
FAIL = "FAIL"
INCONCLUSIVE = "INCONCLUSIVE"

NIS_THRESHOLD = 11.344866730144373
MIN_INLIERS = 20
DEFAULT_INSPECTION_RADIUS = 0.35
DEFAULT_INSPECTION_YAW = 0.03490658503988659

REREGISTER_ONLY_PHASES = ("RETURN_TO_UGV", "WAIT_REREGISTRATION", "RESUME_HANDOFF")


@dataclass
class Verdict:
    scenario: str
    status: str
    reasons: tuple = ()
    evidence: dict = field(default_factory=dict)

    def with_status(self, status, extra_reasons=()):
        self.status = status
        self.reasons = tuple(self.reasons) + tuple(extra_reasons)
        return self


def parse_echo_csv(text):
    """Parse one `rostopic echo -p` document into dicts keyed by header name.

    The `%time` column is normalized to seconds regardless of whether the
    publisher emitted seconds or nanoseconds, so cross-topic comparisons in
    the graders share one unit.
    """
    lines = [line for line in text.splitlines() if line.strip()]
    if len(lines) < 2:
        return []
    header = lines[0].lstrip("%").split(",")
    rows = []
    for line in lines[1:]:
        cells = line.split(",")
        if len(cells) != len(header):
            continue
        rows.append(dict(zip(header, cells)))

    stamps = []
    for row in rows:
        try:
            stamps.append(float(row["time"]))
        except (KeyError, TypeError, ValueError):
            continue
    nanosecond_times = any(abs(stamp) > 1.0e6 for stamp in stamps)
    for row in rows:
        try:
            stamp = float(row["time"])
        except (KeyError, TypeError, ValueError):
            continue
        if nanosecond_times:
            stamp /= 1.0e9
        row["time"] = repr(stamp)
    return rows


def _f(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def phase_events(rows):
    return [(_f(row, "time"), row.get("field.data", "")) for row in rows]


def action_events(rows):
    return [(_f(row, "time"), row.get("field.data", "")) for row in rows]


def scalar_events(rows):
    return [(_f(row, "time"), _f(row, "field.data")) for row in rows]


def accepted_events(rows):
    """Return (time, revision, cov_xx, cov_yy, cov_yaw) per accepted update."""
    events = []
    for row in rows:
        events.append((
            _f(row, "time"),
            int(_f(row, "field.revision")),
            _f(row, "field.pose.covariance0"),
            _f(row, "field.pose.covariance7"),
            _f(row, "field.pose.covariance35"),
        ))
    return events


def estimate_events(rows):
    """Return (time, cov_xx, cov_yy, cov_yaw) per continuous estimate."""
    events = []
    for row in rows:
        events.append((
            _f(row, "time"),
            _f(row, "field.pose.covariance0"),
            _f(row, "field.pose.covariance7"),
            _f(row, "field.pose.covariance35"),
        ))
    return events


def observation_times(rows):
    return [_f(row, "time") for row in rows]


def has_phase(phases, name):
    return any(value == name for _, value in phases)


def first_phase(phases, name):
    for stamp, value in phases:
        if value == name:
            return stamp
    return None


def _nearest_estimate_at_or_before(estimates, stamp):
    best = None
    for event in estimates:
        if event[0] <= stamp and (best is None or event[0] > best[0]):
            best = event
    return best


def _estimate_near(estimates, stamp, window):
    chosen = None
    for event in estimates:
        if abs(event[0] - stamp) <= window and (
            chosen is None or abs(event[0] - stamp) < abs(chosen[0] - stamp)
        ):
            chosen = event
    return chosen


def _trace(event):
    if event is None:
        return float("nan")
    return event[1] + event[2] + event[3]


def _trace_around(estimates, stamp, window=6.0):
    """Return (max-trace event before stamp, min-trace event after stamp).

    Receipt-time jitter between topics makes nearest-row comparisons fragile;
    window extremes are robust: a fresh accepted registration can only reduce
    the estimate trace, so the pre-window maximum captures the grown value and
    the post-window minimum captures the reset value regardless of skew.
    """
    before = None
    after = None
    for event in estimates:
        if not math.isfinite(event[0]):
            continue
        delta = event[0] - stamp
        if delta <= 0.0 and delta >= -window:
            if before is None or _trace(event) > _trace(before):
                before = event
        elif delta > 0.0 and delta <= window:
            if after is None or _trace(event) < _trace(after):
                after = event
    return before, after


def _yaw95(event):
    if event is None:
        return float("nan")
    return 1.959964 * math.sqrt(max(0.0, event[3]))


def grade_a(
    phases,
    actions,
    confidence,
    inliers,
    accepted,
    estimates,
    goals,
    budget_radius,
    budget_yaw,
):
    verdict = Verdict(scenario="A", status=FAIL)

    revision_one = next(
        (event for event in accepted if event[1] >= 1), None
    )
    verdict.evidence["revision1"] = int(revision_one[1]) if revision_one else 0
    if revision_one is None:
        return verdict.with_status(FAIL, ("no initial accepted registration",))

    inlier_at = max(
        (value for stamp, value in inliers if stamp >= revision_one[0] - 1.0),
        default=None,
    )
    verdict.evidence["inliers"] = inlier_at
    if inlier_at is None or inlier_at < MIN_INLIERS:
        verdict.reasons += ("inlier_count below %d" % MIN_INLIERS,)

    direct = [event for event in actions if event[1] == "DIRECT"]
    reregister = [event for event in actions if event[1] == "REREGISTER"]
    verdict.evidence["DIRECT"] = int(bool(direct))
    verdict.evidence["REREGISTER"] = int(bool(reregister))

    if confidence:
        verdict.evidence["confidence"] = confidence[0][1]
    if reregister and not direct:
        grown = _nearest_estimate_at_or_before(estimates, reregister[0][0])
        verdict.evidence["sigma_xy"] = (
            round(math.sqrt(max(grown[1], grown[2])), 4) if grown else None
        )
        verdict.evidence["sigma_yaw_deg"] = (
            round(math.degrees(math.sqrt(max(0.0, grown[3]))), 4) if grown else None
        )
        verdict.status = INCONCLUSIVE
        verdict.reasons += (
            "REREGISTER emitted: input uncertainty exceeded the scenario "
            "budget (INCONCLUSIVE_A_INPUT)",
        )
        return verdict
    if not direct:
        verdict.reasons += ("no DIRECT action observed",)
        return verdict
    if not confidence:
        verdict.evidence["confidence"] = None
        return verdict.with_status(FAIL, ("confidence radius not published",))

    forbidden = [
        value for _, value in phases if value in REREGISTER_ONLY_PHASES
    ]
    verdict.evidence["FORBIDDEN_PHASES"] = ",".join(sorted(set(forbidden)))
    if forbidden:
        verdict.reasons += ("reregistration phases appeared in DIRECT run",)

    dispatch = first_phase(phases, "DISPATCH")
    overwatch = first_phase(phases, "OVERWATCH")
    goal = min((stamp for stamp, _ in goals), default=None)
    verdict.evidence["DISPATCH"] = int(dispatch is not None)
    verdict.evidence["OVERWATCH"] = int(overwatch is not None)
    verdict.evidence["GOAL"] = int(goal is not None)
    if dispatch is None:
        verdict.reasons += ("DISPATCH phase missing",)
    if overwatch is None:
        verdict.reasons += ("OVERWATCH phase missing",)
    if goal is None:
        verdict.reasons += ("no UGV goal published",)
    if dispatch is not None and goal is not None and goal < dispatch:
        verdict.reasons += ("goal published before DISPATCH",)

    conf_time, conf_value = confidence[0]
    verdict.evidence["confidence"] = conf_value
    verdict.evidence["budget_radius"] = budget_radius
    verdict.evidence["budget_yaw"] = budget_yaw
    conf_estimate = _nearest_estimate_at_or_before(estimates, conf_time)
    verdict.evidence["sigma_xy"] = (
        round(math.sqrt(max(conf_estimate[1], conf_estimate[2])), 4)
        if conf_estimate else None
    )
    verdict.evidence["sigma_yaw_deg"] = (
        round(math.degrees(math.sqrt(max(0.0, conf_estimate[3]))), 4)
        if conf_estimate else None
    )
    if not math.isfinite(conf_value):
        verdict.reasons += ("confidence radius not published",)
    elif conf_value > budget_radius:
        verdict.reasons += (
            "confidence %.4f exceeds budget %.4f" % (conf_value, budget_radius),
        )
    if conf_estimate is not None:
        yaw95 = _yaw95(conf_estimate)
        verdict.evidence["yaw95"] = round(yaw95, 4)
        if yaw95 > budget_yaw:
            verdict.reasons += (
                "yaw95 %.4f exceeds budget %.4f" % (yaw95, budget_yaw),
            )

    return verdict.with_status(PASS if not verdict.reasons else FAIL)


def grade_b(
    phases,
    actions,
    accepted,
    revision_values,
    estimates,
    goals,
    observation_dest_times,
    hide_time,
):
    verdict = Verdict(scenario="B", status=FAIL)

    revision_one = next(
        (event for event in accepted if event[1] >= 1), None
    )
    verdict.evidence["revision1"] = int(revision_one[1]) if revision_one else 0
    if revision_one is None:
        return verdict.with_status(FAIL, ("no initial accepted registration",))

    max_revision = max(
        (int(value) for _, value in revision_values if math.isfinite(value)),
        default=0,
    )
    newer = [event for event in accepted if event[1] > 1]
    verdict.evidence["max_revision"] = max_revision
    verdict.evidence["accepted_newer"] = len(newer)
    if max_revision != 1 or newer:
        verdict.reasons += ("revision advanced without clean observation",)

    if hide_time is None:
        verdict.reasons += ("hide control never issued",)
    else:
        window = [
            stamp for stamp in observation_dest_times if stamp > hide_time + 3.0
        ]
        verdict.evidence["obs_after_hide"] = len(window)
        if window:
            verdict.reasons += (
                "observations still flowing %.1fs after hide" % (window[0] - hide_time),
            )

    reregister = [event for event in actions if event[1] == "REREGISTER"]
    verdict.evidence["REREGISTER"] = int(bool(reregister))
    if not reregister:
        verdict.reasons += ("REREGISTER action never emitted",)
        return verdict.with_status(FAIL)

    action_time = reregister[0][0]
    return_time = first_phase(phases, "RETURN_TO_UGV")
    wait_time = first_phase(phases, "WAIT_REREGISTRATION")
    verdict.evidence["RETURN_TO_UGV"] = int(return_time is not None)
    verdict.evidence["WAIT_REREGISTRATION"] = int(wait_time is not None)
    if return_time is None or wait_time is None:
        verdict.reasons += ("return/wait phases missing",)
    elif not (action_time - 1.0 <= return_time <= wait_time):
        verdict.reasons += ("return/wait phase ordering violated",)

    baseline = _estimate_near(estimates, revision_one[0], 3.0)
    grown = _nearest_estimate_at_or_before(estimates, action_time)
    verdict.evidence["cov_before"] = round(_trace(baseline), 6)
    verdict.evidence["cov_after"] = round(_trace(grown), 6)
    if not (_trace(grown) > _trace(baseline)):
        verdict.reasons += ("process covariance did not grow before REREGISTER",)

    after = [
        value for stamp, value in phases
        if stamp > action_time and value in ("RESUME_HANDOFF", "DISPATCH", "OVERWATCH")
    ]
    late_goals = [stamp for stamp, _ in goals if stamp > revision_one[0]]
    verdict.evidence["DISPATCH"] = int(bool(after))
    verdict.evidence["GOAL"] = int(bool(late_goals))
    if after:
        verdict.reasons += ("dispatch-like phase after REREGISTER: %s" % after[0],)
    if late_goals:
        verdict.reasons += ("UGV goal published despite unresolved registration",)

    return verdict.with_status(PASS if not verdict.reasons else FAIL)


def grade_c(
    phases,
    actions,
    accepted,
    revision_values,
    estimates,
    goals,
):
    verdict = Verdict(scenario="C", status=FAIL)

    revision_one = next(
        (event for event in accepted if event[1] >= 1), None
    )
    verdict.evidence["revision_before"] = int(revision_one[1]) if revision_one else 0
    if revision_one is None:
        return verdict.with_status(FAIL, ("no initial accepted registration",))

    revision_two = [event for event in accepted if event[1] == 2]
    newer = [event for event in accepted if event[1] > 1]
    verdict.evidence["revision_after"] = (
        max((event[1] for event in accepted), default=0)
    )
    verdict.evidence["revision2_count"] = len(revision_two)
    if not revision_two or len(newer) != 1:
        verdict.reasons += (
            "expected exactly one registration event producing revision 2",
        )
        return verdict.with_status(FAIL)
    revision_two = revision_two[0]

    reregister = [event for event in actions if event[1] == "REREGISTER"]
    verdict.evidence["REREGISTER"] = int(bool(reregister))
    if not reregister:
        verdict.reasons += ("REREGISTER action never emitted",)

    return_time = first_phase(phases, "RETURN_TO_UGV")
    wait_time = first_phase(phases, "WAIT_REREGISTRATION")
    resume_time = first_phase(phases, "RESUME_HANDOFF")
    dispatch_time = first_phase(phases, "DISPATCH")
    overwatch_time = first_phase(phases, "OVERWATCH")
    goal = min((stamp for stamp, _ in goals), default=None)

    verdict.evidence["RETURN_TO_UGV"] = int(return_time is not None)
    verdict.evidence["WAIT_REREGISTRATION"] = int(wait_time is not None)
    verdict.evidence["RESUME_HANDOFF"] = int(resume_time is not None)
    verdict.evidence["DISPATCH"] = int(dispatch_time is not None)
    verdict.evidence["OVERWATCH"] = int(overwatch_time is not None)
    verdict.evidence["GOAL"] = int(goal is not None)

    chain = (return_time, wait_time, resume_time, dispatch_time)
    if any(stage is None for stage in chain):
        verdict.reasons += ("reregistration phase chain incomplete",)
    else:
        if not chain[0] <= chain[1]:
            verdict.reasons += ("return/wait ordering violated",)
        if not revision_two[0] <= chain[2]:
            verdict.reasons += ("RESUME_HANDOFF before revision 2",)
        if not chain[2] <= chain[3]:
            verdict.reasons += ("resume/dispatch ordering violated",)
    if overwatch_time is None:
        verdict.reasons += ("OVERWATCH phase missing",)
    if goal is None:
        verdict.reasons += ("no UGV goal published",)
    if dispatch_time is not None and goal is not None and goal < dispatch_time:
        verdict.reasons += ("goal published before DISPATCH",)

    before, after = _trace_around(estimates, revision_two[0])
    verdict.evidence["cov_before"] = round(_trace(before), 6)
    verdict.evidence["cov_after"] = round(_trace(after), 6)
    if not (_trace(before) > 3.0 * max(_trace(after), 1.0e-6)):
        verdict.reasons += ("estimate trace did not drop across revision 2",)
    if before is not None and after is not None:
        verdict.evidence["sigma_xy_before"] = round(math.sqrt(max(before[1], before[2])), 4)
        verdict.evidence["sigma_xy_after"] = round(math.sqrt(max(after[1], after[2])), 4)
        verdict.evidence["sigma_yaw_before"] = round(
            math.degrees(math.sqrt(max(0.0, before[3]))), 4
        )
        verdict.evidence["sigma_yaw_after"] = round(
            math.degrees(math.sqrt(max(0.0, after[3]))), 4
        )
        if after[1] > before[1] or after[2] > before[2]:
            verdict.reasons += ("translation sigma did not drop across revision 2",)
        if after[3] > before[3]:
            verdict.reasons += ("yaw sigma did not drop across revision 2",)

    return verdict.with_status(PASS if not verdict.reasons else FAIL)


def grade_d(
    phases,
    actions,
    accepted,
    revision_values,
    estimates,
    goals,
    statuses,
    innovations,
    observation_dest_times,
    hide_time,
    outlier_time,
):
    verdict = Verdict(scenario="D", status=FAIL)

    revision_one = next(
        (event for event in accepted if event[1] >= 1), None
    )
    verdict.evidence["revision_before"] = int(revision_one[1]) if revision_one else 0
    if revision_one is None:
        return verdict.with_status(FAIL, ("no initial accepted registration",))
    if hide_time is None:
        verdict.reasons += ("hide control never issued",)
        return verdict.with_status(FAIL)
    pre_outlier_window = [
        stamp for stamp in observation_dest_times
        if hide_time + 3.0 < stamp <= (outlier_time if outlier_time is not None else hide_time + 3.0)
    ]
    verdict.evidence["obs_after_hide"] = len(pre_outlier_window)
    if outlier_time is None:
        verdict.reasons += ("outlier control never issued",)
        return verdict.with_status(FAIL)

    max_revision = max(
        (int(value) for _, value in revision_values if math.isfinite(value)),
        default=0,
    )
    newer = [event for event in accepted if event[1] > 1]
    verdict.evidence["revision_after"] = max_revision
    if max_revision != 1 or newer:
        verdict.reasons += ("revision advanced despite outlier injection",)

    rejected_statuses = [
        (stamp, value) for stamp, value in statuses
        if value == "REJECTED" and stamp > outlier_time
    ]
    nis_values = [
        value for stamp, value in innovations
        if stamp > outlier_time and math.isfinite(value)
    ]
    peak_nis = max(nis_values, default=float("nan"))
    verdict.evidence["REJECTED"] = len(rejected_statuses)
    verdict.evidence["NIS"] = (
        round(peak_nis, 4) if math.isfinite(peak_nis) else None
    )
    if not rejected_statuses and not any(
        value > NIS_THRESHOLD for value in nis_values
    ):
        verdict.reasons += ("no REJECTED status and no NIS above threshold",)
    if nis_values and all(value <= NIS_THRESHOLD for value in nis_values):
        verdict.reasons += ("innovation stayed below the NIS gate",)

    resume_time = first_phase(phases, "RESUME_HANDOFF")
    dispatch_time = first_phase(phases, "DISPATCH")
    verdict.evidence["RESUME_HANDOFF"] = int(resume_time is not None)
    verdict.evidence["DISPATCH"] = int(dispatch_time is not None)
    verdict.evidence["GOAL"] = int(bool(goals))
    if resume_time is not None:
        verdict.reasons += ("RESUME_HANDOFF must not occur after outlier",)
    if dispatch_time is not None:
        verdict.reasons += ("DISPATCH must not occur after outlier",)
    if goals:
        verdict.reasons += ("UGV goal must not be published in outlier scenario",)

    before, after_event = _trace_around(estimates, outlier_time)
    verdict.evidence["cov_before"] = round(_trace(before), 6)
    verdict.evidence["cov_min_after"] = (
        round(_trace(after_event), 6) if after_event is not None else None
    )
    if after_event is not None and before is not None:
        if _trace(after_event) < _trace(before) - 1e-6:
            verdict.reasons += ("covariance falsely improved after outlier",)

    wait_time = first_phase(phases, "WAIT_REREGISTRATION")
    verdict.evidence["WAIT_REREGISTRATION"] = int(wait_time is not None)
    if wait_time is None:
        verdict.reasons += ("mission never reached WAIT_REREGISTRATION",)

    return verdict.with_status(PASS if not verdict.reasons else FAIL)

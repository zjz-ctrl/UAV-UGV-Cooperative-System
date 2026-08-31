# Task 1 Brief: Pure Target And Goal Covariance

Read this first. It is the complete requirement for this task.

## Context

Task 8 already provides `UncertaintyBudget` in `src/air_ground_bringup/src/air_ground_bringup/target_handoff.py`. Task 1 extends that pure ROS-free module with target sample covariance, standoff geometry, registration lever-arm propagation, and a split-covariance handoff result. Later tasks will consume these exact pure interfaces.

## Files

- Modify `src/air_ground_bringup/src/air_ground_bringup/target_handoff.py`.
- Modify `src/air_ground_bringup/test/test_target_handoff.py`.
- Do not edit any other production or test file.

## Required Interfaces

```python
sample_target_covariance(
    samples_xy,
    variance_floor,
    pose_covariances=None,
    range_axes=None,
    range_variance=0.0,
) -> numpy.ndarray | None

standoff_goal(target_xy, anchor_xy, standoff)
    -> (mean_xyyaw, target_jacobian) | None

registration_execution_covariance(
    target_in_registration_frame,
    registration_covariance,
) -> numpy.ndarray | None

evaluate_handoff(...) -> HandoffResult
```

`HandoffResult` must expose the transformed origin mean/covariance, standoff goal mean, sensing-only goal covariance, registration-only goal covariance, final goal covariance, Task 8 action, and confidence values needed by the future ROS adapter. Use a dataclass or equally explicit immutable value object.

## Required Math

Sample covariance:

```text
P_sample = unbiased covariance(samples_xy), ddof=1
P_sensing = P_sample + variance_floor^2 I
P_target = P_sensing + mean(per-sample projected UAV pose covariance)
                         + configured front range-axis term
```

Range term per valid unit axis:

```python
range_variance * np.outer(unit_range_axis, unit_range_axis)
```

Standoff for `r = target - anchor`, `q = ||r||`, and `u = r/q`:

```text
goal_xy = target_xy - standoff * u
goal_yaw = atan2(r_y, r_x)
J_xy = I - (standoff / q) * (I - u u^T)
J_yaw = [-r_y / q^2, r_x / q^2]
```

Covariance split:

```text
P_origin = R_origin_uav P_target_uav R_origin_uav^T
P_goal_sensing = J_target P_origin J_target^T
P_goal_registration = J_registration P_registration J_registration^T
P_goal = P_goal_sensing + P_goal_registration
```

At a 15 m lever arm with one-degree registration yaw sigma, registration-only lateral variance must include approximately `(15 * radians(1))^2`. Include translation/yaw cross-covariances. `P_origin` must not change when only registration covariance changes. The registration contribution must appear exactly once in `P_goal`.

Pass `P_goal_registration` and `P_goal_sensing[:2,:2]` separately to `UncertaintyBudget`; never pass the combined covariance in both slots.

## Validation

Return `None` or an invalid `HandoffResult` that selects `HOLD` for malformed input. Cover fewer than two samples, nonfinite values, invalid floor/range variance, degenerate range axes, invalid shapes, asymmetric/non-PSD covariance, nonpositive standoff, and target-anchor distance too small for stable linearization.

Preserve all existing Task 8 behavior and action strings. Keep `/air_ground/handoff/confidence_radius` out of this pure task; no ROS code belongs here.

## TDD Steps

1. Add `TargetSampleCovarianceTest`, run it, and record a genuine RED caused by missing behavior.
2. Implement the smallest sample covariance code and run the focused test GREEN.
3. Add `HandoffCovarianceTest`, including one-time 15 m/1 degree lever-arm assertions, run RED.
4. Implement standoff and split propagation, run focused GREEN.
5. Run the entire `test_target_handoff` module and preserve all Task 8 tests.
6. Self-review for double counting, finite/PSD validation, and unrelated changes.

## Commands

```bash
python3 -m unittest \
  src.air_ground_bringup.test.test_target_handoff.TargetSampleCovarianceTest -v
python3 -m unittest \
  src.air_ground_bringup.test.test_target_handoff.HandoffCovarianceTest -v
python3 -m unittest src.air_ground_bringup.test.test_target_handoff -v
```

## Project Rulings

- The registration transform translation is the Task 9 standoff anchor.
- Target sample scatter and mean projected UAV pose covariance are added once; front range variance is one extra sensing-axis term.
- Registration covariance is excluded from target/origin covariance and added once only to executable goal covariance.
- This workspace has no Git metadata. Do not initialize Git or attempt commits.
- Do not implement Task 2 or any ROS adapter in this task.

## Report

Write a complete report to `.superpowers/sdd/2026-08-29-task9-common-frame-handoff/task-1-report.md` containing:

- status (`DONE`, `DONE_WITH_CONCERNS`, `NEEDS_CONTEXT`, or `BLOCKED`),
- files changed,
- RED command and observed failure,
- GREEN/full-suite commands and exact counts,
- formulas and interface signatures implemented,
- self-review findings and concerns.

Return only the status, one-line test summary, and concerns. Do not dispatch subagents.

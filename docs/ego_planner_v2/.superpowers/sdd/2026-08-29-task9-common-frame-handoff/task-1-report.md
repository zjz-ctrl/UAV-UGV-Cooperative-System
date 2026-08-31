# Task 1 Report: Pure Target And Goal Covariance

## Status

`DONE`

## Files Changed

- `src/air_ground_bringup/src/air_ground_bringup/target_handoff.py`
  - Added pure sample covariance, standoff geometry, registration lever-arm propagation, immutable handoff results, and split handoff evaluation.
  - Preserved the Task 8 `UncertaintyBudget` behavior and action strings.
- `src/air_ground_bringup/test/test_target_handoff.py`
  - Added `TargetSampleCovarianceTest` and `HandoffCovarianceTest` coverage.
  - Preserved all existing Task 8 policy tests.
- `.superpowers/sdd/2026-08-29-task9-common-frame-handoff/task-1-report.md`
  - Added this required task report.

No Git metadata was created and no commit was attempted.

## TDD Evidence

### Sample Covariance RED

Command:

```bash
python3 -m unittest \
  src.air_ground_bringup.test.test_target_handoff.TargetSampleCovarianceTest -v
```

Observed genuine RED after correcting the test harness placement:

```text
Ran 5 tests in 0.002s
FAILED (failures=18)
```

Each failure named the absent production interface:

```text
AssertionError: unexpectedly None : sample_target_covariance is missing
```

The failure count includes the malformed-input subtests. An earlier invocation also exposed an accidental test-class placement error; that test-only error was corrected and the command was rerun to obtain the genuine missing-behavior RED above before production implementation.

### Sample Covariance GREEN

Initial focused GREEN after the minimal implementation:

```text
Ran 5 tests in 0.006s
OK
```

Fresh final command and result:

```bash
python3 -m unittest \
  src.air_ground_bringup.test.test_target_handoff.TargetSampleCovarianceTest -v
```

```text
Ran 5 tests in 0.004s
OK
```

### Handoff Covariance RED

Command:

```bash
python3 -m unittest \
  src.air_ground_bringup.test.test_target_handoff.HandoffCovarianceTest -v
```

Observed genuine RED:

```text
Ran 5 tests in 0.001s
FAILED (failures=5)
```

Each failure named the absent pure handoff interfaces:

```text
AssertionError: unexpectedly None : pure handoff interface is missing
```

### Handoff Covariance GREEN

Initial focused GREEN after standoff and split propagation implementation:

```text
Ran 5 tests in 0.014s
OK
```

Self-review then found that the standoff degeneracy threshold incorrectly depended on absolute world coordinates. A focused translation-invariance test produced a genuine RED:

```bash
python3 -m unittest \
  src.air_ground_bringup.test.test_target_handoff.HandoffCovarianceTest.test_standoff_degeneracy_check_is_translation_invariant -v
```

```text
Ran 1 test in 0.001s
FAILED (failures=1)
AssertionError: unexpectedly None : standoff degeneracy must depend on local target-anchor geometry
```

After making the threshold depend only on local geometry, the focused regression passed 1/1. The fresh final handoff command and result were:

```bash
python3 -m unittest \
  src.air_ground_bringup.test.test_target_handoff.HandoffCovarianceTest -v
```

```text
Ran 6 tests in 0.014s
OK
```

### Full Suite

Command:

```bash
python3 -m unittest src.air_ground_bringup.test.test_target_handoff -v
```

Fresh final result:

```text
Ran 24 tests in 0.035s
OK
```

This count consists of 13 preserved Task 8 policy tests, 5 sample covariance tests, and 6 handoff covariance tests.

Additional syntax verification:

```bash
python3 -m py_compile \
  src/air_ground_bringup/src/air_ground_bringup/target_handoff.py \
  src/air_ground_bringup/test/test_target_handoff.py
```

Result: exit status 0 with no output.

## Interfaces Implemented

```python
sample_target_covariance(
    samples_xy,
    variance_floor,
    pose_covariances=None,
    range_axes=None,
    range_variance=0.0,
)

standoff_goal(target_xy, anchor_xy, standoff)

registration_execution_covariance(
    target_in_registration_frame,
    registration_covariance,
)

evaluate_handoff(
    target_xy,
    target_covariance,
    origin_from_uav,
    registration_covariance,
    standoff,
    inspection_radius,
    inspection_yaw,
)
```

`HandoffResult` is a frozen dataclass. Its valid array values are defensive read-only copies. It exposes:

- `valid`
- `origin_mean`
- `origin_covariance`
- `goal_mean`
- `goal_sensing_covariance`
- `goal_registration_covariance`
- `goal_covariance`
- `action`
- `registration_radius`
- `target_radius`
- `confidence_radius`
- `yaw_confidence`

Malformed evaluation input returns an invalid `HandoffResult` with `action == HOLD`, absent means/covariances, and NaN confidence values. The three lower-level math functions return `None` for malformed input.

## Formulas Implemented

For selected XY samples:

```text
P_sample = cov(samples_xy, ddof=1)
P_sensing = P_sample + variance_floor^2 I
P_target = P_sensing
         + mean(validated per-sample pose covariances)
         + range_variance * mean(unit_axis unit_axis^T)
```

Every supplied range axis is normalized before its outer product. Optional pose and range products are each added once.

For `r = target - anchor`, `q = ||r||`, and `u = r/q`:

```text
goal_xy = target_xy - standoff u
goal_yaw = atan2(r_y, r_x)
J_xy = I - (standoff / q) (I - u u^T)
J_yaw = [-r_y / q^2, r_x / q^2]
```

For registration translation/yaw uncertainty and target lever arm `p`:

```text
J_registration = [[1, 0, -p_y],
                  [0, 1,  p_x],
                  [0, 0,    1]]
P_goal_registration = J_registration P_registration J_registration^T
```

The full evaluation uses:

```text
origin_mean = translation_origin_uav + R_origin_uav target_uav
P_origin = R_origin_uav P_target_uav R_origin_uav^T
P_goal_sensing = J_target P_origin J_target^T
P_goal = P_goal_sensing + P_goal_registration
```

At a 15 m x-axis lever arm and one-degree registration yaw sigma, the registration-only y variance is exactly checked against `(15 * radians(1))^2`. The registration Jacobian tests also use nonzero x/y/yaw cross-covariances.

## Validation And Self-Review

- Fewer than two samples, wrong shapes, nonfinite values, negative/nonfinite floor or range variance, mismatched per-sample inputs, degenerate range axes, and asymmetric/non-PSD pose covariances return `None`.
- Standoff geometry rejects malformed/nonfinite vectors, nonpositive standoff, and target-anchor separation too small for stable linearization.
- The standoff degeneracy test is translation invariant; absolute world coordinates do not affect otherwise identical local geometry.
- Registration propagation rejects malformed/nonfinite target vectors and wrong-shape, asymmetric, or non-PSD covariance.
- Evaluation validates target and registration covariance before propagation and validates each propagated covariance for finite, symmetric, PSD output.
- `origin_covariance` contains target sensing/UAV-pose uncertainty only and is unchanged when only registration covariance changes.
- `goal_registration_covariance` is added exactly once to `goal_sensing_covariance` to form `goal_covariance`.
- `UncertaintyBudget` receives `goal_registration_covariance` and `goal_sensing_covariance[:2, :2]` separately. It never receives the combined covariance in both slots.
- Mutation review: omitting unbiased scatter, using floor instead of floor squared, omitting/duplicating pose or range terms, skipping range-axis normalization, changing either standoff Jacobian, omitting lever-arm yaw propagation, dropping cross-covariances, adding registration to origin covariance, omitting/doubling registration in the goal, passing combined covariance twice, making results mutable, or restoring an absolute-coordinate degeneracy threshold fails at least one focused test.
- No ROS imports, topics, adapters, Task 2 behavior, or `/air_ground/handoff/confidence_radius` changes were introduced.

## Concerns

None within Task 1 scope. The covariance propagation is intentionally first-order, and near-degenerate target-anchor geometry is rejected as required.

---

## Review Fix Round 1/5

### Status

`DONE_WITH_CONCERNS`

The round 1 critical frame-conflation finding and all requested important test gaps are addressed. The concern is the separately ruled, deferred minor deep-immutability limitation documented below.

### Files Changed

- `src/air_ground_bringup/src/air_ground_bringup/target_handoff.py`
  - Added a distinct registration mean transform to `evaluate_handoff`.
  - Added registration mean yaw to `registration_execution_covariance`.
  - Separated origin-target transformation from the registration anchor and lever-arm calculation.
  - Inverse-transformed the origin target into the registration frame and rotated the registration yaw derivative back into origin axes.
- `src/air_ground_bringup/test/test_target_handoff.py`
  - Added distinct UAV/registration transform fixtures, including different translations and yaws.
  - Added registration-anchor isolation, nonzero UAV-yaw anisotropic covariance, rotated registration-yaw Jacobian, and independent radius assertions.
  - Replaced collinear range axes with differently oriented axes and a hand-derived mean outer-product expectation.
- `.superpowers/sdd/2026-08-29-task9-common-frame-handoff/task-1-report.md`
  - Appended this review-fix record.

No other production, test, or task file was edited. No Git operation was attempted.

### Root Cause

The original `evaluate_handoff` accepted only `origin_from_uav` and used that one transform for three distinct roles:

1. Transforming the UAV-frame target mean/covariance into origin.
2. Selecting the registration standoff anchor.
3. Defining the registration yaw lever arm.

The original `registration_execution_covariance` also accepted no registration mean yaw, so its yaw derivative `[-p_y, p_x]` was valid only when the registration transform yaw was zero or when `p` was already expressed in origin axes. The API therefore could not represent distinct `origin<-UAV` and `origin<-UGV-odom` transforms.

### RED Evidence

The new frame-separation tests were added before production changes.

Command:

```bash
python3 -m unittest \
  src.air_ground_bringup.test.test_target_handoff.HandoffCovarianceTest.test_registration_yaw_jacobian_rotates_registration_frame_lever \
  src.air_ground_bringup.test.test_target_handoff.HandoffCovarianceTest.test_registration_translation_changes_anchor_but_not_origin_target \
  src.air_ground_bringup.test.test_target_handoff.HandoffCovarianceTest.test_uav_rotation_propagates_anisotropic_target_and_split_radii \
  -v
```

Observed RED:

```text
Ran 3 tests in 0.001s
FAILED (failures=3)
```

The exact failures identified the missing API boundaries:

```text
'registration_yaw' not found ... registration propagation needs the registration mean yaw
'origin_from_registration' not found ... evaluate_handoff needs a distinct registration mean transform
```

The range-axis behavior already used the correct mean product, so the strengthened test was mutation-checked. Production was temporarily changed to use only the first normalized axis, then this command was run:

```bash
python3 -m unittest \
  src.air_ground_bringup.test.test_target_handoff.TargetSampleCovarianceTest.test_combines_each_sensing_term_once \
  -v
```

Observed mutation RED:

```text
Ran 1 test in 0.037s
FAILED (failures=1)
Max absolute difference: 0.35
```

The incorrect first-axis result was:

```text
[[ 2.583333, -0.626667],
 [-0.626667,  1.983333]]
```

The independently derived expected mean-axis result was:

```text
[[ 2.233333, -0.51    ],
 [-0.51    ,  2.333333]]
```

The correct mean aggregation was restored immediately after the mutation RED.

### GREEN Evidence

Frame-separation GREEN using the same three-test command:

```text
Ran 3 tests in 0.008s
OK
```

Range-axis restoration GREEN:

```bash
python3 -m unittest \
  src.air_ground_bringup.test.test_target_handoff.TargetSampleCovarianceTest.test_combines_each_sensing_term_once \
  -v
```

```text
Ran 1 test in 0.003s
OK
```

Fresh final focused sample command:

```bash
python3 -m unittest \
  src.air_ground_bringup.test.test_target_handoff.TargetSampleCovarianceTest \
  -v
```

```text
Ran 5 tests in 0.005s
OK
```

Fresh final focused handoff command:

```bash
python3 -m unittest \
  src.air_ground_bringup.test.test_target_handoff.HandoffCovarianceTest \
  -v
```

```text
Ran 9 tests in 0.019s
OK
```

Fresh final full-module command:

```bash
python3 -m unittest src.air_ground_bringup.test.test_target_handoff -v
```

```text
Ran 27 tests in 0.040s
OK
```

The 27 tests comprise 13 preserved Task 8 policy tests, 5 sample covariance tests, and 9 handoff covariance tests.

Fresh syntax command:

```bash
python3 -m py_compile \
  src/air_ground_bringup/src/air_ground_bringup/target_handoff.py \
  src/air_ground_bringup/test/test_target_handoff.py
```

Result: exit status 0 with no output.

### Corrected Interfaces

```python
registration_execution_covariance(
    target_in_registration_frame,
    registration_yaw,
    registration_covariance,
)

evaluate_handoff(
    target_xy,
    target_covariance,
    origin_from_uav,
    origin_from_registration,
    registration_covariance,
    standoff,
    inspection_radius,
    inspection_yaw,
)
```

There is no backward-compatibility overload because later tasks have not consumed the prior incomplete Task 1 interface and the new ruling requires the additional physical transform explicitly.

### Corrected Frame Math

For UAV-frame target `p_u`, `origin<-UAV` mean `(t_u, theta_u)`, and registration mean `(t_r, theta_r)`:

```text
target_origin = t_u + R(theta_u) p_u
P_origin = R(theta_u) P_target_uav R(theta_u)^T

anchor_origin = t_r
target_registration = R(theta_r)^T (target_origin - t_r)
```

Only the UAV transform contributes to `target_origin` and `P_origin`. The registration translation is used only as the standoff anchor and in the registration lever calculation.

For `target_registration = [p_x, p_y]`, registration yaw derivative is:

```text
d(R(theta_r) p) / d(theta_r)
  = [-sin(theta_r) p_x - cos(theta_r) p_y,
      cos(theta_r) p_x - sin(theta_r) p_y]
```

The registration Jacobian is now:

```text
J_registration = [[1, 0, d_x],
                  [0, 1, d_y],
                  [0, 0,   1]]
```

and the registration-only covariance remains:

```text
P_goal_registration = J_registration P_registration J_registration^T
```

`P_goal_registration` is still added exactly once to `P_goal_sensing`. The original 15 m / one-degree regression remains green and continues to assert exactly `(15 * radians(1))^2` registration-only lateral variance.

### Review Finding Resolution

- Critical, distinct transforms: resolved. `origin_from_uav` transforms only the target mean/covariance; `origin_from_registration` supplies the anchor and registration-frame inverse transform.
- Critical, rotated yaw derivative: resolved with the exact ruled derivative and a nonzero-yaw literal covariance fixture.
- Important, registration-translation isolation: resolved. Varying only registration translation leaves origin mean/covariance exactly unchanged while changing goal mean and registration lever covariance.
- Important, anisotropic UAV covariance rotation: resolved with a 90-degree UAV transform yaw and a distinct zero-yaw registration transform.
- Important, independent policy radii: resolved with literal expectations for `registration_radius`, `target_radius`, and `confidence_radius` from separate covariance products.
- Important, range-axis mutation sensitivity: resolved with x-axis, y-axis, and diagonal per-sample axes; the mutation RED proves using only one axis is detected.
- Existing 15 m / one-degree registration contribution: preserved exactly once.
- Existing Task 8 behavior: all 13 policy tests remain green.

### Self-Review And Concerns

- Origin mean/covariance have no registration mean or covariance input in their calculation.
- Goal geometry uses registration translation, not UAV transform translation, as anchor.
- The target is inverse-transformed into the registration frame before lever propagation.
- The registration yaw derivative is rotated by registration mean yaw and includes covariance cross-terms through full `J P J^T` multiplication.
- `UncertaintyBudget` still receives registration-only goal covariance and sensing-only goal XY covariance separately.
- Registration uncertainty still appears exactly once in final goal covariance and never in origin covariance.
- Invalid registration transform shape/nonfinite values and nonfinite registration yaw return invalid/HOLD or `None` without raising.
- No ROS, Task 2, Task 10, Git, or unrelated changes were introduced.

Concern: the ledger’s deferred minor remains. `HandoffResult` is a frozen dataclass and arrays are returned read-only, but NumPy arrays that own their buffers can restore their write flag. This round intentionally does not expand scope into a stronger deep-immutable array representation.

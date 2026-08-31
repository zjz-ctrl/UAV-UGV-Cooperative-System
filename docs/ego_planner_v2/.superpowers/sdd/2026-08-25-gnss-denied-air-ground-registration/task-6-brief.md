# Task 6: Implement the SE(2) uncertainty filter and innovation gate

## Milestone

Milestone 2: Uncertainty-Aware Opportunistic Re-Registration, checkpoint M2-A.

## Files

- Modify: `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py`
- Modify: `src/air_ground_coordinate_transform/config/registration.yaml`
- Test: `src/air_ground_coordinate_transform/test/test_registration_estimator.py`

## Binding Research State

The state is the planar transform

```text
x = [t_x, t_y, psi]^T = ^O T_G
```

where `O` is `air_ground_origin` and `G` is the UGV experimental odometry
frame. Units are metres, metres, radians. `P` is the complete 3x3 covariance
with units and cross-covariances implied by that state order.

## Interfaces

- `RegistrationFilter(initial_mean, initial_covariance, process_noise)`
- `predict(dt, uav_distance, ugv_distance) -> FilterState`
- `update(batch: BatchEstimate, mahalanobis_threshold: float) -> UpdateResult`
- `FilterState`: `mean`, `covariance`, `revision`, `stamp`, `initialized`
- `UpdateResult`: `accepted`, `innovation`, `mahalanobis`, `mean`,
  `covariance`, `revision`, `reason`
- Expose `filter.state` and `filter.initialized` for Task 7.
- `(None, None)` initial mean/covariance creates an uninitialized filter;
  the first valid batch initializes revision 1 without gating against a fake
  prior. A supplied valid initial mean/covariance creates revision 1.

## Prediction Formula

The empirical relative-drift model is a random walk, not SLAM. Prediction does
not move the mean:

```text
x^- = x
q_translation =
    translation_time_variance_rate * dt
  + translation_uav_distance_variance_rate * uav_distance
  + translation_ugv_distance_variance_rate * ugv_distance
q_yaw =
    yaw_time_variance_rate * dt
  + yaw_uav_distance_variance_rate * uav_distance
  + yaw_ugv_distance_variance_rate * ugv_distance
P^- = P + diag([q_translation, q_translation, q_yaw])
```

All six rates are finite and nonnegative. Physical units:

- translation time rate: `m^2/s`
- translation distance rates: `m^2/m`
- yaw time rate: `rad^2/s`
- yaw distance rates: `rad^2/m`

`dt`, `uav_distance`, and `ugv_distance` must be finite and nonnegative. Zero
time and zero travel produce exactly zero covariance growth. Prediction keeps
revision unchanged and advances the state stamp by `dt` only when initialized.

## Update Formula And Statistical Gate

For batch mean `z` and covariance `R`:

```text
innovation = z - x^-
innovation[2] = wrap_angle(innovation[2])
S = P^- + R
d^2 = innovation.T @ solve(S, innovation)
```

`d^2` is the normalized innovation squared. Gate against the configured
three-degree-of-freedom chi-square threshold. The default is:

```text
innovation_mahalanobis_threshold: 11.344866730144373
```

which is `chi2.ppf(0.99, df=3)`, a nominal 1% false-rejection probability
under the Gaussian model. Do not add SciPy as a runtime dependency.

If accepted:

```text
K = P^- @ inv(S)
x_new = wrap_xyyaw(x^- + K @ innovation)
P_new = (I-K) @ P^- @ (I-K).T + K @ R @ K.T
```

Use the Joseph covariance update, then symmetrize and PSD-check. The accepted
update sets `stamp=batch.stamp` and increments revision exactly once. A gated
update leaves mean, covariance, stamp, and revision unchanged.

Validate all means/covariances for finite values and exact shapes. Covariances
must be symmetric PSD; `S` must be solvable and statistically usable. Return
stable rejection reasons for invalid batches, singular innovation covariance,
and Mahalanobis gate rejection. Programmer errors in prediction inputs and
invalid constructor/process-noise parameters should raise `ValueError`.

## TDD Requirements

1. Write failing covariance-growth tests before implementation. Verify every
   diagonal grows for positive time/travel, the exact formula matches a hand
   calculation, the mean/revision are unchanged, and all-zero inputs do not
   grow covariance.
2. Write failing update tests before implementation:
   - prior yaw `+179 deg`, measurement `-179 deg` gives `+2 deg` innovation;
   - a measurement just inside the chi-square threshold is accepted;
   - a gross translation/yaw outlier is rejected with unchanged state;
   - accepted update uses the independently calculated Joseph result;
   - covariance remains finite, symmetric, and PSD;
   - invalid/nonfinite/non-PSD covariance and singular `S` are rejected with
     explicit reasons.
3. Test uninitialized first-batch initialization and monotonic accepted-update
   revisions.
4. Run 100 seeded drift/update sequences. Intermittent measurements must yield
   finite state, symmetric PSD covariance, monotonic revisions, and lower final
   wrapped-state RMSE than prediction-only for at least 95 sequences. The test
   must model real random-walk error and noisy intermittent measurements, not
   read Gazebo truth or tune each seed independently.
5. Mutation-sensitivity evidence must show the tests fail if yaw wrapping,
   Mahalanobis gating, Joseph covariance, or any process-noise contribution is
   removed.

## Configuration And Calibration Documentation

Add all six process-noise rates and the chi-square threshold to
`registration.yaml`, with comments documenting units and physical meaning.
Document that rates are estimated before held-out evaluation from intervals
without accepted visual updates: regress empirical transform-error variance
growth against elapsed time, UAV travel, and UGV travel using constrained
nonnegative regression. Freeze coefficients before M2 held-out seeds. Check the
innovation threshold using held-out NIS coverage against the selected 99%
chi-square quantile; do not tune against mission success.

## Compatibility And Safety

- Do not modify `takeoff_registration.py`, launch files, one-shot state, or ROS
  behavior in Task 6. Task 7 performs integration.
- The one-shot baseline must remain byte-for-behavior compatible and revision 1.
- No Gazebo truth topic/service, experiment truth, or offline alignment may
  enter the filter or decision path.
- Strict RED -> GREEN TDD. If an allowed test fails unexpectedly, invoke
  `systematic-debugging` before fixing.
- Do not execute `roslaunch`, `roscore`, `rostest`, Gazebo, PX4, RViz, rosbag,
  topic waits, or any long-running process. Allowed checks are pure tests,
  `py_compile`, static config parsing, and bounded catkin builds.
- The workspace has no Git metadata. Do not initialize Git or claim commits.

## M2-A Evidence

Record exact formulas, parameter values/units/calibration method, 100-sequence
Monte Carlo summary, focused/full regression results, and residual manual
validation needs. Stop after Task 6; do not start Task 7.

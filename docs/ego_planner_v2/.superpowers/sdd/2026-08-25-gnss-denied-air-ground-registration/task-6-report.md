# Task 6 Report: SE(2) Uncertainty Filter And Innovation Gate

## Status

**Implementation and pure verification complete for Task 6 / M2-A.** No commit was
created because this workspace has no Git metadata and the task prohibits Git
operations. Task 7 integration was not started.

The ROS-independent filter, configuration, tests, Monte Carlo evaluation, and
mutation checks are complete. The configured process coefficients still require
the prescribed empirical regression on real no-update intervals before M2
held-out evaluation; this is listed under Manual Validation Needs rather than
being hidden by simulator or experiment truth.

## State, Covariance, And Formulas

The filter state is the planar transform

```text
x = [t_x, t_y, psi]^T = ^O T_G
```

where `t_x` and `t_y` are metres and `psi` is radians. `P` is the complete 3x3
covariance in that state order. It retains `tx-ty`, `tx-yaw`, and `ty-yaw`
cross-covariances; neither prediction nor update reduces it to a diagonal.

`FilterState` contains `mean`, `covariance`, `revision`, `stamp`, and
`initialized`. The live value is stored internally as `_state`; the public
`state` property and every returned state/result are independent array snapshots.
A supplied valid prior starts at revision 1 and stamp 0. An
uninitialized filter has revision 0 and stamp 0; its first valid batch becomes
revision 1 without comparison to a fake prior.

Prediction is the exact empirical random walk:

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

Prediction leaves mean and revision unchanged. For an initialized filter it
advances stamp by `dt`; for an uninitialized filter it does not advance stamp.
Zero time and travel produce exactly zero growth.

For a valid batch mean `z` and covariance `R`, update uses:

```text
nu = z - x^-
nu[2] = wrap_angle(nu[2])
S = P^- + R
d^2 = nu.T @ solve(S, nu)
K = solve(S.T, P^-.T).T
x_new = wrap_xyyaw(x^- + K @ nu)
P_new = (I-K) @ P^- @ (I-K).T + K @ R @ K.T
P_new = 0.5 * (P_new + P_new.T)
```

`d^2` is the normalized innovation squared (NIS). `S` must be positive definite,
solvable, and yield finite nonnegative NIS. The Joseph covariance is checked for
finite values, symmetry, and PSD after symmetrization. It is never repaired by
eigenvalue clamping.

Accepted updates set stamp to the batch stamp and increment revision exactly
once. Batch stamps must be finite and nonnegative; an initialized filter returns
`stale_batch` when `batch.stamp < state.stamp`, while equality is allowed. Gated,
invalid-batch, stale-batch, and singular-innovation rejections leave every state
field unchanged. Stable returned rejection reasons are `invalid_batch`,
`stale_batch`, `singular_innovation_covariance`, and `mahalanobis_gate`. Invalid constructor,
process-noise, prediction, and gate-threshold programmer inputs raise
`ValueError`.

## Round 0 Historical Log (Superseded)

This table is retained only as the round-0 implementation log. Review I5 found
that it does not establish strict test-first chronology, and this report no
longer presents it as auditable TDD evidence. Review Fix Round 1 below supersedes
it with a complete production-block reset and observed RED/GREEN outputs.

| Recorded behavior | Recorded RED | Recorded GREEN |
|---|---|---|
| Exact prediction growth, unchanged mean/revision, all diagonals grow | `RegistrationFilter` absent; 1 focused failure | Exact focused case passed |
| Zero time/travel gives zero covariance growth | Production addition was removed before this focused test; `RegistrationFilter` absent; 1 failure | Prediction class passed 2/2 |
| Uninitialized prediction and public state | Missing `initialized`; 1 error | Prediction class passed 5/5 after validation work |
| Constructor/process validation | Negative/nonfinite rates and malformed priors accepted; missing key leaked `KeyError` | Focused constructor validation passed |
| Prediction input validation | Six negative/nonfinite cases failed to raise | Focused prediction validation passed |
| First valid batch initializes without fake-prior gate | `update` absent; 1 focused failure | Focused initialization test passed |
| Wrapped `+179 deg -> -179 deg` innovation | Initialized update returned no result; 1 failure | Wrapped yaw test passed with `+2 deg` innovation |
| Gross NIS outlier leaves state unchanged | Initialized update returned no result; 1 failure | Gate rejection test passed |
| Just-inside chi-square boundary acceptance | Accepted path returned no result; 1 failure | Boundary test passed at threshold minus `1e-9` |
| Full Joseph update and PSD result | Accepted path absent; 1 error | Independent literal mean/covariance fixture passed |
| Invalid/nonfinite/non-symmetric/non-PSD batches | Invalid data was accepted or leaked shape errors | Invalid-batch focused test passed for all cases |
| Invalid first batch remains uninitialized | Non-PSD first batch initialized filter; 1 failure | Focused test passed |
| Singular `S` rejection | `numpy.linalg.LinAlgError: Singular matrix` leaked | Stable-reason focused test passed |
| Monotonic accepted revisions | Accepted transition was absent during focused RED; 1 failure | Revisions `[2, 3, 4]` passed |
| Invalid gate threshold | Three cases failed to raise and malformed text leaked `TypeError` | Focused threshold validation passed |
| Nonnumeric constructor objects | Two `TypeError`s leaked | Both now normalize to `ValueError`; focused test passed |
| 100-sequence acceptance evaluation | Added only after all production behavior was green | Monte Carlo focused test passed in 3.153 s initially and in final regression |

Final fresh focused result: **29/29 passed in 3.276 s**.

## Process Parameters

These values are one frozen coefficient set shared by all synthetic sequences;
there is no per-seed tuning.

| Name | Value | Unit | Physical meaning | Calibration requirement |
|---|---:|---|---|---|
| `translation_time_variance_rate` | 0.0004 | `m^2/s` | Common `tx`/`ty` drift variance per elapsed second | Fit jointly by constrained nonnegative regression on no-accepted-update intervals |
| `translation_uav_distance_variance_rate` | 0.0009 | `m^2/m` | Common `tx`/`ty` drift variance per metre of UAV travel | Same joint nonnegative regression |
| `translation_ugv_distance_variance_rate` | 0.0016 | `m^2/m` | Common `tx`/`ty` drift variance per metre of UGV travel | Same joint nonnegative regression |
| `yaw_time_variance_rate` | 0.000025 | `rad^2/s` | Relative-yaw drift variance per elapsed second | Fit yaw variance growth by constrained nonnegative regression |
| `yaw_uav_distance_variance_rate` | 0.000049 | `rad^2/m` | Relative-yaw drift variance per metre of UAV travel | Same joint nonnegative regression |
| `yaw_ugv_distance_variance_rate` | 0.0001 | `rad^2/m` | Relative-yaw drift variance per metre of UGV travel | Same joint nonnegative regression |
| `innovation_mahalanobis_threshold` | 11.344866730144373 | dimensionless NIS | Three-state innovation acceptance boundary | Freeze at selected quantile; check held-out NIS coverage, never mission success |

Calibration must use intervals without accepted visual updates. Regress empirical
transform-error variance growth jointly against elapsed time, UAV travel, and UGV
travel with coefficients constrained nonnegative. Freeze all six coefficients
before M2 held-out seeds. The configuration comments record this method and the
physical units. The task did not read simulator or experiment truth to claim that
the current numerical rates are field-calibrated.

## Gate Statistics

The threshold `11.344866730144373` is `chi2.ppf(0.99, df=3)`. Under a correctly
specified three-dimensional Gaussian innovation model it gives a nominal 1%
false-rejection probability. SciPy is not a runtime dependency. Held-out
validation must compare empirical NIS coverage with this frozen 99% chi-square
quantile and must not tune the threshold against mission completion.

## 100-Sequence Monte Carlo

Exact setup:

- 100 independent `numpy.random.RandomState` streams with seeds 73000 through
  73099.
- Each sequence starts synthetic truth and both estimates at `[0, 0, 0]` with
  zero initial covariance, then runs 80 steps.
- Every step uses `dt=0.2 s`, UAV distance sampled uniformly from `[0.12, 0.18)`
  m, and UGV distance sampled uniformly from `[0.025, 0.05)` m.
- Synthetic truth follows an independent Gaussian SE(2) random walk whose
  diagonal increment covariance is computed from all six configured rates and
  the exact prediction formula. Truth yaw is wrapped after each increment.
- The update-enabled and prediction-only filters receive the same `dt`, travel,
  and synthetic truth realization. Prediction-only receives no measurements.
- Every fourth step produces a noisy measurement, for 20 opportunities per
  sequence and 2000 total. Noise is independent Gaussian with full covariance:

```text
R = [[ 0.000400,  0.000080,  0.000010],
     [ 0.000080,  0.000625, -0.000015],
     [ 0.000010, -0.000015,  0.000100]]
```

- Every measurement uses the fixed `11.344866730144373` gate. No seed has
  individually selected noise, parameters, timing, or threshold.
- Final state error wraps yaw. Per-sequence wrapped-state RMSE is
  `sqrt(mean([tx_error^2, ty_error^2, wrapped_yaw_error^2]))`; this scalar mixes
  state units only as a comparison diagnostic. Component RMSEs retain their
  metre/metre/radian interpretation.

Fresh final summary:

| Metric | Update-enabled | Prediction-only |
|---|---:|---:|
| Improved sequences | **99 / 100** | comparison baseline |
| `tx` final RMSE | 0.02385568 m | 0.14805080 m |
| `ty` final RMSE | 0.02138058 m | 0.14953587 m |
| wrapped yaw final RMSE | 0.00820835 rad | 0.03421932 rad |
| aggregate wrapped-state RMSE | 0.019092753310491765 | 0.12308665964084933 |

There were 1979 accepted updates and 21 gate rejections. At every step, both
filter states and covariances were finite, covariance symmetry held to `1e-12`,
and minimum covariance eigenvalue was at least `-1e-12`. Accepted revisions
increased by exactly one and rejected revisions remained unchanged.

## Mutation Sensitivity

Every mutation below was temporary, its focused test was observed failing, and
the original production line was restored immediately. A post-restoration set of
the exact prediction, yaw, gate, and Joseph tests passed 4/4, followed by the
fresh 29/29 and 50/50 final suites.

| Removed behavior | Focused failure evidence |
|---|---|
| Yaw innovation wrapping | Expected `+0.0349065850`, observed `-6.2482787221` rad; 1 failure |
| Mahalanobis gate branch | Gross outlier became accepted; 1 failure |
| Joseph measurement-noise term `K R K.T` | Independent covariance mismatched 100%; maximum absolute difference 0.09862051 |
| Time contributor, both translation and yaw terms | Exact prediction mismatch; maxima 0.2 translation variance and 0.02 yaw variance |
| UAV-distance contributor, both translation and yaw terms | Exact prediction mismatch; maxima 0.6 translation variance and 0.06 yaw variance |
| UGV-distance contributor, both translation and yaw terms | Exact prediction mismatch; maxima 1.2 translation variance and 0.12 yaw variance |

Each of the six individual process-rate terms was also removed separately. The
same exact-formula test failed with the expected missing contribution: 0.2,
0.02, 0.6, 0.06, 1.2, and 0.12 for translation-time, yaw-time,
translation-UAV, yaw-UAV, translation-UGV, and yaw-UGV respectively.

## One-Shot And No-Truth Audit

- `takeoff_registration.py`, launch files, node wiring, and one-shot state logic
  were not edited. `RegistrationFilter` is not integrated into the node in Task
  6; Task 7 remains responsible for integration.
- The original concurrent one-shot zero-to-one transition test passed separately
  1/1 and within the 29-test focused suite. Its frozen revision remains 1.
- The source node SHA-256 observed during audit was
  `98348694b753fabe7765b7f20459fed88e89720c06071e064011c54f0a3d7828`.
  There is no Git metadata with which to claim a historical hash comparison, so
  the evidence is the untouched file set plus behavior regression rather than a
  fabricated VCS claim.
- Static production searches found no `RegistrationFilter` or new process/gate
  parameter use in the one-shot script and no Gazebo-truth, ground-truth,
  experiment-truth, or `/truth` reference in the production estimator package.
- The Monte Carlo truth is a local seeded synthetic random walk inside the pure
  test. It is not a topic, service, bag, offline alignment, or filter input.

## Bounded Verification

Only allowed bounded commands were run:

| Check | Fresh result |
|---|---|
| Focused `test_registration_estimator.py` | 29/29 passed in 3.276 s |
| Full pure package tests: `test_se2.py`, `test_odom_buffer.py`, `test_registration_estimator.py` | 50/50 passed in 3.350 s |
| Monte Carlo summary command with `>=95` assertion | 99/100 improved; assertion passed |
| One-shot concurrency regression alone | 1/1 passed in 0.003 s |
| `python3 -m py_compile` on production and focused test | Exit 0, no output |
| `yaml.safe_load` plus seven numeric-key assertions | Exit 0; all six rates and threshold parsed numerically |
| `catkin_make --pkg air_ground_coordinate_transform -j2` | Exit 0; `[100%] Built target coordinate_transform_node` |

No `roslaunch`, `roscore`, `rostest`, Gazebo, PX4, RViz, rosbag, topic wait, or
long-running process was executed.

## Modified Files

- `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py`
- `src/air_ground_coordinate_transform/test/test_registration_estimator.py`
- `src/air_ground_coordinate_transform/config/registration.yaml`
- `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-6-report.md`

## Self-Review

- Full SE(2) mean and 3x3 covariance/cross-covariance are retained.
- Prediction uses all six exact nonnegative variance-rate terms and never moves
  the mean.
- Update wraps yaw innovation, computes NIS with `solve`, uses the fixed 3-DOF
  statistical gate, and applies the full Joseph covariance equation.
- Invalid covariance is rejected rather than clamped. The filter does not call
  the pre-existing sample/batch covariance-conditioning helper.
- Accepted updates are the only operation that advances revision; prediction and
  all rejection paths preserve it.
- Input validation has focused coverage for shape, finite values, symmetry, PSD,
  singular `S`, negative/nonfinite process/prediction/gate values, paired prior
  initialization, and nonnumeric constructor objects.
- No SciPy dependency, ROS dependency, truth source, compatibility shim, node
  integration, or unrelated refactor was added.
- Mutation edits were fully restored, as demonstrated by final focused and full
  passing suites.

## Manual Validation Needs

1. Collect real calibration intervals with no accepted visual update and estimate
   the six rates by the documented constrained nonnegative regression. Replace
   provisional coefficients only before, never during, M2 held-out evaluation.
2. Freeze those coefficients and check empirical held-out NIS coverage against
   the 99% three-degree-of-freedom chi-square threshold. Do not optimize mission
   success or tune per seed.
3. In Task 7, integrate elapsed time and UAV/UGV distance accumulation into the
   node without changing the one-shot baseline path.
4. Perform ROS/vehicle timing, frame, and field validation later under an
   explicitly approved integration procedure; none was attempted in Task 6.

## Review Fix Round 1

### Disposition I1-I5

| Finding | Disposition | Verified evidence and change |
|---|---|---|
| I1 state aliasing | **Valid, fixed** | The previous public `state` arrays mutated the live prior and `state` was rebindable. The filter now stores `_state`; `state`, `predict`, constructor inputs, accepted/rejected `UpdateResult`, and old snapshots use independent array copies. Mutation tests alter every exposed array and verify the subsequent complete filter state. |
| I2 noncausal stamps | **Valid, fixed** | Batch stamp is now finite and nonnegative. For initialized filters, a strict older stamp returns stable `stale_batch`; equal stamp is accepted. Initialization, prediction, accepted update, gate rejection, negative stamp, and stale stamp tests verify monotonicity and full-state preservation. |
| I3 unusable `S` and inverse | **Valid, fixed** | Explicit `inv(S)` was removed. NIS and gain share one diagonally equilibrated Cholesky factorization. Scaled conditioning and all intermediate finite checks map failures to `singular_innovation_covariance` without state mutation. |
| I4 missing edges | **Valid, fixed** | Added alias mutation, negative/stale stamp, scaled-safe tiny SPD, scaled near-singular SPD, prediction overflow rollback, exact gate equality, and complete rejected-state tests. The `>=` gate mutation is retained below with observed RED/GREEN commands. |
| I5 chronology | **Valid, fixed by reset** | Round-0 claims are marked superseded. New I1-I4 tests first failed against the then-current block; the complete `FilterState`/`UpdateResult`/`RegistrationFilter` block was then deleted, all 27 Task 6 tests including Monte Carlo were observed RED, and the block was rebuilt from those retained tests. |

No pushback was required for I1-I5. Deferred ledger items M1-M4 were not
implemented. Prediction overflow was handled only because I4 explicitly requires
that edge and its minimal production rollback behavior.

### Auditable TDD Reset

All commands below were run from the workspace root with no ROS process.

Current-block I1-I4 RED command:

```bash
PYTHONPATH="src/air_ground_coordinate_transform/src" python3 -m unittest \
  src.air_ground_coordinate_transform.test.test_registration_estimator.RegistrationFilterPredictionTest \
  src.air_ground_coordinate_transform.test.test_registration_estimator.RegistrationFilterUpdateTest
```

Observed output:

```text
Ran 26 tests in 0.012s
FAILED (failures=8)
```

The eight failures were live `state` mutation, public-state rebinding,
prediction-result aliasing, prediction overflow not raising, negative stamp
initialization, stale batch acceptance, explicit-inverse failure for scaled-safe
SPD, and acceptance of unusable scaled correlation.

The complete Task 6 production block was then deleted. Reset RED command,
including the retained 100-sequence Monte Carlo test:

```bash
PYTHONPATH="src/air_ground_coordinate_transform/src" python3 -m unittest \
  src.air_ground_coordinate_transform.test.test_registration_estimator.RegistrationFilterPredictionTest \
  src.air_ground_coordinate_transform.test.test_registration_estimator.RegistrationFilterUpdateTest \
  src.air_ground_coordinate_transform.test.test_registration_estimator.RegistrationFilterMonteCarloTest
```

Observed feature-absent output:

```text
Ran 27 tests in 0.003s
FAILED (failures=2, errors=51)
```

The errors are repeated subtest references to the deliberately absent
`RegistrationFilter`; the two explicit failures are the tests that first check
the feature symbol. This is the fix-round RED, not a claim about the original
round-0 implementation chronology.

The first rebuild run found one unexpected failure:

```text
Ran 27 tests in 4.328s
FAILED (failures=1)
```

`systematic-debugging` was invoked before editing. Diagnostics showed that
`sqrt(1e-320)^2` is a valid subnormal scale product but `errstate(all="raise")`
classified its underflow flag as fatal even though the equilibrated matrix was
identity and gain/Joseph outputs were finite. A standalone hypothesis check with
underflow ignored produced eigenvalues `[1,1,1]`, finite gain, and zero posterior
covariance. The minimal correction ignores underflow while still raising
overflow, invalid, and divide errors; all resulting intermediates remain
explicitly finite-checked. Rebuild GREEN was then:

```text
Ran 27 tests in 4.662s
OK
```

After strengthening accepted/rejected result assertions and using an independently
accumulated synthetic Monte Carlo clock, the same retained suite remained GREEN:

```text
Ran 27 tests in 4.947s
OK
```

The synthetic clock change does not relax production causality. It replaces
`step * dt` with `simulation_stamp += dt`, avoiding 800 artificial `stale_batch`
decisions caused solely by multiplication-versus-addition floating rounding.
Instrumentation before the correction counted exactly 800 stale rejections, 11
NIS gates, and 1189 accepted updates; the corrected physical schedule restores
1979 accepted and 21 NIS-gated updates.

Exact-equality mutation command:

```bash
PYTHONPATH="src/air_ground_coordinate_transform/src" python3 -m unittest \
  src.air_ground_coordinate_transform.test.test_registration_estimator.RegistrationFilterUpdateTest.test_exact_mahalanobis_threshold_equality_is_accepted
```

With the temporary production mutation `mahalanobis > threshold` to
`mahalanobis >= threshold`, output was `FAILED (failures=1)` because equality was
rejected. After restoring `>`, the same command ran 1 test and returned `OK`.
The unmutated test uses the exactly representable case `S=I`, `nu=[1,0,0]`, and
threshold 1, so it freezes the intended `d^2 <= threshold` convention without a
rounding-dependent near-boundary fixture.

### Scaled Conditioning Method

For `S = P + R`, define diagonal scaling

```text
D = diag(sqrt(diag(S)))
C = D^-1 S D^-1
y = D^-1 nu
```

This removes diagonal units and magnitude before assessing conditioning. The
scaled reciprocal condition estimate is

```text
r_scaled = lambda_min(C) / lambda_max(C)
```

and the update rejects when

```text
r_scaled < sqrt(epsilon_float64) = 1.4901161193847656e-08
```

The threshold bounds the first-order solve error scale `condition(C) * epsilon`
to approximately `sqrt(epsilon)` while avoiding an arbitrary raw-unit condition
number for the mixed `[m,m,rad]` state. The retained near-singular correlation
fixture has `r_scaled` approximately `5e-11` and is rejected. In contrast,
`S=diag(1,1e-320,1)` scales exactly to `C=I`; it is mathematically usable for the
given PSD prior and is accepted without exception.

One Cholesky factor `L L.T = C` is reused through two triangular solves:

```text
d^2 = y.T solve(C, y)
K = (P D^-1 solve(C, I)) D^-1
```

The implementation forms neither `inv(S)` nor `inv(C)`. It checks finite values
for innovation, `S`, scaling, scaled matrix/eigenvalues, Cholesky factor, scaled
innovation and solve, NIS, scaled prior, gain, state correction, posterior mean,
and Joseph covariance. Factorization, solve, condition, overflow, divide,
nonfinite, or invalid-posterior failures return
`singular_innovation_covariance` with all five internal state fields unchanged.

### Round 1 Modified Files

- `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py`
- `src/air_ground_coordinate_transform/test/test_registration_estimator.py`
- `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-6-report.md`

`registration.yaml`, `takeoff_registration.py`, launch files, one-shot state,
and ROS behavior were not modified in this fix round.

### Round 1 Self-Review

- `_state` is never returned directly; every public state/result path copies
  NumPy mean/covariance arrays, and `state` has no setter.
- Constructor input mutation, current snapshot mutation, prediction result
  mutation, old snapshot mutation, and accepted/rejected result mutation all
  have direct behavioral tests.
- Negative stamp is `invalid_batch`; strict older stamp is `stale_batch`; equal
  stamp is accepted. Every rejection keeps mean, covariance, revision, stamp,
  and initialized unchanged.
- Prediction computes growth, candidate stamp, and candidate covariance before
  assignment. Nonfinite arithmetic raises `ArithmeticError` and preserves state.
- Full 3x3 covariance/cross-covariance, six-rate process growth, wrapped yaw,
  solve-based NIS, exact `<=` gate boundary, and Joseph update remain intact.
- M1 wrapped supplied-prior yaw, M3 covariance validation tolerances, and M4 YAML
  provenance labeling remain deferred exactly as requested. M2 is otherwise
  deferred; only the I4-mandated overflow edge was minimally handled.
- No truth source, one-shot/node integration, SciPy dependency, ROS process, Git
  action, or unrelated refactor was introduced.

### Round 1 Manual Concerns

- The six YAML process rates remain provisional pending the prescribed real
  no-update constrained nonnegative regression and held-out freeze.
- Task 7 integration and real ROS/vehicle timing/frame validation remain manual
  future work and were not started here.

### Round 1 Final Bounded Verification

| Check | Fresh observed result |
|---|---|
| Focused `test_registration_estimator.py` | 40/40 passed in 4.586 s |
| Full pure package `test_se2.py`, `test_odom_buffer.py`, `test_registration_estimator.py` | 61/61 passed in 4.917 s |
| Independent 100-sequence summary | 99/100 improved; 1979 accepted, 21 NIS-gated |
| Update-enabled component RMSE | `[0.02385568 m, 0.02138058 m, 0.00820835 rad]` |
| Prediction-only component RMSE | `[0.14805080 m, 0.14953587 m, 0.03421932 rad]` |
| Aggregate wrapped-state RMSE | 0.019092753310491765 versus 0.12308665964084933 prediction-only |
| One-shot concurrency regression | 1/1 passed in 0.002 s; revision remains 1 |
| `py_compile` for estimator, `se2.py`, and focused test | Exit 0, no output |
| YAML parse and seven numeric parameter assertions | Exit 0 |
| Bounded `catkin_make --pkg air_ground_coordinate_transform -j2` | Exit 0; target built 100% |
| Forbidden inverse/mutation-remnant static search | No matches |
| Production truth-reference static search | No matches |

No ROS, simulator, bag, topic wait, truth source, Git operation, or long-running
process was used in Review Fix Round 1.

## Review Fix Round 2

### R1-NI1 Disposition

**Valid, fixed.** Finite input validation did not guarantee that
`measurement_mean - state.mean` was representable. With prior mean
`[-1e308, 0, 0]`, measurement mean `[1e308, 0, 0]`, and caller
`np.errstate(all="raise")`, NumPy raised `FloatingPointError` before the existing
finite check and no `UpdateResult` was returned.

Innovation subtraction and yaw wrapping now execute inside the narrow local
policy:

```python
with np.errstate(over="raise", invalid="raise"):
    innovation = measurement_mean - self._state.mean
    innovation[2] = wrap_angle(innovation[2])
```

`FloatingPointError` and `OverflowError` return the existing
`singular_innovation_covariance` rejection. The change does not alter global
warning configuration, suppress underflow/divide behavior, or affect the normal
innovation path.

### Round 2 RED / GREEN

Focused command:

```bash
PYTHONPATH="src/air_ground_coordinate_transform/src" python3 -m unittest \
  src.air_ground_coordinate_transform.test.test_registration_estimator.RegistrationFilterUpdateTest.test_finite_extreme_means_reject_overflowing_innovation_without_exception
```

Observed RED before production edit:

```text
FloatingPointError: overflow encountered in subtract
AssertionError: finite extreme innovation raised FloatingPointError(...)
Ran 1 test in 0.001s
FAILED (failures=1)
```

Observed GREEN after the local guard:

```text
Ran 1 test in 0.001s
OK
```

The retained test now covers both translation and yaw-coordinate extreme finite
differences under caller `np.errstate(all="raise")`. For each case it asserts:

- `accepted == False`;
- `reason == "singular_innovation_covariance"`;
- default pre-innovation rejection semantics: zero innovation and `NaN` NIS;
- returned mean, covariance, and revision match the prior; and
- internal mean, covariance, revision, stamp, and initialized are all unchanged.

### Round 2 Modified Files

- `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py`
- `src/air_ground_coordinate_transform/test/test_registration_estimator.py`
- `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-6-report.md`

No configuration, one-shot, node, launch, ROS integration, or deferred Minor was
modified.

### Round 2 Self-Review

- Measurement conversion remains under its existing conversion exception guard.
- Finite shape/value checks still precede initialized update arithmetic.
- Measurement-state subtraction and yaw wrapping are now in one local
  overflow/invalid guard; failures cannot expose a partially computed innovation.
- Pre-innovation numerical rejection deliberately calls `_rejection` without an
  innovation argument, preserving zero innovation and `NaN` Mahalanobis.
- `_rejection` copies prior arrays and no `_state` assignment occurs before this
  path, so all five state fields remain unchanged.
- Normal wrapped-yaw, scaled conditioning, NIS gate, full Joseph update, and
  six-rate prediction code were not changed.
- Deferred Minors, truth access, ROS processes, simulation, long-running
  processes, and Git operations remain out of scope.

### Round 2 Final Bounded Verification

| Check | Fresh observed result |
|---|---|
| Focused `test_registration_estimator.py` | 41/41 passed in 4.640 s |
| Full pure package suite | 62/62 passed in 4.850 s |
| Independent 100-sequence Monte Carlo | 99/100 improved; 1979 accepted, 21 NIS-gated |
| Update-enabled aggregate wrapped-state RMSE | 0.019092753310491765 |
| Prediction-only aggregate wrapped-state RMSE | 0.12308665964084933 |
| `py_compile` for estimator, `se2.py`, and focused test | Exit 0, no output |
| YAML parse and seven numeric parameter assertions | Exit 0 |
| Bounded `catkin_make --pkg air_ground_coordinate_transform -j2` | Exit 0; target built 100% |

No ROS, simulator, truth source, topic wait, bag, long-running process, or Git
operation was used in Review Fix Round 2.

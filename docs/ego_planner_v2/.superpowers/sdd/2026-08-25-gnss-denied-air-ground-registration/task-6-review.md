# Task 6 Fresh Algorithm And Code Review

## Verdicts

- **Spec Compliance: FAIL**
- **Code Quality: FAIL**
- **Task 7 integration: NOT READY**. The six-rate prediction and nominal EKF
  equations are correct, but the mutable public state, non-causal stamp handling,
  and incomplete near-singular innovation rejection must be fixed first.

## Findings Summary

- Critical: 0
- Important: 5
- Minor: 4
- Highest severity: Important

## Critical Findings

None.

## Important Findings

### I1. `FilterState` is not a snapshot and permits silent external corruption

- **Files:**
  `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py:28-34,73-99,109-111,128-151`
- **Failure scenario:** `filter.state.mean[0] = 9`,
  `filter.state.covariance[0, 0] = 99`, or mutation of the object returned by
  `predict()` immediately changes the filter's internal prior. The whole public
  `filter.state` attribute can also be rebound. A frozen dataclass prevents field
  rebinding but does not make its NumPy arrays immutable.
- **Mathematical consequence:** the next prediction and NIS use an externally
  altered `x` or `P`; PSD, units, revision, and initialization invariants can be
  bypassed without validation. This is especially unsafe when Task 7 hands state
  arrays to ROS publication/conversion code.
- **Evidence:** a bounded probe changed the live state to mean `[9, 8, 0]` and
  covariance entry `99` solely through references returned by `state`/`predict`.
  `UpdateResult` arrays are copied and do not have this alias-to-filter problem.
- **Minimum fix:** store an internal `_state`; expose a read-only property that
  returns deep array copies (or arrays made non-writeable), return independent
  snapshots from `predict`, and prevent external assignment to the live state.
  Add mutation tests for constructor inputs, `filter.state`, prediction results,
  accepted/rejected `UpdateResult`, and old snapshots after later transitions.

### I2. Accepted stale batches can move filter time backward

- **Files:**
  `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py:163-175,177-195,234-249`;
  `docs/superpowers/plans/2026-08-25-gnss-denied-air-ground-registration.md:679-687`
- **Failure scenario:** after odometry callbacks predict the state to time `t`, a
  delayed observation window with `batch.stamp < state.stamp` can be accepted and
  line 239 replaces the current stamp with the old batch stamp. Negative first
  batch stamps are also accepted because only finiteness is checked. A bounded
  probe accepted stamp `-4.0` and set the live state stamp to `-4.0`.
- **Mathematical consequence:** the state no longer denotes a causal posterior at
  a monotonic epoch. Task 7's incremental elapsed-time/travel prediction can then
  double-count process variance, publish out-of-order estimates, or associate a
  current covariance with an old epoch.
- **Minimum fix:** define a causal timestamp contract before integration. Reject
  negative or older batches with a stable reason, or predict/update at the batch
  epoch and then replay the accumulated motion to the current epoch. Assert
  monotonic stamps across initialization, prediction, acceptance, and rejection.

### I3. A numerically unusable `S` can escape the specified rejection path

- **Files:**
  `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py:199-220,229-233`
- **Failure scenario:** valid PSD `P=diag(1,0,1)` and
  `R=diag(0,1e-320,0)` produce an SPD matrix for which Cholesky and the zero-NIS
  solve complete, but explicit `inv(S)` overflows. The update emitted a runtime
  warning and raised `ArithmeticError` instead of returning
  `singular_innovation_covariance`. Other ill-conditioned inputs can yield an
  unstable gain before the same late failure.
- **Mathematical consequence:** Cholesky success establishes positive
  definiteness in floating point, not adequate conditioning. Forming `S^-1`
  magnifies error by its condition number and is unnecessary: the full gain is
  `K = solve(S.T, P.T).T`. The current exception path breaks the stable rejection
  API even though state assignment has not yet occurred.
- **Minimum fix:** compute the gain with a solve using the same factorization as
  NIS, apply an explicit scale-aware usability/conditioning criterion, validate
  all intermediate values, and convert every factorization/solve/usability
  failure into `singular_innovation_covariance` while preserving all state
  fields. Add a bounded near-singular SPD test.

### I4. The green suite omits the edge cases that expose all three integration blockers

- **File:**
  `src/air_ground_coordinate_transform/test/test_registration_estimator.py:51-158,160-417`
- **Failure scenario:** the focused suite has no tests for mutation of returned
  state, nonmonotonic/negative batch stamps, near-singular SPD `S`, or prediction
  arithmetic overflow. It also tests only `threshold - 1e-9`, not exact threshold
  equality. Thus all 29 tests pass while I1-I3 remain reproducible.
- **Mathematical consequence:** singular-zero `S` is not representative of an
  ill-conditioned positive `S`; and an inside-only gate test does not freeze the
  intended `d^2 <= threshold` boundary convention. Correlated `P/R` and the full
  Joseph matrix are covered well by lines 282-330, but that does not exercise
  lifecycle or conditioning behavior.
- **Minimum fix:** add independent edge tests for every listed case, including
  exact equality (currently accepted by the deliberate `>` comparison), and
  assert complete state preservation on every numerical rejection.

### I5. The report's chronology does not establish strict RED-to-GREEN TDD

- **File:**
  `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-6-report.md:82-100,180-199`
- **Failure scenario:** line 100 states that the required 100-sequence test was
  added only after all production behavior was green, so it had no recorded RED
  against the implementation it was meant to drive. Line 85 describes removing
  production to manufacture a RED for the zero-growth test rather than recording
  an original test-first failure. The mutation observations are detailed and
  plausible, but are prose-only temporary edits and are not independently
  reproducible from a retained mutation command or artifact.
- **Reason:** passing tests and post-hoc mutation sensitivity demonstrate current
  behavioral discrimination, not the binding development chronology required by
  strict TDD.
- **Minimum fix:** correct the evidence claim and obtain an explicit process
  waiver, or redo the affected behavior under an auditable RED/GREEN workflow.
  Preserve repeatable mutation commands or a mutation harness for future claims.

## Minor Findings

### M1. A supplied prior does not satisfy the wrapped-yaw state contract

- **File:**
  `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py:80-99`
- **Failure scenario:** an initial yaw of `3*pi` remains `3*pi`, outside the
  `[-pi, pi)` API interval, until an accepted update. Prediction preserves that
  out-of-contract value and Task 7 could publish it directly.
- **Mathematical reason:** equivalent angles represent the same rotation, but the
  filter contract requires a canonical coordinate chart; residual wrapping does
  not canonicalize the stored prior.
- **Minimum fix:** initialize with `wrap_xyyaw(mean)` and add constructor and
  prediction range tests.

### M2. Finite validated prediction inputs can corrupt covariance with infinity

- **File:**
  `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py:113-150`
- **Failure scenario:** finite rates of `1e308` and finite `dt=2` pass validation,
  overflow both growth terms, and commit an all-infinite covariance.
- **Mathematical reason:** finiteness is not closed under floating-point
  multiplication/addition. Once `P` contains infinity, later statistical checks
  no longer provide a valid filter state.
- **Minimum fix:** compute growth and candidate covariance before assignment,
  require both to be finite and PSD, raise `ValueError`/`ArithmeticError` as the
  chosen programmer/numerical contract dictates, and leave state unchanged.

### M3. Covariance validity tolerances are absolute and scale-blind

- **File:**
  `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py:101-107,229-230`
- **Failure scenario:** the same `1e-12` tolerance can accept a materially negative
  eigenvalue for a very small covariance yet reject harmless roundoff asymmetry
  on a very large covariance. It is applied to a mixed-unit `[m,m,rad]`
  covariance without normalization.
- **Mathematical reason:** symmetry and PSD roundoff bounds should scale with the
  matrix norm/eigenvalue scale and machine epsilon. Symmetrizing the Joseph result
  is specified and appropriate, but the subsequent fixed absolute eigenvalue
  bound is not a conditioning model.
- **Minimum fix:** document and use a scale-aware symmetry/PSD tolerance (ideally
  after state scaling), while continuing to reject rather than eigenvalue-clamp
  invalid input covariances.

### M4. Production YAML does not label its numerical rates as provisional

- **File:**
  `src/air_ground_coordinate_transform/config/registration.yaml:24-45`
- **Failure scenario:** Task 7 can load the six concrete values directly, while
  only the separate report explicitly says they are not field-calibrated. A paper
  run can therefore record them as calibrated coefficients by mistake.
- **Reason:** units, physical meanings, constrained-regression procedure, freeze
  rule, and 3-DoF 99% gate are documented correctly, but calibration status and
  provenance are part of reproducibility too.
- **Minimum fix:** mark the current values `PROVISIONAL / NOT FIELD-CALIBRATED` in
  the YAML and require calibration dataset/version/date metadata before held-out
  execution. This is documentation hardening, not a demand to perform dynamic
  calibration in Task 6.

## Algorithm Audit

- The prediction implements all six variance rates exactly once with the stated
  units. It does not square rates again, moves no mean, preserves revision, gives
  exact zero growth for zero inputs, and retains every existing 3x3
  cross-covariance.
- Yaw innovation direction is correct: prior `+179 deg` to measurement `-179 deg`
  yields `+2 deg`. `wrap_angle` uses `[-pi, pi)`, and accepted posterior yaw is
  wrapped.
- NIS is correctly `nu.T @ solve(P+R, nu)`. The threshold is the 3-DoF 0.99
  chi-square quantile and the current `>` comparison intentionally accepts exact
  equality. No SciPy runtime dependency was added.
- The gain and Joseph update use complete 3x3 matrices; independent NumPy
  calculation reproduced the correlated fixture covariance and positive
  eigenvalues. The explicit inverse remains the numerical defect in I3.
- Uninitialized semantics are otherwise coherent: state is revision 0/stamp 0,
  prediction is inert, a valid first batch copies its covariance and initializes
  revision 1 without fake-prior gating, rejection leaves it uninitialized, and
  later accepted revisions increment once.
- The Monte Carlo comparison is fair within its declared matched Gaussian model:
  both filters share one truth walk and process schedule, prediction-only starts
  from the same exact prior, measurements occur every fourth step, and yaw errors
  are wrapped. Fresh execution reproduced 99/100 improved, component RMSEs
  `[0.02385568 m, 0.02138058 m, 0.00820835 rad]` versus
  `[0.14805080 m, 0.14953587 m, 0.03421932 rad]`, with 1979 accepted and 21
  rejected updates. The aggregate scalar mixes metres and radians, but the test
  labels it as a comparison diagnostic and retains component metrics. No
  per-seed tuning exists in code; preregistration of the chosen contiguous seed
  range cannot be proven from the unversioned workspace.

## Compatibility, Truth, And Residual Risk

- Static review found no Gazebo/experiment truth reference or offline alignment
  in production estimator code. Synthetic truth is confined to the pure test.
- The current `takeoff_registration.py` SHA-256 is
  `98348694b753fabe7765b7f20459fed88e89720c06071e064011c54f0a3d7828`, matching
  the report, and it does not consume `RegistrationFilter`. With no Git/history
  or earlier independent hash, historical byte-for-byte non-modification cannot
  be proven; this remains a manual audit residual, not a code finding.
- Real no-update interval calibration, held-out NIS coverage, ROS timing/frame
  validation, and dynamic one-shot compatibility were not performed. They remain
  correctly disclosed manual residual risks and are not counted as findings.

## Fresh Verification

- `python3 -m unittest ...test_registration_estimator.py -v`: 29/29 passed in
  3.132 s.
- Independent Monte Carlo summary: 99/100 improved; 1979 accepted, 21 rejected.
- Independent correlated-matrix calculation matched NIS/gain/Joseph covariance.
- `py_compile` for estimator, `se2.py`, and focused test: exit 0.
- YAML parse: all six rates and threshold are numeric floats.
- No ROS, simulator, truth source, topic wait, or long-running process was used.

---

## Fix Round 1 Scoped Re-Review

### Verdicts

- **Spec Compliance: FAIL**
- **Code Quality: FAIL**
- **Task 6 closure: BLOCKED** by one remaining Important numerical rejection
  path. Keep Task 7 paused.
- New fix-round findings: **0 Critical, 1 Important**.

### I1-I5 Dispositions

| Finding | Disposition | Re-review evidence |
|---|---|---|
| I1 state aliasing | **ADDRESSED** | `RegistrationFilter` stores `_state`; the read-only `state` property returns `_snapshot()` copies, `predict()` returns that property, and accepted/rejected results copy all arrays (`registration_estimator.py:74-95,107-128,172-179,208-220,337-366`). Tests mutate constructor inputs, current state snapshots, prediction results, accepted/rejected results, innovations, and old snapshots, and verify all five live state fields; public `state` assignment raises `AttributeError` (`test_registration_estimator.py:184-241,503-533`). No public array aliases the live prior. |
| I2 noncausal stamps | **ADDRESSED** | Nonfinite/negative stamps return `invalid_batch`; strict older stamps return `stale_batch`; equality is accepted (`registration_estimator.py:191-220`). Initialization and prediction advance stamps causally, while every rejection leaves all five state fields unchanged. Direct tests cover negative initialization, stale rejection, equal acceptance, initialization/prediction/accept/reject monotonicity, and full state records (`test_registration_estimator.py:535-601`). |
| I3 unusable `S` and inverse | **PARTIAL** | The filter update contains no explicit inverse. Diagonal equilibration, scaled eigenvalue ratio, one reused Cholesky factor, NIS, and gain algebra are correct (`registration_estimator.py:227-349`); 100 random full correlated 3x3 `P/R` cases matched an independent `solve` plus Joseph calculation. Tiny `diag(1,1e-320,1)` is safely accepted, and scaled correlation with reciprocal condition about `5e-11` is stably rejected. However, R1-NI1 below disproves the report's claim that every nonfinite intermediate returns a stable reason without exception. |
| I4 missing edge tests | **ADDRESSED** | The retained tests directly exercise all requested edges: constructor/state/predict/result/old-snapshot mutation, state rebinding, negative/stale/equal stamps, prediction overflow rollback, tiny usable SPD, unusable near-singular SPD, exact gate equality, and complete five-field preservation (`test_registration_estimator.py:184-241,503-658`). The exact boundary fixture uses `S=I`, `nu=[1,0,0]`, threshold `1`, so changing `>` to `>=` necessarily fails without rounding ambiguity. The newly discovered innovation-overflow path requires a separate regression under R1-NI1, but does not invalidate the listed I4 mutation evidence. |
| I5 TDD chronology | **ADDRESSED** | The report now explicitly labels round 0 as superseded and does not present it as original auditable TDD (`task-6-report.md:80-107`). It records the old-block eight-failure run, complete production-block deletion with all 27 retained filter/Monte Carlo tests RED, the unexpected rebuild failure and debugging rationale, subsequent GREEN runs, and exact-boundary mutation RED/GREEN (`task-6-report.md:280-390`). This is an honestly labeled fix-round reset rather than a fabricated claim about round-0 chronology. In an unversioned workspace the command transcript is the available audit artifact. |

### New Important Finding

#### R1-NI1. Innovation overflow can still escape as `FloatingPointError`

- **Files:**
  `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py:222-225`;
  `src/air_ground_coordinate_transform/test/test_registration_estimator.py:616-658`;
  `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-6-report.md:429-434,445-463`
- **Failure scenario:** an initialized finite state mean
  `[-1e308, 0, 0]` and finite batch mean `[1e308, 0, 0]` overflow at
  `innovation = measurement_mean - self._state.mean`. Under
  `np.errstate(all="raise")`, the update raises `FloatingPointError: overflow
  encountered in subtract` before reaching the finite check. The complete live
  state remains unchanged, but no `UpdateResult` or stable
  `singular_innovation_covariance` reason is returned.
- **Mathematical reason:** finite IEEE-754 operands do not imply a representable
  finite difference. Unlike the later `S`, scaling, solve, gain, correction, and
  Joseph operations, the innovation subtraction is outside the local controlled
  `errstate` and exception handler. The report's claim that all intermediate
  nonfinite paths map to a rejection is therefore too broad.
- **Why Important:** `update()` promises stable numerical rejection rather than
  an exception for unusable statistical intermediates. Caller/global NumPy error
  policy can currently change the API behavior, and the 40-test suite does not
  detect it.
- **Minimum fix:** compute innovation and yaw wrapping inside a local
  `np.errstate(over="raise", invalid="raise")` guarded by
  `FloatingPointError`/`OverflowError`, return
  `singular_innovation_covariance` on failure, and add a test using finite extreme
  means under `np.errstate(all="raise")` that asserts the reason and all five
  unchanged state fields.

### Regression And Algorithm Audit

- Fresh focused suite: **40/40 passed** in 4.814 s.
- Fresh full pure suite (`test_se2.py`, `test_odom_buffer.py`,
  `test_registration_estimator.py`): **61/61 passed** in 4.853 s.
- Fresh Monte Carlo: **99/100 improved**, 1979 accepted and 21 NIS-gated;
  component and aggregate RMSEs reproduce the implementation report.
- Independent random-matrix audit: 100 full correlated 3x3 `P/R` updates matched
  solve-derived gain, wrapped mean, and complete Joseph covariance.
- Prediction still applies every one of the six variance-rate terms exactly once,
  preserves full cross-covariance, and leaves mean/revision unchanged. Prediction
  overflow now raises before assignment and preserves complete state.
- Wrapped `+179 deg -> -179 deg` innovation remains `+2 deg`; posterior yaw is
  wrapped to `[-pi,pi)`. NIS remains `nu.T solve(S,nu)`, exact threshold equality
  is accepted, and the configured threshold remains the 3-DoF 99% chi-square
  quantile.
- The filter gain path has no explicit `inv`. The file still has a pre-existing
  geometry transform inverse at `registration_estimator.py:496`, unrelated to
  `S` or this fix; the report's "forbidden inverse" search should be understood as
  filter-update scoped.
- `py_compile` passed for estimator, `se2.py`, and the focused test. YAML parsing
  confirmed all seven filter parameters remain numeric.
- Current `takeoff_registration.py` SHA-256 remains
  `98348694b753fabe7765b7f20459fed88e89720c06071e064011c54f0a3d7828`; it has no
  `RegistrationFilter`, process/gate parameter, or truth-source reference. Static
  estimator review found no production truth leak. One-shot revision behavior
  remains covered in the 61-test pure suite.
- Deferred M1, M3, and M4 from round 0 remain Minor residuals. The prediction
  overflow part of M2 is fixed. Real calibration and dynamic ROS validation remain
  manual residual risks and are not promoted to findings.

### Closure Decision

Do **not** close Task 6 yet. Address R1-NI1 with a focused RED/GREEN regression,
rerun the bounded pure suites, then request another scoped re-review. Task 7 must
remain paused in the meantime.

---

## Fix Round 2 Scoped Re-Review

### Disposition

- **R1-NI1: ADDRESSED**
- New fix-round findings: **0 Critical, 0 Important**
- **Spec Compliance: PASS** for the Task 6 closure scope, with previously recorded
  Minor residuals still deferred.
- **Code Quality: PASS** for the Task 6 closure scope.
- **Task 6 may close at M2-A; pause before starting Task 7.**

### R1-NI1 Verification

- The innovation operation is now narrowly guarded at
  `registration_estimator.py:222-229`. Only subtraction and yaw wrapping execute
  under local `np.errstate(over="raise", invalid="raise")`; only
  `FloatingPointError` and `OverflowError` are translated to
  `singular_innovation_covariance`.
- Independent probes used finite prior/measurement pairs
  `[-1e308,0,0] -> [1e308,0,0]` and
  `[0,0,-1e308] -> [0,0,1e308]` under caller
  `np.errstate(all="raise")`. Both returned `accepted=False`, zero innovation,
  `NaN` Mahalanobis value, and reason `singular_innovation_covariance` without an
  exception. Mean, covariance, revision, stamp, and initialized were all exactly
  unchanged.
- The local context restores the caller's NumPy error policy. Its catch is not
  broad: an independently injected `TypeError` from `wrap_angle` propagated
  rather than being converted to a numerical rejection.
- The normal wrapped-yaw path is unchanged. Prior `+179 deg` and measurement
  `-179 deg` still produced a `+0.034906585039886195 rad` innovation and continued
  through the ordinary NIS/update path.

### Test Sensitivity

- The retained regression at `test_registration_estimator.py:485-513` covers
  translation-coordinate and yaw-coordinate overflow separately under caller
  `all="raise"`.
- It is behaviorally mutation-sensitive: without the production numerical catch,
  either subcase raises before returning and the test's explicit `self.fail`
  fires, matching the recorded Round 2 RED. Wrong acceptance, wrong reason,
  exposed partial innovation/NIS, changed returned prior, or mutation of any of
  the five internal state fields also fails direct assertions.
- Existing wrapped-yaw and invalid-programmer-input tests remain separate, so the
  numerical regression does not redefine normal yaw or validation behavior.

### Regression Audit

- Fresh focused estimator suite: **41/41 passed** in 4.672 s.
- Fresh full pure suite (`test_se2.py`, `test_odom_buffer.py`,
  `test_registration_estimator.py`): **62/62 passed** in 4.837 s.
- Fresh Monte Carlo: **99/100 improved**, 1979 accepted and 21 NIS-gated, with
  aggregate wrapped-state RMSE `0.019092753310491765` versus
  `0.12308665964084933` prediction-only.
- The complete correlated 3x3 gain/Joseph fixture, six-rate exact prediction,
  exact NIS boundary, scaled near-singular rejection, state snapshots, causal
  stamps, and one-shot revision test all remain green.
- `py_compile` passed for estimator, `se2.py`, and the focused test.
- `takeoff_registration.py` SHA-256 remains
  `98348694b753fabe7765b7f20459fed88e89720c06071e064011c54f0a3d7828`.
  Static production search found no truth reference; no ROS, simulator, bag,
  topic wait, or long-running process was used.

### Closure Decision

R1-NI1 is fully addressed and the fix introduces no new Critical or Important
finding. All Task 6 Critical/Important findings are closed. Task 6 may be closed
with the documented deferred Minors and manual calibration/dynamic-validation
residuals; stop at M2-A and keep Task 7 paused.

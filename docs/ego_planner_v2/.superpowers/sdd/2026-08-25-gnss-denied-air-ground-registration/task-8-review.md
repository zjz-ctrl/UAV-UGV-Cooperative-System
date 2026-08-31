# Task 8 Fresh Read-Only Review

Date: 2026-08-26

## Verdicts

- **Spec Compliance: FAIL**
- **Code Quality: FAIL**
- **Gate: enter Task 8 fix; do not enter coordinator verification yet**

The review found **9 findings: 1 Critical, 7 Important, and 1 Minor**. The
highest severity is **Critical**.

## Critical

### C1. The research mission uses legacy frame IDs against experimental-frame TF edges

- **File:line:** `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch:91-110`; related defaults at `src/air_ground_bringup/scripts/uav_sphere_mission.py:136-138` and registration-node parameters at `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch:82-83`.
- **Failure scenario:** The research launch remaps the mission's UAV and UGV odometry topics to `/air_ground_experiment/uav/odom` and `/air_ground_experiment/ugv/odom`, whose producers label poses `air_ground_experiment/uav_odom` and `air_ground_experiment/ugv_odom`. The registration node publishes its registered TF edges under those experimental frame IDs. The mission node is not passed either frame parameter, so it retains `iris_0/odom` and `ugv_0/odom`. `reregistration_command()` therefore asks for `ugv_0/odom -> air_ground_origin -> iris_0/odom`, while only the experimental-frame edges exist. RETURN cannot obtain a command and RESUME/legacy target publication cannot resolve the preserved target. A static extraction reproduced `mission_frames None None` versus registration frames `air_ground_experiment/uav_odom air_ground_experiment/ugv_odom`.
- **Root cause:** Frame parameters were set only on `takeoff_registration`, not on `uav_sphere_mission`, and the launch test validates the registration consumer but never derives and checks the mission's frame parameters.
- **Minimal fix:** Pass `uav_odom_frame="air_ground_experiment/uav_odom"` and `ugv_odom_frame="air_ground_experiment/ugv_odom"` to the research mission. Add producer-derived assertions that the mission frame parameters equal the actual perturbation destinations.

## Important

### I1. A HOLD decision cannot recover from a new registration covariance for the same preserved target

- **File:line:** `src/air_ground_bringup/scripts/uav_sphere_mission.py:210-216`, `src/air_ground_bringup/scripts/uav_sphere_mission.py:416-420`.
- **Failure scenario:** An invalid/unavailable covariance produces `HOLD`. A later finite, low covariance snapshot arrives while the same final target remains preserved and no new target sample arrives. `process_final_estimate()` sees the same target timestamp and only retries `DIRECT`; pending `HOLD` returns without reevaluating the policy. The bounded stub reproduction remained `FINAL_ESTIMATE/HOLD` after replacing NaN covariance with a direct-budget covariance.
- **Root cause:** Policy deduplication is keyed only by `preserved_target_stamp`; it has no registration-covariance version/value in the decision identity and no recovery path for pending `HOLD`.
- **Minimal fix:** Track the last evaluated target identity plus a fresh/changed estimate identity or covariance value. Reevaluate a pending `HOLD` when registration covariance meaningfully changes, while suppressing identical periodic estimate snapshots so each actual policy decision emits one action/radius event.

### I2. WAIT_REREGISTRATION advances on a revision even when rendezvous odometry/TF is missing

- **File:line:** `src/air_ground_bringup/scripts/uav_sphere_mission.py:712-720`.
- **Failure scenario:** During WAIT, `reregistration_command()` returns `None` because UGV odometry or either TF hop is unavailable. If a newer revision is present on the same tick, the code still enters `RESUME_HANDOFF`. The bounded reproduction with a missing transform and revision `3 -> 4` produced `RESUME_HANDOFF`, rather than a safe WAIT hold.
- **Root cause:** Missing-data command handling and revision handling are independent in the WAIT branch; the revision transition is not gated on a valid recomputed rendezvous command.
- **Minimal fix:** On a missing rendezvous command, hold the last command and remain in WAIT (while still enforcing timeout). Evaluate the newer-revision transition only after the current rendezvous command is valid.

### I3. Both timeout branches allow success observed after the deadline

- **File:line:** `src/air_ground_bringup/scripts/uav_sphere_mission.py:697-711`, `src/air_ground_bringup/scripts/uav_sphere_mission.py:712-722`.
- **Failure scenario:** At `elapsed > registration_move_timeout`, simultaneous arrival wins and enters WAIT because arrival returns before the timeout check. At `elapsed > reregistration_timeout`, a newer revision wins and enters RESUME because timeout is the `elif`. Bounded reproductions at 31 s and 61 s produced `WAIT_REREGISTRATION` and `RESUME_HANDOFF`, respectively.
- **Root cause:** Success predicates precede timeout predicates, and accepted-revision receipt time is not recorded.
- **Minimal fix:** Define deadline precedence explicitly and enforce it before success transitions. If events received just before a delayed timer tick must count, store the accepted-update receipt time and compare that event time with the WAIT deadline.

### I4. Mission callback state is not synchronized, so revision/baseline monotonicity and selected-window timestamp are not atomic

- **File:line:** `src/air_ground_bringup/scripts/uav_sphere_mission.py:210-229`, `src/air_ground_bringup/scripts/uav_sphere_mission.py:280-293`, `src/air_ground_bringup/scripts/uav_sphere_mission.py:350-351`, `src/air_ground_bringup/scripts/uav_sphere_mission.py:398-410`, `src/air_ground_bringup/scripts/uav_sphere_mission.py:677-693`.
- **Failure scenario:** ROS subscriber callbacks and the timer can run on different threads. Two accepted-update callbacks can interleave their compare/assign operations and write an older revision last. A revision can arrive after `phase` is exposed as WAIT but before `baseline_revision` is read, causing a post-entry event to be absorbed into the baseline. A nadir callback can append between `stable_target()`'s selected-window snapshot and `preserve_final_estimate()` reading `nadir_samples[-1]`, so the preserved timestamp need not belong to the final window that produced the target. The FINAL branch also preserves once before the disagreement check and then a second time inside `process_final_estimate()`, widening that race.
- **Root cause:** New Task 8 shared state has no lock or serialized event boundary; preservation reads mutable sample state instead of receiving the selected sample timestamp as part of the stable result.
- **Minimal fix:** Protect registration covariance/revision, phase entry/baseline capture, sample selection, and final preservation with one lock or equivalent serialized state adapter. Return the selected latest timestamp from `stable_target()`, preserve exactly once from that immutable result, and make accepted revision compare/assign atomic.

### I5. Exact PSD validation rejects a mathematically PSD covariance due to a tiny numerical eigenvalue

- **File:line:** `src/air_ground_bringup/src/air_ground_bringup/target_handoff.py:63-72`.
- **Failure scenario:** The rank-one PSD matrix `[[1.0, 0.1], [0.1, 0.01]]` has exact eigenvalues `0` and `1.01`, but NumPy returned `[-1.73472348e-18, 1.01]`. The policy rejected it and returned `HOLD` with NaN confidence. This is a numerical false HOLD, not a materially negative covariance. The same raw eigenvalue path could also raise an uncaught numerical linear-algebra exception on pathological finite input.
- **Root cause:** PSD validation requires computed `lambda_min >= 0.0` with no scale-aware numerical tolerance; accepted near-symmetric arrays are not symmetrized, and eigen/radius calculations have no numerical exception boundary or tiny-negative clamp.
- **Minimal fix:** Symmetrize after the material-symmetry check, reject eigenvalues below a small scale-aware negative tolerance, clamp accepted tiny negatives to zero for radius calculation, and convert `LinAlgError`/numeric failures to the required invalid `HOLD` state.

### I6. The mission imports `tf.transformations` without declaring the `tf` runtime dependency

- **File:line:** `src/air_ground_bringup/scripts/uav_sphere_mission.py:17-18`; `src/air_ground_bringup/package.xml:23-47`.
- **Failure scenario:** On a clean dependency-resolved installation where `tf2_ros` is present but ROS 1 `tf` is not independently installed, the mission fails during module import before `rospy.init_node()`/`Mission()` startup. The new NumPy and coordinate-message dependencies are declared, but the direct `tf.transformations` import is not.
- **Root cause:** `package.xml` declares `tf2_ros` but not the separately imported `tf` package.
- **Minimal fix:** Add the direct `tf` runtime dependency (and matching catkin dependency declarations if required by the package's dependency policy).

### I7. The tests bypass constructor/callback/timer integration and do not check mission frame forwarding or ROS message types

- **File:line:** `src/air_ground_bringup/test/test_reregistration_state_machine.py:68-94`, `src/air_ground_bringup/test/test_reregistration_state_machine.py:126-180`, `src/air_ground_bringup/test/test_launch_wiring.py:48-67`, `src/air_ground_bringup/test/test_launch_wiring.py:186-260`.
- **Failure scenario:** State tests AST-extract the class, allocate it with `__new__`, replace publishers/commands, and never execute imports, `Mission.__init__`, real subscriber bindings, timer construction, or callback concurrency. Launch tests verify the registration node's producer-derived frames but not the mission's frame parameters; they inspect topic literals but not subscriber/publisher message classes or callback bindings. All 9 launch tests and all 10 state tests pass while C1, I1, I2, and I3 are reproducible.
- **Root cause:** Method-level production reuse is useful but is treated as sufficient node integration coverage; the static launch contract stops at the registration node.
- **Minimal fix:** Retain focused method tests, and add one bounded full-module/full-constructor ROS-stub test that validates imports, parameters, topic types, callbacks, timer, and monotonic callback behavior. Extend launch tests to derive the mission's frame IDs from perturbation producers and add explicit HOLD recovery, WAIT missing-data-plus-revision, deadline-order, and selected-window timestamp cases.

## Minor

### M1. Catkin does not register the launch-wiring or legacy waypoint regression suites

- **File:line:** `src/air_ground_bringup/CMakeLists.txt:38-41`.
- **Failure scenario:** `catkin_make run_tests` registers only `test_target_handoff.py` and `test_reregistration_state_machine.py`. The Task 8 launch argument/default/forwarding checks and Task 3 waypoint/one-shot regressions run only when someone knows to invoke them manually, so normal package tests can miss launch or legacy regressions.
- **Root cause:** CMake registration was limited to the two newly created files even though Task 8 extends `test_launch_wiring.py` and relies on `test_registration_waypoint.py` as a required regression.
- **Minimal fix:** Register both existing suites under `CATKIN_ENABLE_TESTING`.

## Verified Compliant Behavior

- `UncertaintyBudget` uses the exact constants `5.991464547` and `1.959964`, computes the combined XY covariance before its largest eigenvalue, exposes all four numeric properties, handles shape/nonfinite/material-asymmetry/nonpositive-threshold cases as `HOLD`, preserves inclusive DIRECT equality, and implements the required yaw/target/registration precedence. I5 is the numerical PSD exception.
- Registration covariance maps all nine ROS covariance entries selected by axes `(0, 1, 5)`. The estimate callback does not change revision; the accepted callback uses only `RegistrationUpdate.revision` and does not overwrite covariance.
- The Task 7 producer publishes one atomic accepted update only on an accepted decision. Timer publication republishes continuous estimate/revision snapshots but does not manufacture accepted-update events. No production `header.seq` application use was found.
- DIRECT safely retries target TF resolution without dispatching early. REOBSERVE clears nadir samples and returns to sensing; REREGISTER preserves target mean, provisional covariance, timestamp, and handoff tuple. HOLD retains position/target and does not dispatch, subject to I1 recovery.
- RETURN uses latest UGV odometry pose/yaw, rotates body `dx/dy`, performs `ugv odom -> origin -> UAV odom`, commands registration altitude and legacy home heading, recomputes each tick, and checks inclusive XY/altitude/speed arrival. C1 prevents this chain in the actual research launch.
- WAIT captures the baseline in `set_phase()`, and sequential equal/older/newer behavior is correct. I4 covers the callback race; I2/I3 cover missing-data and timeout ordering.
- RESUME uses the preserved UAV-odom target and current TF through the existing two-output publication path. Both resolutions must succeed; no detector is called; preserved fields remain; UGV dispatch occurs only on the subsequent DISPATCH tick.
- Default opt-in is false, the research launch exposes and forwards all five Task 8 arguments, and the final demo explicitly sets false. Publisher/subscriber topics and production message types match the brief. Setup, NumPy runtime dependency, and the two new focused CMake tests are present.
- No scoped production truth input, Task 9 relay/controller behavior, `header.seq` use, or new TF broadcaster was found.

## Fresh Evidence

- `py_compile` over all scoped Python production/tests: exit 0.
- XML parse for `package.xml` and both scoped launches: `XML_OK 3`.
- Static mission audit: `header_seq 0`, `truth_inputs []`, `broadcasters 0`.
- Focused production policy suite: `8/8 OK`.
- Focused AST state suite: `10/10 OK`.
- Launch-wiring suite: `9/9 OK` despite C1.
- Pure/stub probes reproduced C1, I1, I2, I3, and I5 as described above.

## Residual Risk

Per the dynamic-execution prohibition, ROS launch resolution, actual callback scheduling, latched delivery, TF transport timing, simulator/vehicle behavior, and M2-C remain externally unverified. No `roslaunch`, `roscore`, `rostest`, Gazebo, PX4, RViz, rosbag, topic wait, truth read, Git operation, subagent, or long-running process was used. This is residual risk only; it is not an additional finding.

# Re-review Round1

Date: 2026-08-26

## Round1 Verdicts

- **Spec Compliance: FAIL**
- **Code Quality: FAIL**
- **Gate: do not enter coordinator verification; enter another Task 8 fix round**

Round1 leaves **3 findings: 0 Critical, 2 Important, and 1 Minor**. The highest
remaining severity is **Important**. Original C1 and I1-I3/I6-I7 are resolved.
I4 and I5 are substantially fixed, but the independent probes below found one
uncovered edge in each area. Original M1 remains recorded as Minor exactly as
directed and is not required to be fixed.

## Critical

No remaining Critical findings.

## Important

### R1-I1. A stable final estimate is not preserved before the disagreement error transition

- **File:line:** `src/air_ground_bringup/scripts/uav_sphere_mission.py:710-725`; related preservation path at `src/air_ground_bringup/scripts/uav_sphere_mission.py:419-434` and incomplete test at `src/air_ground_bringup/test/test_reregistration_state_machine.py:837-856`.
- **Failure scenario:** FINAL has a valid stable result, but its distance from `handoff_target_odom` exceeds `maximum_camera_disagreement`. The code enters `ERROR_COORDINATE` and returns at lines 719-723 before `process_final_estimate()` can preserve the result. A bounded probe instrumented `preserve_final_estimate()` and observed `ERROR_COORDINATE`, zero preservation calls, and `preserved_target_odom is None`. This violates the binding requirement to preserve a stable final estimate before any transition. The new exactly-once test covers only the successful publication path and therefore remains green.
- **Root cause:** Removing the original duplicate preservation moved the sole preservation call below the disagreement guard rather than separating preservation from policy evaluation.
- **Minimal fix:** In FINAL, preserve the immutable `(target, spread, selected_stamp)` exactly once immediately after `stable_target()` succeeds, then run disagreement/error handling and, only on success, evaluate/publish the already-preserved target without preserving it again. Add assertions for exactly one call on the normal path and exactly one call before the disagreement error transition.

### R1-I2. A late numerical exception leaves partially finite properties on an invalid HOLD budget

- **File:line:** `src/air_ground_bringup/src/air_ground_bringup/target_handoff.py:40-51`; incomplete exception test at `src/air_ground_bringup/test/test_target_handoff.py:150-158`.
- **Failure scenario:** Both covariance validations and the first radius calculation succeed, then a later `eigvalsh` call raises `LinAlgError` while calculating the target or combined radius. The constructor catches the exception and `choose_action()` safely returns `HOLD`, but already assigned properties are not reset. A staged bounded probe that raised on the fourth eigenvalue call produced `HOLD` with `registration_radius=2.447746830658759` while the other properties were NaN. Invalid/numerically failed input therefore does not leave all exposed evaluation properties nonfinite as required. The new test raises on the first call only, so it cannot detect partial assignment.
- **Root cause:** Radius fields are assigned incrementally inside the exception boundary, and the exception handler returns without restoring the initialized all-NaN invalid state.
- **Minimal fix:** Compute all four values in local variables and assign the public properties only after every numerical operation succeeds, or reset every property to NaN in every failure path. Extend the mocked failure test across validation, registration radius, target radius, and combined radius call positions.

## Minor

### M1. Catkin still does not register the launch-wiring or legacy waypoint regression suites

- **File:line:** `src/air_ground_bringup/CMakeLists.txt:39-42`.
- **Status:** Unchanged from the original review and explicitly deferred by scope. It remains Minor and is not required for this fix gate.

## Original Finding Revalidation

- **C1 resolved:** Actual perturbation destinations are `air_ground_experiment/uav_odom` and `air_ground_experiment/ugv_odom`; fresh XML extraction showed exact equality with both registration and mission `uav_odom_frame`/`ugv_odom_frame` parameters.
- **I1 resolved:** Independent production-method probes produced `HOLD -> DIRECT -> DISPATCH` for the same preserved target after covariance recovery; a pending DIRECT degraded to REREGISTER without a second TF resolution; covariance drift inside DIRECT emitted one action and one radius only.
- **I2 resolved:** Separate missing-UGV-odom, missing-first-TF-hop, and missing-second-TF-hop probes all remained in WAIT despite a newer revision, then entered `ERROR_REGISTRATION` after timeout.
- **I3 resolved:** At exact deadline equality, RETURN accepted arrival and WAIT accepted a newer revision. At one microsecond strictly late, both entered `ERROR_REGISTRATION` before success.
- **I4 mostly resolved:** Fresh AST inspection confirmed the same `state_lock` context in state/frozen/UAV odom/UGV odom/estimate/accepted-update/front/nadir callbacks, `set_phase`, final processing, and `tick`. Baseline capture and revision compare/assign are under the RLock. `stable_target()` snapshots a tuple and returns the selected tail timestamp. A bounded two-thread probe confirmed a callback waits while tick owns the lock and completes after the external call releases; no self-deadlock was observed. R1-I1 is the remaining preservation-order edge.
- **I5 mostly resolved:** Rank-one PSD and accepted near-symmetry produce finite DIRECT budgets; material negative values `-1e-12` and `-1e-8` are rejected, while the tiny `-1e-14` tolerance edge is clamped. Early `LinAlgError` becomes HOLD. R1-I2 is the remaining late-exception state edge.
- **I6 resolved:** `tf` is present in `find_package`, `catkin_package`, and direct package `<depend>` topology.
- **I7 resolved:** The added test loads the complete production script and executes full `Mission.__init__` under scoped module/ROS stubs. It verifies imported message identities, bound callbacks and owning instance, publishers, parameter defaults, and the 30 Hz timer. This is meaningful constructor integration rather than AST method extraction; actual ROS ABI/runtime availability remains a prohibited dynamic concern, not a local finding.

## Regression And Safety Revalidation

- The fresh four-suite run completed `52/52 OK`, including Task 3 waypoint behavior, legacy uncertainty-disabled direct dispatch, constructor integration, and Round1 focused tests. The two Important probes above demonstrate the remaining coverage gaps despite that GREEN result.
- `py_compile` for all six Round1 changed Python production/test files exited 0.
- Package and both launch XML files parsed successfully.
- Final demo remains explicitly `uncertainty_aware_handoff=false`; research default remains false and all five arguments remain forwarded.
- Static production audit found zero application `header.seq`, truth inputs, Task 9 relay tokens, or new TF broadcasters.
- `dispatch_goal()` has one production call site and remains only in the DISPATCH branch, so no UGV goal is published before DISPATCH.

## Round1 Residual Risk

The shared RLock intentionally covers bounded TF lookups and publisher/service
side effects. The bounded contention probe showed waiting callbacks resume after
the external call returns, and no synchronous self-callback/deadlock path was
found. Real ROS publisher transport, service stalls, callback scheduling, TF
timing, launch resolution, and M2-C remain external residual risks under the
dynamic prohibition. No ROS master, launch, rostest, simulator, truth read, Git
operation, subagent, or long-running process was used.

# Re-review Round2

Date: 2026-08-26

## Round2 Verdicts

- **Spec Compliance: FAIL**
- **Code Quality: FAIL**
- **Gate: do not enter coordinator verification; one scoped Task 8 numerical fix remains**

Round2 leaves **2 findings: 0 Critical, 1 Important, and 1 Minor**. The highest
remaining severity is **Important**. R1-I1 is resolved. R1-I2 is resolved for
validation, all radius stages, yaw, and nonfinite results, but not for an
exception raised by the final finite-check stage. M1 remains Minor and deferred
without scope expansion.

## Critical

No remaining Critical findings.

## Important

### R2-I1. A staged exception in the final finite check still escapes instead of producing HOLD plus four NaNs

- **File:line:** `src/air_ground_bringup/src/air_ground_bringup/target_handoff.py:40-58`; incomplete staged test at `src/air_ground_bringup/test/test_target_handoff.py:160-213`.
- **Failure scenario:** Validation, registration/target/combined radius, and yaw calculations all succeed. On the third `math.isfinite` call (the first local-result finite check after the two threshold checks), a staged `FloatingPointError` escapes the constructor. The bounded reproduction printed `finite_check_exception FloatingPointError finite-check calls 3`; no `UncertaintyBudget` instance is returned to report `HOLD`. This contradicts the Round2 requirement that a failure at any late numerical/finite-check stage produce `HOLD` with all four public properties NaN.
- **Root cause:** The numerical `try/except` ends at line 51, while the local-result `math.isfinite` loop is outside it at lines 52-58. The new staged test injects eigensolve and yaw-square-root failures but does not inject an exception from the finite-check operation itself.
- **Minimal fix:** Move the all-finite gate into the same numerical exception boundary, or wrap it with the same exception conversion before committing locals. Extend the test with a staged first-local finite-check exception and retain the existing nonfinite-return case; both must leave `HOLD` and four NaNs.

## Minor

### M1. Catkin still does not register the launch-wiring or legacy waypoint regression suites

- **File:line:** `src/air_ground_bringup/CMakeLists.txt:39-42`.
- **Status:** Unchanged, explicitly deferred, and not required to be fixed in this scope.

## R1-I1 Revalidation

- FINAL obtains one immutable `(target, spread, selected_stamp)` from `stable_target()` and immediately preserves that exact object once.
- Independent normal and disagreement probes both recorded `preserve` as the first relevant event. Normal FINAL then published the preserved target and entered DISPATCH; disagreement then entered `ERROR_COORDINATE` with preserved mean/covariance/stamp intact.
- `process_final_estimate()` now accepts no sample result and delegates only to `_process_preserved_target_locked()`, so current-covariance retry cannot preserve again. A bounded retry probe recorded zero extra preservation calls and `HOLD -> DIRECT -> DISPATCH` on the same preserved target.
- DIRECT, HOLD, REOBSERVE, and REREGISTER focused behavior remains green, including pending-DIRECT degradation and same-action publication suppression.

## R1-I2 Revalidation

- Staged `LinAlgError` at covariance validation and registration, target, or combined radius calls 1 through 5 each produced `HOLD` and four NaNs.
- Staged yaw `FloatingPointError` produced `HOLD` and four NaNs.
- A late local `inf` returned from combined radius was rejected by the finite gate with `HOLD` and four NaNs.
- Rank-one PSD, accepted near-symmetry, tiny-negative clamp, materially negative rejection, exact formula, and action behavior remain correct in fresh probes.
- Production does not mutate covariance inputs. Mutating both caller arrays after construction left all budget properties/action unchanged, confirming no retained alias.
- R2-I1 is the only remaining scoped numerical failure.

## Regression And Safety Revalidation

- Fresh focused policy/state suites: `34/34 OK`.
- Fresh Task 3 waypoint and launch-wiring suites: `20/20 OK`.
- `py_compile` for all four Round2 modified production/test files exited 0.
- Legacy default remains `uncertainty_aware_handoff=false`.
- Static production audit found zero application `header.seq`, truth/Task 9 inputs, or TF broadcasters.
- `dispatch_goal()` remains a single call in the DISPATCH branch, so no goal is published early.

## Round2 Residual Risk

Dynamic ROS/TF timing and M2-C remain externally unverified under the explicit
prohibition. No ROS master, launch, rostest, simulator, truth read, Git operation,
subagent, or long-running process was used.

# Re-review Round3

Date: 2026-08-26

## Round3 Verdicts

- **Spec Compliance: PASS**
- **Code Quality: PASS WITH DEFERRED MINOR**
- **Gate: may enter coordinator verification**

Round3 leaves **1 finding: 0 Critical, 0 Important, and 1 Minor**. The highest
remaining severity is **Minor**. R2-I1 is resolved. The only remaining item is
M1, which was explicitly deferred and does not block this gate.

## Critical

No remaining Critical findings.

## Important

No remaining Important findings.

## Minor

### M1. Catkin still does not register the launch-wiring or legacy waypoint regression suites

- **File:line:** `src/air_ground_bringup/CMakeLists.txt:39-42`.
- **Status:** Unchanged, explicitly deferred, and non-blocking for coordinator verification.

## R2-I1 Revalidation

- After the two threshold finite checks succeed, staged `FloatingPointError`, `ValueError`, and `OverflowError` at each of the four local-result `math.isfinite` positions all returned a constructed invalid budget with `HOLD` and four NaNs.
- Nonfinite registration-radius, target-radius, combined-radius, and yaw results each returned `HOLD` with four NaNs.
- The all-local finite gate is inside the same supported numerical exception boundary as covariance validation, three radius calculations, and yaw. Public properties and `_valid=True` are committed only after that gate completes.
- Staged covariance-validation, registration-radius, target-radius, combined-radius, and yaw failures remain `HOLD` with four NaNs.
- Valid exact formula, sum-before-eigen, equality, action precedence, rank-one PSD, accepted near-symmetry, tiny-negative clamp, and materially negative rejection remain correct in independent probes.
- The Round3 test covers the first local finite-check exception and nonfinite return. Independent review additionally exercised every later local finite-check position and every supported exception type.

## Fresh Evidence

- Focused policy suite: `13/13 OK`.
- `py_compile` for `target_handoff.py` and `test_target_handoff.py`: exit 0.
- Static constructor inspection found each public confidence property assigned only at initialization and at the post-check commit; `_valid=True` is set only after those assignments.
- Static production audit remains zero for application `header.seq`, truth/Task 9 inputs, and TF broadcasters.
- `dispatch_goal()` remains a single call in the DISPATCH branch; no early goal path was introduced.

## Round3 Residual Risk

Dynamic ROS/TF timing and M2-C remain externally unverified under the explicit
prohibition. No ROS master, launch, rostest, simulator, truth read, Git operation,
subagent, or long-running process was used.

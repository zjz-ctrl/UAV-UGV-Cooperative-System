# Task 8 Report: Uncertainty-Triggered UAV Re-registration

Date: 2026-08-26

Status: `DONE_WITH_CONCERNS`. All allowed pure/stub, compile, parse, static
safety, and bounded build gates pass. Dynamic M2-C remains deliberately
unexecuted under the environment ruling.

## Scope

Implemented only Task 8:

- Pure deterministic `UncertaintyBudget` policy with `DIRECT`, `REOBSERVE`,
  `REREGISTER`, and `HOLD`.
- Separate continuous registration covariance and atomic accepted revision
  mission inputs.
- Opt-in mission flow `FINAL_ESTIMATE -> RETURN_TO_UGV ->
  WAIT_REREGISTRATION -> RESUME_HANDOFF -> DISPATCH`.
- Complete UAV-odom target preservation and current-TF re-resolution.
- Research launch opt-in and final-demo explicit opt-out.
- Catkin Python packaging, NumPy runtime dependency, and focused tests.

Not implemented: Task 9 target-handoff ROS node, common-frame covariance relay,
goal-tracking controller, anomaly covariance publication, truth input, or a new
TF broadcaster.

## Recovery Evidence

The coordinator-confirmed interruption checkpoint was retained rather than
restarted:

- Existing pure policy suite: fresh `7/7 GREEN` at recovery.
- Registration-input state suite: expected `2/2 RED`, both failures naming the
  missing `registration_estimate_callback` and
  `accepted_registration_callback`.
- Mission, launch files, CMake/package, setup, and report had no other Task 8
  implementation at recovery.

The policy suite was later strengthened to eight tests without repeating its
original RED cycle.

## TDD Evidence

### Cycle 1: Pure Policy

RED command:

```bash
python3 -m unittest -v src/air_ground_bringup/test/test_target_handoff.py
```

Observed RED before policy production code: seven tests ran with 14 assertion
failures (including subtests), all reporting
`air_ground_bringup.target_handoff policy is missing`.

Initial GREEN command was identical. Result: `Ran 7 tests`, `OK` after one
fixture-literal correction. The correlated covariance expected value was
independently recomputed with:

```bash
python3 -c "from decimal import Decimal, getcontext; getcontext().prec=40; print((Decimal('5.991464547') * Decimal('0.07')).sqrt())"
```

Output:

```text
0.6476129386369608350800743997598255687777
```

The mutation review added an orthogonal-axis combined-covariance case and exact
equality threshold. Its first hand-derived combined literal was also corrected
after this independent command:

```bash
python3 -c "from decimal import Decimal, getcontext; getcontext().prec=40; print((Decimal('5.991464547') * Decimal('0.05')).sqrt()); print((Decimal('5.991464547') * Decimal('0.04')).sqrt())"
```

Output:

```text
0.5473328305062651730060624854178230278478
0.4895493661317518241690884518764932892052
```

Final policy GREEN: `Ran 8 tests in 0.003s`, `OK`.

### Cycle 2: Registration Inputs

Recovered RED command:

```bash
python3 -m unittest -v src/air_ground_bringup/test/test_reregistration_state_machine.py
```

Observed expected RED: `Ran 2 tests`, `FAILED (failures=2)`. The failures were:

```text
mission is missing the continuous registration estimate callback
mission is missing the atomic accepted-registration callback
```

Minimal implementation added the ROS `(0, 1, 5)` 3x3 covariance extraction and
monotonic `message.revision` callback. GREEN command was identical. Result:
`Ran 2 tests in 0.017s`, `OK`.

Evidence protected by this cycle:

- A continuous estimate changes all nine selected covariance entries but leaves
  revision unchanged.
- An accepted update changes revision but cannot overwrite the continuous
  covariance.
- `header.seq=999` with `revision=7` produces revision 7, and a later
  `header.seq=1000` with older revision 6 is ignored.

### Cycle 3: Mission State

RED command:

```bash
python3 -m unittest -v src/air_ground_bringup/test/test_reregistration_state_machine.py
```

Observed clean state RED after retaining the two input GREENs: `Ran 10 tests`,
`FAILED (failures=6)`. Failures identified the missing dynamic rendezvous,
final-estimate policy handler, exact return/wait transition, and both timeout
paths. Four existing negative/safe-hold behaviors already passed.

Minimal state implementation then produced one harness failure: the exact-flow
fixture had started artificially in `RETURN_TO_UGV`, so no preserved target
existed and the production resume guard correctly held. The fixture was changed
to enter `RETURN_TO_UGV` through the real registration-dominated final policy.

Final Cycle 3 GREEN: `Ran 10 tests in 0.117s`, `OK`.

Behavioral evidence:

- Dynamic rendezvous transformed `(9, 22, 0)` from UGV odom to `(109, 222, 0)`
  in origin and then `(99, 202, 0)` in UAV odom. After changing UGV pose/yaw,
  the next calculation used `(22, 31, 0)` and resolved `(112, 211, 0)`, proving
  no cached waypoint.
- A revision arriving during `RETURN_TO_UGV` became the WAIT-entry baseline.
  Equal and older values held; only a strictly greater revision advanced.
- XY, altitude, and inclusive `speed <= 0.15` are all required for arrival.
- Move timeout and re-registration timeout independently enter
  `ERROR_REGISTRATION`.
- Missing UGV odometry or either TF hop holds without a goal.
- Resume calls only the existing target publication path; the test makes target
  detection fail if invoked.
- No UGV goal callback occurs before the next tick in `DISPATCH`.
- `DIRECT`, `REOBSERVE`, `REREGISTER`, `HOLD`, and disabled legacy behavior have
  distinct mutation-sensitive outcomes.

### Cycle 4: Packaging And Wiring

RED command:

```bash
python3 -m unittest -v src/air_ground_bringup/test/test_launch_wiring.py
```

Observed RED: `Ran 9 tests`, `FAILED (failures=8)`. The new contracts failed for
missing mission defaults/topics, missing `setup.py`/CMake test registration,
five missing research arguments, and final-demo missing explicit opt-out.

Minimal wiring implementation added the package/setup contract, runtime
dependency, subscriptions/publications, launch arguments, and explicit final
opt-out. GREEN command was identical. Result: `Ran 9 tests in 0.028s`, `OK`.

## Formula And Boundaries

The production policy uses exactly:

```text
r95(Pxy) = sqrt(5.991464547 * lambda_max(Pxy))
yaw95 = 1.959964 * sqrt(P_registration[2,2])
combined_radius = r95(P_registration_xy + P_target_xy)
```

Validation requires registration shape 3x3, target shape 2x2, finite entries,
material symmetry (`rtol=1e-7`, `atol=1e-10`), PSD eigenvalues, and finite
strictly positive meter/radian thresholds. Any invalid/unbounded input yields
`HOLD`, with all four exposed confidence properties set to `NaN`.

Action order is:

1. Invalid: `HOLD`.
2. `combined_radius <= inspection_radius` and `yaw95 <= inspection_yaw`:
   `DIRECT`.
3. Yaw excess: `REREGISTER`.
4. Planar excess and `target_radius > registration_radius`: `REOBSERVE`.
5. Planar excess and registration radius equal/larger: `REREGISTER`.

The equality test uses literal radius `0.2447746830658759` and yaw
`0.01959964`, catching `<` substitutions. The orthogonal-axis test distinguishes
`r95(Preg + Ptarget) = 0.5473328305` from either separate radius
`0.4895493661`.

## State And Preservation

- Registration covariance starts unbounded (`NaN`) and can only be replaced by
  `/air_ground/registration/estimate` continuous snapshots.
- Registration revision starts at zero and can only increase from explicit
  `RegistrationUpdate.revision`; `Header.seq` is never read.
- `set_phase("WAIT_REREGISTRATION")` atomically captures the current accepted
  revision as the baseline. Therefore a revision received while returning is
  part of the baseline and cannot satisfy WAIT.
- Every return/wait tick reads latest UGV odometry, rotates `registration_dx/dy`
  by UGV body yaw, and resolves the point through `ugv_odom ->
  air_ground_origin -> uav_odom`.
- Stable target preservation includes UAV-odom mean tuple, isotropic
  `max(observed_final_spread, target_sigma_floor)^2` 2x2 covariance, latest
  selected timestamp, and current handoff tuple.
- Resume retains all preserved fields and calls `publish_final_target()` so both
  origin and UGV resolutions must succeed before `DISPATCH`.
- Policy action and combined radius publish once per new target decision. A
  repeated HOLD timer tick does not create another event; a pending DIRECT may
  retry TF resolution without republishing the decision.
- Disabled mode bypasses the policy and preserves legacy immediate target
  publication/dispatch behavior.

## Modified Files

- `src/air_ground_bringup/setup.py` (new)
- `src/air_ground_bringup/src/air_ground_bringup/__init__.py` (new)
- `src/air_ground_bringup/src/air_ground_bringup/target_handoff.py` (new)
- `src/air_ground_bringup/test/test_target_handoff.py` (new)
- `src/air_ground_bringup/test/test_reregistration_state_machine.py` (new)
- `src/air_ground_bringup/scripts/uav_sphere_mission.py`
- `src/air_ground_bringup/test/test_launch_wiring.py`
- `src/air_ground_bringup/test/test_registration_waypoint.py` (NumPy supplied to
  its existing AST initializer harness)
- `src/air_ground_bringup/CMakeLists.txt`
- `src/air_ground_bringup/package.xml`
- `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch`
- `src/air_ground_bringup/launch/air_ground_final_demo.launch`

## Final Command Results

### Focused And Task 3 Regressions

```bash
python3 -m unittest -v \
  src/air_ground_bringup/test/test_target_handoff.py \
  src/air_ground_bringup/test/test_reregistration_state_machine.py \
  src/air_ground_bringup/test/test_registration_waypoint.py \
  src/air_ground_bringup/test/test_launch_wiring.py
```

First combined run: `Ran 37 tests`, one error in the pre-existing Task 3 AST
harness because the newly selected `registration_covariance` assignment needed
NumPy in its namespace. No production change was made for this harness issue.

Fresh rerun after the harness correction:

```text
Ran 37 tests in 0.241s
OK
```

Final focused-only rerun:

```bash
python3 -m unittest -v \
  src/air_ground_bringup/test/test_target_handoff.py \
  src/air_ground_bringup/test/test_reregistration_state_machine.py
```

```text
Ran 18 tests in 0.134s
OK
```

### Task 7 Pure/Stub Regressions

Commands used `PYTHONPATH="src/air_ground_coordinate_transform/src"`:

```text
test_registration_coordinator.py: Ran 19 tests in 0.028s, OK
test_registration_node_adapter.py: Ran 11 tests in 0.015s, OK
test_ugv_coordinate_monitor.py: Ran 4 tests in 0.003s, OK
```

Total permitted Task 7 pure/stub regression result: `34/34 OK`.

An additional direct invocation of `test_registration_node.py` was attempted and
failed before collecting tests with `ModuleNotFoundError: No module named
'rospy'`. That file is not a pure/stub suite in this unsourced shell, so it was
not rerun or represented as a product failure. No ROS master or process started.

### Compilation

```bash
python3 -m py_compile \
  src/air_ground_bringup/setup.py \
  src/air_ground_bringup/src/air_ground_bringup/__init__.py \
  src/air_ground_bringup/src/air_ground_bringup/target_handoff.py \
  src/air_ground_bringup/scripts/uav_sphere_mission.py \
  src/air_ground_bringup/test/test_target_handoff.py \
  src/air_ground_bringup/test/test_reregistration_state_machine.py \
  src/air_ground_bringup/test/test_registration_waypoint.py \
  src/air_ground_bringup/test/test_launch_wiring.py
```

Result: exit 0, no output.

### XML And Static Safety

XML/package parse result:

```text
XML_OK=3
```

Static mission audit result:

```text
STATIC_SAFETY_OK truth_topics=0 header_seq=0 broadcasters=0
```

The audit asserted absence of `/gazebo/get_model_state`,
`/gazebo/model_states`, application `header.seq`, and `TransformBroadcaster` in
the mission.

### Bounded Build

```bash
timeout 120s catkin_make --pkg air_ground_bringup -j2
```

Result: exit 0. Catkin configured with testing enabled, found
`/usr/bin/nosetests3`, generated the bringup package, and installed the
devel-space `uav_sphere_mission.py` wrapper. Existing workspace warnings included
missing optional VTK executable targets, disabled PCL pcap/png/libusb features,
deprecated Gazebo classic packages, and unrelated Eigen/system-lib export
warnings; none stopped configuration or the requested make.

## Self-Review

- Largest eigenvalue, covariance sum-before-eigenvalue, exact constants,
  inclusive equality, invalid input, yaw precedence, and equal-dominance branch
  each have an independently literalized test.
- Continuous predicted covariance and atomic revision are separate callbacks;
  accepted update pose is deliberately ignored and no `header.seq` application
  read exists.
- WAIT baseline capture occurs only on actual phase entry. A revision arriving
  during RETURN is baseline, not completion.
- Return/wait rendezvous is recomputed from current UGV odometry every tick via
  exactly two requested transform hops; missing odometry or either hop safely
  holds.
- Target mean, provisional covariance, source timestamp, and handoff tuple are
  preserved. Resume performs TF re-resolution only and cannot call detection.
- `DIRECT`, `REOBSERVE`, `REREGISTER`, and `HOLD` effects are distinct, and one
  policy event is emitted per target decision.
- `dispatch_goal()` remains reachable only in the `DISPATCH` tick branch.
- Research and mission defaults are false; final-demo explicitly writes false.
- No truth input, Task 9 behavior, common-frame canonical covariance relay,
  target-handoff ROS node, controller replacement, or second TF broadcaster was
  added.

## Manual M2-C Procedure (Written Only, Not Run)

Start a clean external ROS/simulator session and run the research launch with
opportunistic registration and Task 8 explicitly enabled:

```bash
roslaunch air_ground_bringup air_ground_inspection_experiment.launch \
  registration_mode:=opportunistic \
  uncertainty_aware_handoff:=true \
  inspection_radius:=0.35 \
  inspection_yaw:=0.03490658503988659 \
  target_sigma_floor:=0.02 \
  reregistration_timeout:=60.0 \
  trial_id:=m2-c-manual-0001 \
  output_directory:=/tmp/air_ground_experiments/m2-c-manual-0001
```

Observe in separate terminals:

```bash
rostopic echo /air_ground/handoff/action
rostopic echo /air_ground/handoff/confidence_radius
rostopic echo /air_ground/mission_phase
rostopic echo /air_ground/registration/accepted_update
rostopic echo /air_ground/registration/estimate
rostopic echo /air_ground/red_sphere/origin_point
rostopic echo /air_ground/red_sphere/ugv_odom_point
rostopic echo /air_ground/ugv_goal
```

Manual success criteria:

- The estimate covariance evolves continuously without manufacturing accepted
  revisions; accepted-update revisions are explicit and monotonic.
- A registration-dominated/yaw-excess decision emits one `REREGISTER` and one
  confidence radius, then phases appear in exact order `RETURN_TO_UGV`,
  `WAIT_REREGISTRATION`, `RESUME_HANDOFF`, `DISPATCH`.
- The return setpoint tracks the latest stopped UGV pose and uses the configured
  body-relative offsets and registration altitude.
- No equal/older revision exits WAIT. A revision received before WAIT entry is
  included in baseline. Only a later strictly newer accepted revision exits.
- No `/air_ground/ugv_goal` message appears before `DISPATCH`.
- Resume republishes both target frame resolutions from the preserved UAV-odom
  target without new sphere sensing; then the UGV goal may publish.
- Missing TF/odom causes a hold, and either configured timeout produces
  `ERROR_REGISTRATION` without a goal.
- A direct-budget trial emits `DIRECT`; target-dominated excess emits
  `REOBSERVE`; invalid covariance emits `HOLD`; each event occurs once per final
  target decision.
- The final-demo launch remains one-shot/legacy with uncertainty handoff false.
- No autonomy node subscribes to Gazebo/world truth, and no duplicate TF edge is
  broadcast.

## Concerns

- Dynamic ROS, TF timing, launch resolution, simulator, and physical M2-C
  behavior remain externally unverified by explicit ruling.
- Task 8 target covariance is intentionally provisional and isotropic; Task 9
  still owns unbiased sensor/sample covariance and common-frame relay.
- The direct non-stub Task 7 node test cannot import `rospy` in the unsourced
  verification shell; the permitted coordinator, node-adapter, and monitor
  regressions all pass.

## Review Fix Round 1

Date: 2026-08-26

Review source: `task-8-review.md`. C1 and I1-I7 were independently checked
against the current code and confirmed. M1 remains explicitly deferred and was
not fixed in this round.

### Finding Dispositions

| Finding | Disposition | Evidence |
|---|---|---|
| C1 research frame mismatch | Fixed | Research mission now receives producer-contract UAV/UGV experimental frame IDs; the launch test derives both from perturbation producers and compares mission and registration consumers |
| I1 same-target covariance reevaluation | Fixed | Preserved targets are evaluated against current continuous covariance on every pending FINAL tick; policy publications occur only for a new target identity or action transition |
| I2 WAIT missing-command revision advance | Fixed | WAIT returns after last-command hold when current UGV odometry/either TF hop cannot produce a rendezvous; timeout still runs first |
| I3 deadline precedence | Fixed | Strictly expired move/wait deadlines enter `ERROR_REGISTRATION` before arrival/revision checks; exact deadline equality still permits success |
| I4 unsynchronized mission state/window timestamp | Fixed | One `threading.RLock` serializes callbacks, phase/baseline, sample append/selection, preservation, and tick transitions; selected timestamp is returned in the immutable stable result and FINAL preserves once |
| I5 numerical PSD false HOLD | Fixed | Material symmetry is checked before symmetrization; scale-aware tiny-negative tolerance, nonnegative clamp, and numerical exception conversion are applied |
| I6 undeclared direct `tf` dependency | Fixed | `tf` is a catkin component/export and package direct dependency |
| I7 missing full constructor integration | Fixed as test coverage | Added bounded full-script import/full-`Mission.__init__` ROS-stub integration; no additional production change was needed after C1/I1-I6 |
| M1 CMake registration breadth | Deferred as directed | Existing CMake test registration was not broadened |

### C1 RED/GREEN

Focused command:

```bash
python3 -m unittest -v \
  src.air_ground_bringup.test.test_launch_wiring.LaunchWiringTest.test_research_registration_frames_follow_actual_perturbation_producers
```

RED:

```text
Ran 1 test in 0.002s
FAILED (failures=1)
AssertionError: None != 'air_ground_experiment/uav_odom'
```

The test obtains `destination_frame` from the actual UAV/UGV perturbation nodes,
then compares those values to both registration and mission frame parameters. It
does not use isolated duplicate expected constants.

GREEN:

```text
Ran 1 test in 0.002s
OK
```

### I1 RED/GREEN

Method-level covariance transition command:

```bash
python3 -m unittest -v \
  src.air_ground_bringup.test.test_reregistration_state_machine.ReregistrationStateMachineTest.test_same_target_hold_recovers_when_continuous_covariance_becomes_direct \
  src.air_ground_bringup.test.test_reregistration_state_machine.ReregistrationStateMachineTest.test_pending_direct_is_rechecked_before_tf_retry_dispatch \
  src.air_ground_bringup.test.test_reregistration_state_machine.ReregistrationStateMachineTest.test_same_target_and_action_suppress_duplicate_policy_events
```

RED: `Ran 3 tests`, two failures. HOLD published only `['HOLD']` instead of
`['HOLD', 'DIRECT']`; degrading pending DIRECT published only `['DIRECT']`
instead of `['DIRECT', 'REREGISTER']`. Identical-action suppression already
passed.

GREEN:

```text
Ran 3 tests in 0.026s
OK
```

Timer-level stale-window command:

```bash
python3 -m unittest -v \
  src.air_ground_bringup.test.test_reregistration_state_machine.ReregistrationStateMachineTest.test_tick_reevaluates_preserved_target_after_sample_window_stales
```

RED: `Ran 1 test`, one failure; stale sample window left publications at
`['HOLD']`. GREEN: `Ran 1 test in 0.020s`, `OK`, covering both HOLD-to-DIRECT
recovery and pending-DIRECT-to-REREGISTER degradation through real `tick()`.

Event semantics after the fix:

- Current registration covariance is used each pending FINAL tick even if
  `stable_target()` no longer has a fresh window.
- Decision identity is preserved target timestamp/mean/target covariance/handoff
  plus the resulting action transition.
- Identical target/action suppresses action and confidence publications even if
  continuous covariance drifts within the same action region.
- Pending DIRECT reevaluates policy before each TF retry. It may retry target TF
  without a duplicate decision event, but degradation changes action and blocks
  stale-safe dispatch.
- HOLD-to-DIRECT and any other action boundary crossing publish one new action
  and radius. REOBSERVE/REREGISTER immediately leave FINAL and therefore remain
  one-transition actions.

### I2/I3 RED/GREEN

Focused command:

```bash
python3 -m unittest -v \
  src.air_ground_bringup.test.test_reregistration_state_machine.ReregistrationStateMachineTest.test_wait_requires_current_rendezvous_before_accepting_new_revision \
  src.air_ground_bringup.test.test_reregistration_state_machine.ReregistrationStateMachineTest.test_deadline_expiry_precedes_late_arrival_or_revision \
  src.air_ground_bringup.test.test_reregistration_state_machine.ReregistrationStateMachineTest.test_exact_deadline_boundary_still_allows_success
```

RED: `Ran 3 tests`, two failures. Missing TF plus a newer revision produced
`RESUME_HANDOFF`; arrival at 31 seconds produced `WAIT_REREGISTRATION`. The
exact-boundary case already passed.

GREEN:

```text
Ran 3 tests in 0.055s
OK
```

The timer safety ruling is now explicit: `elapsed > deadline` errors first;
`elapsed == deadline` may still accept arrival/revision. WAIT only compares
revision after obtaining and publishing a valid current rendezvous command.

### I4 RED/GREEN

Focused command:

```bash
python3 -m unittest -v \
  src.air_ground_bringup.test.test_reregistration_state_machine.ReregistrationStateMachineTest.test_stable_window_timestamp_remains_canonical_after_later_append \
  src.air_ground_bringup.test.test_reregistration_state_machine.ReregistrationStateMachineTest.test_final_tick_preserves_selected_result_exactly_once \
  src.air_ground_bringup.test.test_reregistration_state_machine.ReregistrationStateMachineTest.test_task8_callbacks_phase_and_tick_enter_one_shared_lock
```

RED: `Ran 3 tests`, three failures: stable result length `2 != 3`, two
preservation calls instead of one, and shared-lock entry count `0`.

GREEN:

```text
Ran 3 tests in 0.026s
OK
```

Lock semantics:

- A single `threading.RLock` is created before mission state initialization.
- State/frozen/UAV odom/UGV odom/registration callbacks and both sample callbacks
  enter it before reading or mutating shared mission state.
- Accepted revision compare/assign is entirely inside the lock.
- `set_phase()` holds the same lock while phase, start time, and WAIT baseline
  are changed, so a post-entry accepted update cannot be absorbed into baseline.
- `tick()` and final processing use the same reentrant boundary. Sample append
  and `stable_target()` selection therefore cannot interleave.
- `stable_target()` snapshots an immutable tuple and returns `(mean, spread,
  latest_selected_stamp)`. Preservation consumes that stamp and never reads the
  mutable deque tail. FINAL invokes preservation exactly once.
- Publisher and bounded TF lookup side effects remain inside the serialized tick
  to preserve one coherent state snapshot. `RLock` permits same-thread phase
  publication/reentry; other subscriber callbacks may wait for the existing
  0.1-second TF lookup bound. No mission publisher is known to synchronously call
  back into this object.

### I5 RED/GREEN

Focused command:

```bash
python3 -m unittest -v \
  src.air_ground_bringup.test.test_target_handoff.TargetHandoffPolicyTest.test_rank_one_psd_roundoff_is_clamped_instead_of_held \
  src.air_ground_bringup.test.test_target_handoff.TargetHandoffPolicyTest.test_accepted_near_symmetric_covariance_is_symmetrized \
  src.air_ground_bringup.test.test_target_handoff.TargetHandoffPolicyTest.test_linear_algebra_failure_becomes_invalid_hold
```

RED: `Ran 3 tests`, two failures and one error. The rank-one PSD matrix returned
HOLD, near-symmetric covariance returned NaN, and `LinAlgError` escaped.

The first symmetrization fixture was then rejected correctly because its average
was genuinely indefinite by about `5e-10`; before accepting evidence, the test
was corrected to opposite asymmetry around `0.1`. Independent literal command:

```bash
python3 -c "from decimal import Decimal, getcontext; getcontext().prec=50; print((Decimal('5.991464547')*Decimal('1.01')).sqrt())"
```

Output: `2.4599551200113387434482689734796898118671235941944`.

Final GREEN:

```text
Ran 3 tests in 0.001s
OK
```

Implementation checks material symmetry first, symmetrizes, and rejects only
eigenvalues below `-64 * eps * max(1, max_abs_covariance)`. Accepted tiny
negative roundoff is clamped to zero for radius/yaw square roots. `LinAlgError`,
floating-point, value, and overflow failures leave the initialized NaN
properties and produce HOLD.

### I6 RED/GREEN

Focused command:

```bash
python3 -m unittest -v \
  src.air_ground_bringup.test.test_launch_wiring.LaunchWiringTest.test_direct_tf_import_has_catkin_and_manifest_dependency
```

RED: `Ran 1 test`, one failure because `tf` was absent from the parsed catkin
component list. GREEN: `Ran 1 test in 0.008s`, `OK`. The test parses the direct
`tf.transformations` import, `find_package`, `catkin_package`, and package direct
dependencies.

### I7 Integration Coverage

Command:

```bash
python3 -m unittest -v \
  src.air_ground_bringup.test.test_reregistration_state_machine.FullMissionInitializationTest.test_full_module_constructor_wires_ros_contract_and_serialized_callbacks
```

Result on first run after the preceding fixes:

```text
Ran 1 test in 0.005s
OK
```

This is not an AST test. It executes the full mission module and complete
`Mission.__init__` under scoped ROS stubs that implement time/duration,
publishers, subscribers, service proxies, TF buffer/listener, transformations,
message modules, and timer construction. It verifies:

- all five parameter defaults and an actual RLock;
- action/confidence publisher message types;
- accepted-update/estimate subscriber message types, bound callback names, and
  owning mission instance;
- one 30 Hz timer bound to `tick`;
- covariance slot behavior and monotonic explicit revision under the real bound
  callbacks, including irrelevant transport sequence values.

### Task 3 Harness Adjustment

After introducing the lock, the unchanged Task 3 assertions produced five
errors because their `Mission.__new__` fixtures lacked `state_lock`:

```text
Ran 10 tests in 0.079s
FAILED (errors=5)
AttributeError: 'Mission' object has no attribute 'state_lock'
```

Only the test fixtures were given `threading.RLock`; no waypoint assertion was
weakened. Fresh result: `Ran 10 tests in 0.082s`, `OK`.

### Round 1 Modified Files

- `src/air_ground_bringup/src/air_ground_bringup/target_handoff.py`
- `src/air_ground_bringup/scripts/uav_sphere_mission.py`
- `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch`
- `src/air_ground_bringup/CMakeLists.txt`
- `src/air_ground_bringup/package.xml`
- `src/air_ground_bringup/test/test_target_handoff.py`
- `src/air_ground_bringup/test/test_reregistration_state_machine.py`
- `src/air_ground_bringup/test/test_launch_wiring.py`
- `src/air_ground_bringup/test/test_registration_waypoint.py`
- `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-8-report.md`

### Round 1 Final Verification

Task 8 plus Task 3 command:

```bash
python3 -m unittest -v \
  src/air_ground_bringup/test/test_target_handoff.py \
  src/air_ground_bringup/test/test_reregistration_state_machine.py \
  src/air_ground_bringup/test/test_registration_waypoint.py \
  src/air_ground_bringup/test/test_launch_wiring.py
```

Result:

```text
Ran 52 tests in 0.395s
OK
```

Task 7 pure/stub regressions, each with
`PYTHONPATH="src/air_ground_coordinate_transform/src"`:

```text
test_registration_coordinator.py: Ran 19 tests in 0.025s, OK
test_registration_node_adapter.py: Ran 11 tests in 0.013s, OK
test_ugv_coordinate_monitor.py: Ran 4 tests in 0.003s, OK
```

Total Task 7 result: `34/34 OK`.

Changed Python compilation:

```text
python3 -m py_compile [six changed production/test Python files]
exit 0, no output
```

XML/package parse:

```text
XML_OK=3
```

Static audit:

```text
STATIC_SAFETY_OK truth_topics=0 header_seq=0 broadcasters=0 rlocks=1
```

Bounded build:

```bash
timeout 120s catkin_make --pkg air_ground_bringup -j2
```

Result: exit 0. Catkin found `tf`, configured testing, regenerated the bringup
devel wrapper, and completed the requested package make. Existing unrelated VTK,
PCL optional-feature, Gazebo deprecation, and Eigen/system-lib export warnings
remain non-fatal.

### Round 1 Self-Review

- Research mission frame parameters are tied by test to actual perturbation
  producer destinations, so either producer rename breaks the consumer contract.
- Continuous covariance never increments accepted revision. Current covariance
  may change a retained target action, but same-target/same-action snapshots do
  not generate timer event spam.
- A pending DIRECT never resolves/dispatches before reevaluation under current
  covariance.
- WAIT command validity gates revision, but not timeout; strict deadline expiry
  precedes all success, while equality remains allowed.
- Revision compare/assign and WAIT baseline entry share the same RLock as tick.
- Stable selected timestamp is immutable, preservation occurs once, and resume
  still does no detection.
- Rank-one PSD, near-symmetric symmetrization, materially negative PSD rejection,
  and linear-algebra failure paths are covered by behavior assertions.
- `tf` is declared directly in CMake export and package manifest.
- Full constructor coverage validates runtime imports/types/bindings/timer; method
  tests remain for precise state mutations.
- M1 is still deferred; legacy opt-out, no truth, no application `header.seq`, no
  Task 9 behavior, no TF broadcaster, and UGV goal only in DISPATCH remain intact.

### Round 1 Concerns

- ROS launch resolution, real callback scheduling, TF transport timing, and
  dynamic M2-C remain externally unverified under the no-ROS-process ruling.
- Holding the RLock across a bounded TF lookup can delay subscriber callbacks by
  up to the existing 0.1-second lookup timeout; this is the minimal coherent-state
  safety tradeoff and needs observation during external M2-C.
- M1 remains deferred by coordinator direction, so launch/legacy suites still
  require the explicit bounded command documented above rather than normal
  catkin test discovery.

## Review Fix Round 2

Date: 2026-08-26

Scope was limited to Re-review Round1 findings R1-I1 and R1-I2. M1 remains
deferred and unchanged.

### Dispositions

| Finding | Disposition | Result |
|---|---|---|
| R1-I1 stable final not preserved before disagreement error | Fixed | FINAL preserves immutable `(target, spread, selected_stamp)` once immediately after stable selection, before disagreement handling; policy processing consumes preserved fields only |
| R1-I2 late numerical failure leaves partial properties | Fixed | All four confidence values are calculated in locals and atomically committed only after every numerical stage and finite check succeeds |
| M1 CMake registration breadth | Deferred | No CMake test-registration change in Round 2 |

### R1-I1 RED

Command:

```bash
python3 -m unittest -v \
  src.air_ground_bringup.test.test_reregistration_state_machine.ReregistrationStateMachineTest.test_disagreement_error_preserves_immutable_final_before_transition
```

Observed result:

```text
Ran 1 test in 0.009s
FAILED (failures=1)
```

The ordering-sensitive event list contained only
`('phase', 'ERROR_COORDINATE')`; it was missing the expected preceding
`('preserve', final)` event. This reproduced zero preservation calls on a valid
stable final with excessive handoff disagreement.

### R1-I1 GREEN

The minimal production rearrangement is:

1. `stable_target()` returns immutable `(target, spread, selected_stamp)`.
2. FINAL immediately calls `preserve_final_estimate(final)` exactly once.
3. FINAL then applies the handoff disagreement guard and may enter
   `ERROR_COORDINATE`.
4. If the guard passes, `_process_preserved_target_locked()` applies legacy or
   uncertainty policy using only preserved fields. It cannot preserve again.

Method-level tests now make the same separation explicitly with a test-only
`preserve_and_process()` helper. Repeated covariance reevaluation invokes
`process_final_estimate()` without replacing the preserved target.

Focused command:

```bash
python3 -m unittest -v \
  src.air_ground_bringup.test.test_reregistration_state_machine.ReregistrationStateMachineTest.test_disagreement_error_preserves_immutable_final_before_transition \
  src.air_ground_bringup.test.test_reregistration_state_machine.ReregistrationStateMachineTest.test_final_tick_preserves_selected_result_exactly_once \
  src.air_ground_bringup.test.test_reregistration_state_machine.ReregistrationStateMachineTest.test_direct_reobserve_and_hold_have_distinct_safe_effects \
  src.air_ground_bringup.test.test_reregistration_state_machine.ReregistrationStateMachineTest.test_reregister_preserves_complete_target_before_return
```

Result:

```text
Ran 4 tests in 0.060s
OK
```

Evidence includes:

- Disagreement path event order is exactly preserve then
  `ERROR_COORDINATE` transition.
- Preserved mean, covariance `0.03^2 I`, and selected stamp `99.9` equal the
  immutable stable result even on error.
- Normal FINAL path calls preservation exactly once.
- DIRECT, HOLD, REOBSERVE, and REREGISTER effects remain unchanged.

### R1-I2 RED

Command:

```bash
python3 -m unittest -v \
  src.air_ground_bringup.test.test_target_handoff.TargetHandoffPolicyTest.test_failure_at_any_numeric_stage_resets_all_public_properties
```

The test stages failures at:

- registration covariance validation eigensolve call 1;
- target covariance validation eigensolve call 2;
- registration radius eigensolve call 3;
- target radius eigensolve call 4;
- combined radius eigensolve call 5;
- yaw confidence square-root call 4, after all three radius square roots.

Observed result:

```text
Ran 1 test in 0.003s
FAILED (failures=3)
```

Target-radius and combined-radius `LinAlgError`, plus yaw-stage
`FloatingPointError`, left one or more earlier public properties finite. Both
validation stages and registration-radius failure already retained all initial
NaNs.

### R1-I2 GREEN

Registration radius, target radius, combined radius, and yaw confidence now use
locals inside the numerical exception boundary. After all operations succeed,
all four locals must be finite before one commit assigns the public properties
and marks the budget valid. Every earlier return therefore retains the
constructor's four NaNs and invalid state.

Focused rerun:

```text
Ran 1 test in 0.003s
OK
```

The existing valid rank-one PSD, near-symmetric symmetrization, scale-aware
negative tolerance, exact formula, equality, and action-precedence tests remain
GREEN.

### Round 2 Modified Files

- `src/air_ground_bringup/scripts/uav_sphere_mission.py`
- `src/air_ground_bringup/src/air_ground_bringup/target_handoff.py`
- `src/air_ground_bringup/test/test_reregistration_state_machine.py`
- `src/air_ground_bringup/test/test_target_handoff.py`
- `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-8-report.md`

Production metadata and launch files were unchanged in Round 2.

### Round 2 Verification

Focused policy and state suites:

```bash
python3 -m unittest -v \
  src/air_ground_bringup/test/test_target_handoff.py \
  src/air_ground_bringup/test/test_reregistration_state_machine.py
```

```text
Ran 34 tests in 0.268s
OK
```

Full Task 8 plus Task 3 four-suite command:

```bash
python3 -m unittest -v \
  src/air_ground_bringup/test/test_target_handoff.py \
  src/air_ground_bringup/test/test_reregistration_state_machine.py \
  src/air_ground_bringup/test/test_registration_waypoint.py \
  src/air_ground_bringup/test/test_launch_wiring.py
```

```text
Ran 54 tests in 0.379s
OK
```

Task 7 pure/stub regressions:

```text
test_registration_coordinator.py: Ran 19 tests in 0.026s, OK
test_registration_node_adapter.py: Ran 11 tests in 0.014s, OK
test_ugv_coordinate_monitor.py: Ran 4 tests in 0.003s, OK
Total: 34/34 OK
```

Changed Python compilation: exit 0, no output.

XML/package parsing:

```text
XML_OK=3
```

Static safety audit:

```text
STATIC_SAFETY_OK truth_topics=0 header_seq=0 broadcasters=0 dispatch_calls=1
```

Fresh bounded build:

```bash
timeout 120s catkin_make --pkg air_ground_bringup -j2
```

Result: exit 0. Catkin configured and completed the requested bringup package
make. Existing unrelated optional VTK/PCL, Gazebo deprecation, and dependency
export warnings remain non-fatal.

### Round 2 Self-Review

- A valid stable final is preserved before every transition, including
  disagreement error, and exactly once in normal/error paths.
- Policy processing no longer accepts a final sample result and cannot
  accidentally preserve it again.
- Preservation remains under the shared RLock and retains selected timestamp,
  target covariance, and handoff tuple.
- DIRECT retry/current-covariance reevaluation still operates on the same
  preserved target; HOLD/REREGISTER/REOBSERVE semantics and event suppression are
  unchanged.
- No public confidence property is assigned before all validations, three radius
  calculations, yaw confidence, and finite checks finish.
- Failures at every staged numerical call leave `HOLD` and four NaNs.
- Valid PSD/tolerance behavior, no truth, no application `header.seq`, no Task 9
  relay, no TF broadcaster, legacy false, and goal-only-DISPATCH constraints
  remain covered.
- M1 remains deferred exactly as directed.

### Round 2 Concerns

- Dynamic ROS/TF timing and M2-C remain externally unverified under the no-ROS
  process ruling.
- M1 remains deferred, so launch/waypoint suites still depend on the explicit
  four-suite command rather than normal catkin test discovery.

## Review Fix Round 3

Date: 2026-08-26

Scope was limited to Re-review Round2 finding R2-I1. M1 remains deferred and
unchanged.

### Disposition

| Finding | Disposition | Result |
|---|---|---|
| R2-I1 finite-gate exception escapes constructor | Fixed | The local-result all-finite gate is inside the same numerical exception boundary as validation/radii/yaw, before atomic property commit |
| M1 CMake registration breadth | Deferred | No metadata or test-registration change in Round 3 |

### RED

Added
`test_finite_gate_exception_and_nonfinite_result_remain_invalid`, which patches
`air_ground_bringup.target_handoff.math.isfinite`. Calls 1 and 2 allow the
inspection-radius and inspection-yaw threshold checks; call 3, the first local
result finite check after successful validation, three radii, and yaw, raises
`FloatingPointError("local_result_finite_check")`.

Focused command:

```bash
python3 -m unittest -v \
  src.air_ground_bringup.test.test_target_handoff.TargetHandoffPolicyTest.test_finite_gate_exception_and_nonfinite_result_remain_invalid
```

Observed RED:

```text
Ran 1 test in 0.001s
FAILED (errors=1)
FloatingPointError: local_result_finite_check
```

The traceback ended at the constructor generator expression calling
`math.isfinite`, proving the exception escaped before an `UncertaintyBudget`
instance could return HOLD.

The same test retains the nonfinite-result case by staging `_r95` results
`(1.0, 2.0, inf)` and requiring HOLD plus four NaNs.

### GREEN

Minimal production change: the all-local finite gate moved into the existing
`try` block. The exception tuple remains unchanged:
`LinAlgError`, `FloatingPointError`, `ValueError`, and `OverflowError`. Public
properties are still assigned only after the try block completes, so every
exception before commit retains the initialized invalid state and four NaNs.

Focused rerun:

```text
Ran 1 test in 0.001s
OK
```

No formula, PSD tolerance, material-symmetry threshold, finite-return behavior,
or action branch changed.

### Round 3 Modified Files

- `src/air_ground_bringup/src/air_ground_bringup/target_handoff.py`
- `src/air_ground_bringup/test/test_target_handoff.py`
- `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-8-report.md`

### Round 3 Verification

Focused policy suite:

```bash
python3 -m unittest -v src/air_ground_bringup/test/test_target_handoff.py
```

```text
Ran 13 tests in 0.008s
OK
```

Full Task 8 plus Task 3 four-suite command:

```bash
python3 -m unittest -v \
  src/air_ground_bringup/test/test_target_handoff.py \
  src/air_ground_bringup/test/test_reregistration_state_machine.py \
  src/air_ground_bringup/test/test_registration_waypoint.py \
  src/air_ground_bringup/test/test_launch_wiring.py
```

```text
Ran 55 tests in 0.396s
OK
```

Task 7 pure/stub regressions:

```text
test_registration_coordinator.py: Ran 19 tests in 0.024s, OK
test_registration_node_adapter.py: Ran 11 tests in 0.014s, OK
test_ugv_coordinate_monitor.py: Ran 4 tests in 0.003s, OK
Total: 34/34 OK
```

Changed/scoped Python compilation: exit 0, no output.

XML/package parsing:

```text
XML_OK=3
```

Static safety audit:

```text
STATIC_SAFETY_OK truth_topics=0 header_seq=0 broadcasters=0 dispatch_calls=1
```

Fresh bounded build:

```bash
timeout 120s catkin_make --pkg air_ground_bringup -j2
```

Result: exit 0; CMake build-system check and requested bringup package make
completed.

### Round 3 Self-Review

- Threshold finite checks still precede covariance numerical work and retain
  their existing invalid-return behavior.
- Validation, registration radius, target radius, combined radius, yaw, local
  finite-check exception, and nonfinite local-return stages all produce HOLD and
  four NaNs.
- Atomic public-property assignment remains after all numerical work and the
  finite gate.
- Valid rank-one PSD, near-symmetry, scale-aware tiny-negative tolerance,
  formulas, equality, and action precedence remain unchanged and GREEN.
- No mission, launch, dependency, truth, revision, Task 9, TF broadcaster, or
  dispatch behavior changed.
- M1 remains deferred exactly as directed.

### Round 3 Concerns

- Dynamic ROS/TF and M2-C remain externally unverified under the no-ROS-process
  ruling.
- M1 remains deferred, so launch/waypoint tests still require the explicit
  four-suite command rather than normal catkin test discovery.

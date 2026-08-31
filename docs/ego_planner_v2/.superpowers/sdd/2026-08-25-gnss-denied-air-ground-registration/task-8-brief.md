# Task 8 Brief: Uncertainty-Triggered UAV Re-registration

## Goal

Implement only Task 8 from the authoritative plan. Add a deterministic pure
uncertainty policy and integrate the approved UAV mission flow:

```text
FINAL_ESTIMATE -> RETURN_TO_UGV -> WAIT_REREGISTRATION
-> RESUME_HANDOFF -> DISPATCH
```

Do not implement Task 9's target-handoff node, common-frame covariance relay,
goal-tracking controller, or anomaly-covariance publication.

## Files

- Create `src/air_ground_bringup/setup.py`.
- Create `src/air_ground_bringup/src/air_ground_bringup/__init__.py`.
- Create `src/air_ground_bringup/src/air_ground_bringup/target_handoff.py`.
- Create `src/air_ground_bringup/test/test_target_handoff.py`.
- Create `src/air_ground_bringup/test/test_reregistration_state_machine.py`.
- Modify `src/air_ground_bringup/scripts/uav_sphere_mission.py`.
- Modify `src/air_ground_bringup/CMakeLists.txt` and `package.xml` only as
  required for the Python package, NumPy runtime, and Task 8 tests.
- Modify `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch`
  and `air_ground_final_demo.launch` for explicit opt-in/default compatibility.
- Extend existing bounded launch-wiring tests for the new launch contract.

## Binding Constraints

- Runtime autonomy must not consume Gazebo or experiment truth.
- `air_ground_final_demo.launch` remains behaviorally one-shot: uncertainty
  handoff is explicitly disabled there.
- Research launch defaults uncertainty handoff to disabled and exposes an
  opt-in argument. M2-C manual execution uses registration mode
  `opportunistic` plus uncertainty handoff enabled.
- The target remains stored in the UAV odom frame and is re-resolved through
  current TF after a strictly newer registration revision. Do not treat a
  one-time UGV-frame target as canonical.
- Task 7's `/air_ground/registration/accepted_update` supplies atomic revision
  events. `/air_ground/registration/estimate` supplies the continuously
  predicted current covariance. Never use `Header.seq` as revision.
- A visual frame is not a revision. `WAIT_REREGISTRATION` advances only when
  `current_revision > baseline_revision` captured on entry to that phase.
- UGV goal publication remains impossible before `DISPATCH`.
- Re-registration uses the latest stopped UGV odometry, the current registered
  transform chain, the existing body-relative registration offsets, safe
  registration altitude, waypoint tolerance, and speed check.
- No ROS master, launch, rostest, simulator, rosbag, topic wait, truth read,
  Git operation, or long-running process may be used in this environment.

## Ruling: Explicit Constructor Inputs

The plan's three-argument interface conflicts with its requirement that both
meter and radian thresholds be explicit. Use:

```python
UncertaintyBudget(
    registration_covariance,
    target_covariance,
    inspection_radius,
    inspection_yaw,
)
```

All four values are required; there are no hidden learned weights or module
threshold globals. `choose_action()` takes no additional arguments.

## Pure Policy Contract

In `target_handoff.py`, expose string constants `DIRECT`, `REOBSERVE`,
`REREGISTER`, and `HOLD`, plus `UncertaintyBudget`.

Validate covariance arrays as finite, symmetric positive-semidefinite arrays.
Registration covariance must expose x/y/yaw as a 3x3 matrix. Target covariance
must expose XY as a 2x2 matrix. Invalid shapes, nonfinite entries, materially
asymmetric matrices, negative eigenvalues, or nonpositive/nonfinite thresholds
produce `HOLD` and nonfinite confidence values rather than exceptions.

Use exactly:

```text
r95(Pxy) = sqrt(5.991464547 * largest_eigenvalue(Pxy))
yaw95    = 1.959964 * sqrt(P_registration[2, 2])
combined_radius = r95(P_registration_xy + P_target_xy)
```

Action precedence is deterministic:

1. Invalid/unbounded inputs: `HOLD`.
2. `combined_radius <= inspection_radius` and
   `yaw95 <= inspection_yaw`: `DIRECT`.
3. `yaw95 > inspection_yaw`: `REREGISTER`.
4. Outside the planar budget with target radius greater than registration
   radius: `REOBSERVE`.
5. Outside the planar budget with registration radius greater than or equal to
   target radius: `REREGISTER`.

Expose numeric `registration_radius`, `target_radius`, `confidence_radius`,
and `yaw_confidence` properties for evaluation.

## Mission Parameters And Topics

Add parameters with these provisional, explicit defaults:

```text
~uncertainty_aware_handoff: false
~inspection_radius: 0.35             # meters
~inspection_yaw: 0.03490658503988659 # radians, 2 degrees
~target_sigma_floor: 0.02             # meters
~reregistration_timeout: 60.0         # seconds
```

Subscribe to:

- `/air_ground/registration/accepted_update` as `RegistrationUpdate` for the
  current accepted revision.
- `/air_ground/registration/estimate` as `PoseWithCovarianceStamped` for the
  latest predicted registration covariance.

Publish the selected action as `String` on `/air_ground/handoff/action` and the
combined 95% planar radius as `Float64` on
`/air_ground/handoff/confidence_radius`. Publish once per policy decision; do
not turn timer snapshots into new policy events.

The research launch must expose and pass all five parameters. The final-demo
launch must explicitly set `uncertainty_aware_handoff=false`.

## Target Preservation

When a stable final estimate is available, preserve before any transition:

- target mean tuple in UAV odom,
- 2x2 isotropic provisional target covariance using
  `max(final_spread, target_sigma_floor) ** 2`,
- latest selected observation timestamp,
- current handoff target tuple.

Task 9 owns replacement with unbiased sensor/sample covariance. Task 8 must not
preempt that work.

`DIRECT`: publish the current final target and enter `DISPATCH`.

`REOBSERVE`: clear the stale final samples and return to
`CENTER_OVER_SPHERE`; this is the only action allowed to rerun target sensing.

`HOLD`: retain position, preserved target, and current phase; never dispatch an
unsafe goal.

`REREGISTER`: preserve target and enter `RETURN_TO_UGV` without clearing it.

## Re-registration State Machine

`RETURN_TO_UGV`:

- Recompute the latest visual rendezvous command every tick from UGV odometry.
- Apply `registration_dx/dy` in the UGV body heading, transform the resulting
  point `ugv_odom -> air_ground_origin -> uav_odom`, and command the existing
  registration altitude.
- Require XY and altitude tolerances plus `speed <= 0.15` before advancing.
- On arrival, capture `baseline_revision = current_revision` and enter
  `WAIT_REREGISTRATION`.
- Missing TF/odom holds safely. `registration_move_timeout` enters
  `ERROR_REGISTRATION`.

`WAIT_REREGISTRATION`:

- Hold/recompute the same rendezvous command.
- Same or older revision never advances.
- A strictly newer revision enters `RESUME_HANDOFF`.
- `reregistration_timeout` enters `ERROR_REGISTRATION`.

`RESUME_HANDOFF`:

- Re-resolve and republish the preserved UAV-odom target through the current
  registration TF by calling the existing target publication path.
- Do not call target detection or clear preserved fields.
- Enter `DISPATCH` only after both origin- and UGV-frame target resolutions
  succeed. Missing TF holds safely.

## TDD Cycles

Write and run each focused RED before production changes.

1. Policy RED: exact largest-eigenvalue formula, low/direct,
   target-dominated/reobserve, registration-dominated/reregister, yaw-only
   reregister, invalid/nonfinite/non-PSD/threshold hold, and equality boundary.
2. Registration-input RED: covariance mapping from ROS `(0, 1, 5)` slots,
   continuous predicted covariance updates without revision increments, and
   atomic revision callback that never reads `header.seq`.
3. State RED: exact phase sequence, dynamic UGV-body rendezvous, safe arrival,
   baseline captured on WAIT entry, equal/older/newer revision behavior, both
   timeouts, preserved target fields, no goal before dispatch, re-resolve on
   resume, missing-TF hold, `DIRECT`, `REOBSERVE`, and `HOLD` behavior.
4. Wiring RED: Python package setup, CMake test registration, research opt-in
   parameters, final-demo explicit opt-out, and no truth subscription.

Tests must exercise production pure policy and mission adapter behavior, not
duplicate formulas or test-only state machines. Use ROS stubs/AST extraction as
needed; do not execute ROS.

## Verification

- Focused Task 8 tests.
- Existing Task 3 launch/waypoint tests and Task 7 coordinator/node/monitor
  bounded regressions.
- `py_compile` for changed Python production and tests.
- XML/package parsing and static truth/topic audit.
- Bounded `catkin_make --pkg air_ground_bringup -j2`.
- Write but do not run any dynamic M2-C procedure.

Report RED/GREEN evidence, exact policy boundaries, state transition evidence,
modified files, all commands/results, and self-review to `task-8-report.md`.

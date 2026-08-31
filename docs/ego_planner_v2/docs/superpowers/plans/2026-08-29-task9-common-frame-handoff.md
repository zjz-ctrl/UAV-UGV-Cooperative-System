# Task 9 Common-Frame Handoff Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish a covariance-bearing UAV anomaly, transform it into the canonical `air_ground_origin` frame, and publish an executable standoff goal only from a fresh revision-consistent registration snapshot that passes the Task 8 uncertainty policy.

**Architecture:** The UAV mission owns anomaly sampling and publishes sensing-plus-UAV-pose covariance in its experimental odometry frame. A new relay atomically snapshots the target, a revision-bearing continuous registration state, and the matching accepted registration update; it publishes the canonical origin estimate without registration uncertainty and adds registration translation/yaw uncertainty exactly once to the executable inspection goal. The existing Task 8 policy action names and `/air_ground/handoff/confidence_radius` interface remain unchanged.

**Tech Stack:** ROS Noetic (`rospy`, `tf2_ros`), Python 3, NumPy, `geometry_msgs`, existing `RegistrationUpdate`, `unittest`, rostest, catkin.

**Spec:** `docs/superpowers/specs/2026-08-25-gnss-denied-air-ground-registration-design.md`; original Task 9 steps are in `docs/superpowers/plans/2026-08-25-gnss-denied-air-ground-registration.md:783-842`.

## Global Constraints

- Keep `/air_ground/handoff/confidence_radius`; do not publish or migrate to `/confidence_radius`.
- `/air_ground/anomaly/origin_estimate` contains target sensing plus UAV pose uncertainty and no registration uncertainty.
- `/air_ground/inspection_goal` adds registration translation/yaw uncertainty and its lever-arm effect exactly once.
- Evaluate one target, one continuous registration state, and one accepted update from one relay lock snapshot; continuous and accepted revisions must match.
- Revision mismatch, stale timestamps, invalid frames, or invalid/nonfinite covariance produce `HOLD`, publish a NaN confidence radius, and publish no executable inspection goal.
- Never use `Header.seq` as application identity.
- Gazebo truth is evaluation-only and may not enter the mission, relay, policy, or target covariance.
- Exactly one registration TF broadcaster remains.
- Preserve the Task 8 action strings `DIRECT`, `REOBSERVE`, `REREGISTER`, and `HOLD`.
- Preserve legacy red-sphere topics as diagnostic outputs only.
- Do not implement Task 10 goal tracking or Task 11 visual confirmation.
- The workspace is intentionally unversioned. Do not initialize Git and do not add commit steps.

## File Structure

- Modify `src/air_ground_bringup/src/air_ground_bringup/target_handoff.py`: pure covariance, standoff, snapshot validation, and relay evaluation math.
- Modify `src/air_ground_bringup/scripts/uav_sphere_mission.py`: retain selected samples, propagate UAV pose covariance, and publish the UAV-frame anomaly.
- Create `src/air_ground_bringup/scripts/target_handoff_node.py`: ROS adapter and atomic cache/snapshot boundary.
- Modify `src/air_ground_coordinate_transform/scripts/takeoff_registration.py`: publish revision-bearing continuous state using existing `RegistrationUpdate`.
- Modify `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch`: start the relay only for uncertainty-aware research runs.
- Modify bringup and coordinate-transform CMake/package wiring only where required by the new script/topic tests.
- Extend `src/air_ground_bringup/test/test_target_handoff.py`: pure math and invalid-input tests.
- Extend `src/air_ground_bringup/test/test_reregistration_state_machine.py`: mission publication and Task 8 state regressions.
- Extend `src/air_ground_coordinate_transform/test/test_registration_node_adapter.py`: continuous state publication and ordering tests.
- Create `src/air_ground_bringup/test/test_target_handoff_node.py`: ROS-adapter tests with stubs.
- Create `src/air_ground_bringup/test/inspection_relay.test` and `src/air_ground_bringup/test/test_inspection_relay.py`: bounded live ROS integration assertions.

---

### Task 1: Pure Target And Goal Covariance

**Files:**
- Modify: `src/air_ground_bringup/src/air_ground_bringup/target_handoff.py`
- Test: `src/air_ground_bringup/test/test_target_handoff.py`

**Interfaces:**
- Produces: `sample_target_covariance(samples_xy, variance_floor, pose_covariances=None, range_axes=None, range_variance=0.0) -> numpy.ndarray | None`.
- Produces: `standoff_goal(target_xy, anchor_xy, standoff) -> (mean_xyyaw, target_jacobian) | None`.
- Produces: `registration_execution_covariance(target_in_registration_frame, registration_covariance) -> numpy.ndarray | None`.
- Produces: `evaluate_handoff(...) -> HandoffResult` with separate `origin_covariance`, `goal_covariance`, action, and confidence values.

- [ ] **Step 1: Add failing unbiased covariance and front-range tests**

Use a fixed XY cluster and assert `numpy.cov(..., ddof=1)`. Assert the configured floor is applied in variance units and that

```python
range_variance * np.outer(unit_range_axis, unit_range_axis)
```

increases only the front-camera range direction. Reject fewer than two samples, nonfinite samples, invalid floors, and degenerate range axes.

- [ ] **Step 2: Run the focused RED tests**

Run:

```bash
python3 -m unittest \
  src.air_ground_bringup.test.test_target_handoff.TargetSampleCovarianceTest -v
```

Expected: failures naming the missing `sample_target_covariance` behavior.

- [ ] **Step 3: Implement the minimal sample covariance function**

Compute unbiased XY covariance, add `variance_floor ** 2 * I`, add the mean per-sample UAV pose contribution once, then add configured front range variance once. Validate finite, symmetric, positive-semidefinite output before returning it.

- [ ] **Step 4: Add failing standoff and registration lever-arm tests**

For `r = target - anchor`, `q = ||r||`, and `u = r/q`, require:

```text
goal_xy = target_xy - standoff * u
goal_yaw = atan2(r_y, r_x)
J_xy = I - (standoff / q) * (I - u u^T)
J_yaw = [-r_y / q^2, r_x / q^2]
```

At a 15 m lever arm with one-degree registration yaw sigma, require the registration-only lateral variance to contain approximately:

```text
(15 * radians(1))^2
```

Assert the final goal covariance contains this term once, fails if it is absent, and fails if it is doubled. Include nonzero x/y/yaw cross-covariances.

- [ ] **Step 5: Run the lever-arm RED tests**

Run:

```bash
python3 -m unittest \
  src.air_ground_bringup.test.test_target_handoff.HandoffCovarianceTest -v
```

Expected: failures naming missing standoff/registration propagation.

- [ ] **Step 6: Implement pure handoff propagation**

Use these two independent covariance products:

```text
P_origin = R_origin_uav * P_target_uav * R_origin_uav^T
P_goal_sensing = J_target * P_origin * J_target^T
P_goal = P_goal_sensing + J_registration * P_registration * J_registration^T
```

`P_origin` must be unchanged when only `P_registration` changes. Supply `P_goal_sensing[:2,:2]` and the registration-only 3x3 contribution to `UncertaintyBudget`; do not pass an already combined covariance as both arguments.

- [ ] **Step 7: Run all policy and covariance tests**

Run:

```bash
python3 -m unittest src.air_ground_bringup.test.test_target_handoff -v
```

Expected: all existing Task 8 policy tests and new Task 9 covariance tests pass.

---

### Task 2: Revision-Bearing Continuous Registration State

**Files:**
- Modify: `src/air_ground_coordinate_transform/scripts/takeoff_registration.py`
- Test: `src/air_ground_coordinate_transform/test/test_registration_node_adapter.py`
- Test: `src/air_ground_coordinate_transform/test/test_registration_update_serialization.py`

**Interfaces:**
- Produces: `/air_ground/registration/state` as latched `air_ground_coordinate_transform/RegistrationUpdate` for each continuously published initialized state.
- Preserves: `/air_ground/registration/estimate`, `/accepted_update`, and `/revision`.

- [ ] **Step 1: Add failing publisher and message-identity tests**

Require each continuous state message to carry one header, one full planar covariance, and the same explicit `revision` as the state being serialized. Verify that timer prediction can increase covariance without increasing revision. Continue asserting that `Header.seq` is not used as identity.

- [ ] **Step 2: Run the focused RED tests**

Run:

```bash
python3 -m unittest \
  src.air_ground_coordinate_transform.test.test_registration_node_adapter -v
```

Expected: failures because `/air_ground/registration/state` is not published.

- [ ] **Step 3: Implement state serialization and publication**

Create one helper that serializes a filter state and revision into both the legacy `PoseWithCovarianceStamped` and the new `RegistrationUpdate`. On accepted updates, publish continuous state before accepted update. On timer prediction, publish continuous state with the unchanged current revision.

- [ ] **Step 4: Run registration tests**

Run:

```bash
python3 -m unittest discover \
  -s src/air_ground_coordinate_transform/test -p 'test_*.py' -v
```

Expected: all bounded coordinate-transform tests pass.

---

### Task 3: Mission Anomaly Estimate Publication

**Files:**
- Modify: `src/air_ground_bringup/scripts/uav_sphere_mission.py`
- Test: `src/air_ground_bringup/test/test_reregistration_state_machine.py`

**Interfaces:**
- Produces: `/air_ground/anomaly/uav_estimate` as latched `PoseWithCovarianceStamped` in `uav_odom_frame` using the selected observation stamp.
- Preserves: legacy red-sphere diagnostic publications.
- Consumes: relay actions instead of publishing policy actions in uncertainty-aware mode.

- [ ] **Step 1: Add failing sample-retention and covariance tests**

Extend sample fixtures with per-observation UAV pose covariance projected to target XY using:

```text
J_pose = [[1, 0, -dy], [0, 1, dx]]
P_pose_at_target = J_pose * P_uav_xyyaw * J_pose^T
```

Require final covariance to equal unbiased sample covariance plus the mean selected UAV pose contribution once plus the variance floor once. Require nonfinite sample/covariance rejection.

- [ ] **Step 2: Run mission RED tests**

Run:

```bash
python3 -m unittest \
  src.air_ground_bringup.test.test_reregistration_state_machine -v
```

Expected: failures because the mission still publishes no UAV anomaly estimate and uses isotropic spread covariance.

- [ ] **Step 3: Implement selected-sample covariance and anomaly publication**

Preserve the selected sample set and selected timestamp. Publish target x/y/z and XY covariance in slots `(0,1,6,7)`, identity orientation, and no registration covariance. Use the observation stamp rather than `rospy.Time.now()`.

- [ ] **Step 4: Move uncertainty-aware action ownership to the relay**

In enabled mode, publish the anomaly and wait for a fresh relay action. Keep disabled compatibility mode on the legacy direct path. `RESUME_HANDOFF` republishes the preserved anomaly and waits for relay reevaluation; it must not unconditionally dispatch. Enabled mode must not publish `/air_ground/ugv_goal`.

- [ ] **Step 5: Run Task 3 and Task 8 mission regressions**

Run:

```bash
python3 -m unittest \
  src.air_ground_bringup.test.test_registration_waypoint \
  src.air_ground_bringup.test.test_reregistration_state_machine -v
```

Expected: body-relative registration geometry and all Task 8 state transitions remain green.

---

### Task 4: Atomic Target Handoff Relay

**Files:**
- Create: `src/air_ground_bringup/scripts/target_handoff_node.py`
- Create: `src/air_ground_bringup/test/test_target_handoff_node.py`
- Modify: `src/air_ground_bringup/CMakeLists.txt`
- Modify: `src/air_ground_bringup/package.xml`

**Interfaces:**
- Consumes: `/air_ground/anomaly/uav_estimate` (`PoseWithCovarianceStamped`).
- Consumes: `/air_ground/registration/state` (`RegistrationUpdate`).
- Consumes: `/air_ground/registration/accepted_update` (`RegistrationUpdate`).
- Publishes: `/air_ground/anomaly/origin_estimate` (`PoseWithCovarianceStamped`).
- Publishes: `/air_ground/inspection_goal` (`PoseWithCovarianceStamped`) only for `DIRECT`.
- Publishes: `/air_ground/handoff/action` (`String`).
- Publishes: `/air_ground/handoff/confidence_radius` (`Float64`).

- [ ] **Step 1: Add failing atomic snapshot tests**

Require one lock-protected immutable snapshot of target, continuous state, and accepted update. Require:

```text
state.revision == accepted.revision > 0
state.frame_id == accepted.frame_id == origin_frame
state.stamp >= accepted.stamp
target/state age <= configured maximum
```

Require generation recheck before publication so an input callback during computation suppresses stale output.

- [ ] **Step 2: Add failing HOLD/no-goal tests**

Cover revision mismatch, either callback arrival order, stale/future/zero stamps, wrong frames, invalid quaternion, invalid shape, asymmetric/non-PSD/nonfinite covariance, and TF failure. Every case must publish `HOLD`, publish NaN confidence, and add zero goal messages.

- [ ] **Step 3: Run relay adapter RED tests**

Run:

```bash
python3 -m unittest \
  src.air_ground_bringup.test.test_target_handoff_node -v
```

Expected: import or behavior failures because the relay does not exist.

- [ ] **Step 4: Implement the minimal ROS adapter**

Use one `RLock`, deep-copy messages into a snapshot, perform TF/covariance math outside the lock, then compare the snapshot generation before publishing. Do not subscribe to truth topics and do not instantiate a TF broadcaster.

- [ ] **Step 5: Run relay and safety tests**

Run:

```bash
python3 -m unittest \
  src.air_ground_bringup.test.test_target_handoff_node \
  src.air_ground_bringup.test.test_launch_wiring -v
```

Expected: relay behavior and no-truth/no-extra-broadcaster checks pass.

---

### Task 5: Research Launch And Integration Contract

**Files:**
- Modify: `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch`
- Modify: `src/air_ground_bringup/CMakeLists.txt`
- Modify: `src/air_ground_bringup/package.xml`
- Create: `src/air_ground_bringup/test/inspection_relay.test`
- Create: `src/air_ground_bringup/test/test_inspection_relay.py`
- Test: `src/air_ground_bringup/test/test_launch_wiring.py`

**Interfaces:**
- Starts the relay in the uncertainty-aware research launch.
- Leaves `air_ground_final_demo.launch` compatibility defaults unchanged.

- [ ] **Step 1: Add failing launch and integration assertions**

Assert exact topic types, experimental frame forwarding, relay parameters, one action publisher, one confidence publisher, one registration TF broadcaster, and no autonomy truth subscribers.

- [ ] **Step 2: Wire script installation, dependencies, launch, and rostest**

Install `target_handoff_node.py`, declare required ROS dependencies, register pure tests and `inspection_relay.test`, and pass inspection radius/yaw, standoff, age limits, frames, and topic names through the research launch.

- [ ] **Step 3: Run bounded unit/static integration tests**

Run:

```bash
python3 -m unittest discover \
  -s src/air_ground_bringup/test -p 'test_*.py' -v
python3 -m unittest discover \
  -s src/air_ground_coordinate_transform/test -p 'test_*.py' -v
```

Expected: all tests pass.

- [ ] **Step 4: Run bounded build and launch checks**

Run:

```bash
catkin_make --pkg air_ground_coordinate_transform air_ground_bringup -j2
roslaunch --check air_ground_bringup air_ground_inspection_experiment.launch
roslaunch --check air_ground_bringup air_ground_final_demo.launch
```

Expected: build and both launch checks exit zero.

---

### Task 6: Task 9 Verification And Stop Gate

**Files:**
- Modify: `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/progress.md`
- Create: `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-9-report.md`

**Interfaces:**
- Records Task 9 evidence only.

- [ ] **Step 1: Run a fresh complete bounded suite**

Run pure/stub tests, Python compilation, XML parsing, package builds, and static audits for truth consumers, TF broadcasters, topic ownership, covariance double counting, and canonical goal frames.

- [ ] **Step 2: Run the minimum dynamic verification if the environment remains available**

Use a fresh isolated ROS/Gazebo/PX4 session. Verify one matched revision publishes origin estimate and a DIRECT inspection goal in `air_ground_origin`; then create a revision mismatch or stale state and verify `HOLD` with no additional goal. Do not start Task 10 or use the Task 10 controller contract as an acceptance dependency.

- [ ] **Step 3: Request a Task 9 scope review**

Review only Task 9 files against the original plan and the three approved constraints. Treat any Task 10 goal-tracking implementation as scope creep.

- [ ] **Step 4: Write the Task 9 report and stop**

Report changed files, exact topic flow, covariance formulas, test/build/dynamic evidence, and residual limitations. Update the ledger to `Task 9 complete` only when evidence supports it. Stop before Task 10.

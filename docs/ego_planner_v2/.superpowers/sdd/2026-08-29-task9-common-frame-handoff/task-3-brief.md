# Task 3 Brief: Mission Anomaly Estimate Publication

Read this first. It is the complete requirement for this task.

## Context

The UAV mission currently stores `(x,y,z,stamp)` samples, reduces final uncertainty to isotropic radial spread, publishes the Task 8 action itself, and creates a one-time `/air_ground/ugv_goal`. Task 9 moves uncertainty evaluation/action publication to the relay. The mission must publish a covariance-bearing UAV-frame anomaly and consume relay actions while preserving all Task 8 state semantics and the disabled legacy path.

Task 1 now provides `sample_target_covariance` in `air_ground_bringup.target_handoff`.

## Files

- Modify `src/air_ground_bringup/scripts/uav_sphere_mission.py`.
- Modify `src/air_ground_bringup/test/test_reregistration_state_machine.py`.
- Do not edit relay, launch, package, CMake, coordinate-transform, or Task 10 files.

## Required Topic Ownership

Mission produces:

```text
/air_ground/anomaly/uav_estimate
geometry_msgs/PoseWithCovarianceStamped
latched
frame_id = configured uav_odom_frame
stamp = selected observation stamp
```

Mission consumes:

```text
/air_ground/handoff/action
std_msgs/String
DIRECT | REOBSERVE | REREGISTER | HOLD
```

In uncertainty-aware mode the mission must no longer advertise or publish `/air_ground/handoff/action` or `/air_ground/handoff/confidence_radius`; the Task 4 relay is their sole publisher. Legacy red-sphere topics remain diagnostic outputs. Disabled compatibility mode retains the existing direct legacy `/air_ground/ugv_goal` path.

## Sample And Covariance Behavior

For each accepted camera observation, retain the target point, source stamp, and the UAV odometry pose contribution projected to target XY. Extract full `(x,y,yaw)` covariance from ROS axes `(0,1,5)` including cross terms. For target offset `(dx,dy)` from the UAV pose:

```text
J_pose = [[1, 0, -dy],
          [0, 1,  dx]]
P_pose_at_target = J_pose P_uav_xyyaw J_pose^T
```

Reject nonfinite/asymmetric/non-PSD UAV covariance and nonfinite samples rather than preserving them.

`stable_target` must retain the exact selected sample set used for the center/spread decision. `preserve_final_estimate` must call:

```python
sample_target_covariance(
    selected_xy,
    variance_floor=self.target_sigma_floor,
    pose_covariances=selected_pose_covariances,
)
```

The final product is unbiased selected-sample covariance plus `target_sigma_floor**2 I` plus the mean projected UAV-pose covariance, each once. It contains no registration covariance.

Publish x/y/z, identity orientation, and exact XY covariance slots `(0,1,6,7)`; all unused covariance slots stay zero.

## Action/State Behavior

- Publishing a new anomaly begins one handoff request generation and clears prior terminal action state.
- Accept relay actions only while the mission is waiting in `FINAL_ESTIMATE` or `RESUME_HANDOFF` for the current request. The planned String interface has no revision token; use a nonlatched subscription and local request/awaiting state, never `Header.seq`.
- `DIRECT`: mark request complete and transition to `DISPATCH`.
- `REOBSERVE`: clear nadir samples and return to `CENTER_OVER_SPHERE`.
- `REREGISTER`: preserve target, transition to `RETURN_TO_UGV`, and wait for a newer accepted registration revision exactly as Task 8 does.
- `HOLD`: remain in the current waiting phase and continue accepting a later fresh relay action; publish no goal.
- `RESUME_HANDOFF`: republish the preserved anomaly once as a new request and wait for relay reevaluation. Never dispatch unconditionally.
- In uncertainty-aware `DISPATCH`, do not call legacy `dispatch_goal` and do not publish `/air_ground/ugv_goal`; advance to `OVERWATCH` after the relay has already published `/air_ground/inspection_goal`.
- Disabled mode keeps existing `publish_final_target`, `DISPATCH`, and `dispatch_goal` compatibility behavior.

## TDD Requirements

1. Add failing tests for full planar odometry covariance projection with yaw lever arm and cross terms.
2. Add failing tests proving the exact selected sample set and observation stamp feed unbiased covariance, floor, and pose contributions once.
3. Add invalid/nonfinite sample and covariance rejection tests.
4. Add failing publisher message tests for frame, selected stamp, identity orientation, exact XY slots, zero unused slots, and no registration covariance dependence.
5. Add constructor tests proving anomaly publisher/action subscriber and absence of mission action/confidence publishers in enabled architecture.
6. Rewrite Task 8 action tests to drive the public action callback and preserve DIRECT/REOBSERVE/REREGISTER/HOLD behavior.
7. Prove HOLD can later become DIRECT, stale/out-of-phase action is ignored, re-registration preserves target, and RESUME republishes then waits.
8. Prove enabled mode never invokes legacy goal publication; prove disabled mode remains unchanged.
9. Record genuine RED before production changes, then GREEN and full regression counts.

## Commands

```bash
python3 -m unittest \
  src.air_ground_bringup.test.test_reregistration_state_machine -v
python3 -m unittest \
  src.air_ground_bringup.test.test_registration_waypoint \
  src.air_ground_bringup.test.test_reregistration_state_machine -v
python3 -m py_compile \
  src/air_ground_bringup/scripts/uav_sphere_mission.py \
  src/air_ground_bringup/test/test_reregistration_state_machine.py
```

## Rulings And Constraints

- Target covariance includes selected sample scatter, floor, and explicitly projected UAV pose covariance. It excludes registration covariance.
- Task 4 owns action/confidence publication; do not implement or fake the relay here.
- The target `PoseWithCovarianceStamped` is registration-independent; local request generation protects callback ordering but is not transported in `Header.seq`.
- A prior latched executable goal cannot be retracted in Task 9; this mission must at least never publish a legacy goal in enabled mode.
- No truth, new TF broadcaster, Task 10 goal tracker, Task 11 inspection, ROS/Gazebo/PX4 run, Git initialization, or unrelated refactor.

## Report

Write `.superpowers/sdd/2026-08-29-task9-common-frame-handoff/task-3-report.md` with status, files changed, RED/GREEN evidence and counts, exact sample/covariance formula, topic ownership, state transitions, compatibility evidence, self-review, and concerns. Return only status/tests/concerns. Do not dispatch subagents.

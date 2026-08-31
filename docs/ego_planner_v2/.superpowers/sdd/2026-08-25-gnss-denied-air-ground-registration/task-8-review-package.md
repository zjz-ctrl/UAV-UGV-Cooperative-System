# Task 8 Review Package

## Repository State

This workspace has no Git metadata. Review the complete current files listed
below against `task-8-brief.md`, `task-8-report.md`, the plan/spec sections, and
the Task 7 registration interfaces. Task 9 has not started.

## Files In Scope

- `src/air_ground_bringup/setup.py`
- `src/air_ground_bringup/src/air_ground_bringup/__init__.py`
- `src/air_ground_bringup/src/air_ground_bringup/target_handoff.py`
- `src/air_ground_bringup/scripts/uav_sphere_mission.py`
- `src/air_ground_bringup/CMakeLists.txt`
- `src/air_ground_bringup/package.xml`
- `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch`
- `src/air_ground_bringup/launch/air_ground_final_demo.launch`
- `src/air_ground_bringup/test/test_target_handoff.py`
- `src/air_ground_bringup/test/test_reregistration_state_machine.py`
- `src/air_ground_bringup/test/test_launch_wiring.py`
- `src/air_ground_bringup/test/test_registration_waypoint.py`

## Binding Interfaces

- `RegistrationUpdate.revision` is the only accepted-event identity;
  continuous `PoseWithCovarianceStamped` snapshots supply predicted covariance.
- Policy formulas/constants, validation, action precedence, parameter defaults,
  topics, and phase semantics are exact in `task-8-brief.md`.
- `WAIT_REREGISTRATION` baseline is captured on phase entry; only a strictly
  newer accepted revision advances.
- Rendezvous applies body-relative offsets to latest UGV pose and resolves
  `ugv_odom -> air_ground_origin -> uav_odom` every tick.
- Preserved UAV-odom target is re-resolved after a new revision; no new target
  sensing occurs unless action is `REOBSERVE`.
- No UGV goal before `DISPATCH`; timeout/missing-data paths must fail safe.
- Uncertainty-aware behavior is opt-in. The final demo remains legacy one-shot.
- No truth input, Task 9 implementation, or new TF broadcaster is allowed.
- Dynamic ROS/Gazebo/rostest execution is prohibited here and remains external
  acceptance, not local evidence.

# Task 7 Review Package

## Repository State

No Git metadata exists. Review complete current Task 7 files against the brief,
specification, and report. Task 8 has not started.

## Files In Scope

- `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_coordinator.py`
- `src/air_ground_coordinate_transform/scripts/takeoff_registration.py`
- `src/air_ground_coordinate_transform/config/registration.yaml`
- `src/air_ground_coordinate_transform/launch/coordinate_transform.launch`
- `src/air_ground_coordinate_transform/CMakeLists.txt`
- `src/air_ground_coordinate_transform/test/test_registration_coordinator.py`
- `src/air_ground_coordinate_transform/test/test_ugv_coordinate_monitor.py`
- `src/air_ground_coordinate_transform/test/registration_node.test`
- `src/air_ground_coordinate_transform/test/test_registration_node.py`
- `src/air_ground_bringup/scripts/ugv_coordinate_monitor.py`
- `src/air_ground_bringup/test/test_launch_wiring.py`

## Binding Constraints

- One visual frame never changes revision. One accepted robust-window/filter
  update is exactly one registration event and increments revision once.
- First accepted batch initializes revision 1. Rejections and predictions leave
  current registration and revision unchanged.
- Every decided window is consumed once; samples cannot overlap events.
- Hidden intervals perform prediction/covariance growth only.
- `one_shot` remains revision 1 and legacy `FROZEN`; repeated modes retain
  initialized `/frozen=True` and use tracking/degraded/update/reject statuses.
- Task 6 filter mathematics is reused, never copied or simplified.
- Stale/invalid/singular/gate/insufficient reasons remain explicit.
- Exactly one registration TF broadcaster remains.
- No Gazebo/experiment truth may enter production estimation/gating/revision.
- Written rostests and dynamic M2-B checks are intentionally unrun under the
  no-long-process ruling.

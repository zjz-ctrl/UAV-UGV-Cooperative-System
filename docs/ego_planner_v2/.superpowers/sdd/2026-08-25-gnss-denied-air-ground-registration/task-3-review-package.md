# Task 3 Review Package

## Repository State

No Git metadata exists. Review complete current files against the Task 3 brief.
Task 1/2 behavior is pre-existing and must not be regressed.

## Files In Scope

- `src/air_ground_bringup/launch/uav_sitl.launch`
- `src/air_ground_bringup/launch/mvp_system.launch`
- `src/air_ground_bringup/launch/air_ground_final_demo.launch`
- `src/air_ground_bringup/scripts/uav_sphere_mission.py`
- `src/air_ground_bringup/test/test_registration_waypoint.py`
- `src/air_ground_bringup/test/test_launch_wiring.py`
- `src/air_ground_coordinate_transform/config/registration.yaml`
- `src/air_ground_coordinate_transform/launch/coordinate_transform.launch`

## Binding Constraints

- Registration offset is UAV-home-body-relative and supports independent dx/dy.
- Every registration mission phase uses the same rotated waypoint.
- UAV and UGV spawn yaw are independent and fully forwarded.
- Research registration defaults to visual relative yaw.
- `air_ground_final_demo.launch` explicitly retains fixed-yaw compatibility.
- Preserve Task 2 input-frame, covariance, revision, and one-shot interfaces.
- Do not consume Gazebo truth in autonomy code.
- Dynamic M1-A cases remain unverified under the no-long-process ruling.
- No `roslaunch`, `roscore`, `rostest`, simulator, PX4, RViz, rosbag, topic waits,
  or long processes may run in the current environment.

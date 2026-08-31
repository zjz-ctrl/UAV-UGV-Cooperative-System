# Task 1 Review Package

## Repository State

This workspace has no Git metadata, so no base/head SHA or generated diff exists. Review
is limited to Task 1's brief, report, and the complete current contents of the files below.

## Files In Scope

- `src/air_ground_coordinate_transform/setup.py`
- `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/__init__.py`
- `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/se2.py`
- `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/odom_buffer.py`
- `src/air_ground_coordinate_transform/test/test_se2.py`
- `src/air_ground_coordinate_transform/test/test_odom_buffer.py`
- `src/air_ground_coordinate_transform/CMakeLists.txt`
- `src/air_ground_coordinate_transform/package.xml`

## Binding Global Constraints

- Preserve `air_ground_final_demo.launch` as the one-shot compatibility baseline.
- Gazebo truth is evaluation-only and must not publish control, target, registration, or mission-decision topics.
- Runtime autonomy may not consume `/gazebo/get_model_state`, `/gazebo/model_states`, or `/air_ground_experiment/truth/*`.
- Use pure, importable Python for SE(2), covariance, and odometry interpolation math.
- `OdomBuffer` interface is `append(stamp, x, y, z, yaw)`, `append_odometry(message)`, `interpolate(stamp)`, and `distance_since(stamp)`.
- This task must not implement Task 2 registration behavior.

## Test Evidence

Read `task-1-report.md`; reviewers should not rerun the same suite unless a specific code concern is not answered there.

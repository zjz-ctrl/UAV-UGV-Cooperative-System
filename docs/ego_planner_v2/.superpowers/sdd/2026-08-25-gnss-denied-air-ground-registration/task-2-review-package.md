# Task 2 Review Package

## Repository State

No Git metadata exists. Review the complete current contents of every file listed below
against the brief. The original implementer was interrupted by a forbidden foreground
launch; the recovery report explicitly separates bounded evidence from unrun dynamic
checks.

## Files In Scope

- `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py`
- `src/air_ground_coordinate_transform/scripts/takeoff_registration.py`
- `src/air_ground_coordinate_transform/test/test_registration_estimator.py`
- `src/air_ground_coordinate_transform/test/registration_node.test`
- `src/air_ground_coordinate_transform/test/test_registration_node.py`
- `src/air_ground_coordinate_transform/CMakeLists.txt`
- `src/air_ground_coordinate_transform/config/registration.yaml`

## Binding Constraints

- Preserve all legacy registration topics and TF edges in one-shot mode.
- Add covariance-bearing `/air_ground/registration/estimate` and monotonic `/revision`.
- First freeze is revision `1`; contradictory later samples cannot update one-shot state.
- Build the complete 3-D matrix chain before projecting the final transform to SE(2).
- Do not consume Gazebo truth.
- No `roslaunch`, `roscore`, `rostest`, Gazebo, PX4, RViz, rosbag, or long topic waits may run in the current environment.
- Dynamic checks omitted because of that ruling must remain clearly unverified, not claimed.

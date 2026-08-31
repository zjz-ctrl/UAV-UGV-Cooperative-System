# Task 3: Support arbitrary spawn geometry and relative heading

## Milestone

Milestone 1: Multi-Initial-Pose One-Shot Registration Baseline.

## Files

- Modify: `src/air_ground_bringup/launch/uav_sitl.launch`
- Modify: `src/air_ground_bringup/launch/mvp_system.launch`
- Modify: `src/air_ground_bringup/launch/air_ground_final_demo.launch`
- Modify: `src/air_ground_bringup/scripts/uav_sphere_mission.py`
- Modify: `src/air_ground_coordinate_transform/config/registration.yaml`
- Modify: `src/air_ground_coordinate_transform/launch/coordinate_transform.launch`
- Test: `src/air_ground_bringup/test/test_registration_waypoint.py`

## Interfaces

- Produce launch args: `uav_yaw`, `ugv_yaw`, `registration_dx`, `registration_dy`.
- Produce `registration_waypoint(home_x, home_y, home_yaw, dx, dy) -> (x, y)`.
- Change the research default to estimate visual relative yaw; the compatibility launch may explicitly retain fixed yaw.

## Steps

1. Write the failing body-relative waypoint test:

   ```python
   def test_registration_offset_rotates_with_home_heading(self):
       x, y = registration_waypoint(2.0, 3.0, math.pi / 2, 1.6, 0.0)
       self.assertAlmostEqual(x, 2.0, places=6)
       self.assertAlmostEqual(y, 4.6, places=6)
   ```

2. Run the focused test and verify the expected failure is the missing `registration_waypoint`.
3. Implement the body-relative waypoint:

   ```python
   def registration_waypoint(home_x, home_y, home_yaw, dx, dy):
       c, s = math.cos(home_yaw), math.sin(home_yaw)
       return home_x + c * dx - s * dy, home_y + s * dx + c * dy
   ```

4. Replace every `home[0] + registration_offset, home[1]` assumption in all registration phases with the resulting `(registration_x, registration_y)`.
5. Add `uav_sitl.launch` argument `yaw` and pass `-Y $(arg yaw)` to `spawn_model`. Add independent `uav_yaw` and `ugv_yaw` arguments to both parent launch files and forward all pose arguments.
6. Keep `air_ground_final_demo.launch` explicitly passing `use_visual_frame_yaw:=false` through the coordinate launch for compatibility. The later research launch will pass `use_visual_frame_yaw:=true`; do not silently change legacy results.
7. Run geometry unit tests and prepare these three manual pose cases, but do not execute them in the current OpenCode environment:

   ```text
   A: UAV yaw 0 deg, UGV yaw 0 deg
   B: UAV yaw 90 deg, UGV yaw -45 deg
   C: UAV yaw -120 deg, UGV yaw 150 deg
   ```

   In each external/manual case, place the UGV at the configured body-relative registration waypoint, cold-start the Demo, and require `FROZEN` without collision.
8. M1-A external verification must record registration completion, estimated heading, and evaluator-side Gazebo-truth transform error for the three cases.

## Current-Environment Constraints

- Strict RED -> GREEN TDD for every production behavior change.
- If an allowed bounded test fails unexpectedly, invoke `systematic-debugging` before fixing.
- Do not execute `roslaunch`, `roscore`, `rostest`, Gazebo, PX4 SITL, RViz, rosbag, topic wait/echo loops, or any long-running process.
- Allowed verification: pure unit tests, `py_compile`, XML well-formedness checks, bounded catkin builds, and static launch wiring inspection.
- Do not read or introduce Gazebo truth in autonomy code. Truth remains external/evaluator-only.
- Preserve Task 2 frame parameters, covariance/revision interfaces, and legacy one-shot behavior.
- The workspace has no Git metadata. Do not initialize Git or claim commits.

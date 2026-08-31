# Task 3 Report: Arbitrary Spawn Geometry and Relative Heading

## Status

PASS for all Task 3 behaviors that can be verified in the current environment.

- Body-relative registration geometry is implemented and covered by pure logic tests.
- Independent UAV/UGV yaw and registration `dx`/`dy` launch wiring is implemented and statically parsed.
- The research default estimates visual relative yaw; `air_ground_final_demo.launch` explicitly retains fixed-yaw behavior.
- Task 2 pure frame, covariance, revision, and one-shot regressions pass.
- Dynamic M1-A checks remain intentionally NOT EXECUTED and must be completed externally.
- No commit was created because this workspace has no Git metadata.

## RED/GREEN Evidence

### 1. Missing body-relative helper API

RED command:

```text
python3 -m unittest src/air_ground_bringup/test/test_registration_waypoint.py -v
```

Key RED output:

```text
FAIL: test_registration_offset_rotates_with_home_heading
AssertionError: unexpectedly None : uav_sphere_mission is missing registration_waypoint
Ran 1 test ... FAILED (failures=1)
```

This is the required initial failure caused specifically by the missing
`registration_waypoint` API.

Minimal GREEN implementation added the specified rotation:

```text
x = home_x + cos(home_yaw) * dx - sin(home_yaw) * dy
y = home_y + sin(home_yaw) * dx + cos(home_yaw) * dy
```

GREEN command:

```text
python3 -m unittest src/air_ground_bringup/test/test_registration_waypoint.py -v
```

Key GREEN output:

```text
test_registration_offset_rotates_with_home_heading ... ok
Ran 1 test ... OK
```

### 2. Mission registration command uses body-relative geometry

The first expanded test run exposed an `AttributeError` because the test fixture
did not provide the legacy field used by the old implementation. No production
change was made from that run. The fixture was corrected with a legacy sentinel
so the accepted RED was a behavioral assertion failure.

Accepted RED command:

```text
python3 -m unittest src/air_ground_bringup/test/test_registration_waypoint.py -v
```

Key RED output:

```text
FAIL: test_registration_command_uses_body_relative_xy
AssertionError: 3.6 != 2.4 within 6 places
Ran 4 tests ... FAILED (failures=1)
```

The minimal production change made `Mission.registration_command()` call the
real helper with `registration_dx` and `registration_dy`.

GREEN command and key output:

```text
python3 -m unittest src/air_ground_bringup/test/test_registration_waypoint.py -v
Ran 4 tests ... OK
```

The same suite covers heading wrap (`-3*pi/2`) and a nonzero lateral body offset
(`dx=1.6`, `dy=-0.4`) with hand-derived expected coordinates `(2.4, 4.6)`.
These cases exercise the same helper behavior introduced by the initial RED.

### 3. Every registration phase uses the same computed XY

The test executes the real `Mission.tick()` pure control flow with external ROS
side effects replaced by a bounded command capture. It covers:

- `MOVE_TO_REGISTRATION`
- `WAIT_REGISTRATION`
- `CLIMB_FOR_SCAN`
- `FRONT_SCAN`
- `FRONT_CONFIRM`

RED command:

```text
python3 -m unittest src/air_ground_bringup/test/test_registration_waypoint.py -v
```

Key RED output:

```text
FAIL ... (phase='CLIMB_FOR_SCAN'): 3.6 != 2.4
FAIL ... (phase='FRONT_SCAN'): 3.6 != 2.4
FAIL ... (phase='FRONT_CONFIRM'): 3.6 != 2.4
Ran 5 tests ... FAILED (failures=3)
```

The minimal GREEN computes `(registration_x, registration_y)` once per tick from
`registration_command()` and uses both coordinates in commands. This historical
suite's three failures established command reuse only; its odometry stayed far
from both reach points, so it did not establish reach-transition sensitivity.
Fix Round 1 below adds that separate evidence.

GREEN command and key output:

```text
python3 -m unittest src/air_ground_bringup/test/test_registration_waypoint.py -v
Ran 5 tests ... OK
```

### 4. Launch and visual-yaw wiring

The initial static test helper reported missing XML nodes as errors. It was
corrected before production edits to report assertion failures. The accepted RED
used XML/YAML parsers and had one failure for each missing production contract.

Accepted RED command:

```text
python3 -m unittest src/air_ground_bringup/test/test_launch_wiring.py -v
```

Key RED output:

```text
FAIL: test_parent_launches_forward_independent_uav_and_ugv_yaw
  missing arg named 'uav_yaw'
FAIL: test_registration_offsets_reach_the_mission_independently
  missing arg named 'registration_dx'
FAIL: test_research_default_uses_visual_yaw_while_legacy_demo_is_fixed_yaw
  False is not True
FAIL: test_uav_yaw_reaches_spawn_model_as_gazebo_yaw
  missing arg named 'yaw'
Ran 4 tests ... FAILED (failures=4)
```

Minimal GREEN added the launch arguments/forwarding, Gazebo `-Y`, visual-yaw
override path, and independent mission offset parameters.

GREEN command and key output:

```text
python3 -m unittest src/air_ground_bringup/test/test_launch_wiring.py -v
Ran 4 tests ... OK
```

### 5. Mission parameter consumption

This behavior was independently restarted under TDD rather than relying only on
launch XML. The test AST-isolates and executes the real constructor assignments
with a pure `get_param` function.

RED command and key output:

```text
python3 -m unittest src/air_ground_bringup/test/test_registration_waypoint.py -v
FAIL: test_mission_loads_independent_body_registration_offsets
AssertionError: None != 1.25
Ran 6 tests ... FAILED (failures=1)
```

GREEN command and key output:

```text
python3 -m unittest src/air_ground_bringup/test/test_registration_waypoint.py src/air_ground_bringup/test/test_launch_wiring.py -v
Ran 10 tests ... OK
```

No allowed test failed unexpectedly after an accepted RED. Therefore the
`systematic-debugging` workflow was not needed.

## Implementation

- Added `registration_waypoint(home_x, home_y, home_yaw, dx, dy)` to
  `uav_sphere_mission.py`.
- Replaced scalar `registration_offset` loading with independent
  `registration_dx` and `registration_dy` parameters.
- Made registration command generation and all five registration-related phases
  use the same body-rotated XY.
- Added UAV `yaw` to `uav_sitl.launch` and passed it to Gazebo spawn as `-Y`.
- Added and forwarded independent `uav_yaw` and `ugv_yaw` through both parent
  launch files.
- Replaced the legacy demo's registration offset launch API with
  `registration_dx`/`registration_dy`, forwarded directly to the mission.
- Changed the registration research default to `use_visual_frame_yaw: true`.
- Added a coordinate launch argument/override path and explicitly passed
  `use_visual_frame_yaw=false` from the legacy final demo.

## Static Launch Audit

`test_launch_wiring.py` parses launch XML and registration YAML rather than
grepping source text. It verifies:

- `air_ground_final_demo.launch` forwards UAV and UGV `x`, `y`, and `yaw` to
  `mvp_system.launch`.
- `mvp_system.launch` forwards UAV `x`, `y`, and `yaw` to `uav_sitl.launch` and
  UGV `x`, `y`, and `yaw` to `spawn_ugv.launch`.
- `uav_sitl.launch` declares `yaw=0.0`; parsed spawn arguments contain `-Y`
  immediately followed by `$(arg yaw)`.
- `registration_dx=1.6` and `registration_dy=0.0` flow from the legacy demo
  launch to the mission node, while a pure constructor test verifies the mission
  consumes both parameter names and defaults.
- `registration.yaml`, `coordinate_transform.launch`, and `mvp_system.launch`
  default visual frame yaw to true.
- `air_ground_final_demo.launch` explicitly sends false through
  `mvp_system.launch` to `coordinate_transform.launch` for fixed-yaw legacy
  compatibility.
- Task 2 input-frame values remain `map/base_link`,
  `ugv_0/odom/ugv_0/base_link`, and
  `iris_0/nadir_camera_optical_frame`.

## Bounded Verification

Fresh Task 3 verification:

```text
python3 -m unittest src/air_ground_bringup/test/test_registration_waypoint.py src/air_ground_bringup/test/test_launch_wiring.py -v
Ran 14 tests in 0.085s
OK
```

Fresh Task 2 pure compatibility verification:

```text
PYTHONPATH="src/air_ground_coordinate_transform/src" python3 -m unittest src/air_ground_coordinate_transform/test/test_registration_estimator.py src/air_ground_coordinate_transform/test/test_se2.py src/air_ground_coordinate_transform/test/test_odom_buffer.py -v
Ran 34 tests in 0.150s
OK
```

The 34 tests include exact input-frame validation, visual/fixed yaw estimator
geometry, covariance propagation and floors, survivor indexing/revision behavior,
and atomic one-shot registration.

Python syntax verification:

```text
python3 -m py_compile "src/air_ground_bringup/scripts/uav_sphere_mission.py" "src/air_ground_bringup/test/test_registration_waypoint.py" "src/air_ground_bringup/test/test_launch_wiring.py"
exit 0; no output
```

XML well-formedness verification:

```text
python3 -c 'import xml.etree.ElementTree as ET; paths=["src/air_ground_bringup/launch/uav_sitl.launch", "src/air_ground_bringup/launch/mvp_system.launch", "src/air_ground_bringup/launch/air_ground_final_demo.launch", "src/air_ground_coordinate_transform/launch/coordinate_transform.launch"]; [ET.parse(path) for path in paths]; print("XML_OK={}".format(len(paths)))'
XML_OK=4
```

Bounded catkin package build (120-second timeout):

```text
catkin_make --pkg air_ground_coordinate_transform air_ground_bringup
[100%] Built target coordinate_transform_node
Running command: "make -j12 -l12" in ".../build/air_ground_coordinate_transform"
Running command: "make -j12 -l12" in ".../build/air_ground_bringup"
exit 0
```

Catkin configuration traversed workspace dependencies and emitted existing
nonfatal VTK imported-target and Eigen/system-library metadata warnings. Neither
requested package failed configuration or compilation.

No `roslaunch` command, including `--check`, was run. No `roscore`, `rostest`,
Gazebo, PX4 SITL, RViz, rosbag, topic wait/echo loop, or other long-running process
was started. No Gazebo truth was read.

## Dynamic M1-A Checks Not Executed

The following cases are prepared for external/manual execution only:

```text
A: UAV yaw 0 deg, UGV yaw 0 deg
B: UAV yaw 90 deg, UGV yaw -45 deg
C: UAV yaw -120 deg, UGV yaw 150 deg
```

For each external case:

1. Configure UAV spawn `x`, `y`, and yaw.
2. Compute the UGV registration position externally as
   `(uav_x + cos(uav_yaw)*dx - sin(uav_yaw)*dy,
   uav_y + sin(uav_yaw)*dx + cos(uav_yaw)*dy)` and place the UGV there with the
   case's independent UGV yaw.
3. Cold-start the Demo.
4. Require registration state `FROZEN` with no collision.
5. Record registration completion and the estimated heading.
6. Record evaluator-side Gazebo-truth transform error. Truth must remain outside
   autonomy code.

None of these three cases was executed in this OpenCode environment.

## Modified Files

- `src/air_ground_bringup/launch/uav_sitl.launch`
- `src/air_ground_bringup/launch/mvp_system.launch`
- `src/air_ground_bringup/launch/air_ground_final_demo.launch`
- `src/air_ground_bringup/scripts/uav_sphere_mission.py`
- `src/air_ground_bringup/test/test_registration_waypoint.py`
- `src/air_ground_bringup/test/test_launch_wiring.py`
- `src/air_ground_coordinate_transform/config/registration.yaml`
- `src/air_ground_coordinate_transform/launch/coordinate_transform.launch`
- `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-3-report.md`

## Self-Review

- Scope is limited to the Task 3 brief's mission, launch, config, and minimal
  pure/static tests plus this report.
- No Task 2 estimator, covariance, revision, frame-validation, or one-shot
  implementation was modified.
- The mission phase test executes real helper, command, and `tick()` logic; it is
  not a source-text assertion.
- Static launch checks use bounded XML/YAML parsing and cover each forwarding
  edge, not just argument declarations.
- Recorded mutation evidence is limited to the two reach predicates exercised in
  Fix Round 1 below. Other behaviors have focused RED/GREEN evidence where
  recorded above, but no broader mutation claim is made without command output.
- Existing unrelated changes were not reverted.
- No Git repository was initialized and no commit is claimed.

## Fix Round 1

### Finding Disposition

Important Finding 1 is resolved. Four focused tests now execute the real
`Mission.tick()` reach predicates with `home=(2.0, 3.0)`, `home_yaw=pi/2`,
`registration_dx=1.6`, and `registration_dy=-0.4`:

- Rotated point `(2.4, 4.6, 1.5)` advances
  `MOVE_TO_REGISTRATION -> WAIT_REGISTRATION`.
- Legacy world-X point `(3.6, 3.0, 1.5)` does not advance
  `MOVE_TO_REGISTRATION`.
- Rotated point `(2.4, 4.6, 4.0)` advances
  `CLIMB_FOR_SCAN -> FRONT_SCAN`.
- Legacy world-X point `(3.6, 3.0, 4.0)` does not advance
  `CLIMB_FOR_SCAN`.

The reviewer correctly identified a test-evidence defect, not a production-logic
defect. Production was correct before this round and has no net change.

### Focused Test And Mutation Evidence

Current-production GREEN immediately after adding the tests:

```text
python3 -m unittest src.air_ground_bringup.test.test_registration_waypoint.RegistrationWaypointTest.test_move_to_registration_advances_at_rotated_waypoint src.air_ground_bringup.test.test_registration_waypoint.RegistrationWaypointTest.test_move_to_registration_does_not_advance_at_legacy_world_x_point src.air_ground_bringup.test.test_registration_waypoint.RegistrationWaypointTest.test_climb_for_scan_advances_at_rotated_waypoint src.air_ground_bringup.test.test_registration_waypoint.RegistrationWaypointTest.test_climb_for_scan_does_not_advance_at_legacy_world_x_point -v
Ran 4 tests in 0.026s
OK
```

This GREEN is not presented as an original feature RED because production was
already correct. Regression sensitivity was established by temporarily changing
both production reach predicates from `(registration_x, registration_y)` to the
former world-axis point `(home_x + registration_dx, home_y)`.

Temporary mutation RED, using the same focused command:

```text
test_move_to_registration_advances_at_rotated_waypoint ... FAIL
  'MOVE_TO_REGISTRATION' != 'WAIT_REGISTRATION'
test_move_to_registration_does_not_advance_at_legacy_world_x_point ... FAIL
  'WAIT_REGISTRATION' != 'MOVE_TO_REGISTRATION'
test_climb_for_scan_advances_at_rotated_waypoint ... FAIL
  'CLIMB_FOR_SCAN' != 'FRONT_SCAN'
test_climb_for_scan_does_not_advance_at_legacy_world_x_point ... FAIL
  'FRONT_SCAN' != 'CLIMB_FOR_SCAN'
Ran 4 tests in 0.028s
FAILED (failures=4)
```

Both predicates were then restored exactly to
`math.hypot(position.x - registration_x, position.y - registration_y)`.

Restored-production focused GREEN:

```text
python3 -m unittest src.air_ground_bringup.test.test_registration_waypoint.RegistrationWaypointTest.test_move_to_registration_advances_at_rotated_waypoint src.air_ground_bringup.test.test_registration_waypoint.RegistrationWaypointTest.test_move_to_registration_does_not_advance_at_legacy_world_x_point src.air_ground_bringup.test.test_registration_waypoint.RegistrationWaypointTest.test_climb_for_scan_advances_at_rotated_waypoint src.air_ground_bringup.test.test_registration_waypoint.RegistrationWaypointTest.test_climb_for_scan_does_not_advance_at_legacy_world_x_point -v
Ran 4 tests in 0.028s
OK
```

Final bounded verification for this round is recorded above: Task 3 `14/14`,
Task 2 `34/34`, `py_compile` exit 0, XML parse `XML_OK=4`, and bounded
`catkin_make --pkg air_ground_coordinate_transform air_ground_bringup` exit 0.

### Fix Round 1 Modified Files

- `src/air_ground_bringup/test/test_registration_waypoint.py`
- `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-3-report.md`

`uav_sphere_mission.py` was temporarily mutated only to prove test sensitivity
and then restored exactly; it has no net Fix Round 1 production change.

### Fix Round 1 Self-Review

- The new tests assert phase outcomes from real `Mission.tick()` control flow,
  not captured commands or source text.
- Positive and negative cases independently distinguish rotated XY from the
  legacy world-X point at nonzero heading.
- Temporary mutation output proves both reach predicates are protected.
- The three ledger-deferred Minor findings were not changed in this round.
- No unexpected test or build failure occurred, so `systematic-debugging` was
  not invoked.
- No prohibited ROS/simulation process or truth input was used.
- No commit was created.

## Concerns

- Dynamic registration completion, collision clearance, estimated-heading
  accuracy, and evaluator-side transform error are not established by bounded
  tests. They remain the required external M1-A acceptance work for cases A/B/C.

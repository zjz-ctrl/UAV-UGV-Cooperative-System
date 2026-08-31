# Task 3 Fresh Read-Only Review

## Verdicts

- **Spec Compliance: PASS.** The current production files implement the specified body-relative geometry, yaw forwarding, visual-yaw research default, and legacy fixed-yaw override. No current `registration_offset` consumer, truth leak, Task 2 frame regression, scope creep, or evident production dependency/build defect was found.
- **Code Quality: NEEDS CHANGES.** Production logic is currently correct by complete-file inspection and the bounded tests pass, but one required reach-check behavior is not exercised by its claimed regression test. Three smaller static-test/integration gaps remain.

## Findings Summary

- Critical: 0
- Important: 1
- Minor: 3
- Highest severity: Important

## Critical Findings

None.

## Important Findings

### 1. The phase regression test does not execute either rotated-XY reach transition

**Files:** `src/air_ground_bringup/test/test_registration_waypoint.py:188-216`, `src/air_ground_bringup/scripts/uav_sphere_mission.py:431-435`, `src/air_ground_bringup/scripts/uav_sphere_mission.py:448-452`, `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-3-report.md:90-117`

The test places odometry at `(100, 100)` for every phase and only asserts captured command coordinates. Consequently, both reach predicates remain false. Replacing either predicate with the former world-axis point, or otherwise using the wrong XY in a reach check, would leave all ten Task 3 tests green even though the UAV could reach the rotated command and then time out without advancing. The current production predicates do use `registration_x` and `registration_y`, so this is a regression-test and report-evidence defect rather than a current mission-logic defect. The report's statement that the phase suite covers use in "commands and reach checks" is not supported by that suite.

**Minimal fix:** Add focused `Mission.tick()` cases with odometry at the rotated waypoint and assert `MOVE_TO_REGISTRATION -> WAIT_REGISTRATION` and `CLIMB_FOR_SCAN -> FRONT_SCAN`; also assert the former unrotated world-X point does not trigger those transitions for a nonzero heading.

## Minor Findings

### 1. UGV yaw is checked only to the include boundary, not through the spawn leaf

**Files:** `src/air_ground_bringup/test/test_launch_wiring.py:53-79`, `src/air_ground_ugv_gazebo/launch/spawn_ugv.launch:6-9`

The static test proves that `mvp_system.launch` passes `ugv_yaw` into an include argument named `yaw`, but it never parses `spawn_ugv.launch`. If that leaf dropped the argument or passed it with the wrong Gazebo option, the test would remain green. Complete-file review confirms the current leaf correctly declares `yaw` and uses `-Y $(arg yaw)`; Gazebo `spawn_model -Y` is a yaw angle in radians, matching the launch values and mission quaternion-derived radians.

**Minimal fix:** Parse `spawn_ugv.launch` in the existing test and assert its `yaw` declaration and adjacent `-Y $(arg yaw)` spawn tokens, mirroring the UAV leaf assertion.

### 2. The legacy fixed-yaw test does not protect rosparam override precedence

**Files:** `src/air_ground_bringup/test/test_launch_wiring.py:96-130`, `src/air_ground_coordinate_transform/launch/coordinate_transform.launch:3-6`, `src/air_ground_coordinate_transform/config/registration.yaml:28`

The test independently finds the YAML default and the node-local `<param>`, but does not assert their order. If the node-local parameter were moved before the YAML load, `registration.yaml` would overwrite a legacy `false` argument with the research `true` default while this static test still passed. The current launch is correctly ordered with `<rosparam>` first and the argument-backed `<param>` second.

**Minimal fix:** Assert that the `use_visual_frame_yaw` `<param>` occurs after the registration YAML `<rosparam>` within the coordinate node, or evaluate the node's effective parameter precedence in a bounded pure launch-structure helper.

### 3. Task 3 tests are not registered with catkin and their YAML dependency is undeclared

**Files:** `src/air_ground_bringup/CMakeLists.txt:1-31`, `src/air_ground_bringup/package.xml:8-41`, `src/air_ground_bringup/test/test_launch_wiring.py:8`

The two Task 3 suites pass when invoked directly, as required by the brief, but `air_ground_bringup` has no testing block that registers them. A normal `catkin_make run_tests` can therefore report success without running these regressions. In a clean dependency-resolved environment, `test_launch_wiring.py` also imports `yaml` without a package test dependency for PyYAML.

**Minimal fix:** Register both pure suites under `CATKIN_ENABLE_TESTING` and declare the corresponding test runner plus `python3-yaml` test dependencies.

## Technical Assessment

### Geometry and mission use

- `registration_waypoint()` implements `home + R(home_yaw) * [dx, dy]` with standard ENU/FLU semantics: positive `dx` is body-forward and positive `dy` is body-left.
- Trigonometric periodicity handles wrapped headings; the `-3*pi/2` test is equivalent to `pi/2` and passes.
- `home_yaw` is extracted from the MAVROS odometry quaternion in radians, consumed by `sin`/`cos`, and sent as a radian `PositionCommand.yaw`. No degree/radian conversion error was found.
- `MOVE_TO_REGISTRATION`, `WAIT_REGISTRATION`, `CLIMB_FOR_SCAN`, `FRONT_SCAN`, and `FRONT_CONFIRM` currently command the same rotated XY. Both applicable reach predicates currently use that XY.

### Launch and compatibility wiring

- UAV yaw flows `air_ground_final_demo.launch -> mvp_system.launch -> uav_sitl.launch -> spawn_model -Y`.
- UGV yaw flows `air_ground_final_demo.launch -> mvp_system.launch -> spawn_ugv.launch -> spawn_model -Y`.
- `registration_dx` and `registration_dy` are declared independently by the launch that owns the mission node and are passed directly to the mission's private parameters; no unnecessary intermediate forwarding layer is required.
- The research path defaults to visual yaw in YAML, `coordinate_transform.launch`, and `mvp_system.launch`. The compatibility demo sends literal `false`; the coordinate node loads YAML before applying the argument-backed override, so the effective legacy value is false.
- Task 2 input frames remain `map/base_link`, `ugv_0/odom -> ugv_0/base_link`, and `iris_0/nadir_camera_optical_frame`.

### Compatibility, scope, and truth

- Workspace-wide source search found no live consumer of `registration_offset`; remaining occurrences are plans, reports, test names, and unused sentinel attributes in tests. No backward-compatibility shim is justified by current consumers.
- No Gazebo truth, model-state topic, or equivalent evaluator truth input is consumed by the reviewed mission or registration implementation.
- The additional static launch test is aligned with Task 3 behavior; no material scope creep was found.

## Evidence Boundary

Fresh bounded verification performed by this reviewer:

```text
python3 -m unittest src/air_ground_bringup/test/test_registration_waypoint.py src/air_ground_bringup/test/test_launch_wiring.py -v
Ran 10 tests in 0.039s
OK

PYTHONPATH="src/air_ground_coordinate_transform/src" python3 -m unittest src/air_ground_coordinate_transform/test/test_registration_estimator.py src/air_ground_coordinate_transform/test/test_se2.py src/air_ground_coordinate_transform/test/test_odom_buffer.py -v
Ran 34 tests in 0.129s
OK

py_compile: 4 reviewed Python files passed
XML parse: 7 scope/direct-context launch files passed
```

The current GREEN state is independently established. The historical RED sequence is internally plausible and identifies specific expected failures, but cannot be independently reconstructed from the complete current files because the workspace has no Git history or retained failing snapshots. The report's broad mutation-check statement at `task-3-report.md:328-331` has no included mutation command/output and must be read as an implementation claim, not fresh reproducible evidence; the reach-check gap above shows that it must not be generalized beyond the mutations actually covered.

No catkin build was rerun during this read-only review, so the report's historical build output was not independently re-established.

## Residual Risk

The prohibited dynamic M1-A cases remain unexecuted. Therefore runtime PX4/Gazebo spawn-yaw behavior, first-odometry home-yaw settling, registration completion, collision clearance, and evaluator-side transform error remain external acceptance risks, not code findings from this review. No ROS launch, ROS master, simulator, PX4, RViz, rosbag, topic wait, or truth read was performed.

## Fix Round 1 Scoped Re-review

### Important Finding 1 Disposition

**RESOLVED.** `test_registration_waypoint.py:103-128` builds bounded mission fixtures around the production `Mission` class loaded from `uav_sphere_mission.py`, and the four tests at `test_registration_waypoint.py:250-276` call the real `Mission.tick()` method rather than checking source text or captured XY commands.

- At rotated registration XY `(2.4, 4.6)`, the tests establish `MOVE_TO_REGISTRATION -> WAIT_REGISTRATION` at registration altitude and `CLIMB_FOR_SCAN -> FRONT_SCAN` at scan altitude.
- At former world-X XY `(3.6, 3.0)`, the tests establish that neither transition occurs.
- Both fixtures use zero speed, exact target altitude, a nonzero `pi/2` home heading, and the same `dx=1.6`, `dy=-0.4`; therefore the asserted phase difference is specifically sensitive to the horizontal reach predicate rather than speed, altitude, timeout, or command publication.

Current production is restored at `uav_sphere_mission.py:431-435` and `uav_sphere_mission.py:448-452`: both predicates use `registration_x` and `registration_y`. Fresh reviewer execution produced focused `4/4 OK` and full Task 3 `14/14 OK`.

The temporary-mutation output in `task-3-report.md:367-395` is technically credible. Under the reported replacement, the rotated and former points are 2.0 m apart while tolerance is 0.25 m, so each positive test must fail to advance and each negative test must incorrectly advance, matching all four recorded failures. The final source and fresh GREEN establish restoration. Because this repository has no Git history or retained mutated snapshot, the historical edit/run/restore chronology itself remains reported evidence rather than independently reconstructable evidence; this does not leave the original regression-sensitivity finding open.

### New Critical/Important Findings

None. The test/report-only fix introduces no new Critical or Important breakage in the reviewed scope.

The three original Minor findings remain ledger-deferred and are neither reopened nor dispositioned by this scoped re-review.

### Scoped Verdict

**PASS.** Original Important Finding 1 is closed; new Critical findings: 0; new Important findings: 0.

# Task 1 Report: SE(2) and Odometry Interpolation Primitives

## Status

DONE

No commit was created because this workspace has no Git metadata, as stated in the task brief.

## Implementation

- Added the importable `air_ground_coordinate_transform` Python package and catkin `setup.py` registration.
- Added `wrap_angle`, `wrap_xyyaw`, `matrix_from_xyyaw`, `xyyaw_from_matrix`, `compose`, and `inverse` using the `[x, y, yaw]` convention and normalized composed yaw.
- Added `transform_pose_covariance` with the two Jacobians and first-order covariance expression specified verbatim by the brief, including heading lever-arm uncertainty.
- Added `OdomBuffer(maxlen, max_bracket)` with the authoritative `append(stamp, x, y, z, yaw)`, `append_odometry(message)`, `interpolate(stamp)`, and `distance_since(stamp)` interfaces.
- Stored `(stamp_sec, x, y, z, yaw)` samples in a bounded deque. Interpolation returns planar `[x, y, yaw]`, uses linear x/y interpolation, shortest-path wrapped yaw interpolation, per-side `max_bracket` rejection, and a recent preceding sample when no following sample exists.
- Kept ROS-message extraction in `append_odometry`; the module imports no ROS package and has no ROS side effects at import time.
- Registered both test files through guarded `catkin_add_nosetests` declarations and added the required `python3-nose` test dependency.

## RED Evidence

### SE(2)

Command:

```bash
source devel/setup.bash
python3 -m unittest discover -s src/air_ground_coordinate_transform/test -p 'test_se2.py' -v
```

Key failure:

```text
ImportError: Failed to import test module: test_se2
ModuleNotFoundError: No module named 'air_ground_coordinate_transform'
Ran 1 test in 0.000s
FAILED (errors=1)
```

This was the expected missing-feature failure: the new Python package and therefore `air_ground_coordinate_transform.se2` did not exist. Because this workspace had no pre-existing top-level Python package, Python reported the missing package rather than the brief's more specific missing `.se2` submodule.

### Odometry Buffer

After SE(2) was green, command:

```bash
source devel/setup.bash
PYTHONPATH="src/air_ground_coordinate_transform/src:${PYTHONPATH}" \
  python3 -m unittest discover -s src/air_ground_coordinate_transform/test -p 'test_odom_buffer.py' -v
```

Key failure:

```text
ImportError: Failed to import test module: test_odom_buffer
ModuleNotFoundError: No module named 'air_ground_coordinate_transform.odom_buffer'
Ran 1 test in 0.000s
FAILED (errors=1)
```

This exactly demonstrated that the requested odometry module was absent while the already-implemented package and SE(2) module were importable.

## GREEN Evidence

Intermediate SE(2) command with the source package on `PYTHONPATH`:

```bash
source devel/setup.bash
PYTHONPATH="src/air_ground_coordinate_transform/src:${PYTHONPATH}" \
  python3 -m unittest discover -s src/air_ground_coordinate_transform/test -p 'test_se2.py' -v
```

Summary: 6 tests ran in 0.001s, all passed (`OK`).

Intermediate odometry command with the source package on `PYTHONPATH`:

```bash
source devel/setup.bash
PYTHONPATH="src/air_ground_coordinate_transform/src:${PYTHONPATH}" \
  python3 -m unittest discover -s src/air_ground_coordinate_transform/test -p 'test_odom_buffer.py' -v
```

Summary: 8 tests ran in 0.001s, all passed (`OK`).

The brief's exact focused command initially still had two import errors because the existing `devel` space predated `catkin_python_setup()`. The required package build regenerated the devel-space Python package link. The exact command was then rerun for fresh final evidence:

```bash
source devel/setup.bash
python3 -m unittest discover -s src/air_ground_coordinate_transform/test -p 'test_*.py' -v
```

Complete passing summary:

```text
test_append_odometry_converts_message_without_ros_imports ... ok
test_distance_since_accumulates_planar_distance ... ok
test_interpolates_translation_linearly ... ok
test_interpolates_yaw_across_pi_by_shortest_path ... ok
test_rejects_interpolation_when_one_side_is_outside_max_bracket ... ok
test_rejects_preceding_sample_outside_max_bracket ... ok
test_returns_pose_at_exact_timestamp ... ok
test_returns_preceding_sample_within_max_bracket ... ok
test_compose_with_identity_preserves_transform ... ok
test_compose_with_inverse_returns_identity ... ok
test_matrix_round_trip_preserves_pose ... ok
test_transform_covariance_includes_heading_lever_arm ... ok
test_wrap_angle_normalizes_to_principal_range ... ok
test_wrap_xyyaw_normalizes_heading_only ... ok

Ran 14 tests in 0.002s
OK
```

Fresh required build command:

```bash
catkin_make --pkg air_ground_coordinate_transform
```

Build summary:

```text
[100%] Built target coordinate_transform_node
```

The command exited successfully. The first configuration pass also found `/usr/bin/nosetests3` and registered the package; it emitted pre-existing workspace warnings from unrelated packages, with no new error from `air_ground_coordinate_transform`.

## Modified Files

- Created `src/air_ground_coordinate_transform/setup.py`
- Created `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/__init__.py`
- Created `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/se2.py`
- Created `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/odom_buffer.py`
- Created `src/air_ground_coordinate_transform/test/test_se2.py`
- Created `src/air_ground_coordinate_transform/test/test_odom_buffer.py`
- Modified `src/air_ground_coordinate_transform/CMakeLists.txt`
- Modified `src/air_ground_coordinate_transform/package.xml`

## Self-Review

- Scope audit: only Task 1 files from the brief were created or modified; no Task 2 registrar or integration behavior was added.
- Interface audit: all required function names and all four authoritative `OdomBuffer` methods are present with the specified argument order.
- Numeric audit: yaw is normalized to `[-pi, pi)`, interpolation crosses `+pi/-pi` by the shortest path, and covariance uses the exact brief Jacobians.
- Isolation audit: production math code contains no `rospy`, Gazebo service, Gazebo model-state, launch, parameter-server, publisher, or subscriber dependency.
- Runtime audit: no existing script, launch, config, executable, or install declaration was removed or behaviorally changed; CMake changes only add Python/test registration.
- Test audit: tests exercise real NumPy/math code and a structurally complete odometry-shaped message without mocking ROS behavior. They cover every requested SE(2), covariance, timestamp, bracketing, interpolation, adapter, and planar-distance behavior.
- Mutation audit: wrong wrap direction, ordinary rather than shortest-path yaw interpolation, z-inclusive distance, missing heading lever arm, missing bracket rejection, or incorrect quaternion yaw conversion would each fail at least one focused test.

## Problems / Concerns

- The initial SE(2) RED message named the absent top-level package rather than the absent `.se2` submodule because no Python package scaffold existed before the test. It still failed for the required missing implementation, and the odometry RED subsequently produced the exact missing-submodule form.
- The brief does not explicitly state unavailable-data return values. The implementation consistently returns `None` when interpolation has no acceptable preceding/bracketing data, matching the requested rejection behavior.
- No implementation blocker remains. No Gazebo truth source was introduced, and no runtime script or launch behavior changed.

## Review Fixes

### Fixes Applied

- Declared NumPy as a production runtime dependency with `<exec_depend>python3-numpy</exec_depend>`.
- Added a non-commuting composition test with a literal expected matrix, distinguishing `first @ second` from `second @ first`.
- Added full mean and covariance assertions for non-zero correlated point and transform covariance matrices. The expected mean and every covariance entry are explicit hand-derived values.
- Added the complementary bracket test where the preceding sample is within `max_bracket` but the following sample is too far away.
- Added an isolated subprocess import test whose import hook rejects ROS modules, proving `odom_buffer.py` remains importable without loading ROS.
- Added bounded-deque eviction coverage for `maxlen`.
- Added planar-distance coverage from an interpolated, non-exact start timestamp.

No permanent SE(2) or odometry implementation change was needed: the review findings identified missing discrimination and edge-case coverage. Temporary regressions were used only to prove that each added test detects its intended bug, then removed before GREEN verification.

### Review RED Evidence

After adding all six behavior tests, temporary mutations reversed transform composition, removed covariance cross-correlations, omitted the following-side bracket check, disabled deque eviction, snapped non-exact distance starts to the next sample, and imported `rospy` from `odom_buffer.py`.

Command:

```bash
source devel/setup.bash
python3 -m unittest discover -s src/air_ground_coordinate_transform/test -p 'test_*.py' -v
```

Key failure output:

```text
test_distance_since_interpolates_non_exact_start ... FAIL
test_maxlen_evicts_oldest_sample ... FAIL
test_module_import_does_not_load_ros_modules ... FAIL
test_rejects_interpolation_when_following_sample_is_too_far ... FAIL
test_compose_applies_second_transform_then_first_transform ... FAIL
test_transform_covariance_propagates_full_correlated_covariances ... FAIL

AssertionError: 3.0 != 4.0 within 7 places (1.0 difference)
AssertionError: array([1., 0., 0.]) is not None
RuntimeError: ROS import attempted: rospy
AssertionError: array([0.5, 0. , 0. ]) is not None
compose actual translation [5.0, 3.0], expected [-3.0, 5.0]
covariance mismatch: 88.9%, maximum absolute difference 0.09

Ran 20 tests in 0.238s
FAILED (failures=6)
```

Each failure was expected and mapped one-to-one to a review finding. The remaining 14 tests passed, showing the failures came from the targeted temporary regressions rather than test setup errors.

### Review GREEN Evidence

After restoring the correct implementation and adding the manifest dependency, focused command:

```bash
source devel/setup.bash
python3 -m unittest discover -s src/air_ground_coordinate_transform/test -p 'test_*.py' -v
```

Complete passing summary:

```text
test_append_odometry_converts_message_without_ros_imports ... ok
test_distance_since_accumulates_planar_distance ... ok
test_distance_since_interpolates_non_exact_start ... ok
test_interpolates_translation_linearly ... ok
test_interpolates_yaw_across_pi_by_shortest_path ... ok
test_maxlen_evicts_oldest_sample ... ok
test_module_import_does_not_load_ros_modules ... ok
test_rejects_interpolation_when_following_sample_is_too_far ... ok
test_rejects_interpolation_when_one_side_is_outside_max_bracket ... ok
test_rejects_preceding_sample_outside_max_bracket ... ok
test_returns_pose_at_exact_timestamp ... ok
test_returns_preceding_sample_within_max_bracket ... ok
test_compose_applies_second_transform_then_first_transform ... ok
test_compose_with_identity_preserves_transform ... ok
test_compose_with_inverse_returns_identity ... ok
test_matrix_round_trip_preserves_pose ... ok
test_transform_covariance_includes_heading_lever_arm ... ok
test_transform_covariance_propagates_full_correlated_covariances ... ok
test_wrap_angle_normalizes_to_principal_range ... ok
test_wrap_xyyaw_normalizes_heading_only ... ok

Ran 20 tests in 0.122s
OK
```

Required package build:

```bash
catkin_make --pkg air_ground_coordinate_transform
```

Build result:

```text
-- Configuring done
-- Generating done
[100%] Built target coordinate_transform_node
```

The command exited successfully. The configuration output continued to include pre-existing missing VTK tool/library and unrelated catkin `Eigen`/`system_lib` warnings from other workspace packages; no new warning or error was attributed to `air_ground_coordinate_transform`.

### Review Modified Files

- Modified `src/air_ground_coordinate_transform/test/test_se2.py`
- Modified `src/air_ground_coordinate_transform/test/test_odom_buffer.py`
- Modified `src/air_ground_coordinate_transform/package.xml`
- Appended this review-fix evidence to `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-1-report.md`

The temporary mutations to `se2.py` and `odom_buffer.py` were fully removed; their final behavior remains the original correct Task 1 implementation.

### Review Self-Check and Concerns

- All six review findings have direct final coverage or a manifest fix.
- The covariance fixture is symmetric but non-diagonal, and both input covariance matrices contain positive and negative cross-correlations.
- The ROS isolation test executes a fresh interpreter and fails on attempted imports of `rospy`, `roslib`, `geometry_msgs`, `nav_msgs`, `tf`, or `tf2_ros`.
- No Task 2 file or behavior was added, and no subagent or reviewer was dispatched.
- No commit was created because the workspace has no Git metadata.
- Residual concern: package build output contains unrelated pre-existing workspace warnings, but the requested package target builds successfully.

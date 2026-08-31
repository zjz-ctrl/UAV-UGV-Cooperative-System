# Task 2 Report: Pure Robust One-Shot Registration

## Status

INITIAL BOUNDED PASS RECORDED; SUPERSEDED BY REVIEW FIX ROUNDS 1 AND 2 BELOW. Dynamic ROS/cold-start acceptance remains pending external execution.

No commit was created. The workspace has no Git metadata, and Git was not initialized.

## Partial Audit

The interrupted implementation had already created or changed the main Task 2 artifacts but had not written a report:

- `registration_estimator.py` already defined `RegistrationSample`, `BatchEstimate`, and `RobustBatchEstimator`.
- `takeoff_registration.py` already imported the Task 1 `OdomBuffer` and SE(2) helpers, accumulated estimator samples, froze once, and published the legacy registration topics plus the new estimate/revision topics.
- The focused estimator unit test, ROS node test, rostest launch file, CMake test registration, and covariance-floor configuration were present.
- No Task 2 report or recoverable command transcript established the original test-before-implementation order.
- Task 1 files and their behavior were retained. No Task 1 or unrelated user change was reverted.

Two implementation issues were found during takeover:

- The initial circular yaw center used all samples. A one-sided group of gross translation outliers could therefore move the yaw center far enough to reject every genuine inlier.
- The ROS adapter projected the static-camera/observation/board chain to SE(2) before applying the UGV inverse. This was not generally equivalent to the brief's required complete matrix equation when the intermediate transform contains non-planar rotation.

The first invocation of the brief's focused Python command also failed during test collection with `ModuleNotFoundError: No module named 'air_ground_coordinate_transform'`. Systematic root-cause analysis showed that the shell contained only the ROS installation on `PYTHONPATH`; the catkin source package was valid and the devel space exposed it after `source devel/setup.bash`. This was an environment setup failure, not an estimator failure. The sourced command was used for all behavioral evidence.

## Implementation

- Kept the pure, ROS-independent dataclasses and estimator API required by the brief.
- Computes the robust translation center with component-wise medians and applies the translation gate before computing the circular yaw center. Translation-rejected gross outliers can no longer contaminate the yaw gate.
- Recomputes the final median translation and circular yaw mean from effective inliers.
- Computes wrapped residual covariance and combines it with mean input measurement covariance, divides by effective inlier count, adds configurable translation/yaw variance floors, and symmetrizes the result.
- Returns `None` when the input count, translation-survivor count, or combined inlier count is below `min_samples`.
- Uses Task 1 `OdomBuffer` instances for timestamp interpolation and Task 1 SE(2) helpers for frozen-frame and downstream pose composition.
- The initial implementation placed the factors in the required multiplication order:

  ```text
  origin_to_uav_odom * uav * base_camera * observation * inverse(ugv * base_board)
  ```

  However, the initial report overstated this as complete 3-D preservation: UAV and UGV odometry had already been reduced to planar poses, and the observation covariance had not been propagated through this chain. Review Fix Round 1 corrects both defects.
- Stops accepting observations as soon as the first estimate is frozen, preserving one-shot behavior.
- Preserves latched `/air_ground/registration/frozen`, `/status`, `/valid`, `/inlier_count`, origin-to-UAV-odom TF, and origin-to-UGV-odom TF behavior.
- Publishes latched `/air_ground/registration/estimate` as `PoseWithCovarianceStamped`, including all `(x, y, yaw)` covariance and cross-covariance slots, and latched `/air_ground/registration/revision` as `UInt32`. Initial revision is `0`; the first and only freeze publishes revision `1`.
- Retains the written rostest for initial status, wrong-frame sample rejection, first freeze, covariance-bearing estimate, TF, and rejection of a contradictory second batch.

## TDD Evidence

### Unrecoverable Original Evidence

原始 RED 证据因前代理中断丢失。

There is no Git history, report, or retained terminal transcript that can prove the interrupted implementer's original RED-before-GREEN ordering. The existing tests passing at takeover cannot reconstruct that chronology, and this report does not claim that they can.

### Takeover RED/GREEN

A new test, `test_translation_outliers_do_not_bias_yaw_gate`, was added before the takeover fix. It names the regression: samples already rejected by the translation gate must not influence the yaw gate.

RED command:

```bash
source devel/setup.bash
python3 -m unittest src/air_ground_coordinate_transform/test/test_registration_estimator.py -v
```

Observed RED:

```text
test_translation_outliers_do_not_bias_yaw_gate ... FAIL
AssertionError: unexpectedly None
Ran 5 tests in 0.004s
FAILED (failures=1)
```

After applying the translation gate before the circular yaw center, the same command ran 5 tests successfully.

### Temporary Mutation Evidence

The estimator was temporarily mutated to disable translation rejection, use a linear yaw mean, and omit input covariance/floors. The focused command then produced four expected failures:

```text
test_circular_mean_handles_samples_across_wrapped_yaw_boundary ... FAIL
test_covariance_includes_input_uncertainty_and_configured_floors ... FAIL
test_returns_none_when_too_few_samples_survive_residual_gates ... FAIL
test_translation_outliers_do_not_bias_yaw_gate ... FAIL
Ran 5 tests in 0.005s
FAILED (failures=4)
```

The temporary mutation was fully removed. A subsequent focused run passed all 5 tests. This is mutation-sensitivity evidence only; it is not represented as original TDD chronology.

## Bounded Verification Evidence

### Pure Tests

Fresh final command:

```bash
source devel/setup.bash
python3 -m unittest \
  src/air_ground_coordinate_transform/test/test_se2.py \
  src/air_ground_coordinate_transform/test/test_odom_buffer.py \
  src/air_ground_coordinate_transform/test/test_registration_estimator.py -v
```

Result:

```text
Ran 25 tests in 0.129s
OK
```

This includes 5 Task 2 estimator tests and all 20 existing pure Task 1 tests. No test requiring a ROS master was included.

### Python Compilation

Command:

```bash
python3 -m py_compile \
  src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py \
  src/air_ground_coordinate_transform/scripts/takeoff_registration.py \
  src/air_ground_coordinate_transform/test/test_registration_estimator.py \
  src/air_ground_coordinate_transform/test/test_registration_node.py
```

Result: exit status `0`, no output.

### XML Parsing

Python `xml.etree.ElementTree` parsed these four files:

- `src/air_ground_coordinate_transform/test/registration_node.test`
- `src/air_ground_coordinate_transform/launch/coordinate_transform.launch`
- `src/air_ground_coordinate_transform/package.xml`
- `src/air_ground_bringup/launch/air_ground_final_demo.launch`

Result: `parsed 4 XML files` with exit status `0`. This checks XML well-formedness only; it is not a substitute for ROS launch resolution.

### Bounded Catkin Build

Fresh final command:

```bash
catkin_make --pkg air_ground_coordinate_transform air_ground_bringup
```

Result: exit status `0`, including:

```text
[100%] Built target coordinate_transform_node
```

The earlier configuring build emitted pre-existing workspace warnings about missing VTK utility/library artifacts and unrelated `Eigen`/`system_lib` declarations. No warning or error was attributed to the Task 2 package changes. The fresh final incremental build was clean and successful.

### Residual Process Check

The single permitted `pgrep` invocation checked for ROS core/master/launch/test, Gazebo, PX4, MAVROS, RViz, and rosbag processes. It returned no matches. No forbidden long-running process was started during this takeover.

## Long-Process Checks Not Run

The following were deliberately not run under the operating ruling and remain pending external execution:

- `rostest air_ground_coordinate_transform registration_node.test`
- `roslaunch --check air_ground_bringup air_ground_final_demo.launch`
- Compatibility Demo cold-start via `air_ground_final_demo.launch`
- M0-B dynamic observations: `FROZEN -> OVERWATCH`, UGV `ARRIVED`, revision staying `1`, final stop near `0.76 m`, and post-shutdown process state

Therefore no dynamic registration-node, TF timing, launch-resolution, cold-start, mission-state, or final-position result is claimed as passing in this environment.

No Gazebo truth topic or service was read or introduced.

## Modified Files

Task 2 implementation/test files present from the interrupted implementation and completed by this takeover:

- Created `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py`
- Modified `src/air_ground_coordinate_transform/scripts/takeoff_registration.py`
- Created `src/air_ground_coordinate_transform/test/test_registration_estimator.py`
- Created `src/air_ground_coordinate_transform/test/registration_node.test`
- Created `src/air_ground_coordinate_transform/test/test_registration_node.py`
- Modified `src/air_ground_coordinate_transform/CMakeLists.txt` to register the estimator test and rostest
- Modified `src/air_ground_coordinate_transform/config/registration.yaml` to configure covariance floors
- Created `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-2-report.md`

Files directly edited during this takeover were `registration_estimator.py`, `takeoff_registration.py`, `test_registration_estimator.py`, and this report. The other listed Task 2 changes were audited and retained from the interrupted implementation.

## Self-Review

- Interface audit: all four requested data/API products have the specified names and fields.
- Robustness audit: randomized inliers/outliers, wrapped yaw, survivor count, PSD/symmetric covariance, input covariance/floors, and translation-before-yaw gating are covered by pure tests.
- Initial equation audit, superseded by Review Fix Round 1: factor order was correct, but UAV/UGV odometry was projected before multiplication. This line must not be read as evidence of full 3-D preservation in the pre-review implementation.
- Compatibility audit: legacy registration topics and both existing TF edges remain; estimate/revision are additive and latched; frozen callbacks reject all later observations.
- Initial covariance slot audit, superseded by Review Fix Round 1: all nine entries mapped to ROS rows/columns `(0, 1, 5)`, but this established slot placement only and did not establish propagation into the registration estimate frame.
- Test audit: the estimator tests exercise real NumPy/math code without ROS imports or mocks. The ROS integration test is written and registered but was not executed.
- Scope audit: no Gazebo truth, experiment package, Task 3 behavior, Git initialization, commit, subagent, or reviewer was used.
- Restoration audit: the temporary mutation is absent, and the final focused and complete pure-test runs are green.

## Concerns

- Dynamic ROS behavior remains unverified locally by explicit ruling. The written rostest may still expose ROS timing, topic-latching, TF-listener, or launch wiring issues when externally run.
- Static XML parsing cannot resolve substitutions, executable discovery, parameters, or runtime dependencies as `roslaunch --check` would.
- Superseded by Review Fix Round 1: the pre-review implementation re-anchored the fixed-yaw mean while retaining the visual covariance. The current implementation transforms the exact visual survivors and their covariances through the re-anchor Jacobian, then recomputes mean/covariance/count/stamp with deterministic yaw variance defined by the configured floor.
- `RobustBatchEstimator` assumes finite, correctly shaped, positive-semidefinite sample covariances. Malformed-input validation was not added because it is outside the Task 2 brief.
- The workspace is unversioned, so exact pre-interruption diffs and original file authorship cannot be reconstructed.

## Review Fix Round 1

### Status

All 8 read-only review findings were accepted after verification against the current code, Task 1 APIs, the Task 2 brief, launch/config conventions, and package metadata. All permitted bounded checks pass. Dynamic ROS and compatibility acceptance remain pending external execution.

No subagent or reviewer was dispatched. No commit was created, and Git was not initialized.

### Finding Disposition

| Finding | Disposition | Evidence and resolution |
|---|---|---|
| Important 1: odometry was planarized before the 3-D chain | Accepted | Task 1 `OdomBuffer.interpolate()` returned only `[x,y,yaw]`, and the adapter rebuilt both poses with zero z/roll/pitch. Added additive `interpolate_full()` preserving translation and quaternion with SLERP while leaving the established planar method unchanged. The adapter now multiplies full odometry poses and projects only the completed transform. |
| Important 2: observation covariance was copied without propagation | Accepted | The old code selected rows/columns `(0,1,5)` directly in the camera frame. Added a checked central-difference 3x6 Jacobian of the complete chain and final SE(2) extraction. Sample covariance is `J C J^T`, symmetrized and projected to PSD before estimation. |
| Important 3: fixed-yaw mean/mask/covariance were inconsistent | Accepted | The default compatibility path independently reconstructed a second mask and retained visual covariance. `estimate_with_inliers()` now returns the exact survivor indices from the estimator calculation. `fixed_yaw_estimate()` transforms only those samples and their covariances through the re-anchor Jacobian, recomputes batch mean/covariance/count/stamp, and defines deterministic fixed-yaw uncertainty as the configured yaw variance floor. |
| Important 4: input frames were not validated | Accepted | Static audit found MAVROS `/local_position/odom` defaults to `map -> base_link`, while the output TF alias is `iris_0/odom`; the UGV plugin publishes `ugv_0/odom -> ugv_0/base_link`; ChArUco publishes the optical frame. Added separate input parent/child/frame parameters and strict non-empty equality checks before origin/deque/buffer acceptance. Current Demo defaults remain accepted without conflating output TF names. |
| Important 5: freeze was not atomic | Accepted | The old callback guard and `try_freeze()` had no critical-section recheck. Added ROS-independent `OneShotRegistrationState`; its lock serializes sample append/estimation, rechecks revision under lock, and stores the frozen value with the only `0 -> 1` transition before any caller publishes it. |
| Important 6: direct `tf` dependency absent | Accepted | Production and rostest import `tf.transformations`. Added `tf` to catkin components/CATKIN_DEPENDS and package build, build-export, and exec dependencies. |
| Minor 1: rostest coverage too planar/exact/narrow | Accepted | Expanded the written rostest with initial revision `0`, empty/wrong frames for each input stream, nonzero 3-D extrinsics, roll/pitch, midpoint odometry interpolation, anisotropic/correlated covariance checks in all nine planar ROS slots, both legacy TF edges, and final one-shot revision retention. It was deliberately not run. |
| Minor 2: report overstated static evidence | Accepted | Corrected the initial status/equation/covariance wording above. The report now separates factor order, full 3-D odometry preservation, covariance propagation, ROS slot mapping, and unrun dynamic evidence. |

No reviewer suggestion was rejected or modified for compatibility reasons. The additive full-pose API preserves every Task 1 planar call/return contract.

### Review RED/GREEN Evidence

The original interrupted-agent RED remains unrecoverable exactly as recorded earlier. The following evidence belongs only to Review Fix Round 1.

#### Full-Pose Odometry Interpolation

RED test: `test_interpolate_full_preserves_translation_and_slerps_quaternion`.

```text
test_interpolate_full_preserves_translation_and_slerps_quaternion ... FAIL
AssertionError: unexpectedly None
Ran 13 tests in 0.127s
FAILED (failures=1)
```

The failure showed that the requested additive full-pose API did not exist. After implementation, all 13 `test_odom_buffer.py` tests passed, including all pre-existing Task 1 planar, bracket, distance, maxlen, and ROS-isolation tests.

#### Complete Chain And Observation Covariance

RED tests:

- `test_full_uav_attitude_is_applied_before_planar_projection`
- `test_observation_covariance_propagates_rotation_cross_terms_and_lever_arm`

```text
test_full_uav_attitude_is_applied_before_planar_projection ... FAIL
test_observation_covariance_propagates_rotation_cross_terms_and_lever_arm ... FAIL
Ran 7 tests in 0.004s
FAILED (failures=2)
```

Both failed because `registration_sample_from_observation` was absent. The full-attitude fixture uses a 90-degree UAV roll, for which the observation translation must rotate out of planar y before final projection. The covariance fixture has a 90-degree camera rotation, board lever arm `[1,2,0]`, anisotropic x/y variance, and nonzero x-y/x-yaw/y-yaw terms. Its hand-derived final covariance is:

```text
[[0.108, 0.025, 0.014],
 [0.025, 0.092, 0.023],
 [0.014, 0.023, 0.010]]
```

After the complete-chain and numerical-Jacobian implementation, all 7 then-current estimator tests passed. The result is checked for the literal covariance, symmetry, and nonnegative eigenvalues.

#### Exact Inliers And Fixed-Yaw Re-Anchor

RED tests:

- `test_estimator_exposes_the_exact_survivor_indices`
- `test_fixed_yaw_reanchor_uses_survivors_and_propagates_covariance`

```text
test_estimator_exposes_the_exact_survivor_indices ... FAIL
test_fixed_yaw_reanchor_uses_survivors_and_propagates_covariance ... FAIL
Ran 9 tests in 0.005s
FAILED (failures=2)
```

Both failed on the missing exact-inlier API. After implementation, the tests passed with exact survivor indices `(0,1,2)`, `inlier_count == 3`, latest inlier stamp `2.0`, literal re-anchored median `[0.900166583353, 2.094837581925, 0]`, full literal covariance, and fixed-yaw variance `0.005^2`.

#### Atomic One-Shot State

RED test: `test_concurrent_updates_make_one_atomic_zero_to_one_transition`.

```text
test_concurrent_updates_make_one_atomic_zero_to_one_transition ... FAIL
AssertionError: unexpectedly None
Ran 10 tests in 0.007s
FAILED (failures=1)
```

The state helper was absent. After implementation, 16 threads released from one barrier produced one factory call, one non-`None` winner, revision `1`, and one frozen value; all 10 then-current estimator tests passed.

#### Input Frame Validation

RED tests:

- `test_odom_requires_exact_nonempty_parent_and_child_frames`
- `test_observation_requires_exact_nonempty_input_frame`

```text
test_observation_requires_exact_nonempty_input_frame ... FAIL
test_odom_requires_exact_nonempty_parent_and_child_frames ... FAIL
Ran 12 tests in 0.009s
FAILED (failures=2)
```

Both failed because strict validators were absent. After implementation, exact expected frames pass and empty/wrong parent, child, or observation frames fail; all 12 estimator tests passed.

The `tf` metadata, YAML, XML, and written rostest changes are configuration/integration artifacts. They did not receive fabricated unit RED evidence; package/XML/build checks below verify the bounded properties available in this environment.

### Final Bounded Verification

Pure command:

```bash
source devel/setup.bash
python3 -m unittest \
  src/air_ground_coordinate_transform/test/test_se2.py \
  src/air_ground_coordinate_transform/test/test_odom_buffer.py \
  src/air_ground_coordinate_transform/test/test_registration_estimator.py -v
```

Result: 33 tests passed. This comprises 8 SE(2), 13 odometry-buffer, and 12 registration estimator/geometry/state/frame tests. No ROS master is required.

Python compilation covered the two pure modules, ROS adapter, two pure test files, and written node test. Result: exit status `0`, no output.

Static XML parsing covered `registration_node.test`, `coordinate_transform.launch`, `package.xml`, and `air_ground_final_demo.launch`. Result: 4 files parsed with exit status `0`.

Bounded build:

```bash
catkin_make --pkg air_ground_coordinate_transform air_ground_bringup
```

Result: exit status `0`; `coordinate_transform_node` built successfully and the Python devel wrapper was regenerated. Configuration continues to emit unrelated pre-existing VTK and other-package Eigen/system_lib warnings.

### Dynamic Checks Not Run

The operating prohibition was followed. None of these were run:

- `rostest air_ground_coordinate_transform registration_node.test`
- Any `roslaunch` command, including launch resolution and compatibility Demo cold-start
- `roscore`, Gazebo, PX4 SITL, RViz, rosbag, or topic wait/echo loops
- M0-B dynamic registration, mission-state, final-position, or shutdown acceptance

The expanded rostest is written evidence only. Its timing, topic latching, TF listener behavior, and runtime frame defaults remain pending external execution. No Gazebo truth source was read.

### Review Modified Files

- Modified `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/odom_buffer.py`
- Modified `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py`
- Modified `src/air_ground_coordinate_transform/scripts/takeoff_registration.py`
- Modified `src/air_ground_coordinate_transform/test/test_odom_buffer.py`
- Modified `src/air_ground_coordinate_transform/test/test_registration_estimator.py`
- Modified `src/air_ground_coordinate_transform/test/registration_node.test`
- Modified `src/air_ground_coordinate_transform/test/test_registration_node.py`
- Modified `src/air_ground_coordinate_transform/config/registration.yaml`
- Modified `src/air_ground_coordinate_transform/CMakeLists.txt`
- Modified `src/air_ground_coordinate_transform/package.xml`
- Updated `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-2-report.md`

### Review Self-Audit

- Task 1 compatibility: `append`, `append_odometry`, planar `interpolate`, and `distance_since` retain their names, arguments, unavailable-data behavior, and planar return convention. `interpolate_full` is additive.
- Full-chain audit: the ROS adapter obtains interpolated `[x,y,z,qx,qy,qz,qw]` poses and the pure helper performs `origin * uav * base_camera * observation * inverse(ugv * base_board)` before extracting final x/y/yaw.
- Covariance audit: a wrapped-yaw central-difference Jacobian covers all six observation coordinates. `J C J^T` includes camera-axis rotation, board lever arm, anisotropy, and cross terms; every sample and final batch covariance is symmetrized/PSD-projected.
- Fixed-yaw audit: the exact estimator indices are authoritative for mean, covariance, count, and stamp. The fixed yaw is deterministic by compatibility policy, with only its configured floor retained as uncertainty.
- Frame audit: invalid odometry is rejected before origin samples, raw deques, or buffers; empty/mismatched observation frames are rejected before sample construction. Input frame parameters are separate from output TF aliases.
- Atomicity audit: sample period recheck, append, estimator invocation, final-value store, and revision transition execute under the one-shot state lock. Sequential or concurrent later observations cannot invoke the update factory after revision `1`.
- Compatibility audit: legacy publishers and both origin-to-odom TF edges remain. New estimate/revision topics remain latched, with initial revision `0` and one frozen revision `1`.
- Metadata audit: classic `tf` is a direct catkin/package dependency rather than a transitive assumption.
- Test-scope audit: no ROS master, launch, simulator, visualization, bag, topic loop, Gazebo truth, subagent, reviewer, Git initialization, or commit was used.

### Review Concerns

- Dynamic ROS/TF/launch/Demo behavior remains unverified by explicit prohibition. The expanded rostest must be run externally before claiming dynamic acceptance.
- The covariance Jacobian is central-difference numeric with a `1e-6` pose-coordinate step. It is checked against a hand-derived nonsingular fixture, but observation attitudes near Euler singularities remain a numerical risk requiring dynamic data validation.
- Frame defaults were verified against the installed MAVROS config and current UGV/ChArUco publishers. A deployment overriding those publishers must also override the new explicit input-frame parameters or its messages will be intentionally rejected.
- The estimator still assumes finite, correctly shaped input means/covariances; malformed numerical input remains outside the Task 2 brief.
- The workspace remains unversioned, so exact historical diffs are unavailable.

## Review Fix Round 2

### Status

The two PARTIAL dispositions and one new Minor from the Round 1 re-review were verified and addressed with bounded changes. All permitted checks pass. The strengthened written rostest remains deliberately unrun, so no dynamic ROS, TF, launch, or Demo acceptance is claimed.

No subagent or reviewer was dispatched. No commit was created, and Git was not initialized.

### Finding Disposition

| Finding | Disposition | Evidence and resolution |
|---|---|---|
| Minor 1 PARTIAL: written rostest was not independently sensitive to the UGV and observation validators | Addressed | `registration_node.test` now starts two additional registration instances with independent input topics, output remaps, TF frame names, and one-shot state. Each instance captures its own origin and receives four otherwise-valid, mutually consistent candidates, matching `minimum_samples=4`. The UGV case varies only invalid UGV frames; its last valid origin UGV odom is `0.30 s` before the first candidate, beyond `max_odom_bracket=0.20 s`. The observation case varies only an empty/wrong observation frame. With either validator present, its isolated node remains at revision `0`; removing that validator admits four samples and makes the corresponding `ACQUIRING_REGISTRATION`/revision assertions fail by freezing. The main fixture no longer receives these two invalid groups, so they cannot accumulate across concerns. |
| Minor 1 PARTIAL: node covariance assertion accepted any nonzero symmetric matrix | Addressed | The main written node test now compares each of the nine ROS covariance slots at rows/columns `(0,1,5)` to an independently evaluated literal 3x3 matrix with `1e-10` tolerance. The constants include the fixture's propagated six-axis observation uncertainty, empirical sample spread, division by four inliers, and configured floors. Their derivation did not import or call `registration_sample_from_observation`, `_batch_estimate`, or another production helper; direct input-covariance copying does not match them. |
| Minor 2 PARTIAL: stale fixed-yaw concern contradicted Round 1 | Addressed | The original concern above is now explicitly labeled superseded and states the current exact-survivor re-anchor/covariance behavior. |
| New Minor: `observation_input_frame` ignored the legacy camera-frame override by default | Addressed | Added pure `resolve_observation_input_frame(get_param, camera_frame)`. The adapter passes the already resolved `self.camera_frame` as the default while preserving an explicit `~observation_input_frame` override. This restores the existing `~nadir_camera_frame` contract without ROS-master-dependent parameter testing. |

### RED/GREEN Evidence

The pure parameter-resolution test was written before the production helper and adapter change:

```text
test_observation_input_frame_defaults_to_legacy_camera_and_allows_override ... FAIL
AssertionError: unexpectedly None
Ran 13 tests in 0.009s
FAILED (failures=1)
```

The failure was specifically the missing resolution function. After adding the minimal helper and using it in the adapter, the focused command passed all 13 registration estimator/geometry/state/frame tests.

The launch and written rostest changes are integration artifacts under the explicit no-ROS-graph ruling. No dynamic RED was fabricated. Their mutation sensitivity is structural: the two validator fixtures have independent node state and each supply all four candidates needed to freeze if its one targeted guard is removed. The covariance assertion uses the following independent literal matrix rather than production-derived expected values:

```text
[[ 1.089813997879532e-04, -6.337021007873122e-06,  1.152242238294521e-06],
 [-6.337021007873122e-06,  1.077442631452827e-04, -1.340244215354868e-06],
 [ 1.152242238294521e-06, -1.340244215354868e-06,  2.551244577052707e-05]]
```

### Final Bounded Verification

Fresh pure command after all code/test edits:

```bash
source devel/setup.bash
python3 -m unittest \
  src/air_ground_coordinate_transform/test/test_se2.py \
  src/air_ground_coordinate_transform/test/test_odom_buffer.py \
  src/air_ground_coordinate_transform/test/test_registration_estimator.py -v
```

Result: 34 tests passed in 0.138 s: 8 SE(2), 13 odometry-buffer, and 13 registration estimator/geometry/state/frame tests. No ROS master is required.

`py_compile` covered the three pure modules, ROS adapter, three pure test files, and written node test. Result: exit status `0`, no output.

`xml.etree.ElementTree` freshly parsed `registration_node.test`, `coordinate_transform.launch`, `package.xml`, and `air_ground_final_demo.launch`. Result: `parsed 4 XML files`, exit status `0`.

The bounded command `catkin_make --pkg air_ground_coordinate_transform air_ground_bringup` completed with exit status `0`; `coordinate_transform_node` built and the registration Python wrapper regenerated. Configuration reproduced the pre-existing unrelated VTK artifact and other-package Eigen/system_lib warnings.

### Dynamic Checks Not Run

No `rostest`, `roslaunch`, `roscore`, Gazebo, PX4, RViz, rosbag, topic wait/echo, or other ROS graph/long-running command was run. The three written registration instances, remapping, latching, scheduling, TF publication, and compatibility Demo remain pending external execution. No Gazebo truth source was read or introduced.

### Round 2 Modified Files

- Modified `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py`
- Modified `src/air_ground_coordinate_transform/scripts/takeoff_registration.py`
- Modified `src/air_ground_coordinate_transform/test/test_registration_estimator.py`
- Modified `src/air_ground_coordinate_transform/test/test_registration_node.py`
- Modified `src/air_ground_coordinate_transform/test/registration_node.test`
- Updated `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-2-report.md`

### Round 2 Residual Risks

- Written ROS timing, latched-topic behavior, remap resolution, TF publication, and launch execution are statically checked only under the explicit prohibition.
- Observation attitudes near Euler singularities remain a numerical conditioning risk for the central-difference RPY Jacobian; the literal node fixture is nonsingular.
- The workspace remains unversioned, so exact historical diffs are unavailable.

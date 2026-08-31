# GNSS-Denied UAV-UGV Registration and Task Relay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a paper-grade UAV-UGV experiment system for arbitrary-initial-pose one-shot registration, uncertainty-aware opportunistic re-registration, and UAV-to-UGV anomaly inspection handoff.

**Architecture:** Keep perception and low-level vehicle control intact, extract registration and handoff math into tested Python modules, and place all synthetic odometry perturbation and Gazebo-truth evaluation in a separate experiment package. Maintain the anomaly target canonically in `air_ground_origin`; online registration revisions update the UGV-resolved goal safely instead of rewriting a one-time odom goal.

**Tech Stack:** ROS Noetic, catkin, Python 3, NumPy, tf2, Gazebo Classic, PX4 SITL, MAVROS, OpenCV, `unittest`/`rostest`, CSV/JSON experiment outputs.

**Spec:** `docs/superpowers/specs/2026-08-25-gnss-denied-air-ground-registration-design.md`

## Global Constraints

- Preserve `air_ground_final_demo.launch` as the one-shot compatibility baseline until the final integration task.
- Do not add collaborative SLAM, multi-robot allocation, or complex map sharing.
- Gazebo truth is evaluation-only and must not publish control, target, registration, or mission-decision topics.
- Do not record rosbag; write deterministic CSV and JSON experiment artifacts directly.
- Use `PoseWithCovarianceStamped` for registration and target estimates; do not create a custom message package in this scope.
- Use covariance indices `(0, 7, 35)` for `(x, y, yaw)` and preserve cross-covariances when transforming estimates.
- Keep `air_ground_origin` anchored to the UAV takeoff frame; opportunistic observations update `air_ground_origin -> ugv experimental odom`.
- Publish exactly one TF broadcaster for each parent-child edge.
- Every dynamic experiment must cold-start ROS, Gazebo, PX4, and MAVROS and must retain failed trial results.
- Unit tests may use synthetic truth. Runtime autonomy may not consume `/gazebo/get_model_state`, `/gazebo/model_states`, or `/air_ground_experiment/truth/*`.
- This workspace currently has no Git metadata. Do not initialize a repository or create commits without explicit user authorization; each task therefore ends with a named verification checkpoint rather than a commit.

## File Structure

### Coordinate estimator

- Create `src/air_ground_coordinate_transform/setup.py`: expose importable Python modules through catkin.
- Create `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/__init__.py`: package marker.
- Create `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/se2.py`: angle, matrix, adjoint/Jacobian, covariance conversion utilities.
- Create `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/odom_buffer.py`: timestamped odometry interpolation and motion accumulation.
- Create `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py`: robust batch estimator and random-walk SE(2) filter.
- Modify `src/air_ground_coordinate_transform/scripts/takeoff_registration.py`: ROS adapter around the pure estimator.
- Modify `src/air_ground_coordinate_transform/config/registration.yaml`: one-shot/opportunistic and covariance parameters.
- Modify `src/air_ground_coordinate_transform/launch/coordinate_transform.launch`: expose mode and topic overrides.
- Create `src/air_ground_coordinate_transform/test/test_se2.py`.
- Create `src/air_ground_coordinate_transform/test/test_odom_buffer.py`.
- Create `src/air_ground_coordinate_transform/test/test_registration_estimator.py`.
- Create `src/air_ground_coordinate_transform/test/registration_node.test` and `test/test_registration_node.py`.

### Mission and handoff

- Create `src/air_ground_bringup/setup.py`.
- Create `src/air_ground_bringup/src/air_ground_bringup/__init__.py`.
- Create `src/air_ground_bringup/src/air_ground_bringup/target_handoff.py`: covariance transformation, uncertainty budgets, and standoff goal generation.
- Create `src/air_ground_bringup/src/air_ground_bringup/goal_tracking.py`: continuously resolve a common-frame goal into UGV odom and detect revision jumps.
- Create `src/air_ground_bringup/scripts/target_handoff_node.py`: ROS adapter and handoff action publisher.
- Create `src/air_ground_bringup/scripts/ugv_anomaly_inspector.py`: independent UGV-camera confirmation.
- Modify `src/air_ground_bringup/scripts/uav_sphere_mission.py`: publish anomaly covariance, request re-observation/re-registration, and wait for inspection completion.
- Modify `src/air_ground_bringup/scripts/ugv_goal_controller.py`: retain common-frame goal and re-resolve each tick.
- Modify `src/air_ground_bringup/launch/air_ground_final_demo.launch`: retain legacy defaults and expose research-mode arguments.
- Create `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch`: paper-system integration launch.
- Create `src/air_ground_bringup/test/test_target_handoff.py`.
- Create `src/air_ground_bringup/test/test_goal_tracking.py`.
- Create `src/air_ground_bringup/test/inspection_relay.test` and `test/test_inspection_relay.py`.

### Experiment isolation and evaluation

- Create `src/air_ground_experiments/package.xml`, `CMakeLists.txt`, and `setup.py`.
- Create `src/air_ground_experiments/src/air_ground_experiments/__init__.py`.
- Create `src/air_ground_experiments/src/air_ground_experiments/frame_perturbation.py`: deterministic SE(2) frame offsets and drift.
- Create `src/air_ground_experiments/src/air_ground_experiments/metrics.py`: metric definitions and CSV row schema.
- Create `src/air_ground_experiments/scripts/odom_perturbation_node.py`: publish experimental odometry and truth metadata.
- Create `src/air_ground_experiments/scripts/observation_gate.py`: deterministic visibility, delay, and gross-outlier injection for ChArUco observations.
- Create `src/air_ground_experiments/scripts/position_command_adapter.py`: convert UAV commands back to raw CXR odom.
- Create `src/air_ground_experiments/scripts/experiment_recorder.py`: evaluation-only world truth and task metrics.
- Create `src/air_ground_experiments/scripts/run_experiment_matrix.py`: seeded cold-start trial runner.
- Create `src/air_ground_experiments/config/one_shot_matrix.yaml`.
- Create `src/air_ground_experiments/config/opportunistic_matrix.yaml`.
- Create `src/air_ground_experiments/config/handoff_matrix.yaml`.
- Create `src/air_ground_experiments/launch/frame_perturbation.launch`.
- Create `src/air_ground_experiments/test/test_frame_perturbation.py` and `test/test_metrics.py`.

### Simulation model

- Modify `src/air_ground_bringup/launch/uav_sitl.launch`: expose UAV spawn yaw.
- Modify `src/air_ground_bringup/launch/mvp_system.launch`: forward UAV/UGV spawn headings and configurable odom topics.
- Reuse the existing UGV RGB camera in `src/air_ground_ugv_gazebo/models/ugv_mvp/model.sdf:91-116`; no new sensor model is required.

---

## Milestone 0: Freeze Existing Behavior and Create Testable Cores

### Task 1: Extract SE(2) and odometry interpolation primitives

**Files:**
- Create: `src/air_ground_coordinate_transform/setup.py`
- Create: `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/__init__.py`
- Create: `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/se2.py`
- Create: `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/odom_buffer.py`
- Modify: `src/air_ground_coordinate_transform/CMakeLists.txt:1-31`
- Modify: `src/air_ground_coordinate_transform/package.xml:8-30`
- Test: `src/air_ground_coordinate_transform/test/test_se2.py`
- Test: `src/air_ground_coordinate_transform/test/test_odom_buffer.py`

**Interfaces:**
- Produces: `wrap_angle(float) -> float`.
- Produces: `wrap_xyyaw(np.ndarray) -> np.ndarray`, normalizing element `2` only.
- Produces: `matrix_from_xyyaw(float, float, float) -> np.ndarray` with shape `(3, 3)`.
- Produces: `xyyaw_from_matrix(np.ndarray) -> np.ndarray` with shape `(3,)`.
- Produces: `compose(np.ndarray, np.ndarray) -> np.ndarray` and `inverse(np.ndarray) -> np.ndarray` for planar transforms.
- Produces: `transform_pose_covariance(mean, covariance, transform_mean, transform_covariance) -> (mean, covariance)` for first-order SE(2) propagation.
- Produces: `OdomBuffer(maxlen: int, max_bracket: float)` with `append(stamp, x, y, z, yaw)`, `append_odometry(message)`, `interpolate(stamp)`, and `distance_since(stamp)`.

- [ ] **Step 1: Add failing SE(2) identity, inverse, wrap, and covariance tests**

```python
def test_compose_with_inverse_returns_identity(self):
    value = np.array([1.2, -0.4, 0.7])
    matrix = matrix_from_xyyaw(*value)
    np.testing.assert_allclose(compose(matrix, inverse(matrix)), np.eye(3), atol=1e-9)

def test_transform_covariance_includes_heading_lever_arm(self):
    point = np.array([10.0, 0.0, 0.0])
    point_cov = np.zeros((3, 3))
    tf_mean = np.zeros(3)
    tf_cov = np.diag([0.01, 0.01, math.radians(1.0) ** 2])
    _, covariance = transform_pose_covariance(point, point_cov, tf_mean, tf_cov)
    self.assertGreater(covariance[1, 1], 0.03)
```

- [ ] **Step 2: Run the SE(2) tests and verify the missing module failure**

Run:

```bash
source devel/setup.bash
python3 -m unittest discover -s src/air_ground_coordinate_transform/test -p 'test_se2.py' -v
```

Expected: `ModuleNotFoundError: No module named 'air_ground_coordinate_transform.se2'`.

- [ ] **Step 3: Implement the minimal SE(2) functions**

Use the state convention `[x, y, yaw]`. Normalize yaw after every composition. For a point transformed by uncertain yaw, use Jacobians:

```python
j_point = np.array([
    [c, -s, -s * px - c * py],
    [s,  c,  c * px - s * py],
    [0.0, 0.0, 1.0],
])
j_transform = np.array([
    [1.0, 0.0, -s * px - c * py],
    [0.0, 1.0,  c * px - s * py],
    [0.0, 0.0, 1.0],
])
covariance = j_point @ point_covariance @ j_point.T + j_transform @ transform_covariance @ j_transform.T
```

- [ ] **Step 4: Add failing interpolation tests**

Cover exact timestamp, linear translation, shortest-path yaw interpolation across `+pi/-pi`, a preceding sample within `max_bracket`, rejection outside `max_bracket`, and accumulated planar distance.

```python
def test_interpolates_yaw_across_pi_by_shortest_path(self):
    buffer = OdomBuffer(maxlen=10, max_bracket=0.08)
    buffer.append(make_odom(1.00, 0.0, 0.0, math.radians(179.0)))
    buffer.append(make_odom(1.10, 1.0, 0.0, math.radians(-179.0)))
    result = buffer.interpolate(rospy.Time.from_sec(1.05))
    self.assertAlmostEqual(abs(result[2]), math.pi, places=5)
```

- [ ] **Step 5: Implement `OdomBuffer` without ROS side effects at import time**

Store `(stamp_sec, x, y, z, yaw)` samples. Keep ROS message conversion in a small `append_odometry(message)` adapter so pure tests can append numeric samples.

- [ ] **Step 6: Register the Python package and tests with catkin**

Add `catkin_python_setup()` and guarded test declarations:

```cmake
catkin_python_setup()
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_se2.py)
  catkin_add_nosetests(test/test_odom_buffer.py)
endif()
```

Add `<test_depend>python3-nose</test_depend>` to `package.xml`.

- [ ] **Step 7: Run the focused tests and package build**

Run:

```bash
python3 -m unittest discover -s src/air_ground_coordinate_transform/test -p 'test_*.py' -v
catkin_make --pkg air_ground_coordinate_transform
```

Expected: all tests pass; package builds with no new error.

- [ ] **Step 8: Verification checkpoint M0-A**

Record the command output and confirm no runtime script or launch behavior changed.

---

### Task 2: Refactor current one-shot registration behind a pure robust estimator

**Files:**
- Create: `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py`
- Modify: `src/air_ground_coordinate_transform/scripts/takeoff_registration.py:17-216`
- Test: `src/air_ground_coordinate_transform/test/test_registration_estimator.py`
- Test: `src/air_ground_coordinate_transform/test/registration_node.test`
- Test: `src/air_ground_coordinate_transform/test/test_registration_node.py`

**Interfaces:**
- Consumes: SE(2) helpers and `OdomBuffer` from Task 1.
- Produces: `RegistrationSample(mean: np.ndarray, anchor: np.ndarray, covariance: np.ndarray, stamp: float)`.
- Produces: `RobustBatchEstimator(min_samples, max_translation_residual, max_yaw_residual).estimate(samples) -> BatchEstimate | None`.
- Produces: `BatchEstimate(mean, covariance, inlier_count, stamp)`.
- Preserves: `/air_ground/registration/frozen`, `/status`, `/valid`, `/inlier_count`, and existing TF edges in `one_shot` mode.

- [ ] **Step 1: Write failing randomized robust-estimation tests**

Generate 20 inliers around `[2.0, -1.0, 0.35]` and 8 gross outliers. Require correct wrapped yaw, at least 18 inliers, symmetric positive-semidefinite covariance, and no estimate when fewer than `min_samples` survive.

```python
estimate = RobustBatchEstimator(20, 0.12, 0.03).estimate(samples)
np.testing.assert_allclose(estimate.mean[:2], [2.0, -1.0], atol=0.03)
self.assertLess(abs(wrap_angle(estimate.mean[2] - 0.35)), 0.01)
self.assertGreaterEqual(estimate.inlier_count, 18)
np.testing.assert_allclose(estimate.covariance, estimate.covariance.T, atol=1e-12)
```

- [ ] **Step 2: Run the focused estimator test and verify failure**

Run:

```bash
python3 -m unittest src/air_ground_coordinate_transform/test/test_registration_estimator.py -v
```

Expected: import failure for `RobustBatchEstimator`.

- [ ] **Step 3: Implement robust median/circular-mean estimation and covariance floors**

Use median translation, circular yaw mean, the current translation/yaw gates, and sample covariance divided by effective inlier count. Enforce configurable floors:

```python
covariance += np.diag([
    minimum_translation_sigma ** 2,
    minimum_translation_sigma ** 2,
    minimum_yaw_sigma ** 2,
])
```

- [ ] **Step 4: Replace duplicated matrix/interpolation code in the ROS node**

Keep observation construction equivalent to the existing equation:

```python
sample = origin_to_uav_odom.dot(uav).dot(base_camera).dot(
    observation_matrix).dot(inverse_matrix(ugv.dot(base_board)))
```

Convert the planar result to `RegistrationSample`. In `one_shot` mode, stop accepting observations after the first estimate exactly as today.

- [ ] **Step 5: Publish a covariance-bearing estimate without breaking old topics**

Add latched:

```text
/air_ground/registration/estimate  geometry_msgs/PoseWithCovarianceStamped
/air_ground/registration/revision  std_msgs/UInt32
```

The pose represents `ugv_0/odom` origin in `air_ground_origin`. Populate `(x,y,yaw)` covariance slots and set revision to `1` on the initial freeze.

- [ ] **Step 6: Add a rostest for initial status, sample rejection, and one-shot freeze**

Publish synthetic UAV odom, UGV odom, and board observations. Assert:

```python
self.assertEqual(wait_for_status(), "FROZEN")
self.assertTrue(rospy.wait_for_message("/air_ground/registration/frozen", Bool).data)
self.assertEqual(rospy.wait_for_message("/air_ground/registration/revision", UInt32).data, 1)
self.assertLess(abs(received.pose.pose.position.x - expected_x), 0.03)
```

Then publish a contradictory second batch and assert revision remains `1` in `one_shot` mode.

- [ ] **Step 7: Run unit tests, rostest, and current launch XML validation**

Run:

```bash
python3 -m unittest src/air_ground_coordinate_transform/test/test_registration_estimator.py -v
rostest air_ground_coordinate_transform registration_node.test
roslaunch --check air_ground_bringup air_ground_final_demo.launch
catkin_make --pkg air_ground_coordinate_transform air_ground_bringup
```

Expected: tests pass and launch resolves.

- [ ] **Step 8: Cold-start the compatibility Demo**

Run:

```bash
source src/air_ground_bringup/scripts/setup_mvp_env.sh
roslaunch air_ground_bringup air_ground_final_demo.launch separate_terminals:=false
```

Expected: `FROZEN -> OVERWATCH`, UGV `ARRIVED`, revision remains `1`, and the final stop remains approximately `0.76 m` from the anomaly.

- [ ] **Step 9: Verification checkpoint M0-B**

Retain the cold-start status, target coordinate, final UGV coordinate, and shutdown-process check as the compatibility record.

---

## Milestone 1: Multi-Initial-Pose One-Shot Registration Baseline

### Task 3: Support arbitrary spawn geometry and relative heading

**Files:**
- Modify: `src/air_ground_bringup/launch/uav_sitl.launch:2-25`
- Modify: `src/air_ground_bringup/launch/mvp_system.launch:10-35`
- Modify: `src/air_ground_bringup/launch/air_ground_final_demo.launch:2-27`
- Modify: `src/air_ground_bringup/scripts/uav_sphere_mission.py:34-65,374-440`
- Modify: `src/air_ground_coordinate_transform/config/registration.yaml:16-25`
- Modify: `src/air_ground_coordinate_transform/launch/coordinate_transform.launch:1-6`
- Test: `src/air_ground_bringup/test/test_registration_waypoint.py`

**Interfaces:**
- Produces launch args: `uav_yaw`, `ugv_yaw`, `registration_dx`, `registration_dy`.
- Produces: `registration_waypoint(home_x, home_y, home_yaw, dx, dy) -> (x, y)`.
- Changes research default: estimate visual relative yaw; compatibility launch may explicitly retain fixed yaw.

- [ ] **Step 1: Write the failing body-relative waypoint test**

```python
def test_registration_offset_rotates_with_home_heading(self):
    x, y = registration_waypoint(2.0, 3.0, math.pi / 2, 1.6, 0.0)
    self.assertAlmostEqual(x, 2.0, places=6)
    self.assertAlmostEqual(y, 4.6, places=6)
```

- [ ] **Step 2: Run the focused test and verify failure**

Run:

```bash
python3 -m unittest src/air_ground_bringup/test/test_registration_waypoint.py -v
```

Expected: missing `registration_waypoint`.

- [ ] **Step 3: Implement the body-relative waypoint and use it in all registration phases**

```python
def registration_waypoint(home_x, home_y, home_yaw, dx, dy):
    c, s = math.cos(home_yaw), math.sin(home_yaw)
    return home_x + c * dx - s * dy, home_y + s * dx + c * dy
```

Replace all `home[0] + registration_offset, home[1]` assumptions with the resulting `(registration_x, registration_y)`.

- [ ] **Step 4: Add UAV spawn yaw and forward all pose arguments**

Add `uav_sitl.launch` argument `yaw` and pass `-Y $(arg yaw)` to `spawn_model`. Add independent `uav_yaw` and `ugv_yaw` arguments to both parent launch files.

- [ ] **Step 5: Separate compatibility and research yaw policies**

Keep `air_ground_final_demo.launch` explicitly passing `use_visual_frame_yaw:=false` through the coordinate launch for compatibility. Set `air_ground_inspection_experiment.launch` later to `use_visual_frame_yaw:=true`; do not silently change legacy results.

- [ ] **Step 6: Run geometry unit tests and three manual pose cases**

Cases:

```text
A: UAV yaw 0 deg, UGV yaw 0 deg
B: UAV yaw 90 deg, UGV yaw -45 deg
C: UAV yaw -120 deg, UGV yaw 150 deg
```

For each case, place the UGV at the configured body-relative registration waypoint, cold-start the Demo, and require `FROZEN` without collision.

- [ ] **Step 7: Verification checkpoint M1-A**

Record registration completion, estimated heading, and Gazebo-truth transform error for the three cases. Truth is collected manually/evaluator-side only.

---

### Task 4: Add deterministic independent-frame perturbation and one-shot metrics

**Files:**
- Create: `src/air_ground_experiments/package.xml`
- Create: `src/air_ground_experiments/CMakeLists.txt`
- Create: `src/air_ground_experiments/setup.py`
- Create: `src/air_ground_experiments/src/air_ground_experiments/__init__.py`
- Create: `src/air_ground_experiments/src/air_ground_experiments/frame_perturbation.py`
- Create: `src/air_ground_experiments/src/air_ground_experiments/metrics.py`
- Create: `src/air_ground_experiments/scripts/odom_perturbation_node.py`
- Create: `src/air_ground_experiments/scripts/observation_gate.py`
- Create: `src/air_ground_experiments/scripts/position_command_adapter.py`
- Create: `src/air_ground_experiments/scripts/experiment_recorder.py`
- Create: `src/air_ground_experiments/launch/frame_perturbation.launch`
- Test: `src/air_ground_experiments/test/test_frame_perturbation.py`
- Test: `src/air_ground_experiments/test/test_metrics.py`

**Interfaces:**
- Produces: `FramePerturbation(initial_xyyaw, drift_rates, seed)`.
- Produces: `FramePerturbation.at(stamp_seconds: float) -> np.ndarray` with state `[x, y, yaw]`.
- Produces: `transform_odom(pose, twist, stamp) -> (pose, twist, truth_transform)`.
- Publishes: `/air_ground_experiment/uav/odom`, `/air_ground_experiment/ugv/odom`.
- Publishes evaluation-only: `/air_ground_experiment/truth/uav_frame`, `/air_ground_experiment/truth/ugv_frame`.
- Republishes: `/air_ground_experiment/charuco/observation` after a seeded visibility, delay, and outlier schedule.
- Produces CSV columns defined by `TRIAL_COLUMNS` in `metrics.py`.

- [ ] **Step 1: Write failing deterministic perturbation tests**

```python
first = FramePerturbation([2.0, -1.0, 0.4], [0.01, 0.0, 0.001], seed=17)
second = FramePerturbation([2.0, -1.0, 0.4], [0.01, 0.0, 0.001], seed=17)
self.assertEqual(first.at(30.0), second.at(30.0))
```

Also assert pose and twist rotate consistently and covariance transforms with the same Jacobian.

- [ ] **Step 2: Write failing metric tests**

Test wrapped yaw error, 2-D handoff error, final inspection distance, success-radius classification, and a failed trial row with a nonempty failure code.

- [ ] **Step 3: Implement pure perturbation and metric modules**

Use a piecewise deterministic random walk sampled at fixed `drift_step_seconds`; never call global `np.random`. Store and expose the seed in every truth message and result row.

- [ ] **Step 4: Implement the odometry ROS adapter**

Parameters must include source/destination topic, source/destination frame, initial `[x,y,yaw]`, translational drift rate, yaw drift rate, drift step, and seed. Rotate pose, linear twist, angular twist, and covariance consistently.

- [ ] **Step 5: Implement the UAV command frame adapter**

Subscribe to `/air_ground_experiment/iris_0/position_cmd`, invert the current injected UAV-frame transform, and publish `/iris_0/position_cmd` for CXR. Preserve z, yaw rate semantics, timestamp, and trajectory flags. Do not subscribe to Gazebo truth.

- [ ] **Step 6: Implement deterministic intermittent-observation gating**

Subscribe to the real ChArUco observation and republish only during configured visibility windows. Use ROS time and a seeded queue to apply configured delay and gross SE(2) outliers. Preserve the original image timestamp plus an explicit injected-delay diagnostic; never synthesize a valid board observation when the detector published none.

- [ ] **Step 7: Implement the evaluation-only recorder**

The recorder may subscribe to `/gazebo/model_states` and experiment truth topics. It must only publish `/air_ground_experiment/evaluation/status`; enforce this by keeping all other publishers out of the class. Write one CSV row and one JSON metadata file per trial, including failures and timeout state.

- [ ] **Step 8: Add package installation and tests**

Register scripts with `catkin_install_python`, config/launch directories with `install`, and unit tests under `CATKIN_ENABLE_TESTING`.

- [ ] **Step 9: Run package tests and static topic audit**

Run:

```bash
python3 -m unittest discover -s src/air_ground_experiments/test -p 'test_*.py' -v
catkin_make --pkg air_ground_experiments
roslaunch --check air_ground_experiments frame_perturbation.launch
```

Audit:

```bash
rosnode info /experiment_recorder
```

Expected: recorder publishes only its evaluation status and writes files; it does not connect to autonomy input topics.

- [ ] **Step 10: Verification checkpoint M1-B**

Run one zero-drift trial and confirm perturbation outputs equal raw odometry up to the configured constant frame transform.

---

### Task 5: Build and execute the one-shot baseline matrix

**Files:**
- Create: `src/air_ground_experiments/scripts/run_experiment_matrix.py`
- Create: `src/air_ground_experiments/config/one_shot_matrix.yaml`
- Create: `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch`
- Modify: `src/air_ground_bringup/CMakeLists.txt:4-30`
- Modify: `src/air_ground_bringup/package.xml:8-40`
- Test: `src/air_ground_experiments/test/test_matrix_expansion.py`

**Interfaces:**
- Produces: `TrialSpec(trial_id, seed, launch_args, timeout_seconds, output_directory)`.
- Produces: `expand_matrix(config: dict) -> list[TrialSpec]` sorted by `trial_id`.
- Produces: cold-start runner exit codes `PASS`, `LAUNCH_FAILED`, `TIMEOUT`, `REGISTRATION_FAILED`, `MISSION_FAILED`.
- Launches: `air_ground_inspection_experiment.launch registration_mode:=one_shot`.

- [ ] **Step 1: Write a failing deterministic matrix expansion test**

```python
trials = expand_matrix(load_fixture())
self.assertEqual(len(trials), 30)
self.assertEqual(trials[0].seed, 1000)
self.assertEqual(trials[-1].trial_id, "one_shot-0029")
```

- [ ] **Step 2: Define the one-shot matrix explicitly**

Use 30 seeds. Sample within declared bounds:

```yaml
uav_x: [-4.0, -2.0]
uav_y: [-2.0, 2.0]
uav_yaw_deg: [-180.0, 180.0]
ugv_heading_offset_deg: [-180.0, 180.0]
uav_frame_xy_m: [-3.0, 3.0]
uav_frame_yaw_deg: [-45.0, 45.0]
ugv_frame_xy_m: [-3.0, 3.0]
ugv_frame_yaw_deg: [-45.0, 45.0]
drift: zero
```

Keep UGV physical spawn at the UAV body-relative registration waypoint so this milestone tests registration rather than board search.

- [ ] **Step 3: Implement cold-start trial lifecycle**

For every trial:

```text
verify no matching ROS/Gazebo/PX4 processes
start roslaunch with explicit arguments
wait for FROZEN or timeout
wait for requested terminal state
request recorder flush
send SIGINT to roslaunch
wait for all child processes to exit
write exit classification
```

Do not reuse a ROS master between trials.

- [ ] **Step 4: Build the integration launch**

Include perturbation nodes between raw odometry and research nodes, route UAV commands through the adapter, run the registration node in `one_shot` mode with visual yaw enabled, and start the evaluator. Keep Gazebo truth topics under the experiment namespace.

- [ ] **Step 5: Run three smoke seeds before the full matrix**

Run:

```bash
rosrun air_ground_experiments run_experiment_matrix.py \
  --config src/air_ground_experiments/config/one_shot_matrix.yaml \
  --trials one_shot-0000,one_shot-0001,one_shot-0002
```

Expected: three result rows, three metadata files, clean shutdown after each trial.

- [ ] **Step 6: Execute all 30 baseline seeds**

Acceptance:

```text
registration completion >= 95%
translation error p95 <= 0.15 m
yaw error p95 <= 2.0 deg
all failures retain a reason code
```

- [ ] **Step 7: Verification checkpoint M1-C**

Freeze the matrix YAML and report the exact software parameters, seed list, success rate, translation/yaw percentiles, and failures. Do not tune thresholds after inspecting held-out seeds.

---

## Milestone 2: Uncertainty-Aware Opportunistic Re-Registration

### Task 6: Implement the SE(2) uncertainty filter and innovation gate

**Files:**
- Modify: `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py`
- Modify: `src/air_ground_coordinate_transform/config/registration.yaml`
- Test: `src/air_ground_coordinate_transform/test/test_registration_estimator.py`

**Interfaces:**
- Produces: `RegistrationFilter(initial_mean, initial_covariance, process_noise)`.
- Produces: `predict(dt, uav_distance, ugv_distance) -> FilterState`.
- Produces: `update(batch: BatchEstimate, mahalanobis_threshold: float) -> UpdateResult`.
- `FilterState` fields: `mean`, `covariance`, `revision`, `stamp`, `initialized`.
- `UpdateResult` fields: `accepted`, `innovation`, `mahalanobis`, `mean`, `covariance`, `revision`, `reason`.

- [ ] **Step 1: Write failing covariance-growth tests**

```python
before = filter.state.covariance.copy()
after = filter.predict(dt=10.0, uav_distance=5.0, ugv_distance=2.0).covariance
self.assertTrue(np.all(np.diag(after) > np.diag(before)))
```

Assert zero time and zero travel do not grow covariance.

- [ ] **Step 2: Write failing wrapped innovation and outlier-gating tests**

Test a state at `+179 deg` and measurement at `-179 deg`; innovation must be `2 deg`, not `358 deg`. Test gross translation/yaw outliers against a configurable chi-square threshold.

- [ ] **Step 3: Implement random-walk prediction**

Use:

```python
q = q_time * dt + q_uav_distance * uav_distance + q_ugv_distance * ugv_distance
P_predicted = P + np.diag(q)
```

Keep separate coefficients for translation and yaw. The process model is explicitly an empirical relative-drift model, not SLAM.

- [ ] **Step 4: Implement identity-measurement EKF update**

```python
innovation = wrap_xyyaw(z - x)
S = P + R
d2 = innovation.T @ np.linalg.solve(S, innovation)
K = P @ np.linalg.inv(S)
x_new = wrap_xyyaw(x + K @ innovation)
P_new = (I - K) @ P @ (I - K).T + K @ R @ K.T
```

Use the Joseph covariance update and symmetrize the result.

- [ ] **Step 5: Test 100 seeded drift/update sequences**

For each sequence, inject intermittent measurements and require finite state, symmetric positive-semidefinite covariance, monotonic revision, and lower final RMSE than prediction-only for at least 95 sequences.

- [ ] **Step 6: Run focused tests**

Run:

```bash
python3 -m unittest src/air_ground_coordinate_transform/test/test_registration_estimator.py -v
```

Expected: all estimator and filter tests pass.

- [ ] **Step 7: Verification checkpoint M2-A**

Record filter parameters and seeded Monte Carlo summary before ROS integration.

---

### Task 7: Integrate repeated observation windows, revisions, and degraded status

**Files:**
- Modify: `src/air_ground_coordinate_transform/scripts/takeoff_registration.py:34-216`
- Modify: `src/air_ground_coordinate_transform/config/registration.yaml:10-31`
- Modify: `src/air_ground_coordinate_transform/launch/coordinate_transform.launch:1-6`
- Modify: `src/air_ground_bringup/scripts/ugv_coordinate_monitor.py:12-91`
- Test: `src/air_ground_coordinate_transform/test/registration_node.test`
- Test: `src/air_ground_coordinate_transform/test/test_registration_node.py`

**Interfaces:**
- Consumes: `RegistrationFilter` from Task 6.
- Publishes statuses: `CAPTURING_ORIGIN`, `ACQUIRING_INITIAL`, `TRACKING`, `DEGRADED`, `UPDATING`, `REJECTED`.
- Publishes: `/air_ground/registration/estimate`, `/revision`, `/innovation`, `/status`, and updated TF.
- Preserves: `/frozen=True` after initial acquisition for compatibility; `frozen` now means initialized, not immutable.
- Supports modes: `one_shot`, `periodic`, and `opportunistic`; periodic mode accepts at most one robust update per configured interval.

- [ ] **Step 1: Extend rostest with a second accepted batch**

Start in `opportunistic` mode, publish the first batch, assert revision `1`, advance odometry travel, publish a consistent shifted batch, and assert revision `2` with reduced covariance.

- [ ] **Step 2: Add rostest cases for rejection and degradation**

Publish one gross-outlier batch and assert revision remains unchanged and status reports `REJECTED`. Advance simulated time/travel without observations until the configured covariance trace threshold and assert `DEGRADED`.

- [ ] **Step 3: Replace the immutable `frozen` early return**

Use mode-specific behavior:

```python
if self.mode == "one_shot" and self.filter.initialized:
    return
```

In opportunistic mode, maintain a fresh observation window, robustly estimate a batch, apply the innovation gate, then clear only the consumed window.

In periodic mode, use the same robust estimator and innovation gate but accept a batch only when `now - last_revision_time >= periodic_update_seconds`. This is an explicit baseline, not a second estimator.

Call `filter.predict()` from odometry callbacks using incremental UAV/UGV travel and elapsed ROS time. Publish covariance growth even when no visual batch is available; do not change the mean during prediction.

- [ ] **Step 4: Publish covariance and revision on every accepted update**

Broadcast the current filtered mean as `air_ground_origin -> ugv experimental odom`. Do not create a second broadcaster. Publish innovation Mahalanobis distance as `Float64` and a throttled rejection reason in logs.

- [ ] **Step 5: Update coordinate-monitor semantics**

Subscribe to revision and covariance-bearing estimate. Print revision, `sigma_x`, `sigma_y`, `sigma_yaw_deg`, and delta relative to the previous revision separately from UGV travel delta.

- [ ] **Step 6: Run node tests and a scripted intermittent-visibility trial**

Use visibility windows:

```text
0-5 s visible
5-35 s hidden
35-40 s visible
40-70 s hidden
70-75 s visible
```

Expected: revisions `1 -> 2 -> 3`; covariance grows while hidden and contracts after accepted windows.

- [ ] **Step 7: Verification checkpoint M2-B**

Confirm one-shot mode still stops at revision `1` and opportunistic mode updates without restarting any node.

---

### Task 8: Add uncertainty-triggered UAV re-registration behavior

**Files:**
- Create: `src/air_ground_bringup/src/air_ground_bringup/target_handoff.py`
- Create: `src/air_ground_bringup/setup.py`
- Create: `src/air_ground_bringup/src/air_ground_bringup/__init__.py`
- Modify: `src/air_ground_bringup/scripts/uav_sphere_mission.py:28-182,320-570`
- Modify: `src/air_ground_bringup/CMakeLists.txt:1-31`
- Test: `src/air_ground_bringup/test/test_target_handoff.py`
- Test: `src/air_ground_bringup/test/test_reregistration_state_machine.py`

**Interfaces:**
- Produces: `UncertaintyBudget(registration_covariance, target_covariance, inspection_radius)`.
- Produces: `choose_action() -> DIRECT | REOBSERVE | REREGISTER | HOLD`.
- Adds mission phases: `RETURN_TO_UGV`, `WAIT_REREGISTRATION`, `RESUME_HANDOFF`.
- Consumes: registration revision and estimate covariance.

- [ ] **Step 1: Write failing policy tests**

```python
self.assertEqual(low_uncertainty.choose_action(), DIRECT)
self.assertEqual(target_dominated.choose_action(), REOBSERVE)
self.assertEqual(registration_dominated.choose_action(), REREGISTER)
self.assertEqual(unbounded_uncertainty.choose_action(), HOLD)
```

Use the 95% planar confidence radius from the largest eigenvalue of the XY covariance.

- [ ] **Step 2: Implement the minimal deterministic policy**

Inputs must be explicit thresholds in meters and radians. Do not hide learned weights in this milestone. Publish the selected action and numeric confidence radius for evaluation.

- [ ] **Step 3: Write failing state-transition tests**

Given `REREGISTER`, require:

```text
FINAL_ESTIMATE -> RETURN_TO_UGV
RETURN_TO_UGV -> WAIT_REREGISTRATION
revision increment -> RESUME_HANDOFF
RESUME_HANDOFF -> DISPATCH
```

Require timeout to enter `ERROR_REGISTRATION` rather than dispatch an unsafe goal.

- [ ] **Step 4: Implement return-to-UGV from the latest registered common pose**

Compute the expected UGV common pose from UGV odometry and the latest registration. Transform it into the UAV odom command frame. Use the existing safe registration altitude and position/speed arrival checks. UGV remains stopped before dispatch, making this first opportunistic rendezvous observable and attributable.

- [ ] **Step 5: Preserve the detected anomaly while revisiting the UGV**

Store target mean, covariance, observation timestamp, and handoff target before leaving. On revision increment, recompute the common-frame target; do not rerun target detection unless the policy chose `REOBSERVE`.

- [ ] **Step 6: Run state tests and one full drifted trial**

Expected phases include:

```text
FINAL_ESTIMATE -> RETURN_TO_UGV -> WAIT_REREGISTRATION
-> RESUME_HANDOFF -> DISPATCH
```

Registration revision must increase before UGV starts.

- [ ] **Step 7: Verification checkpoint M2-C**

Compare the same seed in one-shot and re-registration modes. Record pre/post target handoff error and additional UAV path/time.

---

## Milestone 3: Uncertainty-Aware Target Relay and UGV Close Inspection

### Task 9: Publish anomaly covariance and create the common-frame handoff node

**Files:**
- Modify: `src/air_ground_bringup/src/air_ground_bringup/target_handoff.py`
- Create: `src/air_ground_bringup/scripts/target_handoff_node.py`
- Modify: `src/air_ground_bringup/scripts/uav_sphere_mission.py:224-237,320-372,541-565`
- Modify: `src/air_ground_bringup/CMakeLists.txt:16-27`
- Modify: `src/air_ground_bringup/package.xml:8-40`
- Test: `src/air_ground_bringup/test/test_target_handoff.py`
- Test: `src/air_ground_bringup/test/inspection_relay.test`

**Interfaces:**
- Mission publishes: `/air_ground/anomaly/uav_estimate` as `PoseWithCovarianceStamped`.
- Handoff publishes: `/air_ground/anomaly/origin_estimate` as `PoseWithCovarianceStamped`.
- Handoff publishes: `/air_ground/inspection_goal` as `PoseWithCovarianceStamped` in `air_ground_origin`.
- Handoff publishes: `/air_ground/handoff/action` as `String` and `/confidence_radius` as `Float64`.

- [ ] **Step 1: Write failing sample-covariance tests**

Create a target sample cluster with known XY scatter. Require unbiased covariance, configured variance floor, and increased range-axis variance for front-camera estimates compared with nadir estimates.

- [ ] **Step 2: Write failing transform-propagation tests**

Keep `/air_ground/anomaly/origin_estimate` covariance limited to UAV sensing/pose uncertainty. For the executable inspection goal, combine that covariance with registration uncertainty projected through the UGV-to-target lever arm. At a target 15 m from the registration anchor, inject 1 degree registration-yaw sigma and assert the goal's lateral execution variance includes approximately `(15 * radians(1))^2`.

- [ ] **Step 3: Publish the UAV-frame anomaly estimate**

Replace direct target/goal publication inside `uav_sphere_mission.py` with a covariance-bearing estimate. Keep legacy red-sphere diagnostic topics during transition, but make them outputs only; they must not drive the new controller.

- [ ] **Step 4: Implement `target_handoff_node.py`**

Cache the latest UAV anomaly estimate and registration estimate. Transform the mean and covariance into `air_ground_origin`, generate a standoff goal in the same frame, evaluate the uncertainty budget, and publish only when all source timestamps and revisions are valid.

- [ ] **Step 5: Add integration assertions**

The rostest must assert:

```python
self.assertEqual(origin_target.header.frame_id, "air_ground_origin")
self.assertEqual(inspection_goal.header.frame_id, "air_ground_origin")
self.assertGreater(inspection_goal.pose.covariance[0], 0.0)
self.assertIn(action.data, ("DIRECT", "REOBSERVE", "REREGISTER", "HOLD"))
```

- [ ] **Step 6: Run unit and integration tests**

Run:

```bash
python3 -m unittest src/air_ground_bringup/test/test_target_handoff.py -v
rostest air_ground_bringup inspection_relay.test
catkin_make --pkg air_ground_bringup
```

- [ ] **Step 7: Verification checkpoint M3-A**

Confirm every dispatched goal remains in `air_ground_origin` and no mission node publishes a canonical goal in `ugv_0/odom`.

---

### Task 10: Re-resolve the common-frame UGV goal after registration revisions

**Files:**
- Create: `src/air_ground_bringup/src/air_ground_bringup/goal_tracking.py`
- Modify: `src/air_ground_bringup/scripts/ugv_goal_controller.py:22-118`
- Test: `src/air_ground_bringup/test/test_goal_tracking.py`
- Test: `src/air_ground_bringup/test/inspection_relay.test`

**Interfaces:**
- Produces: `GoalTracker.set_goal(common_goal)`.
- Produces: `GoalTracker.resolve(transform, revision) -> ResolvedGoal`.
- `ResolvedGoal` fields: `x`, `y`, `yaw`, `revision`, `jump_distance`, `safe`.
- Controller consumes `/air_ground/inspection_goal` and `/air_ground/registration/revision`.

- [ ] **Step 1: Write failing dynamic-resolution tests**

Set one common-frame goal, resolve it under revision `1`, alter the transform, resolve revision `2`, and assert the odom goal changes while the stored common goal does not.

- [ ] **Step 2: Write failing jump-safety tests**

Require a revision correction over `max_goal_jump` to return `safe=False`. Require a correction below the threshold to update normally. Require stale TF or stale registration to stop the vehicle.

- [ ] **Step 3: Implement the pure `GoalTracker`**

Store the canonical `PoseWithCovarianceStamped`, never its one-time odom conversion. Resolve position and orientation using the latest transform every tick. Track the last accepted resolved goal and revision.

- [ ] **Step 4: Modify the controller to stop before a large revision jump**

On unsafe resolution:

```text
publish zero Twist
publish status HOLDING_FOR_REGISTRATION
wait for stable transform for configured dwell
accept corrected goal or report REGISTRATION_JUMP
```

Do not interpolate through a correction while the UGV is moving unless the correction is below the small-update threshold.

- [ ] **Step 5: Keep control law behavior unchanged after resolution**

Reuse current heading and speed control once `(goal_x, goal_y, goal_yaw)` is safe. Continue publishing `/air_ground/ugv_goal_odom` as a diagnostic of the currently resolved goal.

- [ ] **Step 6: Run tests and an online-update trial**

Apply a small accepted re-registration while UGV is navigating. Expected: goal diagnostic changes, controller remains bounded, no command exceeds existing `0.35 m/s` and `0.65 rad/s`. Apply a large correction and require a zero command before acceptance.

- [ ] **Step 7: Verification checkpoint M3-B**

Record maximum transformed-goal jump, hold duration, command maxima, and final target error.

---

### Task 11: Close the task loop with independent UGV visual inspection

**Files:**
- Create: `src/air_ground_bringup/scripts/ugv_anomaly_inspector.py`
- Modify: `src/air_ground_bringup/scripts/uav_sphere_mission.py:136-182,562-570`
- Modify: `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch`
- Modify: `src/air_ground_bringup/CMakeLists.txt:16-27`
- Test: `src/air_ground_bringup/test/test_ugv_anomaly_inspector.py`
- Test: `src/air_ground_bringup/test/inspection_relay.test`

**Interfaces:**
- Consumes existing: `/ugv_0/camera/image_raw` and `/ugv_0/camera/camera_info` from `ugv_mvp/model.sdf:91-116`.
- Publishes: `/air_ground/inspection/visible` as `Bool`.
- Publishes: `/air_ground/inspection/bearing` as `PointStamped` normalized optical ray.
- Publishes: `/air_ground/inspection/status` as `String`: `WAITING`, `SEARCHING`, `CONFIRMED`, `NOT_FOUND`, `TIMEOUT`.
- Mission completion phase: `INSPECTION_CONFIRMED`; failure phase: `ERROR_INSPECTION`.

- [ ] **Step 1: Write failing image tests using generated red/non-red frames**

Generate synthetic OpenCV images in memory. Test minimum area, circularity, center ROI, repeated-frame confirmation, and rejection of a red rectangle that fails the configured shape test.

- [ ] **Step 2: Implement the UGV detector using the existing HSV/circularity conventions**

Require `N` confirmations in the last `M` frames and an image timestamp newer than UGV arrival. Publish a normalized bearing from `CameraInfo`; do not use Gazebo truth or the UAV target topic.

- [ ] **Step 3: Add mission completion semantics**

Subscribe to `/air_ground/ugv/arrived` and inspection status. Change the post-dispatch flow:

```text
DISPATCH -> OVERWATCH -> WAIT_INSPECTION
ARRIVED + CONFIRMED -> INSPECTION_CONFIRMED
ARRIVED + timeout/not found -> ERROR_INSPECTION
```

- [ ] **Step 4: Add rostest assertions for success and false arrival**

Arrival without camera confirmation must not complete the mission. Confirmation before arrival must not complete the mission. Only both conditions within freshness limits produce `INSPECTION_CONFIRMED`.

- [ ] **Step 5: Run camera and integration tests**

Run:

```bash
python3 -m unittest src/air_ground_bringup/test/test_ugv_anomaly_inspector.py -v
rostest air_ground_bringup inspection_relay.test
```

- [ ] **Step 6: Run a full cold-start inspection trial**

Expected terminal sequence:

```text
registration TRACKING
handoff DIRECT or REREGISTER then DIRECT
UGV NAVIGATING -> ARRIVED
inspection SEARCHING -> CONFIRMED
mission INSPECTION_CONFIRMED
```

- [ ] **Step 7: Verification checkpoint M3-C**

Record target truth distance, UGV final distance, camera confirmation count, completion time, and any false confirmations.

---

## Milestone 4: Reproducible Paper Experiment Matrix

### Task 12: Run one-shot, periodic, opportunistic, and uncertainty-aware evaluations

**Files:**
- Create: `src/air_ground_experiments/config/opportunistic_matrix.yaml`
- Create: `src/air_ground_experiments/config/handoff_matrix.yaml`
- Modify: `src/air_ground_experiments/scripts/run_experiment_matrix.py`
- Modify: `src/air_ground_experiments/scripts/experiment_recorder.py`
- Modify: `src/air_ground_experiments/src/air_ground_experiments/metrics.py`
- Create: `docs/experiments/gnss_denied_air_ground_protocol.md`
- Test: `src/air_ground_experiments/test/test_metrics.py`
- Test: `src/air_ground_experiments/test/test_matrix_expansion.py`

**Interfaces:**
- Produces one CSV row per trial and one immutable JSON metadata file per trial.
- Produces aggregate CSV grouped by method, drift level, visibility interval, and target range.
- Produces no plot-specific hidden preprocessing; plotting may consume aggregate CSV directly.

- [ ] **Step 1: Lock the method and disturbance factors**

Methods:

```text
no_align, one_shot, periodic, opportunistic, uncertainty_aware, oracle
```

`oracle` is computed by the evaluation package as an offline upper bound from recorded truth. It must never publish a transform or goal into the autonomy graph.

Principal factors:

```text
drift: zero, low, medium, high
visibility interval: 15 s, 30 s, 60 s, never-after-initial
target range: 5 m, 10 m, 15 m
gross observation outlier rate: 0%, 10%, 25%
communication delay: 0 ms, 100 ms, 300 ms
```

Use a fractional factorial primary matrix so the first paper experiment remains tractable. Reserve full Cartesian sweeps for focused ablations.

- [ ] **Step 2: Add failing aggregate-metric tests**

Test median, p95, bootstrap 95% confidence interval with fixed resampling seed, failure-rate denominator including failed launches, outlier rejection rate, and inspection success.

- [ ] **Step 3: Implement mandatory trial schema**

Every row includes:

```text
trial_id, seed, method, software_version, parameter_hash,
initial_uav_pose, initial_ugv_pose, injected_frame_offsets, drift_rates,
visibility_schedule, registration_revisions, registration_error_xy,
registration_error_yaw, registration_nees, target_handoff_error,
goal_jump_max, ugv_final_error, invalid_ugv_distance,
inspection_confirmed, mission_duration, result, failure_code
```

Use `software_version="unversioned-workspace"` until the user supplies a Git repository; include SHA-256 hashes of launch/config/source manifests so runs remain distinguishable.

- [ ] **Step 4: Add preflight and postflight enforcement**

Preflight rejects a trial if matching ROS/Gazebo/PX4 processes exist, output files already exist without `--resume`, or required topics are configured to consume truth namespaces. Postflight waits for complete process cleanup and marks residue as `SHUTDOWN_FAILED`.

- [ ] **Step 5: Execute calibration seeds only**

Use separate calibration seed ranges for process-noise and policy thresholds. Freeze all parameters before test seeds. Save the frozen config hash in `docs/experiments/gnss_denied_air_ground_protocol.md`.

- [ ] **Step 6: Execute at least 20 held-out seeds per principal condition**

Do not remove failures. Resume interrupted matrices by detecting completed trial IDs, not by overwriting result files.

- [ ] **Step 7: Check milestone hypotheses**

Required comparisons:

```text
one-shot vs no-align under independent initial frames
opportunistic vs one-shot under medium drift
uncertainty-aware vs opportunistic-only for inspection success and invalid travel
periodic vs opportunistic for registration cost
all methods vs oracle upper bound
```

Acceptance targets from the spec:

```text
M1 completion >= 95%, translation p95 <= 0.15 m, yaw p95 <= 2 deg
M2 RMSE reduction >= 30% at medium drift, gross-outlier rejection >= 95%
M3 success gain >= 15 percentage points OR invalid travel reduction >= 20%
```

- [ ] **Step 8: Run the complete verification suite**

Run:

```bash
python3 -m unittest discover -s src/air_ground_coordinate_transform/test -p 'test_*.py' -v
python3 -m unittest discover -s src/air_ground_bringup/test -p 'test_*.py' -v
python3 -m unittest discover -s src/air_ground_experiments/test -p 'test_*.py' -v
rostest air_ground_coordinate_transform registration_node.test
rostest air_ground_bringup inspection_relay.test
catkin_make --pkg air_ground_coordinate_transform air_ground_perception air_ground_bringup air_ground_experiments
roslaunch --check air_ground_bringup air_ground_final_demo.launch
roslaunch --check air_ground_bringup air_ground_inspection_experiment.launch
```

Expected: all tests pass, both launches resolve, and the legacy Demo remains usable.

- [ ] **Step 9: Perform plan-level final dynamic verification**

Run one clean trial for each method, then run the frozen held-out matrix. Confirm no background process remains after the last trial. Report exact commands, result file paths, aggregate metrics, failed-run counts, and residual limitations.

- [ ] **Step 10: Verification checkpoint M4**

The system is paper-experiment ready when every table value can be regenerated from the frozen YAML, source/config hashes, CSV rows, and documented aggregation commands without manual rosbag inspection or hidden coordinate alignment.

---

## Milestone Dependency and Review Gates

| Milestone | Depends on | Reviewer may reject independently for |
|---|---|---|
| M0 testable cores | Current Demo | Behavior regression or untestable ROS-coupled math |
| M1 one-shot baseline | M0 | Shared-world-frame leakage or irreproducible seeds |
| M2 opportunistic filter | M1 | Uncalibrated covariance, unsafe update, or weak baseline |
| M3 task relay | M2 | Odom-frame canonical goals or arrival-only success claim |
| M4 paper matrix | M1-M3 | Truth leakage, missing failures, or unreproducible aggregation |

Do not start M2 until M1 meets its acceptance criteria. Do not tune M2 process noise on M4 held-out seeds. Do not claim task-level benefit until M3 uses independent UGV camera confirmation.

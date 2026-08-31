# Task 4: Add deterministic independent-frame perturbation and one-shot metrics

## Milestone

Milestone 1: Multi-Initial-Pose One-Shot Registration Baseline.

## Files

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

## Interfaces

- Produce `FramePerturbation(initial_xyyaw, drift_rates, seed)`.
- Produce `FramePerturbation.at(stamp_seconds: float) -> np.ndarray` with state `[x, y, yaw]`.
- Produce `transform_odom(pose, twist, stamp) -> (pose, twist, truth_transform)`.
- Publish `/air_ground_experiment/uav/odom` and `/air_ground_experiment/ugv/odom`.
- Publish evaluation-only `/air_ground_experiment/truth/uav_frame` and `/air_ground_experiment/truth/ugv_frame`.
- Republish `/air_ground_experiment/charuco/observation` after seeded visibility, delay, and outlier scheduling.
- Produce CSV columns defined by `TRIAL_COLUMNS` in `metrics.py`.

## Steps

1. Write failing deterministic perturbation tests:

   ```python
   first = FramePerturbation([2.0, -1.0, 0.4], [0.01, 0.0, 0.001], seed=17)
   second = FramePerturbation([2.0, -1.0, 0.4], [0.01, 0.0, 0.001], seed=17)
   self.assertEqual(first.at(30.0), second.at(30.0))
   ```

   Also assert pose and twist rotate consistently and covariance transforms with the same Jacobian.
2. Write failing metric tests for wrapped yaw error, 2-D handoff error, final inspection distance, success-radius classification, and a failed trial row with a nonempty failure code.
3. Implement pure perturbation and metric modules. Use a piecewise deterministic random walk sampled at fixed `drift_step_seconds`; never call global `np.random`. Store and expose the seed in every truth message and result row.
4. Implement the odometry ROS adapter. Parameters must include source/destination topic, source/destination frame, initial `[x,y,yaw]`, translational drift rate, yaw drift rate, drift step, and seed. Rotate pose, linear twist, angular twist, and covariance consistently.
5. Implement the UAV command-frame adapter. Subscribe to `/air_ground_experiment/iris_0/position_cmd`, invert the current injected UAV-frame transform, and publish `/iris_0/position_cmd` for CXR. Preserve z, yaw-rate semantics, timestamp, and trajectory flags. Do not subscribe to Gazebo truth.
6. Implement deterministic intermittent-observation gating. Subscribe to the real ChArUco observation and republish only during configured visibility windows. Use ROS time and a seeded queue to apply configured delay and gross SE(2) outliers. Preserve the original image timestamp plus an explicit injected-delay diagnostic; never synthesize a valid board observation when the detector published none.
7. Implement the evaluation-only recorder. It may subscribe to `/gazebo/model_states` and experiment truth topics. It must only publish `/air_ground_experiment/evaluation/status`; enforce this by keeping all other publishers out of the class. Write one CSV row and one JSON metadata file per trial, including failures and timeout state.
8. Add package installation and tests. Register scripts with `catkin_install_python`, config/launch directories with `install`, and unit tests under `CATKIN_ENABLE_TESTING`.
9. Run package pure tests and bounded static topic/structure audit. Build `air_ground_experiments`. XML-parse the launch file. Statically verify the recorder has only the evaluation-status publisher and does not connect to autonomy input topics.
10. M1-B external verification: run one zero-drift trial and confirm perturbation outputs equal raw odometry up to the configured constant frame transform.

## Current-Environment Constraints

- Strict RED -> GREEN TDD for every production behavior change; retain command/output evidence.
- If an allowed bounded test fails unexpectedly, invoke `systematic-debugging` before fixing.
- Do not execute `roslaunch`, `roscore`, `rostest`, Gazebo, PX4 SITL, RViz, rosbag, `rosnode info`, topic wait/echo loops, or any long-running process.
- Allowed verification: pure unit tests, `py_compile`, XML parsing, static AST/source wiring audit, and bounded catkin builds.
- The zero-drift M1-B runtime trial is external/manual only in this environment.
- Gazebo truth is evaluation-only. It must never be imported, subscribed, or routed by perturbation, gate, command adapter, mission, registration, or controller autonomy code.
- The recorder must never publish autonomy inputs; its only ROS publisher is `/air_ground_experiment/evaluation/status`.
- Preserve Tasks 1-3 behavior and the legacy final Demo.
- The workspace has no Git metadata. Do not initialize Git or claim commits.

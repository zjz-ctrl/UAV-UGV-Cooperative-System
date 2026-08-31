# Task 5 Review Package

## Repository State

No Git metadata exists. Review complete current Task 5 files against the brief.

## Files In Scope

- `src/air_ground_experiments/scripts/run_experiment_matrix.py`
- `src/air_ground_experiments/config/one_shot_matrix.yaml`
- `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch`
- `src/air_ground_bringup/CMakeLists.txt`
- `src/air_ground_bringup/package.xml`
- `src/air_ground_experiments/test/test_matrix_expansion.py`

## Binding Constraints

- `expand_matrix` is deterministic per seed, sorted by `trial_id`, 30 trials,
  fixture asserts seed 1000 first and `one_shot-0029` last; bounds exactly as
  the brief's YAML; drift zero.
- UGV physical spawn sits at the UAV body-relative registration waypoint.
- Trial lifecycle: process pre-check, explicit-argument launch start,
  FROZEN-or-timeout wait, terminal-state wait, recorder flush request, SIGINT,
  child-process reaping, exit classification (`PASS`, `LAUNCH_FAILED`,
  `TIMEOUT`, `REGISTRATION_FAILED`, `MISSION_FAILED`); no ROS master reuse
  between trials. Runner verified only via injected fakes in this environment.
- Integration launch: perturbation between raw odometry and research nodes;
  UAV commands routed through adapter; registration `one_shot` with visual yaw
  enabled; evaluator started; truth confined to experiment namespace.
- Matrix YAML frozen for M1-C with exact parameters and seed list documented.
- Preserve Tasks 1-4 behavior/interfaces and the legacy Demo.
- Dynamic smoke seeds / 30-seed matrix / acceptance thresholds remain
  unverified under the no-long-process ruling.
- Never run ROS launch/core/test, simulation, PX4, RViz, rosbag, rosnode/topic
  loops, the runner itself, or any long process in this environment.

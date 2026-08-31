# Task 5: Build and execute the one-shot baseline matrix

## Milestone

Milestone 1: Multi-Initial-Pose One-Shot Registration Baseline.

## Files

- Create: `src/air_ground_experiments/scripts/run_experiment_matrix.py`
- Create: `src/air_ground_experiments/config/one_shot_matrix.yaml`
- Create: `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch`
- Modify: `src/air_ground_bringup/CMakeLists.txt`
- Modify: `src/air_ground_bringup/package.xml`
- Test: `src/air_ground_experiments/test/test_matrix_expansion.py`

## Interfaces

- Produce `TrialSpec(trial_id, seed, launch_args, timeout_seconds, output_directory)`.
- Produce `expand_matrix(config: dict) -> list[TrialSpec]` sorted by `trial_id`.
- Produce cold-start runner exit codes `PASS`, `LAUNCH_FAILED`, `TIMEOUT`, `REGISTRATION_FAILED`, `MISSION_FAILED`.
- Launches `air_ground_inspection_experiment.launch registration_mode:=one_shot`.

## Steps

1. Write a failing deterministic matrix expansion test:

   ```python
   trials = expand_matrix(load_fixture())
   self.assertEqual(len(trials), 30)
   self.assertEqual(trials[0].seed, 1000)
   self.assertEqual(trials[-1].trial_id, "one_shot-0029")
   ```

2. Define the one-shot matrix explicitly. Use 30 seeds sampled within declared
   bounds:

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

   Keep UGV physical spawn at the UAV body-relative registration waypoint so
   this milestone tests registration rather than board search.
3. Implement the cold-start trial lifecycle. For every trial:

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
4. Build the integration launch: include perturbation nodes between raw
   odometry and research nodes, route UAV commands through the adapter, run the
   registration node in `one_shot` mode with visual yaw enabled, and start the
   evaluator. Keep Gazebo truth topics under the experiment namespace.
5. Prepare three smoke seeds command (`one_shot-0000..0002`) and the full
   30-seed execution command, but do NOT run them in this environment.
6. Document acceptance for the full matrix:

   ```text
   registration completion >= 95%
   translation error p95 <= 0.15 m
   yaw error p95 <= 2.0 deg
   all failures retain a reason code
   ```

7. Verification checkpoint M1-C: freeze the matrix YAML and report the exact
   software parameters, seed list, success rate formula, translation/yaw
   percentiles definition, and failure taxonomy. Do not tune thresholds after
   inspecting held-out seeds.

## Current-Environment Constraints

- Strict RED -> GREEN TDD for every production behavior change; retain
  per-behavior RED/GREEN command/output evidence.
- If an allowed bounded test fails unexpectedly, invoke `systematic-debugging`
  before fixing.
- Do not execute `roslaunch` (including the runner itself), `roscore`,
  `rostest`, Gazebo, PX4 SITL, RViz, rosbag, `rosnode info`, topic wait/echo
  loops, or any long-running process. The trial-lifecycle runner must be
  implemented and verified through pure/injected-fake tests only; its real
  execution is external M1-C work.
- Allowed verification: pure unit tests, `py_compile`, XML parsing, static
  AST/source audits, and bounded catkin builds.
- Sampling for the matrix must be deterministic from declared seeds; never use
  global RNG state.
- Preserve Tasks 1-4 behavior, interfaces, and the legacy Demo.
- Gazebo truth remains evaluation-only under the experiment namespace; the
  recorder is the only consumer.
- The workspace has no Git metadata. Do not initialize Git or claim commits.

## Prior-Task Interfaces To Reuse

- `TrialResultWriter`, canonical statuses {COMPLETED, FAILED, TIMEOUT}, failure
  codes, and CSV columns from `air_ground_experiments.metrics`.
- Recorder terminal phase `/air_ground/mission_phase` with
  `INSPECTION_CONFIRMED` success semantics.
- Perturbation launch `frame_perturbation.launch` parameters including shared
  `epoch_seconds` and domain-separated stream labels.
- Registration node `use_visual_frame_yaw:=true` research path and
  `registration_mode` evolution contract (one_shot today).
- Mission body-relative registration waypoint parameters `registration_dx`/
  `registration_dy` and independent `uav_yaw`/`ugv_yaw` spawn arguments.

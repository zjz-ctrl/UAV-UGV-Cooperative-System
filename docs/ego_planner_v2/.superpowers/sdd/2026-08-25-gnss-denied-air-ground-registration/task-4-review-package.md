# Task 4 Review Package

## Repository State

No Git metadata exists. Review complete current Task 4 files against the brief.

## Files In Scope

- `src/air_ground_experiments/package.xml`
- `src/air_ground_experiments/CMakeLists.txt`
- `src/air_ground_experiments/setup.py`
- `src/air_ground_experiments/config/frame_perturbation.yaml`
- `src/air_ground_experiments/launch/frame_perturbation.launch`
- `src/air_ground_experiments/src/air_ground_experiments/__init__.py`
- `src/air_ground_experiments/src/air_ground_experiments/frame_perturbation.py`
- `src/air_ground_experiments/src/air_ground_experiments/metrics.py`
- `src/air_ground_experiments/scripts/odom_perturbation_node.py`
- `src/air_ground_experiments/scripts/observation_gate.py`
- `src/air_ground_experiments/scripts/position_command_adapter.py`
- `src/air_ground_experiments/scripts/experiment_recorder.py`
- `src/air_ground_experiments/test/test_frame_perturbation.py`
- `src/air_ground_experiments/test/test_metrics.py`
- `src/air_ground_experiments/test/test_adapter_serialization.py`
- `src/air_ground_experiments/test/test_package_safety.py`

## Binding Constraints

- Perturbation is deterministic for a seed and fixed drift step, including
  repeated/out-of-order timestamp queries; never use global NumPy RNG.
- Pose, twist, and covariance transform consistently under the injected frame.
- Command routing locally inverts the injected transform and consumes no truth.
- Observation gating never synthesizes detector observations and preserves the
  source image timestamp while diagnosing injected delay/outlier behavior.
- Gazebo/model truth is evaluation-only and confined to the recorder.
- The recorder's only publisher is `/air_ground_experiment/evaluation/status`;
  it cannot publish autonomy inputs.
- Every trial, including failure/timeout, produces schema-valid CSV and JSON.
- Preserve Tasks 1-3 and the legacy Demo.
- Dynamic M1-B remains unverified under the no-long-process ruling.
- Never run ROS launch/core/test, simulation, PX4, RViz, rosbag, rosnode/topic
  loops, or any long process in this environment.

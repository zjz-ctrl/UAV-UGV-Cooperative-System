# GNSS-Denied UAV-UGV Registration and Task Relay Design

## Scope

Evolve `air_ground_final_demo.launch` into a paper-grade experiment system for:

1. One-shot UAV-UGV coordinate registration from varied initial positions and headings.
2. Uncertainty-aware opportunistic re-registration under intermittent mutual visibility.
3. UAV anomaly localization, common-frame target handoff, and UGV close-range inspection.

The first research system remains one UAV and one UGV. It does not add collaborative
SLAM, multi-robot allocation, or complex map sharing.

## Research Boundary

The research state is the planar transform

\[
x = [t_x, t_y, \psi]^T = {}^OT_G,
\]

where `O` is `air_ground_origin` and `G` is the UGV experimental odometry frame.
The UAV origin transform remains anchored to the captured takeoff frame. Relative
observations update the UAV-to-UGV relationship; they do not claim globally drift-free
localization.

The red sphere remains an anomaly proxy so experiments isolate registration and task
handoff instead of introducing an unrelated perception benchmark. Gazebo truth is
available only to the experiment evaluator and must never publish control, registration,
target, or mission-decision topics.

## Architecture

### Coordinate Estimation

Refactor matrix, interpolation, robust batch estimation, covariance computation, and
SE(2) filtering into importable pure-Python modules. The ROS node supports two modes:

- `one_shot`: freeze the first accepted robust batch and reproduce the current baseline.
- `opportunistic`: initialize from the first batch, grow covariance with elapsed time and
  UAV/UGV odometric travel, and fuse later robust batches after innovation gating.

The published registration estimate is a `PoseWithCovarianceStamped` in
`air_ground_origin`. Covariance slots `(x, y, yaw)` use ROS indices `(0, 7, 35)` with
cross-covariances populated where available. A monotonically increasing `UInt32`
revision identifies accepted updates.

### Experimental Odometry

A separate `air_ground_experiments` package applies seeded SE(2) origin offsets,
heading offsets, and deterministic random-walk drift to copied UAV and UGV odometry
topics. This prevents the shared Gazebo ENU axes from making registration artificially
easy. The injected truth is published only under `/air_ground_experiment/truth/*`.

UAV high-level commands expressed in the experimental UAV odom frame pass through a
frame adapter before reaching the existing CXR controller. UGV `cmd_vel` remains a body
velocity and needs no equivalent adapter.

### Target Relay

The UAV mission publishes the final anomaly as `PoseWithCovarianceStamped` in the UAV
experimental odom frame. A target handoff node transforms both mean and covariance into
`air_ground_origin`, keeps that common-frame estimate canonical, and selects one action:

- `DIRECT`: uncertainty is inside the configured inspection budget.
- `REOBSERVE`: target sensing uncertainty dominates and the UAV should refine the target.
- `REREGISTER`: registration uncertainty dominates and the UAV should revisit the UGV.
- `HOLD`: no safe handoff is currently available.

The UGV controller stores the canonical goal in `air_ground_origin` and resolves it into
UGV odom on every control tick. A registration revision that moves the resolved goal more
than the configured jump threshold causes a stop before accepting the corrected goal.

### Close Inspection

The existing UGV front RGB camera provides an independent close-range confirmation. A
UGV anomaly detector publishes visual confirmation and bearing. Mission success requires
both UGV arrival and a valid close-range confirmation; arrival alone is not inspection
success.

### Evaluation

An experiment recorder writes CSV and JSON metadata directly; rosbag is not required.
Every trial has an explicit seed and launch arguments. Metrics include registration
translation/yaw error, covariance consistency, target handoff error, final UGV error,
inspection success, invalid travel, update jump, mission time, and re-registration count.

## Baselines

- `no_align`: intentionally assumes the frames coincide.
- `one_shot`: current robust ChArUco registration, generalized to arbitrary SE(2).
- `periodic`: re-register at a fixed interval when observations are available.
- `opportunistic`: update on accepted intermittent observations.
- `uncertainty_aware`: choose direct handoff, re-observation, or re-registration from the
  uncertainty budget.
- `oracle`: use injected/world truth only as an offline upper bound.

## Safety and Validity Constraints

- No `/gazebo/get_model_state` or `/gazebo/model_states` value may enter autonomy.
- Only one node broadcasts each parent-child TF edge.
- Every dynamic trial starts from a clean ROS, Gazebo, PX4, and MAVROS process state.
- Registration updates are rejected when odometry timestamps, image quality, velocity,
  innovation, or covariance checks fail.
- The initial one-shot Demo remains launchable as a compatibility baseline.
- No hidden offline alignment is applied before reporting public-frame errors.
- Simulated frame perturbations and ground truth are logged with every result row.

## Milestone Acceptance Summary

### M1: One-Shot Baseline

- At least 30 seeded initial-pose trials.
- At least 95% registration completion.
- 95th percentile translation error at most 0.15 m.
- 95th percentile yaw error at most 2 degrees.
- No Gazebo truth topic consumed outside `air_ground_experiments`.

### M2: Opportunistic Registration

- Compare one-shot, periodic, opportunistic, and oracle under identical drift seeds.
- Opportunistic registration reduces relative-transform RMSE by at least 30% against
  one-shot at medium drift.
- At least 95% of configured gross outliers are rejected.
- Registration revisions never drive the UGV through an unbounded goal jump.

### M3: Task Relay

- Anomaly coordinates remain canonical in `air_ground_origin`.
- Mission completion requires UGV arrival plus independent UGV visual confirmation.
- Under the moderate-drift matrix, uncertainty-aware handoff improves inspection success
  by at least 15 percentage points or reduces invalid UGV travel by at least 20% relative
  to one-shot while reporting both metrics.

### M4: Paper Experiments

- At least 20 seeds per principal simulation condition.
- Every figure/table can be regenerated from frozen configuration and CSV/JSON output.
- Failed runs are retained and classified rather than removed.

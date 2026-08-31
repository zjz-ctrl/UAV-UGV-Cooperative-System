# Task 7: Integrate repeated observation windows, revisions, and degraded status

## Milestone

Milestone 2: Uncertainty-Aware Opportunistic Re-Registration, checkpoint M2-B.

## Files

- Modify: `src/air_ground_coordinate_transform/scripts/takeoff_registration.py`
- Modify: `src/air_ground_coordinate_transform/config/registration.yaml`
- Modify: `src/air_ground_coordinate_transform/launch/coordinate_transform.launch`
- Modify: `src/air_ground_bringup/scripts/ugv_coordinate_monitor.py`
- Test/modify: `src/air_ground_coordinate_transform/test/registration_node.test`
- Test/modify: `src/air_ground_coordinate_transform/test/test_registration_node.py`
- A minimal ROS-independent helper/test may be added under
  `air_ground_coordinate_transform` only when needed to test event/window logic
  without a ROS master. It must not duplicate Task 6 filter mathematics.

## Required Interfaces

- Consume `RegistrationFilter` and configured process/gate parameters from Task 6.
- Modes: `one_shot`, `periodic`, `opportunistic`; default `one_shot`.
- Repeated-mode statuses: `CAPTURING_ORIGIN`, `ACQUIRING_INITIAL`, `TRACKING`,
  `DEGRADED`, `UPDATING`, `REJECTED`.
- Preserve legacy one-shot post-registration status `FROZEN`.
- Publish existing `/valid`, `/frozen`, `/status`, `/inlier_count`, `/estimate`,
  `/revision`, both existing TF edges, and `/air_ground/ugv/pose_takeoff`.
- Add `/air_ground/registration/innovation` as `std_msgs/Float64`.
- `/frozen=True` means registration initialized and remains true after rejection.

## Registration Event Semantics

- Filter starts uninitialized. First valid robust batch initializes revision 1.
- A visual frame only appends a candidate sample. It never increments revision.
- A registration event exists only when a fresh robust window forms a batch and
  `RegistrationFilter.update()` accepts it. Exactly then revision increments once.
- A rejected/invalid/stale/gated/inconsistent window leaves current mean,
  covariance, TF, stamp, and revision unchanged.
- Clear the consumed window after every actual batch decision, accepted or
  rejected. Do not reuse samples across registration events.
- In `one_shot`, stop sample acceptance after initialization and preserve revision
  1 plus legacy `FROZEN` behavior.
- In `opportunistic`, attempt a batch as soon as a fresh reliable window is ready.
- In `periodic`, use the same robust estimator and gate, but accept at most one
  batch when `now-last_revision_time >= periodic_update_seconds`; it is a timing
  baseline, not a second estimator.

## Fresh Window And Consistency

- Retain only samples within `registration_window_seconds` and cap at
  `registration_window_max_samples`.
- Respect existing `sample_period`, frame, timestamp bracket, UAV height/speed,
  UAV angular speed, and UGV speed gates before appending.
- Require at least `minimum_samples`, then use the existing
  `RobustBatchEstimator.estimate_with_inliers()` and fixed/visual-yaw policy.
- If a full/capped window cannot form a robust batch, report reason
  `insufficient_inliers`, publish coarse `REJECTED`, clear it, and keep revision.
- Filter rejection reasons (`invalid_batch`, `stale_batch`,
  `singular_innovation_covariance`, `mahalanobis_gate`) must be preserved in
  throttled logs and tests. Publish NIS on `/innovation` whenever computed.
- Observation batches no older than `max_odom_bracket` relative to current filter
  time may be coalesced to current time to absorb callback ordering only. Anything
  older must enter the filter as stale and be rejected; do not silently rewrite
  materially delayed measurements.

## Prediction And Degradation

- From valid monotonic odometry callbacks, accumulate incremental planar distance
  separately for UAV and UGV.
- Advance elapsed time once using one global monotonic prediction stamp. A second
  vehicle callback at an older/equal stamp contributes only its own new distance,
  not duplicate elapsed time.
- Prediction calls Task 6 `predict(dt, uav_distance, ugv_distance)` and therefore
  changes covariance/stamp only, never mean/revision.
- Publish covariance-bearing estimate while hidden so growth is observable.
- Enter `DEGRADED` when initialized repeated-mode covariance trace exceeds
  `degraded_covariance_trace_threshold`; return to `TRACKING` after an accepted
  update contracts it below threshold. Degradation never changes revision.
- Protect prediction/window/update/current publication state with one lock or an
  equivalent serialized pure coordinator; rospy connection threads must not
  create duplicate registration events.

## Publishing

- On first accepted batch: publish valid/frozen true, revision 1, estimate, TF;
  use `FROZEN` in one-shot and `TRACKING` in repeated modes.
- On later accepted batch: publish `UPDATING`, new estimate/TF/revision/inliers/NIS,
  then `TRACKING` or `DEGRADED` according to posterior covariance.
- On rejection: publish `REJECTED`, log exact reason, publish finite NIS when
  available, and continue broadcasting the previous valid transform.
- Keep exactly one broadcaster for `air_ground_origin -> ugv experimental odom`.

## Coordinate Monitor

- Subscribe to revision and covariance-bearing estimate in addition to frozen and
  odometry.
- Log revision, `sigma_x`, `sigma_y`, `sigma_yaw_deg`, and registration delta from
  the previous revision separately from UGV travel delta.

## Configuration

Add/document at least:

- `registration_mode: one_shot`
- `registration_window_seconds`
- `registration_window_max_samples`
- `periodic_update_seconds`
- `degraded_covariance_trace_threshold`
- Task 6 process rates and `innovation_mahalanobis_threshold` remain the sole
  prediction/gate parameters and remain explicitly provisional until calibrated.

Expose mode through `coordinate_transform.launch`. Resolve the Task 5 deferred
finding by actually consuming `~registration_mode` in the node.

## TDD And Verification

1. First write failing pure/stub tests for frame-vs-event semantics, first revision
   1, accepted second window revision 2, rejected batch unchanged revision/state,
   consumed-window clearing, periodic interval, prediction-only covariance growth,
   degradation, and concurrent duplicate-attempt prevention.
2. Extend written rostest for first accepted batch, second consistent batch,
   gross-outlier rejection, degraded status, and one-shot contradictory batch.
   Write it but do not run it in this environment.
3. Tests must prove multiple individual frames cannot increment revision and
   rejected windows cannot alter the valid TF/estimate.
4. Preserve all Task 6 pure tests and current one-shot tests.
5. Run pure/stub tests, `py_compile`, XML/YAML parsing, and bounded catkin build.

## Safety And Runtime Constraint

- Never read Gazebo/experiment truth in registration, gating, prediction, status,
  revision, TF, mission, or monitor decisions.
- Do not run `roslaunch`, `roscore`, `rostest`, Gazebo, PX4, RViz, rosbag, topic
  waits, or any long-running process. Dynamic intermittent-visibility and one-shot
  compatibility trials are external/manual.
- Do not initialize Git or claim commits.

## M2-B External Acceptance

- Scripted windows: visible 0-5 s, hidden 5-35 s, visible 35-40 s, hidden
  40-70 s, visible 70-75 s.
- Repeated mode: revisions exactly `1 -> 2 -> 3`; covariance grows while hidden
  and contracts only after accepted windows.
- One-shot mode: revision remains exactly 1 without node restart.

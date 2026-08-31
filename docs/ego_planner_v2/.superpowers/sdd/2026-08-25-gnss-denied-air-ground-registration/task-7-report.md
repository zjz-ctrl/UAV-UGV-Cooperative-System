# Task 7 Report: Repeated Registration Integration

## Status

**PASS for Task 7 implementation and bounded verification. External M2-B ROS
acceptance remains intentionally unrun and pending.**

- No commit was created and no Git command was used.
- No subagent or reviewer was dispatched.
- No `roslaunch`, `roscore`, `rostest`, Gazebo, PX4, RViz, rosbag, topic wait,
  or other long-running ROS process was run.
- Task 6 `RegistrationFilter.predict()` and `RegistrationFilter.update()` are
  called directly. No NIS, gain, Joseph covariance, 3x3 update, or process-noise
  mathematics was copied into Task 7.

## Event And Revision Invariants

1. The filter starts uninitialized at revision 0.
2. A visual callback can append at most one candidate sample after all existing
   frame, timestamp bracket, height, linear/angular speed, and `sample_period`
   gates pass. Appending never changes revision.
3. A registration event occurs only after the existing
   `RobustBatchEstimator.estimate_with_inliers()` returns a batch and the sole
   Task 6 `RegistrationFilter.update()` accepts it.
4. The first accepted batch initializes revision 1. Each later accepted batch
   increments exactly once through Task 6. Rejections and predictions do not
   increment revision.
5. One `threading.RLock` in `RegistrationCoordinator` serializes window,
   prediction, filter, status, and snapshot state. All ROS callbacks and timer
   publication use that same lock; the coordinator uses reentrancy rather than a
   second lock.
6. `one_shot` stops accepting samples after revision 1 and suppresses later
   prediction, preserving the accepted state and legacy `FROZEN` status.
7. The node has exactly one `RegistrationFilter`, one
   `TransformBroadcaster`, and one call site for the filter update.

## Window Lifecycle

- Samples older than `registration_window_seconds` relative to the newest
  accepted candidate are removed before use.
- The window is capped at `registration_window_max_samples` and cannot be
  configured below `minimum_samples`.
- `opportunistic` attempts immediately when the fresh window yields a robust
  batch.
- `periodic` uses the same estimator/filter but defers an initialized update
  until `now - last_revision_time >= periodic_update_seconds`; it does not own a
  second estimator or state.
- A capped/aged window with fewer than `minimum_samples` robust inliers produces
  `insufficient_inliers`, publishes coarse `REJECTED`, and is cleared.
- Every actual filter decision, accepted or rejected, clears the consumed
  window. Samples are never reused across events.
- Filter reasons `invalid_batch`, `stale_batch`,
  `singular_innovation_covariance`, and `mahalanobis_gate` pass unchanged through
  the coordinator decision and throttled node log. `insufficient_inliers` is the
  explicit pre-filter robust-window reason.
- An initialized batch behind filter time by at most `max_odom_bracket` is
  rebuilt only with its stamp coalesced to current filter time. A materially
  older batch retains its original stamp and is rejected by Task 6 as
  `stale_batch`.

## Prediction, Time, And Distance Accounting

- UAV and UGV odometry each retain their own last valid monotonic stamp and XY
  position. The first callback establishes a baseline; later monotonic callbacks
  contribute incremental planar distance for only that vehicle.
- One global monotonic prediction stamp computes elapsed `dt`. A second vehicle
  callback at an older/equal global stamp contributes its own new distance with
  `dt=0`.
- Repeated modes call the sole Task 6 filter as
  `predict(dt, uav_distance, ugv_distance)`. Prediction changes filter stamp and
  covariance only; mean and revision remain unchanged.
- One-shot bookkeeping can observe odometry, but initialized one-shot state is
  not predicted or degraded.
- The timer republishes the current covariance-bearing estimate while visual
  observations are hidden, making process growth externally observable.
- Initialized repeated mode enters `DEGRADED` only when covariance trace exceeds
  `degraded_covariance_trace_threshold`. An accepted posterior is classified
  `TRACKING` again when its trace contracts below the threshold. Neither status
  transition changes revision.

## Status And Reason Semantics

- Before origin capture: `CAPTURING_ORIGIN`.
- After origin capture and before repeated-mode initialization:
  `ACQUIRING_INITIAL`.
- Legacy one-shot pre-registration status remains `ACQUIRING_REGISTRATION` to
  preserve the existing one-shot rostest contract.
- First repeated acceptance: valid/frozen true, revision 1, estimate/TF/inliers/
  NIS, then `TRACKING` or `DEGRADED` from posterior covariance.
- First one-shot acceptance: valid/frozen true, revision 1, then `FROZEN`.
- Later repeated acceptance: `UPDATING`, accepted estimate/TF/revision/inliers/
  NIS, then `TRACKING` or `DEGRADED`.
- Rejection: `REJECTED`, exact decision reason in throttled logs, finite NIS on
  `/air_ground/registration/innovation` when Task 6 computed it, and the prior
  valid mean/TF/revision retained.
- `/frozen` means initialized, not immutable covariance. It remains true after a
  repeated-mode rejection or degradation.
- The monitor logs revision, `sigma_x`, `sigma_y`, `sigma_yaw_deg`, and
  `registration_delta` at each new revision. Its odometry log labels the
  independent displacement as `travel_delta`.

## RED And GREEN By Behavior

| Behavior | Observed RED | Observed GREEN |
|---|---|---|
| Coordinator boundary | Missing module produced `ModuleNotFoundError` before the API shell | Import/API available |
| Frame versus event; first revision 1 | Behavior-level shell run: expected decision was `None` | Included in 11/11 coordinator pass |
| Accepted second window revision 2 and consumed window | Behavior-level shell run: initial/second decisions absent | Revision exactly 2; window size 0 |
| Gated rejection preserves estimate/TF proxy/stamp/revision | Behavior-level shell run: initialization absent | Exact NumPy state and SE(2) matrix equality |
| Capped insufficient-inlier rejection and clear | Behavior-level shell run: final decision absent | Explicit `insufficient_inliers`, revision 1, window 0 |
| Periodic interval | Behavior-level shell run: first decision absent | No early result; one due result at revision 2 |
| Same-window concurrency | Behavior-level shell run: zero events instead of one | Exactly one accepted result and revision 1 |
| Prediction growth and one global `dt` | Behavior-level shell run: initialization absent | Exact expected full covariance and stamp |
| Degradation and contraction | `ACQUIRING_INITIAL` instead of `TRACKING` | `TRACKING -> DEGRADED -> TRACKING`, revision only on update |
| Coalesced near delay and stale material delay | Behavior-level shell run: no near decision | Near batch accepted at current stamp; old batch `stale_batch` with exact state equality |
| One-shot unchanged status/revision/state | Behavior-level shell run: no initial decision | Later frames/odometry leave `FROZEN`, revision 1, exact state |
| Existing fixed-yaw policy | After API shell, yaw was `0.700066...`, expected `-0.4` | Existing policy callback runs before Task 6 update |
| YAML/launch mode and repeated parameters | `registration_mode` was absent (`None != one_shot`) | Pure XML/YAML wiring test passes |
| Monitor revision uncertainty/delta | `estimate_callback` was absent | ROS-stub callback test passes with hand-derived sigmas/deltas |

During the initial behavior-level RED, two tests dereferenced the intentionally
uninitialized API shell and raised `TypeError`. `systematic-debugging` was invoked;
the root cause was missing prerequisite assertions in test setup. Adding those
assertions produced the intended clean result: all 10 behavior tests failed by
assertion, with no errors, before event logic was implemented.

## Mutation Evidence

Three temporary mutations were applied one at a time, tested, and restored:

1. Disabled the initialized one-shot sample guard. The one-shot test failed with
   `REJECTED != FROZEN`, proving later contradictory frames cannot silently alter
   one-shot status/event behavior.
2. Replaced the global elapsed-time calculation with per-vehicle elapsed time.
   The exact covariance test failed with maximum absolute difference `0.1`,
   proving the second odometry callback cannot duplicate `dt`.
3. Called the Task 6 update twice for one robust window. Both the frame/event test
   and concurrent same-window test failed with revision `2 != 1`, proving the
   tests detect duplicate accepted events.

Additional mutation-sensitive assertions cover consumed-window size, exact
rejected mean/covariance/stamp/revision, exact transform matrix, periodic early
suppression, prediction mean immobility, and accepted posterior contraction.

## One-Shot And No-Truth Audit

- One-shot pure test sends 12 contradictory frames plus two separated odometry
  callbacks after initialization and asserts `FROZEN`, revision 1, exact mean,
  exact covariance, exact stamp, and empty window.
- Existing written one-shot rostest retains its contradictory post-registration
  observation sequence and revision/estimate assertions.
- Production search over `air_ground_coordinate_transform/src` and the modified
  registration node found no `truth`, `ground_truth`, `model_states`, or Gazebo
  truth references.
- The only `truth` matches in the package are pre-existing Task 6 Monte Carlo test
  variables in `test_registration_estimator.py`; they are not production inputs.
- Registration, gating, prediction, statuses, revisions, TF, and monitor behavior
  consume only configured odometry, real observation, and filter state.

## Written But Unrun Rostest

`registration_node.test` and `test_registration_node.py` now describe a separate
`opportunistic` node and assertions for:

- first accepted robust window at revision 1 and `TRACKING`;
- second consistent accepted window at revision 2;
- gross consistent outlier rejected with revision, estimate mean, and TF
  translation unchanged, plus finite NIS;
- hidden odometry prediction producing larger covariance and `DEGRADED` without a
  new revision; and
- the existing one-shot contradictory batch retaining revision 1.

The rostest was deliberately **not run** under the task safety constraint. Only
its Python compilation and XML parsing were run.

## Bounded Tests And Build

Fresh final evidence:

| Check | Result |
|---|---|
| Full pure/stub regression: SE(2), odom buffer, all Task 6 filter/Monte Carlo tests, Task 7 coordinator, monitor stub, launch wiring | **79/79 passed in 4.795 s** |
| `py_compile` for all changed Python production/test files | Exit 0, no output |
| XML parse for `coordinate_transform.launch` and `registration_node.test` | Exit 0 |
| YAML parse, required-key, default-mode, window-size assertions | Exit 0 |
| Bounded `timeout 120s catkin_make --pkg air_ground_coordinate_transform air_ground_bringup -j2` | Exit 0; `coordinate_transform_node` built 100% and both package make steps completed |
| Production no-truth search | No matches |
| Single filter/broadcaster/update audit | 1 constructor, 1 broadcaster, 1 coordinator update call |

The bounded build emitted existing workspace warnings about missing imported VTK
executables/libraries, disabled optional PCL features, deprecated Gazebo Classic
messages, and unrelated Eigen/system library metadata. None was fatal and none
originated in the changed Python/config/XML files.

## External M2-B Manual Draft

This is a draft for an external ROS/simulator-capable environment. It was not
executed here.

1. Before launch, configure the observation gate's
   `visibility_windows` in `frame_perturbation.yaml` as
   `[[0.0, 5.0], [35.0, 40.0], [70.0, 75.0]]`, with
   `visibility_probability: 1.0`, and freeze an explicit seed. The current launch
   does not expose these windows as command-line args, so record this controlled
   config override in the trial provenance.
2. Repeated-mode draft command:

   ```bash
   roslaunch air_ground_bringup air_ground_inspection_experiment.launch \
     registration_mode:=opportunistic use_visual_frame_yaw:=true \
     epoch_seconds:=0.0 timeout_seconds:=85.0 seed:=2707 \
     trial_id:=m2b-opportunistic-2707 \
     output_directory:=/tmp/air_ground_experiments/m2b-opportunistic-2707
   ```

3. Observe/record only interface topics for registration acceptance:
   `/air_ground/registration/revision`, `/status`, `/valid`, `/frozen`,
   `/estimate`, `/innovation`, `/inlier_count`, `/tf`,
   `/air_ground_experiment/charuco/observation`, and
   `/air_ground_experiment/charuco/injected_delay`. Experiment truth may be used
   by the separate evaluator, never by registration or acceptance decisions.
4. Repeated-mode criteria: accepted revisions exactly `1 -> 2 -> 3`, no revision
   during hidden intervals, covariance trace grows during 5-35 s and 40-70 s,
   and contracts only after accepted windows in 35-40 s and 70-75 s. `frozen`
   remains true after revision 1. Rejected windows retain the prior mean/TF/
   revision and emit their exact reason/NIS diagnostics.
5. One-shot compatibility draft command uses the same frozen visibility schedule
   and seed, changing only `registration_mode:=one_shot` and trial/output names.
   Criterion: revision remains exactly 1 through 85 s without node restart,
   status remains `FROZEN`, and contradictory/later windows do not change the
   registration estimate or TF.

## Modified Files

- `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_coordinator.py` (new)
- `src/air_ground_coordinate_transform/scripts/takeoff_registration.py`
- `src/air_ground_coordinate_transform/config/registration.yaml`
- `src/air_ground_coordinate_transform/launch/coordinate_transform.launch`
- `src/air_ground_coordinate_transform/CMakeLists.txt`
- `src/air_ground_coordinate_transform/test/test_registration_coordinator.py` (new)
- `src/air_ground_coordinate_transform/test/test_ugv_coordinate_monitor.py` (new)
- `src/air_ground_coordinate_transform/test/registration_node.test`
- `src/air_ground_coordinate_transform/test/test_registration_node.py`
- `src/air_ground_bringup/scripts/ugv_coordinate_monitor.py`
- `src/air_ground_bringup/test/test_launch_wiring.py`
- `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-7-report.md` (new)

## Self-Review

- The coordinator owns event/window/prediction/status sequencing but no filter
  mathematics and no broadcaster.
- All filter state returned by Task 6 remains copied through its existing
  snapshot contract; rejection compares exact prior values in tests.
- The fixed-yaw branch calls the existing `fixed_yaw_estimate()` through a small
  postprocessor, preserving both visual-yaw and legacy fixed-yaw behavior.
- The node's callback gates remain before sample append.
- Accepted/rejected windows clear before returning their decision, preventing
  callback races from reusing samples.
- Prediction is serialized with observations and timer publication, and the
  global stamp is advanced only by a strictly newer callback.
- Timer publication uses the current filter covariance, not the accepted batch
  covariance, so hidden growth is visible.
- One-shot never reaches repeated prediction or sample acceptance after revision
  1.
- Existing public topics and both TF edges remain; innovation is the only new
  topic. Exactly one TF broadcaster remains.
- Task 6 process rates and NIS threshold are now explicitly marked provisional,
  not field-calibrated, and missing dataset/version/date provenance. No new
  prediction or gate parameter duplicates them.
- No Task 8 behavior or files were changed.

## Concerns

1. Dynamic ROS callback ordering, TF timing, intermittent visibility, and
   one-shot compatibility still require the external M2-B run because rostest and
   ROS processes were prohibited here.
2. The six process rates, degradation threshold, and NIS threshold remain
   provisional. Field no-update calibration plus frozen dataset/version/date and
   held-out NIS coverage are required before interpreting covariance or gate
   performance experimentally.
3. The M2-B visibility schedule is not exposed as a launch argument by the
   pre-existing experiment launch. External trials must make and record the
   controlled YAML override or a later scoped task must add explicit launch
   wiring.
4. The bounded build's unrelated VTK/PCL/Eigen/Gazebo warnings remain in the
   workspace; they did not prevent the requested packages from building.

# Review Fix Round1

## Dispositions

All findings from `task-7-review.md` were reproduced or otherwise verified before
editing and are resolved in the bounded implementation. Task 8 remains gated on
the external M2-B ROS acceptance run described above.

| Finding | Disposition |
|---|---|
| C1 | Fixed. The research launch now derives the registration and monitor contracts from the actual perturbation producers: UAV parent/child `air_ground_experiment/uav_odom`, UGV parent `air_ground_experiment/ugv_odom`, UGV child `ugv_0/base_link`, and matching estimate-derived output aliases. The monitor consumes `/air_ground_experiment/ugv/odom`. |
| I1 | Fixed. Before initialization, vehicle motion is retained as time-bounded segments. A first batch within the coalesce bracket initializes at its causal batch stamp and then directly calls Task 6 `predict()` for the elapsed interval and proportional post-batch UAV/UGV travel. A materially stale first batch is passed directly to Task 6 `update(..., current_stamp=global_prediction_stamp)` and returns `stale_batch` at revision 0. The coordinator again has exactly one Task 6 update call site. |
| I2 | Fixed. Pruning records whether any sample expired. An expired non-robust window now produces exactly one `insufficient_inliers` decision and clears the consumed samples, including production-like jitter where the sample cap is not reached. |
| I3 | Fixed. Serialized `RegistrationCoordinator.tick(now)` evaluates a due periodic window without requiring a new frame. The existing 20 Hz node timer calls it; one complete window yields at most one decision and materially stale data still reaches Task 6. |
| I4 | Fixed. `finite_odometry()` and `OdometryAcceptance` perform frame, finite-value, stamp, and per-vehicle monotonic validation before either the node deque or `OdomBuffer` is mutated. UAV and UGV adapter tests cover duplicate, out-of-order, nonfinite, and invalid-frame input. |
| I5 | Fixed. Each accepted event has one publication path ordered as accepted UGV TF, estimate, revision, then status. Later accepted events keep coordinator state at `UPDATING` until `complete_publication_cycle()` after publication, and timer handling does not duplicate event outputs. |
| I6 | Fixed. Every estimate carries `header.seq=revision`. The monitor joins estimates and revisions by identity under a lock and commits each matched revision once, independent of callback order or timer republication. |
| I7 | Fixed in the written rostest. Both `TransformListener` instances have strong references for the complete buffer lookup lifetime. The test was compiled but not run under the no-ROS-process constraint. |

The M1 coverage concern is addressed by producer-derived launch wiring tests,
ROS-stub odometry/publication tests, both monitor callback orders, duplicates,
mismatches, and listener-lifetime AST coverage. Task 6 mathematics remains tested
through the real Task 6 implementation rather than copied into Task 7 tests.

## Round1 RED And GREEN

| Finding | Observed RED | Observed GREEN |
|---|---|---|
| C1 | Producer labels failed registration validators and output aliases differed from consumed frames | Static producer-to-launch-to-validator/output contract passes |
| I1 | First state remained behind the global clock and a materially stale first batch could initialize | Exact process covariance includes elapsed time/travel; global-clock stale case returns `stale_batch`, revision 0 |
| I2 | Production-like 0.11 s jitter stream slid indefinitely with no decision | Expiry produces one `insufficient_inliers` decision and window size 0 |
| I3 | A ready pre-deadline periodic window had no decision path without a new frame | Timer tick decides once at the deadline; a repeated tick returns no event |
| I4 | Rejected odometry polluted interpolation buffers | All invalid input classes leave both buffers and prediction state unchanged |
| I5 | Revision was observable before accepted TF and `UPDATING` was transient publisher-only state | Revision callback observes the new TF/estimate and coordinator exposes `UPDATING` for the publication cycle |
| I6 | Revision-first delivery committed the previous estimate | Revision-first, estimate-first, mismatch, duplicate, and republication cases commit only identity-matched pairs |
| I7 | Temporary listeners were unreferenced before lookup | AST lifetime check and Python compilation require retained listener assignments |

## Round1 Frame Contract

- UAV topic: `/air_ground_experiment/uav/odom`; accepted parent and child:
  `air_ground_experiment/uav_odom`; output alias:
  `air_ground_experiment/uav_odom`.
- UGV topic: `/air_ground_experiment/ugv/odom`; accepted parent:
  `air_ground_experiment/ugv_odom`; accepted child: `ugv_0/base_link`; output
  alias: `air_ground_experiment/ugv_odom`.
- Registration TF remains estimate-derived from `air_ground_origin` to the
  configured UGV experimental odometry alias. No truth input was introduced.

## Round1 Publication Contract

- One `RegistrationFilter`, one `TransformBroadcaster`, and one Task 6 filter
  update call site remain in production registration integration.
- Frame ingestion does not change revision. Only a Task 6 accepted update
  initializes revision 1 or increments it once.
- Accepted event order is UGV TF, estimate with matching sequence, revision, and
  status. Rejected events retain the prior mean, covariance, stamp, revision, and
  accepted TF.
- One-shot remains `FROZEN` at revision 1 and suppresses subsequent samples and
  prediction.

## Round1 Bounded Verification

Fresh post-edit evidence on 2026-08-25:

| Check | Result |
|---|---|
| Full pure/stub regression including all Task 6 tests and Round1 integration tests | **100/100 passed in 5.029 s** |
| Targeted post-refactor coordinator/adapter regression | **27/27 passed in 0.036 s** |
| `py_compile` for changed production and test Python files | Exit 0, no output |
| XML/YAML parsing for coordinate launch, written rostest, research launch, and registration config | Exit 0 |
| No-truth and architecture AST audit | No truth inputs; 1 filter constructor; 1 broadcaster constructor; 1 filter-update call site |
| `timeout 120s catkin_make --pkg air_ground_coordinate_transform air_ground_bringup -j2` | Exit 0; `coordinate_transform_node` built 100% and both requested package make steps completed |

The build retained the previously documented unrelated VTK/PCL/Eigen/Gazebo
warnings. No `roslaunch`, `roscore`, `rostest`, simulator, truth read, topic wait,
or other long-running ROS process was used.

## Round1 Modified Files

- `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py`
- `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_coordinator.py`
- `src/air_ground_coordinate_transform/scripts/takeoff_registration.py`
- `src/air_ground_bringup/scripts/ugv_coordinate_monitor.py`
- `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch`
- `src/air_ground_bringup/test/test_launch_wiring.py`
- `src/air_ground_coordinate_transform/test/test_registration_coordinator.py`
- `src/air_ground_coordinate_transform/test/test_registration_node_adapter.py`
- `src/air_ground_coordinate_transform/test/test_ugv_coordinate_monitor.py`
- `src/air_ground_coordinate_transform/test/test_registration_node.py`
- `src/air_ground_coordinate_transform/CMakeLists.txt`

## Round1 Concerns

1. Dynamic ROS callback scheduling, TF transport timing, queue behavior, and the
   written one-shot/repeated rostest remain unexecuted by explicit constraint.
   External M2-B acceptance is still required before proceeding to Task 8.
2. Task 6 process rates, degradation threshold, and NIS threshold remain
   provisional and require field calibration with frozen provenance.
3. Existing unrelated workspace dependency warnings remain nonfatal.

# Review Fix Round2

## Root Causes And Dispositions

| Finding | Root cause | Disposition |
|---|---|---|
| R1-I1 | Round1 used top-level `PoseWithCovarianceStamped.header.seq` as registration identity. `rospy.msg.serialize_message()` owns and rewrites that field with the publisher's per-topic transport sequence, while the stub publisher never serialized messages. | Fixed. Registration identity now uses the explicit `uint32 revision` in the generated `RegistrationUpdate` message. The node publishes one latched atomic accepted update per accepted registration event. The monitor consumes only that atomic interface for revision/uncertainty/delta commits; legacy estimate/revision traffic cannot enter its commit path. |
| R1-I2 | The causal pre-initialization catch-up required by repeated modes had no mode guard and therefore changed the initial one-shot covariance and stamp. | Fixed. Material stale-first-batch validation still reaches Task 6 in `one_shot`, `opportunistic`, and `periodic`; post-acceptance elapsed/travel catch-up runs only when `mode != one_shot`. One-shot preserves the accepted robust batch covariance, stamp, revision 1, and `FROZEN` state. |

No Minor finding was changed in this round.

## Round2 RED And GREEN

| Cycle | Observed RED before production change | Observed GREEN |
|---|---|---|
| Message schema | `RegistrationUpdate.msg is missing`; 1/1 source contract test failed | Exact three-field schema passed 1/1 |
| Generation and consumer dependencies | Source contract failed because `message_generation` was absent from coordinate CMake | Coordinate generation/runtime and bringup build/export/runtime consumer contracts passed 1/1; bounded build generated all five language bindings |
| Accepted-event producer | Expected one accepted update but observed `0 != 1` | Accepted event publishes exactly one typed update; a later legacy estimate republication leaves the accepted-update count at one; full adapter suite passed 11/11 |
| Atomic monitor | Four tests failed with missing `accepted_update_callback` | Atomic uncertainty/delta, stale legacy estimate, duplicate/out-of-order, and revision-gap cases passed 4/4 |
| One-shot compatibility | Exact fixture observed covariance diagonal `[0.330101, 0.330101, 0.0330251]` instead of `[0.000101, 0.000101, 0.0000251]`; maximum absolute difference `0.33` | Exact covariance and stamp `1.0` passed; coordinator suite passed 19/19 |

The initial serialization test harness was corrected before evidence was
accepted: `serialize_message()` prepends a four-byte TCPROS length, so generated
message deserialization consumes the payload after that prefix. No harness error
was counted as an expected product RED.

## RegistrationUpdate Contract

`src/air_ground_coordinate_transform/msg/RegistrationUpdate.msg` is:

```text
std_msgs/Header header
uint32 revision
geometry_msgs/PoseWithCovariance pose
```

- `header.stamp` is the filter estimate stamp and `header.frame_id` is the
  estimate origin frame. Neither field encodes revision.
- `revision` is the Task 6 accepted registration revision and is independent of
  ROS publisher transport sequence.
- `pose` contains the same x/y/yaw pose and all mapped 3x3 planar covariance
  entries as the legacy estimate published for that event.
- `/air_ground/registration/accepted_update` is latched with queue size one and
  is published once only from the accepted decision path.
- Accepted order is UGV TF, legacy estimate, atomic accepted update, legacy
  revision, then status. The relative legacy estimate-before-revision order and
  both legacy topics remain unchanged.
- Periodic snapshot publication continues to republish the legacy estimate,
  revision, TF, and status but never manufactures an accepted update.
- The monitor subscribes to the atomic update topic and performs the revision
  order check, uncertainty calculation, registration delta, log, and commit under
  one lock. Duplicate and older revisions are ignored; a forward revision gap is
  accepted.

## Real Serialization Evidence

After the bounded message-generation build, the sourced generated Python class
was exercised directly with `rospy.msg.serialize_message()` and no ROS master:

```text
serialized header.seq 7->91; revision=42; x=1.25; covariance[0]=0.04
```

The automated real-message test also compares stamp, frame, x/y, quaternion, and
all 36 covariance values after deserialization. Thus it demonstrates both the
reviewed top-level sequence rewrite and preservation of the explicit application
identity and estimate payload.

## One-Shot Compatibility

- A batch at stamp `1.0` bracketed by pre-initialization UAV and UGV odometry
  through `1.1` remains at stamp `1.0` in one-shot.
- Its exact covariance diagonal remains
  `[0.000101, 0.000101, 0.0000251]`, with unchanged mean, revision 1, and
  `FROZEN` status.
- The written one-shot rostest again checks the independent legacy literal 3x3
  covariance matrix from Task 2 with `1e-10` tolerance rather than only finite,
  symmetric, and PSD properties.
- The corresponding repeated-mode fixture still catches up to stamp `1.1` with
  exact covariance diagonal `[0.330101, 0.330101, 0.0330251]`.
- A first batch materially behind the global prediction clock still returns Task
  6 `stale_batch`, uninitialized revision 0, in all three modes.

## Package Contract

- `air_ground_coordinate_transform` declares `message_generation`, generates
  `RegistrationUpdate.msg` with `geometry_msgs` and `std_msgs`, exports
  `message_runtime`, and declares its runtime dependency.
- `air_ground_bringup` declares `air_ground_coordinate_transform` as a catkin
  dependency and package `<depend>`, providing build, build-export, and runtime
  topology for the monitor import.
- The bounded catkin ordering placed `air_ground_coordinate_transform` before
  `air_ground_bringup` and generated C++, Python, Node.js, Lisp, and EusLisp
  message artifacts.

## Round2 Bounded Verification

Fresh final evidence on 2026-08-25:

| Check | Result |
|---|---|
| Sourced full pure/stub/real-message suite | **103/103 passed in 4.788 s** |
| Real generated-message serialization test | Passed; transport sequence `7 -> 91`, explicit revision `42` and complete pose/covariance preserved |
| `py_compile` for all Round2 Python production and test files | Exit 0, no output |
| XML/YAML parsing for both package manifests, coordinate launch, written rostest, research launch, and registration config | Parsed 5 XML files and YAML successfully |
| Production identity/publication audit | No `header.seq` application use; legacy topics retained; no monitor cross-topic join |
| Existing architecture audit | One `RegistrationFilter`, one `TransformBroadcaster`, and one Task 6 filter-update call site |
| `timeout 120s catkin_make --pkg air_ground_coordinate_transform air_ground_bringup -j2` | Exit 0; one message generated, `coordinate_transform_node` built, and both requested package make steps completed |

No `roslaunch`, `roscore`, `rostest`, Gazebo, PX4, RViz, rosbag, topic wait,
truth read, Git command, subagent, reviewer, or other long-running ROS process was
used.

## Round2 Modified Files

- `src/air_ground_coordinate_transform/msg/RegistrationUpdate.msg` (new)
- `src/air_ground_coordinate_transform/CMakeLists.txt`
- `src/air_ground_coordinate_transform/package.xml`
- `src/air_ground_coordinate_transform/scripts/takeoff_registration.py`
- `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_coordinator.py`
- `src/air_ground_coordinate_transform/test/test_registration_node_adapter.py`
- `src/air_ground_coordinate_transform/test/test_registration_update_serialization.py` (new)
- `src/air_ground_coordinate_transform/test/test_ugv_coordinate_monitor.py`
- `src/air_ground_coordinate_transform/test/test_registration_coordinator.py`
- `src/air_ground_coordinate_transform/test/test_registration_node.py`
- `src/air_ground_bringup/CMakeLists.txt`
- `src/air_ground_bringup/package.xml`
- `src/air_ground_bringup/scripts/ugv_coordinate_monitor.py`
- `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-7-report.md`

## Round2 Self-Review

- The generated message has one explicit scalar identity and one typed pose; no
  string, JSON, array, frame, or timestamp encoding is used.
- The node keeps the existing estimate and revision publishers and publishes the
  new message only for accepted decisions. Rejections and timer snapshots cannot
  publish it.
- Production registration and monitor code contains no `header.seq` read or
  assignment. The only Round2 sequence assignment is in the real serialization
  test that proves ROS overwrites it.
- The monitor has no estimate/revision cache, pending set, or legacy callback, so
  there is no second commit path.
- The one-shot guard surrounds only post-acceptance catch-up. Stale validation,
  Task 6 update ownership, and repeated-mode process accounting are unchanged.
- Existing one-filter, one-broadcaster, one-update-call, revision, one-shot later
  ignore, no-truth, and publication-order invariants remain covered.

## Round2 Concerns

1. Dynamic ROS queueing, latch delivery, callback scheduling, TF transport, and
   the written one-shot/repeated rostest remain unexecuted by explicit constraint.
   External M2-B acceptance remains required before Task 8.
2. Existing consumers of the legacy estimate/revision topics remain compatible,
   but only the typed accepted-update topic provides an atomic accepted-event
   identity contract.
3. Task 6 process rates and thresholds remain provisional and require frozen
   field-calibration provenance.
4. Existing unrelated VTK/PCL/Eigen/Gazebo workspace warnings remain nonfatal.

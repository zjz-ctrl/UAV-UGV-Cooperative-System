# Task 7 Fresh Read-Only Integration Review

## Verdicts

- **Spec Compliance: FAIL**
- **Code Quality: FAIL**
- **Task 8 gate: DO NOT PROCEED**

The current research launch cannot initialize registration from the experimental
odometry streams. Seven additional integration defects affect stale-measurement
handling, window lifecycle, periodic updates, odometry ordering, publication
consistency, monitor accounting, and the written rostest.

## Critical Findings

### C1. The M2-B research launch rejects both experimental odometry streams and labels the registration TF as raw odometry

- **Files:**
  `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch:64-75`,
  `src/air_ground_experiments/launch/frame_perturbation.launch:16-45`,
  `src/air_ground_experiments/src/air_ground_experiments/frame_perturbation.py:225-239`,
  `src/air_ground_coordinate_transform/config/registration.yaml:5-14`,
  `src/air_ground_coordinate_transform/scripts/takeoff_registration.py:140-205`
- **Failure scenario:** The report's proposed opportunistic M2-B command routes
  `/air_ground_experiment/uav/odom` and `/air_ground_experiment/ugv/odom` into
  registration. The producers label their parents
  `air_ground_experiment/uav_odom` and `air_ground_experiment/ugv_odom`; the UAV
  producer also labels its child `air_ground_experiment/uav_odom` because it uses
  parent-frame twist. The registration node still requires `map -> base_link`
  and `ugv_0/odom -> ugv_0/base_link`, so both callbacks return before buffering,
  origin capture, prediction, or registration. Status remains
  `CAPTURING_ORIGIN` and revision remains 0. A bounded direct validator check
  returned `False` for both produced frame pairs.
- **Additional impact:** If only the validators were relaxed, the launch would
  still publish the estimate-derived TF as `air_ground_origin -> ugv_0/odom`
  and the UAV origin edge as `air_ground_origin -> iris_0/odom`, even though the
  estimator consumed experimental-frame poses. That mislabels the research
  state and violates the required `air_ground_origin -> UGV experimental odom`
  edge.
- **Root cause:** The launch overrides stream topics but does not override the
  corresponding input-frame contracts or output TF aliases.
- **Minimal fix:** In `air_ground_inspection_experiment.launch`, explicitly set
  the two produced parent/child frame pairs and set `uav_odom_frame` and
  `ugv_odom_frame` to the experimental odometry frames. Route the coordinate
  monitor to the same experimental UGV odometry contract. Add a static launch
  test that compares producer destination/child labels with registration input
  validators and output aliases.

## Important Findings

### I1. Initialization can leave filter time behind global prediction time and coalesce materially stale observations

- **File:**
  `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_coordinator.py:74-76,97-100,117-129,197-223`
- **Failure scenario:** Odometry necessarily arrives before a visual sample can
  be formed, so `_prediction_stamp` can already be newer than the first batch by
  up to `max_odom_bracket`. On initialization, `_decision()` only copies the
  filter stamp into `_prediction_stamp` when the latter is `None`; it otherwise
  preserves the newer global stamp. Subsequent `dt` is measured from that newer
  global stamp but added to the older filter stamp, preserving the offset. A
  bounded reproduction initialized a batch at 1.00 after odometry at 1.08,
  advanced odometry to 1.18, and then accepted/coalesced a batch stamped 1.03.
  Its true lag from global prediction time was 0.15 s, greater than the configured
  0.08 s bracket, but its apparent lag from filter state time was only 0.07 s.
- **Root cause:** The coordinator owns two clocks but does not reconcile them
  when an uninitialized filter becomes initialized. It also omits process-time
  accounting for the already observed bracket interval.
- **Minimal fix:** On the first accepted update, reconcile the filter state to
  the current global prediction stamp before exposing the decision, accounting
  for the elapsed interval without changing mean or revision. Add a test where
  pre-initialization odometry is newer than the batch and assert that age greater
  than `max_batch_coalesce_age` relative to the global prediction clock reaches
  Task 6 as `stale_batch`.

### I2. A non-robust aged window can slide forever without rejection or clearing

- **File:**
  `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_coordinator.py:145-185`
- **Failure scenario:** Old samples are removed before `aged` is computed. The
  retained first/last span is therefore always at most the configured age and is
  usually strictly less when sample timing has any jitter. With the production
  settings (`window_seconds=3`, `sample_period=0.1`, `max_samples=60`), the cap is
  also normally unreachable because only about 30 fresh samples fit. A bounded
  stream at 0.11 s spacing with mutually inconsistent samples produced zero
  decisions after 100 frames, retained 28 samples, stayed `TRACKING`, and never
  emitted `insufficient_inliers`.
- **Root cause:** Window expiry is inferred from the post-prune retained span
  instead of recording that samples expired. The cap-based test masks this by
  configuring a cap of four.
- **Minimal fix:** Record whether pruning expired any candidate (or track the
  window's original horizon) and make that condition an actual robust-window
  decision boundary. On insufficient inliers, publish one rejection and clear
  the consumed window. Add jittered timing coverage with production-like
  age/cap/sample-period ratios.

### I3. Periodic mode has no due-time decision path and can lose a complete fresh window

- **Files:**
  `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_coordinator.py:157-166`,
  `src/air_ground_coordinate_transform/scripts/takeoff_registration.py:360-405`
- **Failure scenario:** A complete robust window formed just before the periodic
  deadline is retained but not estimated because it is not due. If visibility
  ends before another frame arrives, the 20 Hz timer only republishes state and
  never attempts the ready window. At the next visibility interval, the first
  frame prunes the old complete window and starts again. A bounded reproduction
  retained three ready samples at revision 1 before the deadline, performed no
  event at the deadline, then pruned them on the next frame and remained at
  revision 1.
- **Root cause:** Periodic eligibility is checked only inside `add_sample()`;
  elapsed time itself cannot trigger a decision.
- **Minimal fix:** Add a serialized coordinator due-time/tick operation and call
  it from the existing timer under the same lock. It should decide at most one
  currently fresh window, clear every actual decision, and still pass materially
  stale batches to Task 6 for `stale_batch` rejection. Add a no-frame-at-deadline
  periodic test.

### I4. Rejected out-of-order or nonfinite odometry is already inserted into interpolation buffers

- **Files:**
  `src/air_ground_coordinate_transform/scripts/takeoff_registration.py:155-161,200-205`,
  `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_coordinator.py:197-222`,
  `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/odom_buffer.py:63-77,94-126`
- **Failure scenario:** The node appends a message to its nearest-message deque
  and `OdomBuffer` before `observe_odometry()` rejects duplicate, out-of-order,
  or nonfinite data. `OdomBuffer.interpolate_full()` uses `bisect` and therefore
  assumes chronological storage. A bounded sequence at stamps 1.00, 1.10, then
  out-of-order 1.05 produced x=7.5 at 1.075; chronological interpolation of the
  same poses is x=55.0. The wrong interpolated pose can then enter a visual
  registration sample even though prediction correctly ignored that odometry.
- **Root cause:** Monotonic/finite acceptance is split after buffer mutation,
  and the buffer itself neither sorts nor rejects invalid ordering.
- **Minimal fix:** Validate finite pose/stamp and per-vehicle monotonicity before
  mutating either node buffer, ideally through one coordinator acceptance result
  used by both prediction and buffering. Add node-adapter tests for duplicate,
  out-of-order, nonfinite, and invalid-frame odometry.

### I5. An accepted revision is visible before its TF, and `UPDATING` is not coordinator state

- **File:**
  `src/air_ground_coordinate_transform/scripts/takeoff_registration.py:121-128,290-301,360-384`
- **Failure scenario:** `publish_decision()` publishes the accepted estimate and
  revision but does not broadcast the accepted UGV transform. That transform is
  sent only by the next 20 Hz timer callback, leaving up to a 50 ms interval in
  which a Task 8 consumer can observe revision N and estimate N while TF still
  represents revision N-1. The lock prevents an in-process state race but does
  not make this split publication snapshot consistent. For later updates,
  `UPDATING` is manually published immediately before the final status through a
  publisher with `queue_size=1`; the coordinator has already stored
  `TRACKING`/`DEGRADED`, so no timer snapshot can reproduce `UPDATING`, and the
  queued transient can be dropped.
- **Root cause:** Event publication is split between `publish_decision()` and the
  periodic publisher, while status transition state is bypassed.
- **Minimal fix:** Serialize the accepted TF and all accepted decision outputs in
  one event-publication path, ordering TF/estimate before exposing the revision.
  Represent `UPDATING` as an observable coordinator transition for at least one
  publication cycle (or provide an equivalent reliable event contract). Test
  output order and TF value at the revision callback boundary.

### I6. The coordinate monitor can associate a new revision with the previous estimate

- **Files:**
  `src/air_ground_bringup/scripts/ugv_coordinate_monitor.py:56-93`,
  `src/air_ground_coordinate_transform/scripts/takeoff_registration.py:299-301,378-382`
- **Failure scenario:** Revision and estimate arrive on independent ROS topics
  and callbacks with no lock or identity join. `publish_decision()` sends
  estimate then revision, while the timer sends revision then estimate; ROS does
  not guarantee cross-topic callback order in either case. If revision 2 arrives
  while `registration_estimate` still contains revision 1, the monitor logs
  revision 2 with revision 1 sigmas/delta and commits
  `registration_revision=2`. The later estimate cannot correct the log because
  subsequent revision 2 messages are ignored.
- **Root cause:** The monitor treats two asynchronously delivered cached values
  as an atomic pair, and the estimate message contains no revision identity.
- **Minimal fix:** Publish an identity that allows the estimate to be matched to
  the revision (for example a consistently populated header sequence under the
  existing message constraint), then only commit/log matched pairs under a
  monitor lock. Add revision-before-estimate, estimate-before-revision, duplicate,
  and timer-republication tests.

### I7. The written rostest destroys both TF listeners before lookup

- **File:**
  `src/air_ground_coordinate_transform/test/test_registration_node.py:481-489,546-552`
- **Failure scenario:** Both tests construct `tf2_ros.TransformListener(tf_buffer)`
  without retaining the returned object. In ROS Noetic Python, the listener's
  `__del__` immediately unregisters `/tf` and `/tf_static`. Under CPython reference
  counting, the temporary listener is destroyed at the end of the expression,
  leaving each new buffer empty; the following lookup times out. Thus the
  deliberately unrun rostest is not currently a runnable validation of TF
  preservation or accepted updates.
- **Root cause:** Listener lifetime is shorter than buffer use, and static
  compilation/XML checks cannot detect it.
- **Minimal fix:** Retain each listener for at least the complete lookup/assertion
  lifetime, preferably as a class fixture, and run the rostest in the permitted
  external ROS environment before accepting M2-B.

## Minor Findings

### M1. Pure/stub wiring tests are too helper-focused to detect the production integration failures

- **Files:**
  `src/air_ground_bringup/test/test_launch_wiring.py:96-156`,
  `src/air_ground_coordinate_transform/test/test_registration_coordinator.py:87-295`,
  `src/air_ground_coordinate_transform/test/test_ugv_coordinate_monitor.py:78-104`
- **Failure scenario:** All bounded tests pass while the research launch rejects
  every odometry message, the monitor can join mismatched revisions/estimates,
  and accepted revision publication can precede TF. The launch test checks the
  coordinate launch's default mode but never parses the research producer/
  consumer frame contract. The monitor stub invokes callbacks only in the happy
  estimate-then-revision order. Coordinator tests bypass the ROS publication
  adapter entirely.
- **Root cause:** Tests validate isolated helpers and selected XML values rather
  than the actual producer -> validator -> estimator -> publisher contract.
- **Minimal fix:** Add bounded static/stub integration tests for experimental
  frame labels, both monitor callback orders, and node-adapter publication
  sequencing. Keep Task 6 mathematics covered only by its existing direct tests.

## Confirmed Compliant Areas

- `RegistrationFilter.update()` has one Task 7 production call site, and Task 6
  remains the sole owner of revision increment, NIS/gate, gain, and covariance
  update mathematics.
- A frame append alone does not increment revision; accepted updates initialize
  revision 1 and later accepted updates add exactly one in the coordinator tests.
- The coordinator's `RLock` covers node buffer/sample decisions, prediction,
  filter updates, and timer snapshots. No duplicate same-window update path was
  found.
- Per-vehicle monotonic distance and one global `dt` accounting inside the
  coordinator are correct for odometry that reaches it in valid order.
- Rejections preserve Task 6 filter mean, covariance, stamp, and revision at the
  filter decision boundary; finite NIS is forwarded to the innovation topic.
- ROS covariance mapping includes x/y/yaw slots and all 3x3 cross-covariances.
- `one_shot` suppresses later sample updates and prediction after initialization,
  preserves `FROZEN`, and the current final-demo include resolves to the
  `one_shot` default.
- No production registration, coordinator, or monitor truth input was found.
  Process rates and the NIS threshold are marked provisional in YAML.

## Verification Evidence

- Read completely: Task 7 brief, implementation report, review package, all
  eleven scope files, Task 6 `RegistrationFilter`, odometry buffer, experiment
  odometry/observation producers, relevant launch chain, plan, and design spec.
- Bounded pure/stub suite: **58/58 passed** in 4.734 s.
- XML/YAML parsing for coordinate launch, rostest launch, final demo, research
  launch, frame perturbation launch, and registration YAML: **passed**.
- Bounded reproductions confirmed the C1 frame-validator result, I1 stale
  coalescing, I2 non-robust sliding-window starvation, I3 periodic due-time loss,
  and I4 unsorted interpolation corruption.
- No ROS master, launch, rostest, Gazebo, PX4, RViz, rosbag, topic wait, truth
  read, or long-running process was used.

## Residual Risk

Dynamic ROS callback scheduling, publisher queue behavior, TF timing, and
one-shot compatibility remain unverified because the review constraints prohibit
ROS execution. These are residual risks, not substitutes for the static and
bounded failures above. M2-B external acceptance remains mandatory after the
Critical and Important findings are fixed.

# Re-review Round1

## Current Verdicts

- **Spec Compliance: FAIL**
- **Code Quality: FAIL**
- **Coordinator verification gate: DO NOT PROCEED**
- **Remaining findings: 0 Critical, 2 Important, 0 Minor**

Round1 resolves C1, I2-I5, and I7. The repeated-mode portion of I1 is fixed with
exact time and both-vehicle travel accounting, but the same implementation
introduces a one-shot compatibility regression. I6 is not fixed under real
`rospy` serialization because `Header.seq` is transport-owned rather than an
application revision field.

## Original Finding Revalidation

| Original finding | Round1 re-review disposition |
|---|---|
| C1 | **Resolved.** Actual Task 4 `populate_odometry()` output labels, research launch validators, output TF aliases, and monitor odometry topic agree for both vehicles. |
| I1 | **Repeated-mode failure resolved; new one-shot regression remains.** Exact pre-initialization elapsed time and proportional UAV/UGV segment travel reach Task 6 prediction, while a material first-batch delay reaches Task 6 stale rejection. The catch-up prediction is incorrectly also applied to one-shot. |
| I2 | **Resolved.** Production-like 0.11 s jitter reaches one `insufficient_inliers` decision, clears, and starts a non-overlapping new window. |
| I3 | **Resolved.** A no-frame deadline tick decides once; a repeated tick is empty; a materially stale retained window returns `stale_batch`. |
| I4 | **Resolved.** Invalid-frame, duplicate, out-of-order, and nonfinite odometry return before both nearest-message deque and `OdomBuffer` mutation. |
| I5 | **Resolved in the bounded adapter.** Accepted UGV TF and estimate precede revision. Opportunistic `UPDATING` is published by the accepted callback and one timer cycle, then reaches its final status on the following timer publication. The periodic event similarly leaves an observable latched `UPDATING` until the next timer. |
| I6 | **Not resolved.** `rospy` replaces the configured estimate `header.seq` with the estimate topic's publisher sequence at serialization. |
| I7 | **Resolved statically.** Both listeners have strong local references spanning their buffer lookups. |
| M1 | **Partially resolved.** New adapter/static tests catch C1 and I1-I5/I7, but publisher stubs do not model `rospy` header sequence rewriting and therefore report I6 green incorrectly. |

## Important Findings

### R1-I1. `estimate.header.seq` cannot identify registration revision under `rospy`

- **Files:**
  `src/air_ground_coordinate_transform/scripts/takeoff_registration.py:369-390,408-458`,
  `src/air_ground_bringup/scripts/ugv_coordinate_monitor.py:59-120`,
  `src/air_ground_coordinate_transform/test/test_registration_node_adapter.py:34-47,49-58,512-526`,
  `src/air_ground_coordinate_transform/test/test_ugv_coordinate_monitor.py:58-77,116-170`
- **Failure scenario:** `publish_estimate()` assigns `header.seq=revision`, but
  ROS Noetic `rospy.msg.serialize_message()` overwrites the top-level header
  sequence with the publisher's per-topic transport sequence. The estimate is
  republished every 50 ms, so transport sequence and registration revision
  immediately diverge. A bounded real-message serialization reproduction did
  the following without a ROS master: revision 1 committed from estimate
  publisher sequence 1; a timer republication of the same revision-1 estimate
  arrived with sequence 2 and was cached as registration estimate 2; the true
  revision-2 estimate arrived with publisher sequence 3; revision 2 then matched
  and committed the stale revision-1 estimate cached under key 2. The monitor
  logged revision 2 with x=1.0 instead of the true x=2.0.
- **Root cause:** The implementation uses a ROS-managed transport field as an
  application identity. The stub `Publisher.publish()` stores the object directly
  and never performs ROS serialization, so all new tests preserve the manually
  assigned sequence and cannot detect the production behavior.
- **Minimal fix:** Use an explicit accepted-update identity contract that is not
  rewritten by ROS and can atomically associate revision with the accepted
  estimate (for example a dedicated paired accepted-update interface), while
  preserving the existing estimate/revision topics. Add a bounded test using
  real `PoseWithCovarianceStamped` plus `rospy.msg.serialize_message()` and cover
  timer republication before revision 2. Do not use top-level `Header.seq` for
  application identity in `rospy`.

### R1-I2. I1 catch-up prediction changes legacy one-shot covariance and estimate stamp

- **Files:**
  `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_coordinator.py:200-227`,
  `src/air_ground_coordinate_transform/test/test_registration_node.py:453-480`,
  `src/air_ground_coordinate_transform/test/test_registration_coordinator.py:123-141,318-339`
- **Failure scenario:** After every first accepted update, `_attempt_window()`
  calls Task 6 `predict()` for pre-initialization time and travel without checking
  mode. Thus one-shot no longer publishes the first robust batch's covariance and
  stamp. A bounded one-sample reproduction with a batch at 1.0 and bracketed
  odometry through 1.1 changed covariance from
  `[0.000101, 0.000101, 0.0000251]` to
  `[0.330101, 0.330101, 0.0330251]` and changed stamp from 1.0 to 1.1, while
  mean and revision remained unchanged. The prior written one-shot rostest's
  exact covariance fixture was replaced by finite/symmetric/PSD-only assertions,
  so it no longer guards the required legacy covariance behavior.
- **Root cause:** The causal catch-up needed by repeated prediction was applied
  to all modes. The one-shot regression test checks only that later callbacks do
  not change the already altered state, not that the initial public state remains
  compatible.
- **Minimal fix:** Keep stale-first-batch validation for all modes, but apply
  post-initialization process catch-up only in repeated modes. In one-shot,
  preserve the accepted robust batch covariance/stamp and restore an exact
  compatibility assertion for the established fixture.

## Resolved Reproduction Evidence

- **C1:** Called the actual Task 4 `populate_odometry()` for parent- and
  body-twist conventions. Produced pairs were
  `air_ground_experiment/uav_odom -> air_ground_experiment/uav_odom` and
  `air_ground_experiment/ugv_odom -> ugv_0/base_link`; both passed the research
  launch validators. Output aliases and monitor topic matched the same contract.
- **I1 repeated behavior:** Independent exact arithmetic produced covariance
  diagonal `[0.330101, 0.330101, 0.0330251]`, stamp 1.1, unchanged mean, and
  revision 1 from a batch at 1.0 plus 0.1 s, 1.0 m UAV, and 0.4 m UGV post-batch
  motion. A batch lagging the global prediction clock by 0.15 s with a 0.08 s
  bracket returned `stale_batch`, revision 0, uninitialized.
- **I2:** An inconsistent 0.11 s stream rejected once at the first expiry,
  cleared to zero, and five later frames formed a new size-five window without
  reusing consumed samples.
- **I3:** A ready periodic window was accepted exactly once by `tick()` without
  a new frame. A separate material-delay case returned Task 6 `stale_batch` and
  cleared.
- **I4:** For both adapters, duplicate, out-of-order, nonfinite, and wrong-frame
  messages left only the two valid messages in nearest deques; interpolation at
  1.05 remained x=5.0 and coordinator odometry retained the valid 1.1 sample.
- **I5:** Actual adapter methods produced accepted order
  `UGV TF -> estimate -> revision`; estimate sequence assignment was visible in
  the stub only. Status publications were `UPDATING, UPDATING, TRACKING` across
  accepted publication and two timer calls.
- **I7:** AST and direct scope inspection found two assigned listener references,
  each local to and retained throughout the test method containing all related
  lookups.

## Regression And Static Evidence

- Full bounded pure/stub suite: **110/110 passed** in 4.843 s. This includes all
  Task 6 estimator/Monte Carlo tests, coordinator, node adapter, monitor, launch,
  one-shot geometry, SE(2), and odometry buffer tests.
- `py_compile` for all Round1 changed Python production/test files: exit 0.
- XML/YAML parsing for research/coordinate/rostest/perturbation launches and
  registration config: passed.
- Production architecture remains one `RegistrationFilter` constructor, one
  `TransformBroadcaster` constructor, and one coordinator call site for Task 6
  `RegistrationFilter.update()`.
- Frame append, prediction, and rejection do not increment revision. One-shot
  still remains `FROZEN` at revision 1 after initialization and ignores later
  samples/prediction; the remaining compatibility defect is its initial
  covariance/stamp.
- Current final-demo launch still resolves through the coordinate launch's
  `registration_mode=one_shot` default.
- No production truth input was found in registration, coordinator, or monitor.

## Residual Risk

Dynamic ROS callback scheduling, TF transport timing, publisher queue behavior,
and the written rostest remain unexecuted because ROS processes are prohibited.
Those are residual risks only. The two Important findings above are bounded,
reproducible failures and must not be downgraded to residual risk.

# Re-review Round2

## Current Verdicts

- **Spec Compliance: PASS**
- **Code Quality: FAIL**
- **Coordinator verification gate: PROCEED**
- **Remaining findings: 0 Critical, 0 Important, 1 Minor**

Round2 resolves both Round1 Important findings. The generated typed update keeps
application revision independent of ROS transport sequence, and the monitor has
one atomic commit path. The one-shot guard now preserves the established first
accepted batch covariance and stamp while repeated modes retain causal
pre-initialization catch-up. The remaining Minor finding affects isolation of
the written multi-node rostest, not the production single-registration-node
launch or the Round2 blocker fixes, so it does not block coordinator
verification.

## Round1 Finding Revalidation

| Round1 finding | Round2 disposition |
|---|---|
| R1-I1 | **Resolved.** `RegistrationUpdate` carries an explicit `uint32 revision` beside its typed `PoseWithCovariance`. Real `rospy` serialization changed `header.seq` from 999 while preserving revision 7, stamp, frame, pose, and all 36 covariance entries. Production registration and monitor code no longer reads or assigns `header.seq`. The monitor subscribes only to the atomic accepted-update interface for registration commits. |
| R1-I2 | **Resolved.** The post-acceptance catch-up at `registration_coordinator.py:220-225` is restricted to non-one-shot modes. An independent exact fixture produced one-shot `FROZEN`, revision 1, stamp 1.0, and covariance diagonal `[0.000101, 0.000101, 0.0000251]`; the same pre-initialization motion in opportunistic mode produced stamp 1.1 and `[0.330101, 0.330101, 0.0330251]`. Material stale-first-batch rejection remains common to all three modes. |

## Minor Finding

### R2-M1. Auxiliary registration nodes in the written rostest share the new latched accepted-update topic

- **Files:**
  `src/air_ground_coordinate_transform/scripts/takeoff_registration.py:148-160`,
  `src/air_ground_coordinate_transform/test/registration_node.test:2-38,40-158`
- **Failure scenario:** The rostest starts four `takeoff_registration.py` nodes.
  It remaps the auxiliary nodes' existing estimate, revision, status, and other
  output topics, but none remaps
  `/air_ground/registration/accepted_update`. All four publishers therefore
  share one absolute latched topic while producing revisions and poses in
  different test frame contracts. The current test has no accepted-update
  subscriber, so this does not fail its existing assertions, but a late
  subscriber or future atomic-interface assertion can receive a
  publisher-dependent latch and cannot attribute the update to the intended
  fixture.
- **Root cause:** The Round2 output topic was added to the node but not to the
  existing auxiliary-node remap sets in the multi-node test launch.
- **Minimal fix:** Remap the accepted-update output for each auxiliary
  validation/repeated node, matching the isolation already applied to its
  estimate and revision outputs. Add a static launch assertion covering all
  registration output topics.

## Atomic Contract Evidence

- The message source is exactly `std_msgs/Header header`, `uint32 revision`, and
  `geometry_msgs/PoseWithCovariance pose`; no string, JSON, array identity, frame,
  or timestamp encoding is used.
- Coordinate CMake/package metadata generates the message with
  `message_generation` and exports/runs with `message_runtime`.
  `air_ground_bringup` declares `air_ground_coordinate_transform` in both CMake
  and package dependency topology. A bounded package build generated all five
  message language bindings and completed both requested package targets.
- Static production inspection found one accepted-update publication call site,
  only inside the accepted decision branch. Rejection and timer snapshot paths
  do not call it. A direct adapter reproduction published one accepted event,
  then 25 legacy estimates and five rejected decisions; the atomic update count
  remained exactly one.
- The accepted path order is UGV TF, legacy estimate, atomic update, legacy
  revision, and status. All existing legacy publishers remain present. The
  timer keeps publishing legacy snapshots but cannot manufacture an accepted
  event.
- The monitor has no legacy estimate/revision callback, cache, pending join, or
  second registration commit path. Revision comparison and commit execute under
  one lock; duplicate/older revisions are ignored and a forward gap is accepted.
  A concurrent 102-message reproduction including revisions 1-100 plus duplicate
  and old deliveries ended at revision 100 with pose `(100, -100)` and could not
  regress state.

## One-Shot And Repeated-Mode Evidence

- The exact one-shot fixture retained the robust batch mean, covariance, stamp
  1.0, revision 1, and `FROZEN`; later sample/prediction suppression remains
  covered by the full coordinator suite.
- The same fixture in opportunistic mode accounted for 0.1 s, 1.0 m UAV travel,
  and 0.4 m UGV travel exactly once, advancing stamp to 1.1 without changing
  mean or revision.
- A mutation probe deliberately applied that repeated-mode catch-up to the
  accepted one-shot filter. It added covariance diagonal
  `[0.33, 0.33, 0.033]` and moved stamp from 1.0 to 1.1, demonstrating that the
  exact compatibility assertion detects removal of the mode guard.
- Existing bounded cases still reject a materially stale first batch through
  Task 6 in one-shot, opportunistic, and periodic mode, without initialization
  or revision increment.

## Fresh Verification Evidence

- Explicit permitted coordinate pure/stub/real-message modules: **97/97 passed
  in 4.703 s**.
- Bringup launch/mission pure modules: **16/16 passed in 0.079 s**. Combined
  bounded suite: **113/113 passed**.
- The real generated-message serialization test passed independently; a second
  mutation probe observed transport sequence 999 and preserved explicit revision
  7 plus the complete typed pose/covariance payload.
- `py_compile` for all Round2 Python production and test files exited 0.
- Five XML files and registration YAML parsed successfully. Independent source
  checks passed for exact message schema, generation/runtime dependencies,
  absence of production `header.seq`, one atomic publish site, one monitor commit
  path, and accepted publication ordering.
- `timeout 120s catkin_make --pkg air_ground_coordinate_transform
  air_ground_bringup -j2` exited 0 and generated/built the requested message and
  package targets.
- One initial over-broad `unittest discover` attempt reached the ROS-dependent
  written test after its pure cases and was killed by the 120 s timeout. No ROS
  master, launch, rostest, Gazebo, PX4, RViz, rosbag, topic wait, truth read, Git
  operation, or subagent was used. The timed-out attempt is not counted as
  evidence; the explicit 113-test rerun above is the bounded result.

## Residual Risk

Dynamic ROS queueing, latch selection/delivery, callback scheduling, and TF
transport timing remain unverified under the review prohibition. The written
one-shot/repeated rostest also remains unexecuted and still requires external
ROS validation. M2-B external acceptance and frozen field-calibration provenance
remain mandatory before Task 8; these constraints do not reopen either resolved
Round1 Important finding.

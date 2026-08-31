# Task 4 Report

## Status

STATIC/PURE IMPLEMENTATION COMPLETE. Dynamic M1-B zero-drift verification remains pending because this environment explicitly prohibits launching ROS/Gazebo/PX4 trials.

No Git repository was initialized and no commit was created.

## RED/GREEN Evidence

### Deterministic frame perturbation

- RED: `python3 test/test_frame_perturbation.py`
- Expected result: `ModuleNotFoundError: No module named 'air_ground_experiments'` before production package/module creation.
- GREEN: the focused suite initially passed 6 tests after the minimal `FramePerturbation` and odometry transform implementation.
- Final GREEN: 11 frame/adapter behavior tests pass, including same seed, different seed, repeated and out-of-order timestamps, fixed-step values, global RNG isolation, full 3-D pose/twist preservation, covariance Jacobians, inverse command conversion, seeded visibility, delay/outlier scheduling, and no synthetic observations.

### One-shot metrics and result persistence

- RED: `python3 test/test_metrics.py`
- Expected result: `ModuleNotFoundError: No module named 'air_ground_experiments.metrics'`.
- GREEN: the focused suite initially passed 6 tests after the metric implementation.
- Final GREEN: 7 metric/persistence tests pass, including wrapped yaw error, 2-D handoff error, 3-D final inspection distance, inclusive success radius, failure/timeout codes, seed retention, CSV append, and per-trial JSON output.

### ROS-free adapter behavior and serialization

- RED: `python3 test/test_adapter_serialization.py`
- Expected result: imports for `diagnostic_json`, `odometry_record`, `populate_odometry`, and `truth_json` were absent.
- GREEN: 4 tests pass against duck-typed complete odometry messages and parsed JSON diagnostics. The tests execute serialization behavior rather than searching for constants.

### Packaging, launch, and autonomy safety

- RED: `python3 test/test_package_safety.py`
- Expected result: package, scripts, launch file, and recorder class did not exist.
- GREEN: 5 AST/XML/package tests pass. They verify launch behavior wiring, installation/test registration, the recorder publisher set, Gazebo confinement, and truth subscriber confinement.
- Additional RED: the seeded visibility test failed with unsupported `visibility_probability`, and launch list tests failed because initial poses were string-valued parameters.
- Additional GREEN: seeded intermittent visibility was added and initial poses now use YAML-parsed `<rosparam subst_value="true">` values.
- One unexpected `NameError` occurred after a test insertion split two assertions from their original test. `systematic-debugging` was invoked, the test boundary was inspected, the misplaced assertions were moved back, and focused plus full regressions passed.

## Implementation

- `FramePerturbation` owns an instance-local `numpy.random.Generator`, caches a fixed-step random walk, wraps yaw, and returns copies so repeated/out-of-order queries cannot mutate or alter a timestamp result.
- `transform_odom` applies one `blockdiag(Rz, Rz)` 6-D frame Jacobian to pose and twist covariance, left-composes the yaw quaternion, rotates linear and angular twist, and preserves z and non-planar quaternion components.
- The odometry adapter validates the source frame, retains the source timestamp and child frame, publishes the two configured experiment odometry streams, and publishes seed-bearing JSON truth messages.
- The command adapter independently reconstructs the injected UAV transform from the same parameters. It does not subscribe to truth. It applies the inverse transform to position, velocity, acceleration, jerk, and yaw while preserving z, yaw rate, timestamps, gains, trajectory IDs, and trajectory flags.
- `ObservationGateSchedule` only queues detector callbacks. Configured ROS-time windows and a seeded keep/drop decision control visibility; seeded delay jitter and gross SE(2) outliers are queued deterministically. Release preserves the image timestamp and emits explicit JSON delay/outlier/seed diagnostics.
- Metrics expose a stable `TRIAL_COLUMNS` schema and reject failed/timeout rows without failure codes.
- `TrialResultWriter` appends exactly one CSV row per call and writes one JSON metadata document per trial, including failed and timeout trials.
- `ExperimentRecorder` gathers Gazebo/model truth, experiment frame truth, the frozen estimate, registration status, and terminal mission phase. It writes success, failure, and timeout records once and publishes only evaluation status.
- Catkin packaging installs all four scripts plus `config/` and `launch/`, declares ROS/Python dependencies, and registers all pure/static tests under `CATKIN_ENABLE_TESTING`.

## Truth/Autonomy Isolation Audit

- AST result: `ExperimentRecorder` contains exactly one `rospy.Publisher` call.
- Exact publisher topic: `/air_ground_experiment/evaluation/status`.
- The recorder has no publisher to odometry, command, observation, registration, mission, or controller input topics.
- `/gazebo/model_states` and `gazebo_msgs` occur only in `experiment_recorder.py` among Task 4 scripts.
- The only subscribers to `/air_ground_experiment/truth/*` are the two recorder subscriptions.
- The odometry adapters produce truth for evaluation, as required; the position command adapter regenerates the deterministic transform locally and never consumes truth.
- Observation and command autonomy paths do not import or subscribe to Gazebo data.

## Bounded Verification

- Full Task 4 pure/static suite: `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s test -p 'test_*.py'`
- Result: 27 tests run, 27 passed, 0 failures/errors.
- Python compilation: all package modules and four scripts passed `python3 -m py_compile`.
- XML parse: `frame_perturbation.launch` and `package.xml` parsed successfully.
- Bounded package build: `catkin_make --pkg air_ground_experiments -DCATKIN_ENABLE_TESTING=ON` exited successfully and generated devel-space wrappers for all four scripts.
- Tasks 1-3 coordinate-transform pure regression: 34 tests passed.
- Legacy bringup/mission pure regression: 14 tests passed.
- No prohibited ROS master, launch, simulator, bag, topic wait, or runtime trial command was executed.

## Dynamic M1-B Pending

- Pending external/manual verification: run one zero-drift M1-B trial and verify each perturbed odometry output equals raw odometry under the configured constant SE(2) frame transform.
- The trial must also validate live topic types, runtime timestamps, CXR command routing, detector-to-gate timing, and recorder outputs.
- This report does not claim dynamic M1-B completion.

## Modified Files

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
- `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-4-report.md`

## Self-Review

- Every production behavior was preceded by a focused failing test and observed failure.
- Behavioral tests exercise NumPy transformations, real queue state, duck-typed complete messages, filesystem output, AST structure, and parsed XML; they are not solely constant searches.
- Full 3-D values are retained while only the injected SE(2) frame action changes x/y/yaw-related components.
- Input dictionaries/messages are copied or populated without changing source timestamps.
- The random walk and observation schedule never call global `np.random` functions.
- Tasks 1-3 and legacy files were not edited.
- No subagent or reviewer was dispatched.

## Concerns

- Dynamic M1-B remains mandatory and pending.
- The bounded catkin configure emitted existing workspace warnings for missing imported VTK executables/libraries and undefined Eigen/system-lib export variables. These did not fail this package build.
- ROS reports Gazebo Classic `gazebo_msgs` as deprecated; it remains required here solely for evaluation-side model truth under the approved brief.
- The random walk indexes elapsed timestamps from zero. The supplied launch is intended for simulation ROS time; applying Unix-epoch wall timestamps directly would create an impractically large fixed-step cache and must be avoided in external wiring.

---

# Review Fix Round 1

Scope: exactly the 4 Critical + 6 Important findings from `task-4-review.md`. The two deferred Minor findings (M1 command-frame validation, M2 setuptools metadata) were left untouched as instructed. No subagent or reviewer was dispatched. Every disposition below was verified against the code and real producers before implementation; none required pushback.

## Dispositions

### C1 — Shared bounded perturbation epoch: ACCEPTED, FIXED

- Evidence confirmed: `at()` indexed fixed steps from absolute zero (`_extend_to` loop), so any large ROS stamp (wall time ~1.8e9 s) materialized O(stamp/step) RNG draws before the first publish; the round-0 report admitted this hazard in Concerns.
- RED: new `PerturbationEpochTest` (3 tests) failed with `TypeError: __init__() got an unexpected keyword argument 'epoch_seconds'`; adapter-level RED showed a pre-epoch odometry/command still published (`AssertionError: 1 != 0`) and the extreme-stamp callback raised while materializing.
- GREEN: `FramePerturbation(..., epoch_seconds=0.0, maximum_elapsed_seconds=None)`; `at()` computes `elapsed = stamp - epoch`, raises `ValueError` immediately for negative elapsed or elapsed beyond the maximum — bounded fail fast, no allocation proportional to the stamp. `launch/frame_perturbation.launch` gained `<arg name="epoch_seconds">` wired identically into all four consumers (`uav_odom_perturbation`, `ugv_odom_perturbation`, `position_command_adapter`, `observation_gate`); both odom/command adapters read it and drop out-of-window messages with a throttled warning instead of publishing.
- Verification: `timeout 30 python3 test/test_frame_perturbation.py` → 14 OK; `test_script_adapters.py` epoch cases OK; `test_package_safety.py::test_all_perturbation_consumers_share_one_epoch_parameter` OK. The 1.8e9 query now raises in microseconds under `timeout 30`.

### C2 — Truth registration on the estimator frame chain with stamp synchronization: ACCEPTED, FIXED

- Evidence confirmed: published estimate is `^O T_G` where `O` is re-anchored at UAV takeoff inside the perturbed stream (`takeoff_registration.py:122-136` anchor rule; `send_tf(origin_frame, ugv_odom_frame, frozen)`), while the recorder compared against `F_uav ∘ F_ugv^-1` using latest unsynchronized truth documents and even discarded the estimate stamp.
- Frame ruling (now normative for evaluation): all planar SE(2), matrices act as p' = R p + t.
  - `F_uav(t)`, `F_ugv(t)`: injected source→experiment transforms from the truth stream (p_experiment = F(t) · p_source).
  - `Delta`: true constant relation between the two SOURCE odometry frames, measured once evaluation-side from resting Gazebo world poses within `~source_relation_window_seconds` (30 s default): `Delta = inv(planar(W_T_uav)) @ planar(W_T_ugv)`.
  - `A`: takeoff anchor of the experiment UAV odometry stream computed with the estimator's exact rule (mean position over the first N samples, circular mean yaw, origin_yaw = −yaw).
  - Ground truth at the estimate stamp: `^O T_G_truth(t̂) = A @ F_uav(t̂) @ Delta @ F_ugv(t̂)^-1`, with both F histories linearly interpolated at `t̂` (yaw interpolated on the wrapped delta). This equals what the published estimate denotes; a constant injected UAV offset is absorbed by A exactly as the estimator absorbs it.
- RED: `TrialTruthEvaluatorTest` import failure, then behavior failures; recorder RED showed wrong-topic/no-sync state (`'/air_ground/mission_phase' not found [...] '/air_ground/demo_phase'`) and success path misclassifying due to the old comparison.
- GREEN: pure `TrialTruthEvaluator` in `metrics.py` (bounded histories, one-shot Delta, estimator-identical anchor, interpolation, returns None when the synchronized tuple is unavailable); `experiment_recorder.py` now keeps the stamped estimate `(t̂, x, y, yaw)`, subscribes `/air_ground_experiment/uav/odom` (evaluation-only) for anchor samples, filters truth by expected domain-separated stream seeds, records Gazebo world poses once during the rest window, computes metrics only through the evaluator, and emits stable failure codes (`INCOMPLETE_TRUTH_SYNC`, `ANOMALY_TRUTH_UNAVAILABLE`) instead of pseudo-metrics when synchronization is impossible.
- Verification: `python3 test/test_metrics.py` → 20 OK (zero-drift truth equals true source relation; interpolation mid-point hand-checked; constant-offset absorption; three unavailable-sync paths return None); `python3 test/test_recorder_evaluation.py` → 7 OK including an end-to-end COMPLETED trial whose estimate matches the chain-derived truth.

### C3 — Real mission topic and terminal/error states: ACCEPTED, FIXED

- Evidence confirmed: producer publishes `/air_ground/mission_phase` (`uav_sphere_mission.py:127`); round-0 recorder subscribed `/air_ground/demo_phase` and accepted only DONE/COMPLETE/COMPLETED, none of which the mission emits; planned success terminal is `INSPECTION_CONFIRMED` (plan §Task 11) and current failures are `ERROR_{TAKEOFF,REGISTRATION,APPROACH,TARGET,COORDINATE,CONTROLLER}`.
- RED: wiring test failed with `/air_ground/demo_phase` still present; completion test recorded TIMEOUT instead of mission results.
- GREEN: `classify_mission_phase()` maps `INSPECTION_CONFIRMED`→SUCCESS, every `ERROR_*`→FAILED with stable code `MISSION_<SUFFIX>` (future-proof for `ERROR_INSPECTION`), everything else PENDING. Recorder defaults `~mission_phase_topic` to `/air_ground/mission_phase` (launch param added explicitly), finalizes success/failure immediately, never waiting for timeout. Tasks 1-3 producer untouched.
- Verification: `MissionPhaseClassifierTest` covers the six current error phases + future inspection phase + transient phases (mutation-sensitive literals taken from the producer source); recorder tests assert immediate FAILED/`MISSION_REGISTRATION` rows and launch names the exact producer topic.

### C4 — Inspection distance is UGV-to-anomaly: ACCEPTED, FIXED

- Evidence confirmed: `metrics()` fed the UAV position as `position` and UGV as target, measuring UAV–UGV separation against `success_radius`.
- RED: success-path test asserted the hand-computed UGV→red_sphere distance and failed under the old metric.
- GREEN: recorder reads the anomaly model (default `red_sphere`, the actual spawned model name) from `/gazebo/model_states` (evaluation-only), computes `final_inspection_distance(ugv_position, sphere_position)` full 3-D, applies the inclusive radius to that value; UAV–UGV separation no longer enters classification. Missing sphere truth yields `ANOMALY_TRUTH_UNAVAILABLE`.
- Verification: `test_inspection_confirmed_records_success_with_ugv_to_anomaly_distance` passes with expected = sqrt(0.3²+0.2²+0.2²).

### I1 — Exactly-once persistence and terminal transition: ACCEPTED, FIXED

- Evidence confirmed: check-then-act `finished` flag raced across subscriber/timer threads; writer appended duplicate rows and silently overwrote JSON; `finished=True` before write permanently suppressed retries after I/O failure.
- RED: duplicate-ID test raised nothing and duplicated the row; rollback test left a partial row; concurrency test produced duplicates.
- GREEN: `TrialResultWriter.write()` runs under a `threading.Lock`, rejects an already-finalized trial id (in-memory set + existing JSON file), writes JSON to a `.partial` temp then appends CSV remembering the byte offset, atomically `os.replace`s the JSON, and on any exception truncates the CSV back and removes the temp so state stays retryable. Recorder `finish()` holds one lock around transition+persist+state-set and publishes status only after a successful write; concurrent timeout/error finalize yields exactly one row.
- Verification: `WriterExactlyOnceTest` (duplicate rejection, simulated `IOError` rollback + successful retry, 16-thread parallel writes → 16 unique rows) and `RecorderCompletionTest.test_timeout_and_error_race_finalize_exactly_once`.

### I2 — Visibility on occurrence time relative to a shared epoch with full timing trace: ACCEPTED, FIXED

- Evidence confirmed: windows were evaluated on `received_at` (breaks for nonzero ROS clocks and late-delivered captures) and diagnostics lacked receipt/actual-release times.
- RED: `GateEpochVisibilityTest` (3 tests) failed on missing `epoch_seconds`, missing `receipt_time`.
- GREEN: `ObservationGateSchedule(epoch_seconds=...)` evaluates window membership on `image_stamp − epoch` (occurrence-based; a capture inside a visible interval passes even if delivered late) and fails fast if a stamp precedes the shared trial epoch; transport delay still schedules from receipt time. `ScheduledObservation` carries `receipt_time`; diagnostics now expose `image_stamp`, `receipt_time`, `scheduled_release`, `actual_release`, `outlier_xyyaw`, stream seed, and trial seed. Gate script uses the same shared epoch parameter and domain seed.
- Verification: gate script tests drive the real callback+timer with fake clock — visible-at-capture/late-receipt observation publishes with preserved image stamp and complete trace; hidden observations produce zero publications and zero diagnostics.

### I3 — Twist convention made explicit, configurable, and honestly labeled: ACCEPTED, FIXED WITH RULING

- Evidence confirmed: twist vectors/covariance were rotated (parent semantics) while `child_frame_id` stayed body — contract-inconsistent output per `nav_msgs/Odometry.msg`.
- Producer audit ruling: UGV source `/ugv_0/odom` comes from `libgazebo_ros_planar_move.so` (`ugv_mvp/model.sdf:119-127`), which publishes body-frame twist with `robotBaseFrame ugv_0/base_link` ⇒ valid body convention. UAV source `/iris_0/mavros/local_position/odom` (mavros LocalPositionPlugin) fills ENU-transformed LOCAL_POSITION_NED velocity — parent/map-expressed values despite a base_link child label (mavros default config `local_position: frame_id: map`). Because the two producers genuinely differ and static certainty about mavros EKF yaw alignment is impossible here, the convention is explicit per node:
  - `twist_convention=parent`: rotate linear/angular twist and covariance with blockdiag(R,R) and set `child_frame_id := destination_frame` (twist truly expressed in the labeled frame).
  - `twist_convention=body`: leave twist and covariance untouched and preserve the physical child frame.
- Launch defaults encode the ruling: UAV node `parent` (matches mavros practice), UGV node `body` (matches planar_move). Downstream Tasks 1-3 gating consumes speed magnitudes/planar norms, invariant under this choice; M1-B must confirm live labels.
- RED/GREEN: `TwistConventionTest` first failed on the unknown kwarg; after GREEN both conventions pass against a hand-expanded non-diagonal 6×6 fixture with z–rotY cross terms (`CROSS_COVARIANCE_EXPECTED_PARENT` computed manually, not via production helpers), plus child-label assertions per convention and script-level plumbing tests (`OdomTwistConventionPlumbingTest`).

### I4 — Domain-separated independent drift streams: ACCEPTED, FIXED

- Evidence confirmed: launch passed the identical trial seed to both perturbation nodes ⇒ identical walks (perfectly correlated drift).
- RED: `DomainSeedTest` failed on missing `domain_seed`; launch-label test failed; node plumbing test showed effective seed == trial seed.
- GREEN: `domain_seed(trial_seed, label)` derives stable per-stream seeds via `SeedSequence([trial_seed, crc32(label)])` (cross-run/platform deterministic, no global RNG); launch passes `seed_domain` ∈ {uav, ugv, gate} to the three seeded nodes; the command adapter uses the `uav` domain so its inversion regenerates exactly the UAV odometry walk (parity test). `truth_json` now reports both `trial_seed` and the effective stream `seed`; gate diagnostics carry both; recorder metadata stores `expected_stream_seeds` and validates incoming truth against them. Trial seed remains in every CSV row.
- Verification: uav ≠ ugv ≠ gate seeds, reproducibility, parity between odom walker and command inverter.

### I5 — Canonical status validation: ACCEPTED, FIXED

- Evidence confirmed: non-canonical statuses slipped through as unclassified failures; TIMEOUT rows could carry `timeout=False`; COMPLETED could carry failure codes.
- RED: 3 failures across contradiction cases.
- GREEN: allowed statuses are exactly {COMPLETED, FAILED, TIMEOUT} (case/space normalized, anything else raises); FAILED/TIMEOUT require a nonempty failure code; COMPLETED forbids one and requires a finite inspection distance; the `timeout` column is derived from status; success ⇔ COMPLETED and finite distance ≤ inclusive radius. Stale round-0 assertions encoding the old contradictions were updated to the canonical API.
- Verification: `test_metrics.py` → 13 writer/validation tests OK; recorder paths emit only canonical statuses.

### I6 — Behavior-depth tests replacing shape-only coverage: ACCEPTED, FIXED THROUGHOUT

- Delivered in this round rather than as one patch: duck-typed real-layout message fixtures and a fake rospy (`test/ros_stubs.py`) now drive the actual script classes — odom adapter end-to-end publication, gate callback/release timing, recorder truth/anchor/model/estimate/phase flows, completion, race, and failure paths. Covariance expectations are independently hand-computed non-diagonal 6×6 matrices with cross terms (I3 fixture). Mission phases, topics, anomaly model, seeds, and diagnostics are asserted against literal producer constants (mutation-sensitive: changing any constant breaks a named test). Per-finding focused RED commands and outputs were captured above instead of relying on initial module-absence errors. The AST publisher-isolation test now resolves module-level topic constants, keeping the isolation proof while allowing honest encapsulation.

## Corrected Round-0 Statements

- "The random walk indexes elapsed timestamps from zero …" (Concerns): resolved by the shared bounded epoch; wall-time stamps now fail fast instead of hanging.
- "… rotates linear twist, angular twist, and preserves z …" and "retains the source timestamp and child frame": superseded by the I3 convention ruling — twist handling is explicit and configurable with honest child labels; z/non-planar preservation is unchanged.
- Test-count claims ("27 tests") reflect round-0 coverage that missed C2-C4/I1-I3, as the review stated; the suite is now 75 tests with callback-level behavioral depth.

## Review Fix Round 1 — Bounded Verification

- Full Task 4 suite: `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s test -p 'test_*.py'` → **75 tests, OK** (0 failures/errors/skips).
- Tasks 1-3 regressions: coordinate-transform pure suites → **34 tests OK**; bringup launch/mission suites → **14 tests OK**.
- `py_compile` on all package modules and four scripts → OK (bytecode directed to `/tmp`).
- XML parse of `frame_perturbation.launch` and `package.xml` → OK.
- Bounded build `catkin_make --pkg air_ground_experiments -DCATKIN_ENABLE_TESTING=ON` → completed successfully, devel wrappers regenerated for all four scripts.
- Stray `scripts/__pycache__` removed; no prohibited ROS/Gazebo/PX4/rostest/bag/topic-wait command executed; no Git operations.

## Residual Dynamic Risks (unchanged or newly noted)

- Dynamic M1-B zero-drift trial remains mandatory and pending; it must additionally confirm the mavros twist-convention default (`parent`) and child-frame labels on live messages, and the EKF-origin assumption behind `Delta` (measured as spawn-relative world poses at rest).
- The recorder's takeoff anchor mirrors the estimator rule over the same experiment stream but samples independently; sub-sample races between the two nodes are possible in principle and would surface as small anchor mismatch — observable via near-zero zero-drift registration error in M1-B.
- Gazebo Classic deprecation warnings persist (evaluation-only dependency).

---

# Review Fix Round 2

Scope: the scoped re-review's open findings NC1, NI1, NI2 only. Deferred NM1-NM3 untouched. No subagent/reviewer dispatched; no prohibited processes; no Git operations.

## Dispositions

### NC1 — Truth-history eviction defeats synchronized evaluation: ACCEPTED, FIXED

- Verified first with a reviewer-equivalent bounded reproduction: an evaluator seeded with a freeze bracket at t∈[10,20] then fed 60 later truth pairs (history_length=8) returned `registration_truth_at(15.0) is None` — exactly the reviewer's 30 Hz / 80 s outcome.
- RED: `TrialTruthEvaluatorTest.test_truth_history_retains_early_freeze_across_high_rate_ingest` → `TypeError: 'NoneType' object is not subscriptable` (`truth_at_freeze` was None).
- Fix decision: full per-trial retention (the review's first option). An intermediate coarse-decimation attempt (`history[::2]` halving) was implemented and rejected by its own test: single-pass decimation dropped the t=14 bracket sample and interpolation degraded to [3.0, 0] instead of [3.5, 0]; multi-pass variants have the same lossy-bracket hazard. Full retention keeps every stamp interpolable for the whole trial at bounded, modest cost (multi-minute trial at 30-50 Hz ≈ ≤10^5 small entries, a few MB). The misleading `history_length` eviction knob was removed entirely rather than left as a foot-gun.
- GREEN: `_truth_history` entries are plain append-only lists with monotonic-stamp ingestion retained; regression test now ingests 700 later pairs (> the old 600-entry deque default) after freezing at t≈15 and requires `[3.5, 0]` — passes.
- Command/output: focused RED as above; post-fix `python3 test/test_metrics.py` → **21 OK** (now 22 with NI1's addition).

### NI1 — Takeoff anchor must mirror the estimator's configurable rule: ACCEPTED, FIXED

- Verified against `takeoff_registration.py:129-136`: `origin_yaw = -mean_yaw` only when `align_origin_to_uav_heading`, else `fixed_origin_yaw`; translation always `-R(origin_yaw) @ center`. The evaluator hardcoded the aligning branch — wrong for the deployed research config (`align_origin_to_uav_heading: false`).
- RED (two layers): pure `test_fixed_yaw_anchor_matches_estimator_branch` failed (anchor yaw was -π/2 instead of fixed 0.25; hand-derived literals `-cos(0.25), -sin(0.25)`); recorder wiring test `test_recorder_mirrors_estimator_anchor_parameters` failed (`True is not false`) until parameters were mirrored. One harness-convention fix during GREEN (params are tilde-prefixed in this fake-rospy harness).
- GREEN: `TrialTruthEvaluator(..., align_origin_to_uav_heading=True, fixed_origin_yaw=0.0)` branches identically to the estimator; `experiment_recorder.py` mirrors both via `~align_origin_to_uav_heading` / `~fixed_origin_yaw`. The recorder metadata string no longer claims "exactly like the estimator" unconditionally; it names the configurable rule.
- Report wording correction: Round-1 claims of "the estimator's exact rule" are superseded — see the correction note appended below.

### NI2 — Recorder race test was vacuous: ACCEPTED, FIXED AND MUTATION-PROVEN

- Verified: harness clock (100 s) never exceeded started+timeout (120 s), so the tick thread was a no-op and the single-row assertion held even without any lock.
- Strengthened test: builds the recorder with `~timeout_seconds: 0.05`, advances the fake clock past start+timeout before racing, and releases two threads through a `threading.Barrier` — one calling `tick(None)` (live TIMEOUT path), one firing `ERROR_TARGET`. Asserts zero thread errors, exactly one CSV row, status ∈ {TIMEOUT, FAILED}.
- Mutation validation (as required): temporarily replaced `with self._finalize_lock:` by an unprotected block in `experiment_recorder.py` → the strengthened test FAILED 5/5 runs with `ValueError("trial 'm1b-fix' has already been finalized")` surfacing through the writer's duplicate guard into the errors assertion (two threads reached finalization). Restored from backup → PASS 3/3 consecutive runs. This proves the test now exercises the race it cites.

## Review Fix Round 2 — Bounded Verification

- Full Task 4 suite: `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s test -p 'test_*.py'` → **78 tests, OK** (75 + retention-expiry + fixed-yaw-anchor + recorder-mirror tests).
- Tasks 1-3 regressions: coordinate-transform pure suites → **34 OK**; bringup suites → **14 OK**.
- `py_compile` on all modules/scripts (bytecode under `/tmp`) → OK. XML parse of launch + package.xml → OK.
- Bounded build `catkin_make --pkg air_ground_experiments -DCATKIN_ENABLE_TESTING=ON` → completed successfully.
- No roslaunch/roscore/rostest/Gazebo/PX4/RViz/rosbag/topic-wait commands; no truth read; no Git operations.

## Modified Files (Round 2)

- `src/air_ground_experiments/src/air_ground_experiments/metrics.py` (full-retention histories; configurable anchor branch)
- `src/air_ground_experiments/scripts/experiment_recorder.py` (mirror anchor params; metadata wording)
- `src/air_ground_experiments/test/test_metrics.py` (+2 tests, incl. retention-expiry regression)
- `src/air_ground_experiments/test/test_recorder_evaluation.py` (+1 mirror test; effective barrier race test)
- `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-4-report.md` (this section + wording corrections)

## Wording Corrections to Earlier Sections

- Round-1 C2 text "computed with the estimator's exact rule" and "estimator-identical anchor": accurate only for the aligning configuration until this round; the evaluator now mirrors `align_origin_to_uav_heading`/`fixed_origin_yaw`, and trials must pass the registration node's actual values to the recorder.

## Self-Review (Round 2)

- Each fix was preceded by a focused failing test whose failure mode matched the finding (None-interpolation; wrong branch literals; vacuous-race exposure via mutation).
- A proposed implementation (decimation) that passed neither honesty nor its own regression was discarded in favor of the simpler correct option; the discarded variant's failure is documented above.
- Memory bound of full retention is stated with arithmetic rather than asserted.
- No Tasks 1-3 files touched; deferred Minors remain open by instruction.

## Residual Dynamic Risks (updated)

- M1-B remains pending and must additionally confirm live truth rates vs. the retention bound (any multi-hour trial would warrant revisiting persistence) and that recorder params receive the trial's real `align_origin_to_uav_heading`/`fixed_origin_yaw`.

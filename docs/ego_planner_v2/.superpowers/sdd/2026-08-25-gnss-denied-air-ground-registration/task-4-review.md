# Task 4 Fresh Read-Only Review

## Verdicts

- **Spec Compliance: FAIL**
- **Code Quality: FAIL**

The pure SE(2) pose action and its command inverse are directionally consistent,
the local NumPy generators do not consume global RNG state, repeated/out-of-order
queries return cached deterministic states, and the full `J C J^T` operations do
retain covariance cross terms under the implementation's deterministic left-action
model. However, the critical timestamp, evaluator-truth, recorder-terminal, and
inspection-metric defects below prevent valid M1 experiment results.

## Critical Findings

### C1. Absolute/nonzero-origin ROS stamps can make the first callback hang or exhaust memory

- **Location:** `src/air_ground_experiments/src/air_ground_experiments/frame_perturbation.py:275-293`; `src/air_ground_experiments/scripts/odom_perturbation_node.py:52-55`; `src/air_ground_experiments/scripts/position_command_adapter.py:50-68`
- **Failure scenario:** `at()` interprets the absolute stamp as elapsed time and materializes every fixed-step state from zero through `floor(stamp / drift_step_seconds)`. A wall-time stamp near `1.8e9` with a one-second step attempts roughly 1.8 billion RNG draws and cached arrays before either adapter publishes. A simulation or bag whose ROS clock starts at a large nonzero value has the same failure. The report acknowledges this limitation at `task-4-report.md:114`, so it is a real correctness defect rather than merely pending dynamic validation. Repeated and out-of-order calls are deterministic only after paying O(max_stamp/step) time and memory.
- **Minimal fix:** define an explicit, shared perturbation epoch and index fixed steps from `(stamp - epoch)`, with the same epoch supplied to the UAV odometry and command adapters. Do not independently use each node's first callback as the epoch because different first source stamps would make command inversion disagree with odometry. Bound retained checkpoints or use a deterministic keyed/counter-based construction so arbitrary queries cannot allocate in proportion to an epoch stamp.

### C2. Recorder compares the estimate against truth in a different frame and at unsynchronized times

- **Location:** `src/air_ground_experiments/scripts/experiment_recorder.py:71-84,103-116`; `src/air_ground_coordinate_transform/scripts/takeoff_registration.py:123-136,258-278`; `src/air_ground_experiments/src/air_ground_experiments/frame_perturbation.py:295-324`
- **Failure scenario:** the perturbation action is `p_experiment = F p_source`, so `F_uav inv(F_ugv)` is the mapping between the two experimental odometry frames. The published registration estimate is instead `^O T_G`, where `O` is re-anchored at UAV takeoff by `origin_to_uav_odom`. The recorder omits that takeoff-origin transform and compares `^O T_G` directly with `F_uav inv(F_ugv)`. Any nonzero UAV spawn or injected UAV offset therefore appears as registration error even for a perfect estimate. Under drift, the recorder also composes the latest UAV and UGV truth documents without matching their stamps to one another or to the frozen estimate.
- **Minimal fix:** record the exact evaluation-side takeoff anchor and compute `^O T_G` with the same frame chain as the estimator, then select/interpolate both injected transforms at the estimate stamp. Include all source stamps in metadata and reject a metric when a synchronized truth tuple is unavailable.

### C3. Recorder listens to the wrong mission phase and does not recognize the specified terminal state

- **Location:** `src/air_ground_experiments/scripts/experiment_recorder.py:58-60,99-102,178-180`; `src/air_ground_bringup/scripts/uav_sphere_mission.py:125-128,175-188`; `docs/superpowers/plans/2026-08-25-gnss-denied-air-ground-registration.md:908-914`
- **Failure scenario:** the Tasks 1-3 mission publishes `/air_ground/mission_phase`, but the recorder subscribes to `/air_ground/demo_phase`. Even if connected to the real producer, it only accepts `DONE`, `COMPLETE`, or `COMPLETED`; the planned successful terminal phase is `INSPECTION_CONFIRMED`, and current mission failures use `ERROR_*`. A successful or explicit mission-failure trial is therefore recorded later as `TRIAL_TIMEOUT` rather than with its real result and failure code.
- **Minimal fix:** parameterize and default the subscriber to `/air_ground/mission_phase`, recognize `INSPECTION_CONFIRMED`, and map every terminal `ERROR_*` phase to a stable nonempty failure code. Test against the actual mission producer constants/topic.

### C4. “Final inspection distance” measures UAV-to-UGV separation, not UGV-to-anomaly error

- **Location:** `src/air_ground_experiments/scripts/experiment_recorder.py:43-50,63-69,117-132`; `src/air_ground_experiments/src/air_ground_experiments/metrics.py:43-49`
- **Failure scenario:** `metrics()` passes the UAV model position as `position` and the UGV model position as `inspection_target`. This value is then compared with `success_radius`. During overwatch the UAV is intentionally above/away from the UGV, so a valid UGV inspection can be marked outside the radius; conversely, UAV/UGV proximity says nothing about UGV error to the red-sphere anomaly.
- **Minimal fix:** obtain the evaluation-only anomaly/inspection-target truth, compute the specified full 3-D distance from the final UGV position to that target, and apply the inclusive radius to that value. Keep UAV-to-UGV separation, if useful, under a separately named diagnostic rather than the success metric.

## Important Findings

### I1. “Exactly once” persistence is neither thread-safe nor enforced by the writer

- **Location:** `src/air_ground_experiments/scripts/experiment_recorder.py:94-102,125-180`; `src/air_ground_experiments/src/air_ground_experiments/metrics.py:118-139`
- **Failure scenario:** subscriber callbacks and the timer can call `finish()` concurrently. The `if self.finished` check and assignment are not protected, so timeout, registration failure, and terminal phase callbacks can each append a CSV row. Independently, calling `TrialResultWriter.write()` twice appends duplicate CSV rows while silently overwriting the same JSON path. Setting `finished=True` before row construction/write also permanently suppresses retries if serialization or I/O fails, leaving no complete CSV+JSON pair.
- **Minimal fix:** guard terminal transition and persistence with one lock/state machine; permit exactly one owner to finalize. Make the writer reject an existing trial ID/JSON file and arrange writes so an exception cannot be reported as a completed finalization. Add a concurrent timeout/status regression test and a duplicate-ID writer test.

### I2. Visibility windows are evaluated against absolute callback receipt time rather than observation time or trial-relative ROS time

- **Location:** `src/air_ground_experiments/src/air_ground_experiments/frame_perturbation.py:102-104,126-155`; `src/air_ground_experiments/scripts/observation_gate.py:83-108`; `src/air_ground_experiments/config/frame_perturbation.yaml:9`
- **Failure scenario:** a configured window such as `[0, 5]` works only when the ROS clock itself is near zero. Starting the node at ROS time 100 drops every observation. An observation captured inside a visible interval but delivered after its end is also dropped because `received_at`, not `image_stamp`, controls visibility. Delay jitter then releases by reception time and may reorder observations, but diagnostics expose only the scheduled delay, not receipt or actual publication time, so that behavior cannot be reconstructed exactly.
- **Minimal fix:** define windows explicitly in trial-relative ROS time and establish a shared trial epoch, or evaluate occurrence visibility from the source image stamp relative to that epoch. Continue scheduling transport delay from receipt time, but record source stamp, receipt time, scheduled release, actual release, and the queue ordering policy.

### I3. Rotated twist data is mislabeled with the unchanged odometry child frame

- **Location:** `src/air_ground_experiments/src/air_ground_experiments/frame_perturbation.py:198-217,295-323`; `src/air_ground_experiments/test/test_adapter_serialization.py:60-84`; `/opt/ros/noetic/share/nav_msgs/msg/Odometry.msg:1-7`
- **Failure scenario:** ROS defines odometry pose in `header.frame_id` and twist in `child_frame_id`. The implementation rotates linear/angular twist and twist covariance as if they were parent-frame vectors, changes only `header.frame_id`, and deliberately preserves `child_frame_id`. A consumer interpreting the output according to `nav_msgs/Odometry` therefore applies body-frame semantics to parent-rotated values. The existing test asserts this inconsistent combination.
- **Minimal fix:** choose and document one ROS-valid convention. For the normal `Odometry` parent-frame left action, preserve body-expressed twist/covariance unchanged with the physical child frame; if the actual producer is known to publish parent-expressed twist despite the message contract, expose and set an explicit matching output twist/child-frame convention rather than silently preserving the old child ID. Add a test based on the real message definition and producer convention.

### I4. UAV and UGV random walks use identical RNG streams, not independent frame drift

- **Location:** `src/air_ground_experiments/launch/frame_perturbation.launch:3,21,34`; `src/air_ground_experiments/src/air_ground_experiments/frame_perturbation.py:269-284`
- **Failure scenario:** both perturbation nodes receive the same seed and generate the same ordered normal samples for x, y, and yaw. Their drift is perfectly correlated (with only initial transforms differing), which invalidates an “independent-frame” disturbance and can substantially cancel or distort the intended relative drift.
- **Minimal fix:** expose separate UAV/UGV stream seeds or derive stable domain-separated seeds from the trial seed (for example, trial seed plus fixed UAV/UGV labels). Store both effective stream seeds in truth and result metadata while retaining the trial seed in every row.

### I5. Trial-row validation permits contradictory or unclassified failure states

- **Location:** `src/air_ground_experiments/src/air_ground_experiments/metrics.py:52-101`
- **Failure scenario:** `build_trial_row(status="ERROR", failure_code="")` yields a non-success row with no reason; `status="TIMEOUT", timed_out=False` yields a TIMEOUT row whose timeout column is false; `status="COMPLETED", failure_code="X"` is labeled completed but unsuccessful. Thus schema-valid-looking rows can violate the binding failure-code and timeout semantics.
- **Minimal fix:** restrict status to a documented enum, require a nonempty failure code for every non-success terminal status, reject failure codes on success statuses, and derive `timeout` from canonical status (or validate exact agreement).

### I6. Tests exercise helpers and shallow AST/XML shape, allowing the real adapter/recorder defects to remain green

- **Location:** `src/air_ground_experiments/test/test_frame_perturbation.py:125-238`; `src/air_ground_experiments/test/test_adapter_serialization.py:48-111`; `src/air_ground_experiments/test/test_package_safety.py:13-115`; `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-4-report.md:9-38,99-102`
- **Failure scenario:** no test instantiates or invokes any ROS script callback with mocked/real message classes; no test covers recorder truth math, real mission phase wiring, completion/failure/timeout races, or duplicate writes. The package test only counts publishers and checks selected XML strings. Consequently all 27 tests pass while C2-C4 and I1-I3 remain. Covariance tests also use diagonal matrices, so 6x6 ordering and transformed cross terms are not independently asserted. The reported import-error RED evidence proves modules were absent, but does not retain per-behavior RED output for most adapter/recorder behavior as required by strict TDD.
- **Minimal fix:** factor only the minimum ROS-free callback/state logic needed for direct tests or mock `rospy` and real message layouts; add mutation-sensitive tests for exact producer topics/terminal values, synchronized truth chains, full non-diagonal 6x6 covariance, source/child frames, and concurrent finalization. Retain focused RED command/output for each added behavior rather than only an initial module import failure.

## Minor Findings

### M1. Position command input frame is never validated

- **Location:** `src/air_ground_experiments/scripts/position_command_adapter.py:29-48,50-77`
- **Failure scenario:** any `PositionCommand` arriving on the configured topic is inversely transformed regardless of `header.frame_id`. A raw-frame command accidentally routed onto that topic is transformed a second time while being relabeled as `iris_0/odom`. Deep-copying does otherwise preserve the actual `PositionCommand.msg` gains, trajectory ID/flag, stamp, and unknown future fields.
- **Minimal fix:** add a required/explicit source-frame parameter, reject mismatches with a throttled warning, and test both accepted and rejected real-message layouts.

### M2. Python build/test dependency metadata is incomplete

- **Location:** `src/air_ground_experiments/package.xml:8-17`; `src/air_ground_experiments/setup.py:3-4`
- **Failure scenario:** package setup imports `setuptools`, and catkin tests import NumPy, but the manifest declares only runtime `python3-numpy` and does not declare `python3-setuptools`. A clean build/test environment need not install these build/test-time Python dependencies merely from the current declarations.
- **Minimal fix:** declare `python3-setuptools` for the setup/build stage and NumPy for the test stage (or use dependency tags that correctly cover all required stages), then verify from a clean dependency resolution environment.

## Confirmed Correct or Acceptable Areas

- `FramePerturbation` owns an instance-local `numpy.random.Generator`; the bounded test confirms global NumPy RNG isolation.
- For practical elapsed stamps, cached states make repeated and out-of-order calls deterministic and returned arrays are copies.
- Pose position and quaternion use the same left SE(2) action; command position/vector/yaw uses its mathematical inverse and does not consume truth.
- The pose covariance uses ROS ordering `(x,y,z,rotX,rotY,rotZ)` and full matrix multiplication, so existing off-diagonal cross terms are propagated under the chosen block-diagonal deterministic rotation Jacobian. The twist frame-label defect is separate.
- `PositionCommand.msg` fields not explicitly changed are retained through `deepcopy`; z and yaw rate remain unchanged under the planar instantaneous transform.
- Observation output is created only from a detector callback, retains the original message/header stamp, and the heap sequence breaks equal-release-time ties deterministically.
- The recorder has one publisher, exactly `/air_ground_experiment/evaluation/status`; Gazebo and experiment truth subscriptions remain confined to the evaluation package. No truth leak into Tasks 1-3 autonomy code was found.
- Scripts are executable and registered for installation; launch/config installation and guarded test registration are present.

## Verification and Residual Risk

- Fresh bounded command: `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s test -p 'test_*.py' -v` from `src/air_ground_experiments`.
- Result: 27 tests ran and passed. This confirms the reported pure/static green state but also demonstrates the coverage gap described in I6.
- No `roslaunch`, `roscore`, `rostest`, Gazebo, PX4, RViz, rosbag, ROS graph/topic wait, or truth-reading command was run. The prohibited dynamic M1-B check, live type negotiation, ROS timer/callback scheduling, and clean-install dependency resolution remain residual risks only, not findings by themselves.
- No Git metadata was created or used. No source, test, config, plan, brief, or implementation report was edited.

## Finding Count

- Critical: 4
- Important: 6
- Minor: 2
- Total: 12
- Highest severity: Critical

---

# Task 4 Scoped Re-Review — Fix Round 1

Scope: the 4 Critical + 6 Important round-0 findings, plus a scan of the fix
diff itself. Read-only; no Git operations; no ROS/Gazebo processes. Fresh
bounded verification run during this review:

- `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s test -p 'test_*.py' -v` → **75 tests, OK** (matches the report claim).
- Independent NumPy scratch check (own R/J construction, no production helpers): declared `CROSS_COVARIANCE_EXPECTED_PARENT` equals `J C J^T` for the fixture → the I3 hand-computed covariance expectation is honest.
- XML parse of `frame_perturbation.launch` and `package.xml` → OK.
- `py_compile` of both modules and all four scripts → OK.
- Bounded simulation of the new truth evaluator (below) → confirms a new Critical defect.

## Dispositions

### C1 (epoch / stamp blowup) — ADDRESSED

- `FramePerturbation(epoch_seconds, maximum_elapsed_seconds)` indexes steps from `stamp − epoch`; both range checks execute strictly before `_extend_to`, so negative or over-limit elapsed raises without any proportional allocation (`frame_perturbation.py:341-364`). Script-level tests confirm pre-epoch odometry/command are dropped unpublished and an extreme stamp rejects instead of materializing.
- One `<arg name="epoch_seconds">` feeds all four consumers (`launch/frame_perturbation.launch:9,24,40,51,64`); the launch-sharing test enforces it. Command adapter shares the UAV epoch, so inversion stays aligned with odometry.
- Residual (Minor, see NM3): shipped defaults remain `epoch_seconds=0.0` with no `maximum_elapsed_seconds`, so misconfigured wall-clock wiring can still hang; a conservative default maximum would convert that into fail-fast.

### C2 (truth frame chain / synchronization) — PARTIAL

Delivered and verified: pure `TrialTruthEvaluator` with estimator-shaped anchor, one-shot `Delta` from resting world poses within a disclosed window, monotonic truth ingestion, wrapped-yaw linear interpolation at the frozen estimate stamp, seed-filtered truth subscription, `None` on any unsynchronized tuple, and stable `INCOMPLETE_TRUTH_SYNC` behavior. Zero-drift, mid-point interpolation, constant-offset absorption, and unavailable-sync paths are tested.

Two defects remain in this newly added path:

- **NC1 (new Critical):** see New Findings.
- **NI1 (new Important):** see New Findings.

### C3 (mission phase wiring) — ADDRESSED

Recorder defaults and launch both use `/air_ground/mission_phase`; `classify_mission_phase` maps `INSPECTION_CONFIRMED`→SUCCESS and every producer-emitted `ERROR_{TAKEOFF,REGISTRATION,APPROACH,TARGET,COORDINATE,CONTROLLER}` (+future `ERROR_INSPECTION`) to stable `MISSION_*` codes — all literals match `uav_sphere_mission.py`. Terminal phases finalize immediately; race-safe. Minor residue (NM2): the legacy follow mission's terminal `COMPLETE` classifies PENDING, so pairing the recorder with the legacy Demo would end TIMEOUT rather than success.

### C4 (inspection distance) — ADDRESSED

`metrics()` now measures full 3-D UGV→anomaly distance (`red_sphere`, matching the spawned model name), applies the inclusive radius to that value, and emits `ANOMALY_TRUTH_UNAVAILABLE` when sphere truth is missing. Test asserts the hand-computed sqrt(0.3²+0.2²+0.2²).

### I1 (exactly-once persistence) — ADDRESSED

Writer: lock-protected, duplicate-ID rejection (memory + existing JSON), `.partial` JSON then CSV append with byte accounting, atomic `os.replace`, exception-path truncation restoring retryability — each behavior directly tested, including genuine 16-thread concurrent writes producing 16 unique rows and simulated-IOError rollback + retry. Recorder: single `_finalize_lock` around transition+persist, status published only after success. Caveat: the recorder-level race test is ineffective as constructed (NI2).

### I2 (visibility basis and trace) — ADDRESSED

Window membership evaluates `image_stamp − epoch` (occurrence-based, late delivery tolerated), pre-epoch stamps fail fast and the gate drops them with a warning; delay still schedules from receipt; diagnostics carry `image_stamp`, `receipt_time`, `scheduled_release`, `actual_release`, outlier, stream seed, and trial seed. Fake-clock script tests verify visible-at-capture/late-receipt publication with preserved image stamp and zero publications/diagnostics for hidden observations.

### I3 (twist convention) — ADDRESSED

Explicit `parent`/`body` modes in both `transform_odom` and `populate_odometry`; `parent` rotates vectors+covariance and relabels `child_frame_id := destination_frame`; `body` preserves both. Launch encodes the audited producer ruling (UAV `parent` for mavros map-frame practice, UGV `body` for planar_move). The non-diagonal cross-term expectation was independently re-derived during this review and matches. Nit: `populate_odometry` defaults `"body"` while `transform_odom` defaults `"parent"` — harmless today because the node passes one convention to both, but an API foot-gun.

### I4 (independent drift streams) — ADDRESSED

`domain_seed(trial_seed, label)` via `SeedSequence([trial_seed, crc32(label)])` is deterministic and label-separated; launch passes `uav`/`ugv`/`gate` labels; the command adapter reuses the `uav` domain with identical constructor inputs, so inversion regenerates the UAV walk (parity tested at seed level and structurally guaranteed by determinism). Truth messages, gate diagnostics, and recorder metadata expose both trial and effective stream seeds; the recorder validates incoming truth against expected stream seeds.

### I5 (canonical statuses) — ADDRESSED

Allowed statuses are exactly {COMPLETED, FAILED, TIMEOUT}; failure codes required on FAILURE_STATUSES, forbidden on COMPLETED (which additionally requires a finite inspection distance); the `timeout` column derives from status; contradiction cases including trailing-space statuses are tested. Recorder paths emit only canonical values. Nit: whitespace-only failure codes (e.g., `" "`) still satisfy the nonempty check.

### I6 (behavior-depth tests) — ADDRESSED

`ros_stubs` duck-typed message layouts (including real `PositionCommand` constants) and a fake rospy now drive the actual script classes end-to-end: odom/command/gate callbacks with publication-count assertions, recorder anchor/truth/model/estimate/phase flows, completion, sync-failure, and duplicate-write paths. Classifier, topic, anomaly-model, seed-domain, twist-convention, and epoch-sharing literals were checked against real producers and are mutation-sensitive. Per-finding focused RED commands are documented; historical RED output remains self-reported text (acceptable here), and one named test needs strengthening (NI2).

## New Findings Introduced by the Fix Diff

### NC1 (Critical). Truth-history eviction makes synchronized evaluation impossible for realistic freeze→finish gaps

- **Location:** `src/air_ground_experiments/src/air_ground_experiments/metrics.py:126-129,170-184`; consumer `src/air_ground_experiments/scripts/experiment_recorder.py:145-169`
- **Failure scenario:** `_truth_history` is a `deque(maxlen=600)`. At typical odometry truth rates (30–50 Hz) that retains only ≈12–20 s. The estimate freezes early (registration completes seconds after visibility) while finalization happens at `INSPECTION_CONFIRMED`/timeout, typically minutes later. Once `t̂` ages out of either history, `_interpolate` returns `None`, `completion_failure_code()` yields `INCOMPLETE_TRUTH_SYNC`, and every genuinely successful trial is recorded as FAILED. Bounded simulation run during this review: 2400 truth pairs at 30 Hz (80 s) → `registration_truth_at(30.0)` returns `None`.
- **Minimal fix:** retain the whole per-trial history or a coarse permanent decimation (a few hundred KB at most), or bracket `t̂` with persisted snapshots; add a retention-expiry regression test (freeze early, ingest >maxlen later samples, require interpolation still succeeds).

### NI1 (Important). Takeoff anchor does not replicate the estimator's configured rule

- **Location:** `src/air_ground_experiments/src/air_ground_experiments/metrics.py:131-148` vs producer `src/air_ground_coordinate_transform/scripts/takeoff_registration.py:129-136`
- **Failure scenario:** the estimator selects `origin_yaw = −mean_yaw` only when `align_origin_to_uav_heading=true`; otherwise it uses `fixed_origin_yaw` (current research config sets `align_origin_to_uav_heading: false`, `fixed_origin_yaw: 0.0`). The evaluator hardcodes the `−mean_yaw` branch, so whenever mean takeoff yaw ≠ 0 under the deployed config, truth and estimate live in frames differing by a constant rotation about the takeoff point — reintroducing C2-class systematic yaw/translation bias that grows with lever arm. Verified numerically: anchors diverge for `align=false`, nonzero yaw. The report's "estimator's exact rule / exactly like the estimator" therefore overclaims.
- **Minimal fix:** mirror `~align_origin_to_uav_heading`/`~fixed_origin_yaw` in the recorder (matching the registration config used by the trial) and branch identically; add a fixed-yaw-anchor equivalence test; soften the report wording until parameterized.

### NI2 (Important). The recorder race test cannot exercise the race

- **Location:** `src/air_ground_experiments/test/test_recorder_evaluation.py:220-238`
- **Failure scenario:** the harness never advances `rospy.now_seconds` past `started + timeout_seconds` (both are 100 s), so the `tick(None)` timeout thread is a no-op; only the ERROR thread finalizes. The single-row assertion passes even if `_finalize_lock` were deleted, so the test cited as I1's recorder-side concurrency proof is vacuous.
- **Minimal fix:** have one thread call `tick` with the fake clock advanced beyond the timeout (or set `~timeout_seconds≈0`) while the other fires a terminal phase concurrently; keep asserting exactly one row with status ∈ {TIMEOUT, FAILED}.

### NM1 (Minor). New test modules are not registered with catkin

- **Location:** `src/air_ground_experiments/CMakeLists.txt:35-40`
- `catkin_add_nosetests` still lists only the four round-0 files; `test_script_adapters.py` and `test_recorder_evaluation.py` (the most valuable suites) never run under catkin testing. The retained round-0 report bullet "registers all pure/static tests" is now stale.

### NM2 (Minor). Legacy mission terminal phase unrecognized

- **Location:** `src/air_ground_experiments/src/air_ground_experiments/metrics.py:65-76`
- `COMPLETE` (legacy `uav_follow_mission.py` terminal) maps to PENDING; recorder runs beside the legacy Demo would record TIMEOUT instead of success. Acceptable for research scope, but worth documenting or mapping.

### NM3 (Minor). Epoch hardening defaults and small nits

- Shipped launch defaults leave `epoch_seconds=0.0` and no `maximum_elapsed_seconds`; wall-clock miswiring recreates the C1 hang mode. A conservative default maximum would fail fast instead.
- Nits: duplicate `import numpy as np` (`metrics.py:11,13`); whitespace-only failure codes pass validation; cosmetic launch indentation of injected param lines.

## Verification and Residual Risk

- All commands listed at the top ran fresh during this review; no prohibited process was started; no Gazebo truth was read; no file outside this review report was modified.
- Unchanged residuals: dynamic M1-B remains mandatory (must additionally confirm the mavros `parent` convention on live messages, the `Delta` at-rest assumption, and — now — anchor-rule parity under the deployed registration config); anchor sample races between recorder and estimator remain possible; RED evidence is self-reported text.

## Re-Review Verdicts

- **Spec Compliance: FAIL** — the new evaluator path still cannot produce valid results for realistic trials (NC1) and truth/estimate frames can silently diverge under the deployed config (NI1).
- **Code Quality: FAIL** — substantial improvement across all six round-0 Important items, but the fix diff introduces 1 Critical + 2 Important issues.

Count: original findings — C1/C3/C4 and I1-I6 ADDRESSED, C2 PARTIAL; new findings — 1 Critical, 2 Important, 3 Minor. Highest severity: Critical.

---

# Task 4 Scoped Re-Review — Fix Round 2

Scope: open findings NC1, NI1, NI2 only, plus a diff scan for new
Critical/Important issues and a memory/complexity check on full truth
retention. Read-only; no ROS/Gazebo processes; no truth read; no Git.

Fresh bounded verification run during this review:

- Full suite `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s test -p 'test_*.py' -v` → **78 tests, OK** (matches the report).
- Reviewer-side replay of the original NC1 scenario against the fixed code (freeze bracket at t≈15, then 2400 later truth pairs ≈ 80 s at 30 Hz): history retains all entries and `registration_truth_at(15.0)` returns `[3.5, 0, 0]`.
- Independent branch-equivalence check with my own estimator-formula reimplementation: evaluator anchor matches for both branches (`align=True` with mean yaw 0.7; `align=False`, `fixed_origin_yaw=0.25`) — rotation and translation exact.
- `py_compile` (metrics.py, experiment_recorder.py) → OK; XML parse of launch + package.xml → OK.

## Dispositions

### NC1 (truth-history eviction) — ADDRESSED

- `_truth_history` is now plain append-only lists with no `maxlen`; monotonic-stamp ingestion retained (`metrics.py:133-136,172-184`). The misleading eviction knob was removed entirely.
- The regression test freezes a bracket around t=15, then ingests 700 later pairs (> the old 600-entry capacity) and requires `[3.5, 0]` — it fails by construction if eviction is reintroduced, so it is genuinely mutation-sensitive against this defect class.
- The report's RED (`NoneType` failure), reviewer-equivalent pre-fix reproduction, and GREEN commands are coherent with the delivered test. The discarded decimation attempt is documented; one quoted intermediate value ("[3.0, 0]") does not reconstruct exactly under my arithmetic for that fixture, but the underlying lossy-bracket objection is valid regardless, and the adopted option is the review's recommended first choice. Not a finding.
- Memory/complexity of full retention: bounded per trial (~≤10^5 small entries ≈ tens of MB even for multi-hour high-rate runs; typical trials far less); `_interpolate` runs once per finalize. No realistic failure scenario → not counted as a finding, matching the review instructions.

### NI1 (anchor rule parameterization) — ADDRESSED

- `TrialTruthEvaluator(align_origin_to_uav_heading, fixed_origin_yaw)` branches exactly like `takeoff_registration.py` (`origin_yaw = −mean_yaw` iff aligning, else fixed; translation `−R(origin_yaw)·center`); independently verified for both branches.
- `experiment_recorder.py:62-68` mirrors both values from `~align_origin_to_uav_heading`/`~fixed_origin_yaw`; the wiring test asserts propagation through the param layer. The fixed-yaw-anchor test uses hand-derived literals (`−cos 0.25, −sin 0.25`) independent of production helpers and flips red if the branch regresses.
- Report overclaim corrected: Round-2 "Wording Corrections" supersedes "estimator's exact rule", and the recorder metadata string now names the configurable rule.
- Residual recorded as NM4 below: the shipped launch does not yet carry these two parameters and the recorder default (align=True) differs from the current research config (false), so value propagation remains an explicit Task 5 runner obligation — disclosed, tested at the unit level, and outside this package's fix scope, hence Minor rather than Important.

### NI2 (vacuous race test) — ADDRESSED

- The test now sets `~timeout_seconds: 0.05`, advances the fake clock past start+timeout before racing, and releases `tick(None)` and `ERROR_TARGET` through `threading.Barrier(2)`; it asserts zero thread errors plus exactly one CSV row with status ∈ {TIMEOUT, FAILED} (`test_recorder_evaluation.py:231-266`).
- Mutation sensitivity verified analytically: without `_finalize_lock`, both threads pass the `finalized` check and the writer's duplicate guard raises into the collected errors, so `assertEqual(errors, [])` fails — consistent with the reported 5/5 red under mutation and 3/3 green after restore. The cited evidence is credible.

## Diff Scan — New Issues

No new Critical or Important issues found in this round's diff. One new Minor observation:

- **NM4 (Minor):** `frame_perturbation.launch` does not pass `align_origin_to_uav_heading`/`fixed_origin_yaw` to the recorder, whose defaults (align=True) contradict the shipped research config (`registration.yaml: align=false`); until Task 5 wires the registration node's actual values, default-wired trials can silently reuse the wrong anchor branch when takeoff yaw ≠ 0. Also cosmetic: the `TrialTruthEvaluator` class docstring (`metrics.py:104,112-113`) still describes an unconditional `origin_yaw=-yaw` rule.

Deferred NM1-NM3 remain open by instruction; dynamic M1-B residual risks updated in the report (must confirm live rates vs retention bound and real anchor-parameter propagation).

## Re-Review Verdicts

- **Spec Compliance: PASS** (static/pure scope of this fix round; NC1, NI1, NI2 all resolved with behavior-level, mutation-sensitive evidence; dynamic M1-B remains externally pending as documented).
- **Code Quality: PASS** (no new Critical/Important; one Minor wiring/docstring note recorded).

Count: NC1/NI1/NI2 all ADDRESSED; new findings — 0 Critical, 0 Important, 1 Minor (NM4).

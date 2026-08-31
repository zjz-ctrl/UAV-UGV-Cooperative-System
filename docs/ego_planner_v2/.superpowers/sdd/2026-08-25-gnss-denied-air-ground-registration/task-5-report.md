# Task 5 Report — One-Shot Baseline Matrix

## Status

IMPLEMENTATION COMPLETE (pure/static scope). The matrix expansion, cold-start
trial-lifecycle runner, frozen one-shot matrix YAML, integration launch, and
bringup manifest dependencies are implemented and verified through 29 new
focused tests plus full regressions. Dynamic M1-C execution (smoke seeds,
30-seed matrix, acceptance evaluation) remains explicitly pending external
execution per the no-long-process ruling.

No Git repository was initialized and no commit was created.
No subagent or reviewer was dispatched.

Environment note discovered during debugging: `/home/zjz/air_ground_cooperation/Ego_Planner_v2`
is a symlink to `/home/zjz/Ego_Planner_v2`; both paths denote the same files.

## Files (brief mapping)

- Created: `src/air_ground_experiments/scripts/run_experiment_matrix.py`
- Created: `src/air_ground_experiments/config/one_shot_matrix.yaml`
- Created: `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch`
- Modified: `src/air_ground_bringup/CMakeLists.txt` (dependency declarations only)
- Modified: `src/air_ground_bringup/package.xml` (dependency declarations only)
- Test: `src/air_ground_experiments/test/test_matrix_expansion.py`

## RED/GREEN Evidence (per behavior)

Test driver convention: `PYTHONDONTWRITEBYTECODE=1 python3 test/test_matrix_expansion.py`
from `src/air_ground_experiments` (29 tests final). Every production behavior
below was preceded by an observed failing run; failure text is abbreviated.

### A. Deterministic matrix expansion (`expand_matrix`, `TrialSpec`)

| # | Behavior | RED observation | GREEN result |
|---|---|---|---|
| A1 | Fixture yields 30 trials; `trials[0].seed==1000`; `trials[-1].trial_id=="one_shot-0029"` | `ModuleNotFoundError: No module named 'run_experiment_matrix'`; after minimal module, `AssertionError: 'one_shot-29' != 'one_shot-0029'` (padding driven by the brief's own snippet) | OK |
| A2 | Sorted by `trial_id` regardless of seed-list order; id follows the seed's rank in the frozen list | Reversed-seed fixture: `AssertionError: 1029 != 1000` at position 0 | OK |
| A3 | Samples stay inside closed bounds; UGV physical spawn = UAV spawn ⊕ R(uav_yaw)·(dx,dy); `ugv_yaw = wrap(uav_yaw+offset)` | `KeyError: 'uav_x'` (no sampling existed); then normalization mismatch `119.224 != -240.776` (exactly 360°) fixed by wrapping the expected value in the test | OK |
| A4 | Same config rerun → field-for-field identical; distinct seeds → distinct parameters | Passed immediately post-A3; mutation-proven: replacing `np.random.default_rng(seed)` with global-entropy `default_rng()` made `test_expansion_is_reproducible_field_for_field` FAIL (`'-3.332101...' != '-3.554451...'`); restored → OK | OK (mutation-proven) |
| A5 | launch_args pin `registration_mode=one_shot`, `use_visual_frame_yaw=true`, zero drift rates, seed/trial_id consistency, timeout/epoch values, per-trial output dir under root ending with trial_id; frame offsets within ±3 m / ±45°(rad) | Passed immediately post-A3; mutation-proven: flipping `registration_mode`→`batch` and drift→`0.01` made the test FAIL; restored → OK | OK (mutation-proven) |
| A6 | `config/one_shot_matrix.yaml` exists, parses, equals the brief bounds exactly, seeds == range(1000,1030), expands equal to fixture | `FileNotFoundError` opening the YAML; then list-equality failed because `TrialSpec` lacked `__eq__` | OK |

### B. Cold-start trial lifecycle runner (injected fakes only)

Fake collaborators in the test file: `FakeProcessManager` (precheck/spawn/
signal/reap/poll recording), `FakeClock`, `FrozenWatchFactory`,
`FakeResultReader`, `RecordingClassificationWriter`.

| # | Behavior | RED observation | GREEN result |
|---|---|---|---|
| B1 | Precheck failure (matching ROS/Gazebo/PX4 process) → `LAUNCH_FAILED`, roslaunch never spawned, classification recorded | `ImportError: cannot import name 'EXIT_LAUNCH_FAILED'` | OK |
| B2 | `popen` raising `OSError` → `LAUNCH_FAILED` | `NotImplementedError: lifecycle continuation pending` | OK |
| B3 | FROZEN never observed within registration timeout → `REGISTRATION_FAILED` reason `frozen_timeout`; SIGINT sent; child reaped; quiet-system re-check (2 prechecks total) | `'LAUNCH_FAILED' != 'REGISTRATION_FAILED'` | OK |
| B4 | Terminal-state mapping from recorder rows: COMPLETED+success→PASS; FAILED `MISSION_APPROACH`→MISSION_FAILED; FAILED {`MISSION_REGISTRATION`,`INCOMPLETE_TRUTH_SYNC`,`ANOMALY_TRUTH_UNAVAILABLE`}→REGISTRATION_FAILED; TIMEOUT/`TRIAL_TIMEOUT`→TIMEOUT; {`OUTSIDE_SUCCESS_RADIUS`, unknown}→MISSION_FAILED (default), raw code always retained | All five mapping tests returned placeholder `'LAUNCH_FAILED'` | OK |
| B5 | Runner-side deadline backstop with no recorder row → TIMEOUT **with** teardown | First RED surfaced a real defect: backstop returned TIMEOUT without sending SIGINT (`[] != [SIGINT]`) — fixed by tearing down before recording | OK |
| B6 | roslaunch exits before any result row → `LAUNCH_FAILED` reason `roslaunch_exited_early` (runner previously never polled the process: ran to TIMEOUT) | `'TIMEOUT' != 'LAUNCH_FAILED'` | OK |
| B7 | `JsonClassificationWriter` creates the per-trial directory and writes `exit.json` (sorted keys) | `ImportError: JsonClassificationWriter` | OK |
| B8 | `run_matrix` runs trials strictly sequentially; exactly one precheck before and one quiet-system verification after each trial (2×N prechecks ⇒ no ROS master reuse); each spawned argv carries its own `trial_id:=one_shot-000N` and `registration_mode:=one_shot` | `AttributeError: no attribute 'run_matrix'` | OK |
| B9 | Real-dependency primitives: `matching_processes(patterns, cmdline_by_pid)` detects rosmaster/roslaunch/gzserver/gzclient/px4 among benign processes; `JsonResultReader` parses `<trial_dir>/<safe_id>.json` or returns None when absent | `ImportError: matching_processes` | OK |
| B10 | Module-level ROS-import ban keeps the suite pure (AST audit of top-level imports for rospy/rosbag/tf) | Guard test (passed immediately by construction; enforces the property as CLI glue landed) | OK |

CLI glue (`main`, `WallClock`, `SystemProcessManager`, `RosFrozenWatch`,
`load_matrix_config`) is thin external-only wiring verified by `py_compile`
and the B10 AST audit; `rospy` is imported lazily inside `RosFrozenWatch`.
`main()` was never executed here.

### Debugging episodes (systematic-debugging invoked)

1. **A2 persistent identical failure after first fix.** Evidence gathering
   (isolated repro + `inspect.getsource`) showed source-on-disk correct while
   execution disagreed. Root cause: ids were assigned from `enumerate`
   positions *before* sorting by seed, so original list positions travelled
   with their seeds (`(0,1029)` became `one_shot-0000`). Fix: rank =
   position within `sorted(seeds)`. Verified OK.
   (Side finding: the apparent path discrepancy came from the workspace
   symlink; harmless.)
2. **Coordinate-transform regression timeout (120 s).** My discovery command
   included `test_registration_node.py`, which is a pre-existing Task-2
   **rostest** requiring a ROS master (`rospy.init_node` blocks). Not a code
   regression; correct pure invocation excludes it (see Bounded Verification).
3. **catkin configure failure after manifest edit.**
   `catkin_package() the catkin package 'air_ground_experiments' has been
   find_package()-ed but is not listed as a build dependency in the
   package.xml`. Added a RED assertion for `build_depend` entries, then added
   `build_depend` + `build_export_depend` for both workspace packages →
   build passed.
4. Small harness fixes recorded honestly: relative-`__file__` parents crash
   (resolved paths), duplicate fake `poll` method, signal-module shadowing in
   the fake manager, recursive node search for the namespaced group,
   `str.index(")")` matching line 1 instead of the find_package block.

## Implementation

- `TrialSpec`: immutable value object (`trial_id`, `seed`, `launch_args`,
  `timeout_seconds`, `output_directory`, `sampled_parameters`) with
  field-wise `__eq__` used by the YAML-freeze equality test.
- `expand_matrix(config)`: validates `drift: zero` and bound completeness;
  assigns `one_shot-%04d` ids by ascending-seed rank; draws all ten pose
  quantities from one instance-local `numpy.random.default_rng(seed)` (PCG64)
  in a fixed key order (`uav_x, uav_y, uav_yaw_deg, ugv_heading_offset_deg,
  uav_frame_dx/dy/yaw, ugv_frame_dx/dy/yaw`); computes the UGV physical spawn
  at the UAV body-relative registration waypoint
  (`ugv = uav ⊕ R(uav_yaw_rad)·(dx,dy)`, `ugv_yaw = wrap(uav_yaw+offset)`);
  emits roslaunch-ready string args including constant frame offsets
  (`uav_initial_xyyaw`/`ugv_initial_xyyaw` in radians), zero drift rates,
  shared `epoch_seconds`, per-trial `output_directory`, `timeout_seconds`;
  returns specs sorted by `trial_id`. No global RNG state is touched.
- `TrialRunner`: sequential cold-start lifecycle with fully injected
  collaborators (process manager, clock, frozen-watch factory, result reader,
  classification writer, optional flush callback):
  precheck forbidden processes → popen roslaunch with explicit args → wait
  FROZEN until registration timeout → poll recorder JSON until terminal row /
  early exit / overall deadline → request recorder flush → SIGINT → bounded
  reap with kill escalation → quiet-system re-check → write per-trial
  classification. Failure taxonomy maps canonical recorder rows onto
  `PASS / LAUNCH_FAILED / TIMEOUT / REGISTRATION_FAILED / MISSION_FAILED`
  (table below); every classification retains the raw failure code and a
  teardown-cleanliness flag.
- `one_shot_matrix.yaml`: frozen 30 seeds (1000–1029), brief bounds verbatim,
  `registration_dx_m: 0.60`, `registration_dy_m: 0.0`, `timeout_seconds: 180`,
  `epoch_seconds: 0.0`, `drift: zero`, launch target coordinates.
- `air_ground_inspection_experiment.launch`: Gazebo/PX4/mavros stack
  (`uav_sitl.launch`, `spawn_ugv.launch`, red-sphere spawn) + raw perception +
  `frame_perturbation.launch` (two odometry perturbers, observation gate,
  command adapter, and the evaluation-only recorder/evaluator) between raw
  sources and research nodes; `takeoff_registration.py` launched directly with
  `registration.yaml` plus overrides to the experiment streams
  (`uav_odom_topic=/air_ground_experiment/uav/odom`,
  `ugv_odom_topic=/air_ground_experiment/ugv/odom`,
  `observation_topic=/air_ground_experiment/charuco/observation`),
  `use_visual_frame_yaw=$(arg use_visual_frame_yaw)` (default true) and
  `registration_mode=$(arg registration_mode)` (default `one_shot`);
  mission + auto-takeoff remapped onto experiment odometry streams with
  position commands routed through the adapter topic
  `/air_ground_experiment/iris_0/position_cmd` (adapter destination remains
  `/iris_0/position_cmd` for CXR/PX4); UGV autonomy stays on the raw side;
  no truth topics are introduced outside the perturbation include.
- Bringup manifests: `air_ground_experiments` and `air_ground_perception`
  declared as catkin components (`find_package`, `CATKIN_DEPENDS`,
  `build_depend`, `build_export_depend`) and runtime deps (`exec_depend`);
  existing `install(DIRECTORY config launch rviz scripts ...)` already covers
  the new launch file — no other changes.

## Matrix Freeze (M1-C checkpoint)

Frozen before any dynamic run; thresholds must not be tuned after inspecting
held-out or any other seeds.

- **Software parameters:** Python 3.8.10; NumPy 1.17.4 (PCG64 via
  `default_rng(seed)`); PyYAML; ROS Noetic catkin workspace. Registration
  research config = `src/air_ground_coordinate_transform/config/registration.yaml`
  verbatim (notably `minimum_origin_samples: 30`, `minimum_samples: 20`,
  `sample_period: 0.10`, `max_odom_bracket: 0.08`,
  `max_translation_residual: 0.12`, `max_yaw_residual: 0.03`,
  `align_origin_to_uav_heading: false`, `fixed_origin_yaw: 0.0`,
  `use_visual_frame_yaw: true`). Perturbation defaults: zero drift rates,
  `drift_step_seconds: 1.0`, shared `epoch_seconds: 0.0`, domain-separated
  stream seeds via `domain_seed(trial_seed, label)` (uav/ugv/gate/uav-command).
  Recorder defaults per `experiment_recorder.py`
  (`success_radius_m: 0.5`, `source_relation_window_seconds: 30`,
  `minimum_anchor_samples: 30`, mission topic `/air_ground/mission_phase`).
  Mission geometry: body-relative waypoint dx=0.60 m, dy=0.0 m; mission phase
  parameters as wired in the launch (front pitch 25°, scan 4.0 m, approach
  3.0 m, center 2.3 m, overwatch 4.0 m, standoff 3.5 m).
- **Seed list (frozen):** trials `one_shot-0000..0029` ↔ seeds `1000..1029`
  (identity mapping; YAML lists them explicitly). Frozen sampled spawns
  (degrees/metres, computed deterministically, reproduced twice):

  ```
  trial_id        seed   uav_x     uav_y    uav_yaw    ugv_yaw    ugv_x     ugv_y
  one_shot-0000   1000  -2.9572   0.4154   -10.4610  -117.2917   -2.3672   0.3064
  one_shot-0001   1001  -2.7748  -1.9372  -112.4318    16.4087   -3.0038  -2.4918
  one_shot-0002   1002  -3.2384  -0.5712    89.1404    49.3575   -3.2294   0.0287
  one_shot-0003   1003  -3.6245  -0.9732    -1.9154    55.9695   -3.0248  -0.9932
  one_shot-0004   1004  -3.9982  -1.4794  -130.1481  -173.1125   -4.3851  -1.9380
  one_shot-0005   1005  -3.8356   1.7222   -76.5777   -26.6969   -3.6963   1.1386
  one_shot-0006   1006  -3.3262   0.4034   -72.7807  -167.2547   -3.1485  -0.1697
  one_shot-0007   1007  -3.8576   1.2639    51.4543   -57.7714   -3.4837   1.7332
  one_shot-0008   1008  -3.6655   1.2522  -101.7242    72.4328   -3.7874   0.6647
  one_shot-0009   1009  -3.8870  -0.6923  -162.0535   119.2240   -4.4578  -0.8772
  one_shot-0010   1010  -3.8100   0.1432   150.3636   -36.3087   -4.3315   0.4399
  one_shot-0011   1011  -3.4318   0.2533  -103.4640   106.0701   -3.5715  -0.3302
  one_shot-0012   1012  -3.3874   1.1226   -94.1629    35.5814   -3.4309   0.5242
  one_shot-0013   1013  -2.4110  -1.6841    62.1575    43.2821   -2.1308  -1.1535
  one_shot-0014   1014  -3.1290   0.3428   143.2673   -39.4036   -3.6098   0.7017
  one_shot-0015   1015  -3.2802   0.4283    50.8698   -95.6734   -2.9015   0.8937
  one_shot-0016   1016  -3.5440  -0.2292   -52.6954   -24.0006   -3.1804  -0.7065
  one_shot-0017   1017  -2.1633   1.2853   170.1286  -154.0076   -2.7544   1.3882
  one_shot-0018   1018  -2.8120  -0.1802   133.7450  -113.0752   -3.2269   0.2533
  one_shot-0019   1019  -3.5661  -0.4160   -11.9217   106.4290   -2.9791  -0.5400
  one_shot-0020   1020  -2.8426  -0.8653  -152.7506   -78.2425   -3.3760  -1.1400
  one_shot-0021   1021  -3.2512  -1.6766   176.9268    37.5698   -3.8504  -1.6444
  one_shot-0022   1022  -3.9989   1.6737    32.3332    84.9173   -3.4920   1.9946
  one_shot-0023   1023  -3.3223   1.6236   147.4567  -123.5429   -3.8281   1.9464
  one_shot-0024   1024  -3.3551  -0.9666   -46.6877    91.9301   -2.9435  -1.4032
  one_shot-0025   1025  -2.3578   0.5160  -121.6278   151.1545   -2.6724   0.0051
  one_shot-0026   1026  -3.6585   0.3805  -109.9548    82.8454   -3.8633  -0.1835
  one_shot-0027   1027  -3.9441   0.2486    23.4583  -152.3959   -3.3936   0.4874
  one_shot-0028   1028  -2.3677  -0.4559    63.1124   -51.5913   -2.0963   0.0792
  one_shot-0029   1029  -2.3264   1.2917  -162.2892    83.9565   -2.8980   1.1092
  ```

  Frame-perturbation offsets (`uav_initial_xyyaw`, `ugv_initial_xyyaw`) are
  likewise frozen per seed through the same RNG; they live in the CSV/JSON
  outputs via the recorder metadata and can be regenerated exactly by rerunning
  `expand_matrix(load(config))`.
- **Success-rate formula:** `completion_rate = (#rows with status==COMPLETED
  AND success==True) / (#rows with status in {COMPLETED, FAILED, TIMEOUT})`.
  Rows missing entirely (runner-level LAUNCH_FAILED) count against completion
  in the denominator as failures; report both denominators if they diverge.
- **Percentile definitions:** over successful trials only, translation error
  p95 = `numpy.percentile(handoff_error_m, 95)` and yaw error p95 =
  `numpy.degrees(numpy.percentile(yaw_error_rad, 95))` using NumPy's default
  linear interpolation; NaN metrics (never expected on success) would exclude
  the trial and must be reported.
- **Failure taxonomy (recorder code → runner exit):**

  | Recorder failure_code | Exit classification | Domain |
  |---|---|---|
  | (COMPLETED, success=True) | PASS | success |
  | TRIAL_TIMEOUT / status TIMEOUT | TIMEOUT | infrastructure/watchdog |
  | MISSION_REGISTRATION | REGISTRATION_FAILED | registration |
  | INCOMPLETE_TRUTH_SYNC | REGISTRATION_FAILED | evaluation sync |
  | ANOMALY_TRUTH_UNAVAILABLE | REGISTRATION_FAILED | evaluation truth |
  | OUTSIDE_SUCCESS_RADIUS | MISSION_FAILED | mission execution |
  | MISSION_TAKEOFF/APPROACH/TARGET/COORDINATE/CONTROLLER | MISSION_FAILED | mission execution |
  | any unknown code (raw code retained) | MISSION_FAILED | conservative default |

- **Acceptance (frozen, from the plan/brief):** registration completion ≥ 95%;
  translation error p95 ≤ 0.15 m; yaw error p95 ≤ 2.0 deg; every failure
  retains a reason code.

## Static Launch Audit (summary)

Verified programmatically by `IntegrationLaunchAuditTest` +
`BringupManifestAuditTest` on parsed XML/text:

- `registration_mode` arg exists with default `one_shot`; runner passes
  `registration_mode:=one_shot` on every trial argv (B8).
- Perturbation layer included between raw sources and research nodes with all
  eight arguments wired (`seed`, initial offsets, drift rates, epoch,
  output_directory, trial_id); UAV/UGV spawn geometry forwarded.
- Research nodes consume experiment streams (registration param overrides;
  mission/takeoff remaps); commands routed through
  `/air_ground_experiment/iris_0/position_cmd` → adapter → `/iris_0/position_cmd`
  (CXR present on the raw side).
- Evaluator/recorder started via the include; no truth topics introduced
  outside `/air_ground_experiment/truth/*` (which lives solely in the include).

## Bounded Verification

- New focused suite: 29 tests OK
  (`PYTHONDONTWRITEBYTECODE=1 python3 test/test_matrix_expansion.py`).
- Full `air_ground_experiments` regression: `python3 -m unittest discover -s
  test -p 'test_*.py'` → **107 tests, OK** (78 prior + 29 new).
- Legacy bringup suites: **14 tests OK** (launch wiring + registration
  waypoint unchanged).
- Tasks 1–3 coordinate-transform pure regressions: **34 tests OK**
  (13 odom_buffer + 13 estimator + 8 se2) via
  `PYTHONPATH=src python3 -m unittest discover …` excluding the pre-existing
  rostest file that requires a ROS master (and is forbidden here anyway).
- `python3 -m py_compile` on `run_experiment_matrix.py` and
  `test_matrix_expansion.py` → OK.
- XML parse OK: `air_ground_inspection_experiment.launch`,
  `air_ground_bringup/package.xml`, `air_ground_experiments/package.xml`.
- YAML parse OK: `one_shot_matrix.yaml` (30 seeds, drift zero).
- Bounded build: `timeout 570 catkin_make --pkg air_ground_bringup
  air_ground_experiments -DCATKIN_ENABLE_TESTING=ON` → completed (exit 0)
  after the package.xml build-dependency fix.
- No roslaunch/roscore/rostest/Gazebo/PX4/RViz/rosbag/rosnode-info/topic-wait
  command was executed; the runner itself was never launched; no Gazebo truth
  was read; no Git operations were performed.

## Explicitly Not Executed Here (external M1-C work)

**Fresh-root rule (M1-C):** run every matrix invocation into a fresh output
root. Never mix rows from smoke and full runs in one directory: a stale
`<trial_id>.json`/`exit.json` makes the runner refuse to start (exit code 3),
and `--force` deletes those whole per-trial directories before rerunning.

1. Smoke seeds (documented, do NOT tune anything based on them before the
   full run):

   ```bash
   rosrun air_ground_experiments run_experiment_matrix.py \
     --config src/air_ground_experiments/config/one_shot_matrix.yaml \
     --output-root /tmp/air_ground_experiments/matrix_one_shot/smoke \
     --trials one_shot-0000,one_shot-0001,one_shot-0002
   # (or python3 src/air_ground_experiments/scripts/run_experiment_matrix.py …)
   ```

   Expected: three result rows, three metadata files, clean shutdown per trial.
2. Full 30-seed matrix into its own fresh root:

   ```bash
   rosrun air_ground_experiments run_experiment_matrix.py \
     --config src/air_ground_experiments/config/one_shot_matrix.yaml \
     --output-root /tmp/air_ground_experiments/matrix_one_shot/full
   ```

   (`--force` exists for deliberate same-root reruns; it removes the stale
   per-trial directories entirely so no row survives from an earlier run.)
3. Acceptance evaluation against the frozen thresholds above, from the
   produced `trials.csv`/per-trial JSON only.

## Modified / Created Files

- Created `src/air_ground_experiments/scripts/run_experiment_matrix.py`
- Created `src/air_ground_experiments/config/one_shot_matrix.yaml`
- Created `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch`
- Modified `src/air_ground_bringup/CMakeLists.txt` (component declarations)
- Modified `src/air_ground_bringup/package.xml` (depend declarations)
- Created `src/air_ground_experiments/test/test_matrix_expansion.py`
- Created `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-5-report.md`

Tasks 1–4 files were not modified (verified by green legacy regressions).

## Self-Review

- Every production behavior has a named test that was watched failing first;
  the two behaviors whose tests passed immediately (determinism pair, A5
  args, B10 guard) were validated via mutation or serve as standing guards.
- Determinism: instance-local PCG64 keyed by the trial seed, fixed draw order,
  no global RNG; same-config reruns compare equal field-for-field (test-enforced),
  and different seeds demonstrably produce different parameters.
- Runner purity: no ROS imports at module import time (AST-audited); real
  implementations exist but only injected fakes ran in this environment.
- Safety: precheck before every trial, quiet-system verification after teardown
  (2×N checks), SIGINT-first with kill escalation, per-trial directories make
  `TrialResultWriter` exactly-once semantics hold across reruns.
- The launch composes only existing prior-task launches/nodes; no prior-task
  file was edited except the two bringup manifest files, minimally, and only
  for dependency declaration correctness (forced by the real catkin error).
- No subagent/reviewer dispatched; no Git operations; no long-running process.

## Concerns

- `run_experiment_matrix.py` is not yet registered in
  `air_ground_experiments/CMakeLists.txt` (`catkin_install_python`), so
  `rosrun` resolution depends on the devel/source layout; the brief's file
  list did not include that package's CMakeLists, so the external executor
  can invoke the script by path, or a later task may register it.
- `takeoff_registration.py` receives `registration_mode` as a node parameter
  today (the node is inherently one-shot via `OneShotRegistrationState`);
  future modes (Task 7+) will need the node to actually consume it. Passing
  it through the launch keeps the contract visible without touching Tasks 1–2
  files.
- Mission/cxr takeoff interplay: both auto_takeoff_trigger and the mission
  publish commands on the adapter input topic sequentially by phase, mirroring
  the legacy demo topology; live confirmation belongs to M1 smoke runs.
- `test_registration_node.py` (pre-existing rostest) hangs if run without a
  master; pure-suite invocations must exclude it (documented above).
- The catkin build emitted the known workspace warnings (gazebo_msgs
  deprecation; unrelated VTK/Eigen messages in earlier tasks' logs); none
  blocked this package build.

---

# Review Fix Round 1

Scope: exactly I1 and I2 from `task-5-review.md`. The six deferred Minors
(M1–M6) were left untouched as instructed. No subagent/reviewer dispatched,
no prohibited processes, no Git operations. Both findings were verified
against the code before implementation; neither required pushback.

## Dispositions

### I1 — Frozen `timeout_seconds` never reached the recorder: ACCEPTED, FIXED

Verification first: grep confirmed the arg at
`air_ground_inspection_experiment.launch:13` has zero references in that file;
`frame_perturbation.launch` contains no `timeout_seconds` anywhere;
`experiment_recorder.py:45` defaults `~timeout_seconds` to 120.0 while the
frozen YAML says 180.0 — every trial would have ended at 120 sim-time seconds,
systematically distorting M1-C statistics (RTF < ~0.67 would let the runner's
wall-clock backstop fire first).

- RED: new `IntegrationLaunchAuditTest.test_frozen_timeout_budget_reaches_the_recorder`
  → `AssertionError: missing arg named 'timeout_seconds'` for
  `frame_perturbation.launch`.
- GREEN: `<arg name="timeout_seconds" default="120.0"/>` added to
  `frame_perturbation.launch` (default preserves the recorder's current
  effective budget for existing Task-4 users) plus
  `<param name="timeout_seconds" value="$(arg timeout_seconds)"/>` on the
  recorder node; the experiment launch now forwards
  `<arg name="timeout_seconds" value="$(arg timeout_seconds)"/>` into the
  include, completing the chain runner arg → experiment launch → include →
  recorder param. Frozen YAML stays 180.0.
- Static test asserts all three links (arg default, param wiring, include
  forwarding). Focused suite 30 OK; Task-4 launch/packaging suites re-run green.

### I2 — Stale-result reuse on reruns: ACCEPTED, FIXED

Verification first: `JsonResultReader.read` returns any existing
`<trial_id>.json`; `main()` had no freshness guard and only `--config/--trials`;
the documented smoke→full workflow shares one fixed output root, so trials
0000–0002 would be classified from smoke-era rows (launched fresh, then the
reader wins the poll race immediately and SIGINTs early); and
`TrialResultWriter.write` refuses existing paths while `exit.json` silently
overwrites — inconsistent rerun semantics.

- RED: three behavioral tests + one glue audit failed with
  `ImportError: cannot import name 'evaluate_stale_outputs'` /
  `'apply_output_root'`, and the audit failed on missing `"--force"`/
  `"--output-root"`.
- GREEN:
  - `stale_trial_ids(specs)` flags per-trial directories already containing
    `<safe_id>.json` or `exit.json`.
  - `evaluate_stale_outputs(specs, force=False)` returns
    `("ok", [])`, `("conflict", ids)` (refuse), or — with `force=True` —
    removes those entire per-trial directories (`shutil.rmtree`) and returns
    `("cleared", ids)`. Whole-directory removal (not file deletion) is
    deliberate: `trials.csv` lives beside the JSON, so deleting only markers
    would still mix rows across invocations.
  - `apply_output_root(specs, root)` returns redirected copies
    (`<root>/<trial_id>`); frozen config roots untouched.
  - `main()` gains `--output-root` (applied before the staleness check so a
    fresh root never needs `--force`) and `--force`; on conflict it refuses
    to run anything and exits 3 with guidance. Fresh-root-per-invocation is
    documented in the M1-C commands section above.
- Tests cover the rejection path (recorder JSON and exit.json markers),
  force-clearing including directory-gone + subsequent-ok assertions, the
  override redirecting every spec without touching the frozen roots, and a
  static guard that `main()` wires both options and calls both helpers
  (behavioral `main()` execution remains prohibited here).

## Unexpected-failure note (systematic-debugging)

The I1 edit initially broke `frame_perturbation.launch` XML well-formedness
(`mismatched tag: line 78`): my replacement dropped the recorder's closing
`</node>`. Root-caused via direct read of the damaged region and repaired by
restoring `</node>`; parse plus Task-4 packaging/launch suites then passed.
No other unexpected failures occurred in this round.

## Review Fix Round 1 — Bounded Verification

- Focused suite: `PYTHONDONTWRITEBYTECODE=1 python3 test/test_matrix_expansion.py`
  → **34 tests, OK** (29 + 5 new: 1 for I1, 4 for I2).
- Full `air_ground_experiments`: `python3 -m unittest discover -s test -p 'test_*.py'`
  → **112 tests, OK** (107 + 5).
- Bringup legacy suites: **14 tests OK**.
- Coordinate-transform pure regressions: **34 tests OK** (13+13+8), rostest
  module excluded as documented.
- `py_compile` on runner + test file → OK; XML parse of
  `frame_perturbation.launch` and `air_ground_inspection_experiment.launch`
  → OK; YAML parse of `one_shot_matrix.yaml` (30 seeds) → OK.
- Bounded build: `timeout 570 catkin_make --pkg air_ground_bringup
  air_ground_experiments -DCATKIN_ENABLE_TESTING=ON` → completed successfully.

## Modified Files (Round 1)

- `src/air_ground_experiments/launch/frame_perturbation.launch`
  (+arg, +recorder param, restored `</node>`)
- `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch`
  (forward `timeout_seconds` into the include)
- `src/air_ground_experiments/scripts/run_experiment_matrix.py`
  (staleness preflight, `--force`, `--output-root`)
- `src/air_ground_experiments/test/test_matrix_expansion.py`
  (+4 tests)
- `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-5-report.md`
  (this section + M1-C fresh-root commands)

## Self-Review (Round 1)

- Both fixes preserve the freeze: YAML timeout stays 180.0; the launch chain
  now actually delivers it; defaults keep prior-task behavior identical.
- Refusal semantics fail fast before any process work; classification files
  are never written over stale runs; provenance of every row is single-invocation.
- Deferred Minors M1–M6 remain open by instruction; nothing else was touched.

---

# Review Fix Round 2

Scope: exactly C1 from the scoped re-review. N1 remains deferred per
instruction; deferred Minors M1–M6 untouched except where noted transparently
below. No subagent/reviewer dispatched, no prohibited processes, no Git.

## Disposition

### C1 — Runner argv omits `output_directory:=`: ACCEPTED, FIXED

Verified first: `_trial_launch_args` carried no `output_directory` key and
`roslaunch_command` forwards only `launch_args`, so every trial would inherit
the experiment launch's default `…/matrix_one_shot/one_shot-0000` — all
recorder rows land in one directory while `JsonResultReader` polls each
trial's real directory forever (systematic TIMEOUTs; `--output-root` voided).

- RED (argv-level): new
  `TerminalMappingTest.test_every_spawned_argv_carries_the_per_trial_output_directory`
  ran a fake 3-trial matrix and failed with
  `AssertionError: 'output_directory:=/tmp/.../one_shot-0000' not found in [...]`
  — the printed argv shows every emitted key except the missing one.
- GREEN: `expand_matrix` now passes the already-computed per-trial path into
  `_trial_launch_args`, which emits `"output_directory": str(output_directory)`
  (name matches the experiment launch's declared arg). The argv-level test
  asserts `output_directory:=<spec.output_directory>` for every spawned trial.
- Static glue audit (new):
  `IntegrationLaunchAuditTest.test_every_emitted_launch_arg_is_declared_by_the_experiment_launch`
  asserts `set(launch_args) ⊆ set(declared <arg> names)` of the experiment
  launch, so any future runner-emitted-but-undeclared argument fails the suite
  instead of being silently dropped by roslaunch.
- RED proven against existing code: the audit immediately flagged exactly
  `['drift_step_seconds']` as emitted-but-undeclared (the latent M1-class
  defect). GREEN declares `<arg name="drift_step_seconds" default="1.0"/>` and
  forwards it into the perturbation include (whose own arg already exists),
  completing that chain too. Scope note: this resolves deferred M1's substance
  ("declare + forward") as a necessary consequence of the mandated glue
  consistency check; recorded here explicitly rather than silently.

## RED/GREEN Commands and Output

- Focused: `PYTHONDONTWRITEBYTECODE=1 python3 test/test_matrix_expansion.py`
  - After RED-1: `Ran 35 tests ... FAILED` with the missing
    `output_directory:=` assertion above.
  - After GREEN-1: `Ran 35 tests ... OK`.
  - After RED-2: `FAILED` with `['drift_step_seconds'] != []`.
  - After GREEN-2: **`Ran 36 tests ... OK`**.

## Modified Files (Round 2)

- `src/air_ground_experiments/scripts/run_experiment_matrix.py`
  (`_trial_launch_args` emits per-trial `output_directory`)
- `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch`
  (+declare/forward `drift_step_seconds`; `output_directory` arg already existed)
- `src/air_ground_experiments/test/test_matrix_expansion.py`
  (+2 tests: argv regression, glue audit)
- `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-5-report.md`
  (this section)

## Self-Review (Round 2)

- The fix restores end-to-end I2 semantics: redirected roots now actually
  reach the recorder; I1's budget likewise flows to its consumer.
- Determinism unaffected: the added key derives deterministically from config
  + trial id (field-for-field equality tests still pass).
- Glue audit direction is deliberate: runner ⊆ declared catches silent drops;
  extra declared args (defaults for manual smoke runs) are legitimate.
- N1 (`rmtree ignore_errors`) untouched per instruction.

---

# Review Fix Round 3

Scope: exactly IA1 from the latest re-review. N1 and M2–M6 remain deferred
per instruction. No subagent/reviewer dispatched, no prohibited processes,
no Git operations.

## Disposition

### IA1 — `--output-root` desyncs argv from the reader directory: ACCEPTED, FIXED

Verified first: `apply_output_root` passed `spec.launch_args` through
untouched, so after a redirect the argv carried
`output_directory:=<frozen root>/<id>` while the reader, exit.json writer,
and stale-preflight all used `<new root>/<id>` (`spec.output_directory`) —
recorder rows land in the frozen tree, the reader never sees them, and the
whole overridden matrix burns its budget into TIMEOUTs.

- RED: new
  `StaleOutputGuardTest.test_output_root_override_syncs_argv_with_the_reader_directory`
  → `AssertionError: '/tmp/air_ground_experiments/matrix_one_shot/one_shot-0000'
  != '/tmp/tmp…/one_shot-0000'` (launch_args kept the frozen path).
- GREEN: `apply_output_root` now builds each spec with
  `dict(spec.launch_args, output_directory=directory)` where `directory`
  is the same redirected path stored on the spec — argv and reader share one
  source of truth by construction. The regression test asserts both
  `launch_args["output_directory"] == <root>/<trial_id>` and that
  `roslaunch_command(spec)` contains `output_directory:=<root>/<trial_id>`.
- Same-defect sweep across all output-directory consumers (grep audit):
  `stale_trial_ids`, `evaluate_stale_outputs` (rmtree), `JsonResultReader`,
  and `JsonClassificationWriter` all already derive from
  `spec.output_directory`; expansion derives the launch_args key from the
  identical string used for the spec attribute. No other hardcoded roots
  remain; the redirect now updates every consumer coherently.

## RED/GREEN Commands and Output

- Focused: `PYTHONDONTWRITEBYTECODE=1 python3 test/test_matrix_expansion.py`
  - RED: `Ran 37 tests ... FAILED` with the assertion mismatch above.
  - GREEN: **`Ran 37 tests ... OK`**.

## Modified Files (Round 3)

- `src/air_ground_experiments/scripts/run_experiment_matrix.py`
  (`apply_output_root` syncs `launch_args["output_directory"]`)
- `src/air_ground_experiments/test/test_matrix_expansion.py` (+1 regression test)
- `.superpowers/sdd/2026-08-25-gnss-denied-air-ground-registration/task-5-review.md`
  untouched; `.superpowers/sdd/.../task-5-report.md` (this section)

## Self-Review (Round 3)

- The reviewer-reproduced failure mode is closed at construction time rather
  than patched at use sites: there is exactly one place that mints a
  redirected spec, and it cannot produce divergent copies anymore.
- Frozen-root canonical workflow unchanged; original specs are never mutated.
- N1/M2–M6 untouched per instruction; no other behavior changed.

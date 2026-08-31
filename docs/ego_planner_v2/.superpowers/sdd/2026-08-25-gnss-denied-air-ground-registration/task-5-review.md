# Task 5 Review — One-Shot Baseline Matrix (fresh read-only review)

Reviewer: Task 5 fresh reviewer. Read-only; no code/tests/config/plans/reports were modified,
no subagents dispatched, no Git operations, no ROS/Gazebo/PX4/rosbag/runner processes executed.
Verification performed: full reads of brief/report/review package/scope files; direct reads of
producers (`frame_perturbation.launch`, `experiment_recorder.py`, `metrics.py`,
`takeoff_registration.py` excerpts, `registration.yaml`, `uav_sitl.launch`, `spawn_ugv.launch`,
`perception.launch`, mission/takeoff/controller scripts, both CMakeLists/package.xml/setup.py);
pure unittest runs; `py_compile`; XML/YAML parse; static regex/frame audits; independent
re-computation of the frozen matrix.

## Evidence Gathered (independent)

- `python3 -m unittest discover` in `air_ground_experiments`: **107 tests OK** (matches report).
- Bringup legacy suites: **14 tests OK** (matches report).
- `py_compile` on runner + test file: OK.
- Freeze-table reproduction: re-ran `expand_matrix(load(one_shot_matrix.yaml))` twice; equal;
  rows `one_shot-0000/-0009/-0029` match the report table digit-for-digit (including the
  `-240.776 → 119.224` wrap case). IDs sorted; seeds 1000..1029; YAML bounds == brief bounds.
- Static arg audit of `air_ground_inspection_experiment.launch`: top-level args parsed; every
  runner-emitted key declared **except** `drift_step_seconds`; `timeout_seconds` declared but
  never referenced anywhere in the launch body.
- roslaunch semantics checked statically in `/opt/ros/noetic/.../xmlloader.py`: unused-arg
  enforcement exists only for `<include>` child args; top-level CLI args that are undeclared are
  silently ignored.
- Frame-topology audit: red-sphere detector publishes camera-frame points (depth-camera fit);
  mission composes them with the (perturbed) experiment odom stream; UGV goal is relabeled into
  raw `ugv_0/odom` coordinates via the registration TF bridge (`ugv_goal_controller.py:45-48`
  looks up goal.frame_id → `ugv_0/odom`); UAV commands return to the raw side via the adapter
  inverse delta. Topology is coherent by design; registration error is the measured quantity.
- `/air_ground/registration/frozen` is a latched Bool published by
  `takeoff_registration.py:91,226` — `RosFrozenWatch` topic matches.
- Recorder JSON writes are atomic (`metrics.py:306-322`, temp + `os.replace`) — no partial-read
  race for `JsonResultReader`.
- Test-method count = 29 (matches report); no `.git` anywhere (report claim holds).

## Verdicts

- **Spec Compliance: PASS_WITH_FINDINGS** — all six brief files delivered; `expand_matrix`
  contract, fixture assertions, bounds equality, determinism, lifecycle shape, integration-launch
  wiring, manifests, smoke/full commands, acceptance text, and M1-C freeze are all present and
  statically verified. Two Important findings below breach the *freeze-fidelity* and *rerun
  workflow* contracts and must be reconciled before any dynamic M1-C execution.
- **Code Quality: PASS_WITH_FINDINGS** — clean decomposition, fully injected collaborators,
  mutation-sensitive behavioral tests, atomic writer assumptions hold. No Critical defects;
  several Minor duplication/robustness/test-depth issues.

Findings totals: **0 Critical / 2 Important / 6 Minor**.

## Findings

### Important

**I1. Frozen `timeout_seconds` never reaches the recorder; effective trial budget is the
recorder's 120 s default, contradicting the M1-C freeze.**
- Where: `src/air_ground_bringup/launch/air_ground_inspection_experiment.launch:13` (arg declared,
  zero references); `src/air_ground_experiments/launch/frame_perturbation.launch:68-74` (recorder
  node receives no `~timeout_seconds`); `experiment_recorder.py:45` (default `120.0`);
  `one_shot_matrix.yaml:11` (`timeout_seconds: 180.0`); freeze doc claims 180 s.
- Failure scenario: every trial ends at 120 **sim-time** seconds regardless of the frozen 180 s.
  Under Gazebo RTF < ~0.67 the runner's 180 s **wall-clock** backstop
  (`run_experiment_matrix.py:322-331`) fires first, classifying as TIMEOUT trials that the
  intended budget would have let complete — acceptance statistics would silently deviate from
  the frozen parameters (bias toward TIMEOUT; completion-rate distortion).
- Minimal fix: forward the value before M1-C — add an optional defaulted arg to
  `frame_perturbation.launch` feeding `<param name="timeout_seconds">` on the recorder
  (backward-compatible), or amend the frozen YAML/doc to the true 120 s budget. Whichever is
  chosen, update the freeze entry *before* any held-out seed inspection.

**I2. Stale-result reuse on reruns: `JsonResultReader` instantly consumes a previous run's
`<trial_id>.json`, and the documented smoke→full workflow collides on the same output root.**
- Where: `run_experiment_matrix.py:108-119` (reader returns any existing JSON),
  `run_experiment_matrix.py:472-500` (`main()` has no `--output-root` override or freshness
  guard), `one_shot_matrix.yaml:13` (fixed root), task-5-report.md §Explicitly Not Executed
  (smoke seeds 0000-0002 followed by full matrix into the same root).
- Failure scenario: after running the three smoke seeds, the full 30-seed run immediately reads
  the smoke-era JSON for trials 0000-0002, sends SIGINT, and records classifications without
  launching those trials — mixed-run provenance, and any environment fix between smoke and full
  runs leaves stale FAILED/TIMEOUT rows counted. Conversely `TrialResultWriter.write`
  (`metrics.py:302-305`) refuses existing paths, so a recorder-side rerun crashes mid-trial
  ("already finalized") while `JsonClassificationWriter` happily overwrites `exit.json`.
  Report's self-review claim "exactly-once semantics hold across reruns" is only true for a
  fresh root.
- Minimal fix: in `main()`/`run_trial`, preflight-refuse (or `--force`-clean) non-empty trial
  dirs when `<id>.json`/`exit.json` exist; add an optional `--output-root` override; document
  "fresh root per matrix invocation" in the M1-C commands.

### Minor

**M1. Runner emits an argument the launch neither declares nor forwards: `drift_step_seconds`.**
- Where: `run_experiment_matrix.py:224` emits `"drift_step_seconds": "1.0"`;
  `air_ground_inspection_experiment.launch:7-26` lacks the arg; roslaunch silently ignores
  undeclared top-level CLI args (verified in xmlloader.py), so perturbers get the value from
  `frame_perturbation.launch:8`'s default — coincidentally also 1.0 today.
- Scenario: either constant changes independently → silent divergence between frozen intent and
  runtime; also masks typos in future args.
- Fix: declare + forward `drift_step_seconds` in the experiment launch, or drop it from
  `launch_args` with a comment pointing at the include default.

**M2. Canonical statuses/failure codes/file-stem duplicated as literals despite the brief's
reuse directive.**
- Where: `run_experiment_matrix.py:60-64` (`REGISTRATION_FAILURE_CODES`), `:69-75`
  (`"COMPLETED"`/`"TIMEOUT"`/`"TRIAL_TIMEOUT"`), `:97-102` (`safe_trial_file_stem` mirroring
  `metrics.TrialResultWriter._safe_trial_id`); canonical constants live in
  `air_ground_experiments/src/air_ground_experiments/metrics.py:216-218`.
- Scenario: any upstream change to statuses, codes, or the stem rule makes the reader miss or
  misfile rows (mass TIMEOUTs) with no failing test, since the mirror is asserted against itself.
- Fix: import the constants/helper from `metrics` (it is rospy-free) or add a guard test asserting
  stem/status equality against the live module.

**M3. Kill-escalation path untested; a second `TimeoutError` escapes `_reap` and aborts the whole
matrix without classification.**
- Where: `run_experiment_matrix.py:363-369`; fakes at `test_matrix_expansion.py:237-243` never
  raise `TimeoutError`, so the SIGKILL branch has zero coverage; if the second `wait` also times
  out, the exception propagates out of `run_trial`/`run_matrix`, skipping all remaining trials.
- Fix: a fake whose first `wait` raises `TimeoutError` (assert kill called, trial recorded), plus
  wrap `_reap` to degrade to `teardown_clean: False` instead of raising.

**M4. Brief lifecycle step "request recorder flush" is wired to a no-op in real execution.**
- Where: `run_experiment_matrix.py:345-348` (callback invoked, `flush_requested=True`),
  `:495` (`flush_recorder=lambda spec: None`); recorder exposes no flush hook (grep: none in
  `experiment_recorder.py` / `frame_perturbation.launch`).
- Scenario: currently harmless — the recorder finalizes synchronously under a lock and the JSON
  appears atomically (`metrics.py:301-322`) — but the recorded flag implies a real handshake that
  does not exist, and a future buffered recorder would silently lose data.
- Fix: comment the no-op honestly, or implement a minimal ack (e.g., require the latched
  evaluation-status message) before claiming flush semantics.

**M5. `registration_mode` reaches the node as an unconsumed parameter (implementer-disclosed).**
- Where: `air_ground_inspection_experiment.launch:71`;
  `takeoff_registration.py` has no `get_param("~registration_mode")` (grep-verified).
- Scenario: mode switching is convention-only; if a later task adds batch mode defaults, the
  launch will appear wired while the node ignores it. Node is genuinely one-shot today
  (`OneShotRegistrationState`), so M1 behavior is correct.
- Fix (Task 7+): consume the param in the node, or log a warning when it != `one_shot`.

**M6. Report calls `TrialSpec` "immutable"; attributes are freely mutable.**
- Where: `run_experiment_matrix.py:144-172` (plain attribute assignment; no `__slots__`/
  `__setattr__` guard); task-5-report.md Implementation §1.
- Scenario: accidental mutation of `launch_args` between expansion and launch would corrupt argv;
  low likelihood, doc/behavior mismatch only.
- Fix: one-line doc correction now; optional `types.MappingProxyType`/read-only properties later.

## Residual Risks (dynamic scope unexecuted — per no-long-process ruling, not findings)

- Real-stack behaviors (PX4/mavros init order, Gazebo RTF, adapter↔auto-takeoff command
  interleaving, charuco gating delays) remain unconfirmed until M1 smoke runs.
- `RosFrozenWatch.wait` (`run_experiment_matrix.py:453-462`) can leak rospy exceptions if the
  master dies mid-FROZEN-wait — would crash the runner instead of yielding LAUNCH_FAILED.
- `SystemProcessManager.popen` uses no session/process-group isolation; kill escalation targets
  only the roslaunch pid, so orphaned grandchildren rely on the quiet-system flag plus the next
  trial's fail-fast precheck (acceptable, but expect cascading LAUNCH_FAILED after a dirty kill).
- `rosrun` resolution of the uninstalled script (disclosed concern #1) — external executor should
  invoke by path or register `catkin_install_python` in a later task.
- Pre-existing rostest hang (`test_registration_node.py` without a master) — environmental,
  correctly documented; excluded here as well.

## Checks That Passed (highlights)

- Determinism: instance-local PCG64, fixed draw order, rank-based ids, id-sorted output, closed-
  bound sampling, wrap-at-±180° consistency; independent recomputation reproduced the frozen
  table exactly; no global RNG anywhere in the module.
- Lifecycle state machine: precheck → spawn → FROZEN-or-timeout → terminal row/early-exit/deadline
  → flush → SIGINT → bounded reap → quiet re-check → classified write; all five exit codes with
  brief-consistent triggers; reader-before-poll ordering means a just-finalized row wins the race
  against a required-node exit.
- Launch wiring: perturbation include sits between raw sources and research nodes with all eight
  args identity-forwarded; commands routed experiment→adapter→raw; visual-yaw + one_shot args
  threaded; truth confined to the include (recorder-only consumers, incl. `/gazebo/model_states`);
  UGV spawn at the body-relative registration waypoint; mission params byte-match the legacy
  final-demo values (red sphere −17.6, pitch 25°, altitudes 4.0/3.0/2.3/4.0, standoff 3.5).
- Manifests: minimal dependency-only additions; `install(DIRECTORY … launch)` already covers the
  new launch; bounded catkin build reported green and consistent with the added build_depends.
- Tests: behavior-driven (real parsing, real arithmetic, argv/document assertions), no
  constant-parroting fake greens detected; RED→GREEN evidence plausible and specific; suite runs
  green in this environment (107 + 14 re-executed by reviewer).

---

# Re-review — Review Fix Round 1 (fresh read-only)

Scope: verify I1/I2 dispositions only; check the fix diff for new Critical/Important.
No code/tests/config/reports modified; no subagents; no Git; no ROS/Gazebo/PX4/rosbag
processes; the runner was not executed. Verification performed: full reads of the four
changed files + updated report; pure unittest runs; py_compile; XML/YAML parse; a static
argv-vs-launch audit of the emitted roslaunch command.

## Evidence Gathered (independent)

- `air_ground_experiments` suite: **112 tests OK** (matches report). Bringup legacy: **14 OK**.
- Coordinate-transform pure regressions (direct-file invocation): **13+13+8 = 34 OK**
  (reviewer's initial `unittest discover` attempts failed purely on path/import mechanics;
  direct execution confirms green — matches report).
- `py_compile` runner+test OK; both changed launch files parse as XML; YAML unchanged (180.0).
- Static forwarding-chain audit for `timeout_seconds`:
  runner `_trial_launch_args` emits `timeout_seconds` (run_experiment_matrix.py:282)
  → experiment launch declares `timeout_seconds` default `180.0` (:13) and forwards it into
  the include (:59)
  → `frame_perturbation.launch` declares `<arg name="timeout_seconds" default="120.0"/>` (:14,
  preserving prior Task-4 behavior) and feeds `<param name="timeout_seconds"
  value="$(arg timeout_seconds)"/>` to the recorder (:76). Chain complete; frozen YAML stays
  180.0; include-child override correctly supersedes the 120.0 default at runtime.
- New I1 test (`test_matrix_expansion.py:765-785`) asserts all three links (default, param
  wiring, include forwarding) — mutation-sensitive by construction.
- I2 behavior: `stale_trial_ids` (:105-120) flags dirs containing `<safe_id>.json` or
  `exit.json`; `evaluate_stale_outputs` (:123-141) returns ok/conflict/cleared, rmtree limited
  to stale specs' own output directories under `force`; `apply_output_root` (:144-158) returns
  redirected copies; `main()` applies `--output-root` **before** the staleness check
  (:550-553), then refuses with guidance and exit 3 (:553-563) before constructing the runner
  or spawning anything. Fresh-root-per-invocation documented in the report's M1-C commands.
- New I2 tests drive real filesystem behavior through tempdirs (both marker types, force-clear
  incl. dir-gone + subsequent-ok, redirect without touching frozen roots) — genuinely
  behavioral, not mock self-proof. `main()` itself is only statically audited
  (`test_main_glue_wires_freshness_guard_and_output_override`, :566-573) — acceptable under the
  no-execution ruling; behavioral `main()` coverage remains a dynamic-stage item.
- Regression impact of the diff on Task-4 launch/packaging: `test_frame_perturbation.py`,
  `test_metrics.py`, adapter/script suites all inside the 112 OK; bringup 14 OK; XML
  well-formedness confirmed (the round's transient `</node>` breakage is repaired).

## Dispositions

- **I1: ADDRESSED** — complete runner→experiment-launch→include→recorder-param chain verified
  statically; default 120.0 preserves Task-4 behavior; frozen YAML remains 180.0; the guarding
  test is mutation-sensitive across all three links.
- **I2: ADDRESSED at the runner layer** — preflight/refuse(exit 3)/`--force` whole-directory
  clear/`--output-root`/M1-C fresh-root docs all present, ordered correctly, and behaviorally
  tested. However its end-to-end guarantee is currently voided by C1 below: redirected
  `spec.output_directory` never reaches the recorder, so even a fresh overridden root cannot
  receive rows.

## New Findings

### Critical

**C1. Runner argv omits `output_directory:=`; the launch's default pins every recorder to
`one_shot-0000`'s directory — the matrix cannot produce valid rows at runtime.**
(Pre-existing since round 0 — missed by this reviewer in the first review; not introduced by
the fix diff, but it defeats both I2's redirect and I1's delivered budget end-to-end.)
- Where: `run_experiment_matrix.py:255-283` (`_trial_launch_args` has no `output_directory`
  key; `roslaunch_command` :313-322 forwards only `launch_args`);
  `air_ground_inspection_experiment.launch:11-12` (`output_directory` default hardcodes
  `/tmp/air_ground_experiments/matrix_one_shot/one_shot-0000`).
- Failure scenario (verified statically): reviewer-generated argv for `one_shot-0007` contains
  no `output_directory:=`. For every trial the recorder writes `trials.csv`/`<id>.json` into
  `…/one_shot-0000/`, while `JsonResultReader` polls `…/<actual_id>/` forever → each trial
  burns the full 180 s wall budget and classifies TIMEOUT; with `--output-root` nothing changes
  recorder-side; smoke seeds 0001+ likewise misdirected. No test covers per-trial
  `output_directory` in argv (B8 asserts only `trial_id:`/`registration_mode:`), so the suite
  stayed green.
- Minimal fix: emit `"output_directory": repr(float-free str(spec.output_directory))` i.e.
  `"output_directory": str(spec.output_directory)` in `expand_matrix`/`_trial_launch_args`
  (value available at spec construction), plus a B8-style assertion that every spawned argv
  carries `output_directory:=<spec dir>`; add the key to the I1-style static chain audit.

### Minor

**N1. `--force` uses `shutil.rmtree(..., ignore_errors=True)`; silent deletion failure still
returns `"cleared"` and proceeds onto stale rows.**
- Where: `run_experiment_matrix.py:140`.
- Failure scenario: EPERM/EBUSY leaves `<id>.json` in place; `evaluate_stale_outputs` reports
  `cleared`; the run then re-reads the stale row — the exact provenance hazard I2 was meant to
  kill, reintroduced on an error path.
- Fix: drop `ignore_errors` (let it raise) or re-check markers after removal and return
  `conflict` if any survive.

No other new Critical/Important introduced by the diff; deferred Minors M1–M6 untouched as
scoped.

## Verdicts (Round 1)

- I1 disposition: ACCEPTED/FIXED, verified. I2 disposition: ACCEPTED/FIXED at the runner layer.
- Fix diff introduces **0 new Critical/Important**; re-review records **1 Critical (C1,
  pre-existing, blocks all dynamic value)** and **1 Minor (N1)**.
- Round verdict: **FIXES VERIFIED — CONDITIONAL PASS; C1 must be fixed (with an argv-level
  regression test) before any M1-C dynamic execution.**

---

# Re-review — Review Fix Round 2 (fresh read-only)

Scope: verify C1 disposition only; scan the fix diff for new Critical/Important
(`drift_step_seconds` chain, glue-audit assertion direction, Task-4 regression impact).
Read-only; no subagents; no Git; no ROS/Gazebo/rosbag/runner processes. Verification:
full reads of changed files + Round-2 report section; pure unittest runs; py_compile;
XML/YAML parse; independent programmatic argv audit incl. the `--output-root` path.

## Evidence Gathered (independent)

- `air_ground_experiments`: **114 tests OK** (112 + 2 new, matches report's focused 36);
  bringup legacy **14 OK**; `py_compile` OK; both launches XML-parse; YAML unchanged
  (`timeout_seconds: 180.0`, 30 seeds). Determinism unaffected: re-expanded matrix equals a
  rerun field-for-field and frozen row `one_shot-0000 uav_x = -2.9572285…` reproduces exactly.
- C1 core fix verified in code: `_trial_launch_args(..., output_directory)` now emits
  `"output_directory": str(output_directory)` (run_experiment_matrix.py:246,260) fed from the
  single per-trial path computed in `expand_matrix` (:298-309) that also populates
  `TrialSpec.output_directory` — one source of truth; name matches the launch declaration
  (`air_ground_inspection_experiment.launch:11`). New argv test
  (`test_matrix_expansion.py:501-520`) drives the real state machine through `run_matrix` +
  fakes and asserts `output_directory:=<spec.output_directory>` per spawned trial — genuinely
  behavioral and mutation-sensitive (removal/rename/misdirection all fail it). RED evidence
  (specific missing-assignment message, 35→FAILED→OK counts) is internally consistent;
  history not replayable, accepted on stated output.
- Glue audit (`:786-797`): direction emitted ⊆ declared is reasonable — declared-but-unemitted
  args are legitimate manual-smoke defaults (e.g., `red_sphere_*`), so bidirectional equality
  would over-constrain; rationale documented in the report. RED proven live against pre-fix
  code flagging exactly `['drift_step_seconds']`. Limitation (accepted): name-subset only,
  single trial's keys (all trials share keys); value-wiring stays covered by the other audits.
- `drift_step_seconds` chain: experiment launch declares default `1.0` (:27) and forwards into
  the include (:57) whose own arg already exists with matching name and identical default
  (`frame_perturbation.launch:8`); runner constant `"1.0"` consistent across all three. Value
  semantics unchanged. Transparently recorded as resolving deferred M1's substance — no scope
  violation (it was the mandated consistency check's direct consequence).
- Task-4 regression impact: none — experiments suites (incl. frame-perturbation/metrics/
  adapter/recorder) green at 114, bringup 14 green, both launches well-formed.

## Disposition

### C1: PARTIALLY ADDRESSED

Fixed and properly tested for the canonical config/frozen-root path: every trial's argv now
carries its own directory, restoring recorder↔reader convergence there (independently
reproduced: argv contains `output_directory:=<spec dir>`).

**However** — new Important introduced by this diff's interaction with the Round-1 override:

### Important (new)

**IA1. `apply_output_root` redirects `spec.output_directory` but not the newly-emitted
`launch_args["output_directory"]` — `--output-root` runs desynchronize argv from the reader.**
- Where: `run_experiment_matrix.py:144-158` (passes `spec.launch_args` through untouched)
  vs `:260` (argv now carries the stale frozen-root path).
- Failure scenario (reviewer-reproduced): after `--output-root /tmp/fresh_root`,
  spec polls `/tmp/fresh_root/<id>` while roslaunch receives
  `output_directory:=/tmp/air_ground_experiments/matrix_one_shot/<id>` → recorder writes the
  frozen tree, reader never sees rows → whole overridden matrix classifies TIMEOUT; if the
  frozen root still held stale rows, the recorder additionally raises "already finalized"
  mid-run. This voids the first remedy the refusal message itself recommends
  (`main()` guidance: "Use a fresh root per matrix invocation (--output-root) …") and
  contradicts Round-2 Self-Review's claim that "redirected roots now actually reach the
  recorder". Rated Important rather than Critical because the frozen-root path (--force or
  cleaned root) — the freeze-documented acceptance workflow — is fully functional, and the
  failure mode yields detectable TIMEOUTs, never false PASSes; it borders Critical due to the
  guided-path nature.
- Minimal fix: in `apply_output_root`, rebuild args as
  `dict(spec.launch_args, output_directory=str(Path(root) / spec.trial_id))`, and extend the
  argv test (or add one) asserting `roslaunch_command(apply_output_root(specs, root)[i])`
  carries the redirected path.

Deferred N1 and M2–M6 remain open per instruction; M1 substance closed transparently.

## Verdicts (Round 2)

- C1 disposition: **PARTIALLY ADDRESSED** (canonical path fixed + behaviorally guarded;
  `--output-root` path regressed/desynced — IA1 must land before M1-C).
- Fix diff introduces: **0 Critical / 1 Important (IA1)**.
- Round verdict: **CONDITIONAL PASS — one focused follow-up required.** Bounded-scope Task 5
  may close only after IA1 is fixed and its regression test is green; dynamic M1-C execution
  remains external regardless.

---

# Re-review — Review Fix Round 3 (fresh read-only)

Scope: verify IA1 only; scan the fix diff for new Critical/Important. Read-only;
no subagents; no Git; no ROS/Gazebo/rosbag processes; runner never executed.
Verification: reads of changed files + Round-3 report section; pure unittest runs;
py_compile; XML/YAML parse; an independent end-to-end consistency probe of the
redirected-spec path (argv vs reader vs exit-writer vs stale-preflight vs originals).

## Evidence Gathered (independent)

- `apply_output_root` (:144-160) now mints each redirected spec from a single computed
  `directory`, used both as `dict(spec.launch_args, output_directory=directory)` and as
  `TrialSpec.output_directory` — argv and every consumer share one source by construction,
  at the only site that creates redirected specs. `dict()` copy plus TrialSpec's own copy
  means original specs are never aliased or mutated (repro-confirmed).
- Same-defect sweep credible: grep audit shows every consumer derives from
  `spec.output_directory` (`JsonClassificationWriter.write`:88, `stale_trial_ids`:111,
  `evaluate_stale_outputs` rmtree:140, `JsonResultReader`:172); expansion mints
  `launch_args["output_directory"]` from the identical string as the spec attribute
  (:263 vs :301-312); no hardcoded roots remain in the runner.
- New regression test (`test_matrix_expansion.py:672-690`) drives real `roslaunch_command`
  on redirected specs and asserts BOTH `launch_args["output_directory"] == <root>/<id>`
  and the argv token `output_directory:=<root>/<id>` — mutation-sensitive in either
  direction (reverting the fix or desyncing either layer fails it). RED evidence (specific
  frozen-vs-tempdir mismatch, 37→FAILED→OK) internally consistent with the diff.
- Reviewer repro beyond the test: for redirected specs — argv token present;
  `JsonResultReader().read` clean-None on fresh dirs; `JsonClassificationWriter` lands
  `exit.json` inside the redirected dir; `stale_trial_ids` flags markers under the redirect
  only; original specs byte-identical before/after (`OVERRIDE_PATH_CONSISTENT`).
- Suites: experiments **115 tests OK** (114+1, matches report's focused 37); bringup legacy
  re-verified verbose: exactly **14 tests OK** (an earlier combined-command run of mine
  reported a stray 43-test discovery from a wrong working directory — invocation artifact,
  not a product change; authoritative verbose run shown here); `py_compile` OK; both launch
  files XML-parse; YAML unchanged (180.0, 30 seeds); determinism intact (reruns equal,
  frozen row `one_shot-0000 uav_x` reproduces exactly).

## Disposition

### IA1: ADDRESSED

Construction-time single-source fix at the only mint site; regression test behavioral and
mutation-sensitive; independent probe confirms argv/reader/writer/preflight coherence on the
override path with originals untouched.

## New Critical/Important scan

Fix diff (runner `apply_output_root` + 1 test) introduces **0 new Critical/Important**.
Deferred N1 (`rmtree ignore_errors`) and M2–M6 remain open, explicitly per instruction.

## Verdicts (Round 3)

- IA1 disposition: **ACCEPTED/FIXED, verified**.
- Fix diff introduces: **0 Critical / 0 Important**.
- Verdict: **PASS — Task 5 may close in bounded scope.** All Critical/Important findings from
  the initial review and fix rounds 1–2 (I1, I2, C1, IA1) are resolved and guarded by
  mutation-sensitive tests; remaining opens are the six+one explicitly deferred Minors and the
  externally-owned M1-C dynamic execution (smoke seeds, 30-seed matrix, acceptance evaluation),
  which stays outside this environment by ruling.

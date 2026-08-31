# SDD ledger — plan: docs/superpowers/plans/2026-08-25-gnss-denied-air-ground-registration.md

## Preflight

- Baseline: Python compilation passed for `takeoff_registration.py`, `uav_sphere_mission.py`, and `ugv_goal_controller.py`.
- Baseline: `catkin_make --pkg air_ground_coordinate_transform air_ground_bringup` passed.
- Ruling: execute in the current workspace because the project has no Git metadata and neither a worktree nor task-range commits can exist. Do not initialize Git. Per-task reports plus task-scoped read-only reviews of all changed files replace Git diff packages. Cost if wrong: reviewers see full current files rather than exact hunks and must rely on the report's changed-file list to constrain scope.
- Ruling: no implementation task may use Gazebo truth except files under `air_ground_experiments` that are evaluation-only. Cost if wrong: any truth leak invalidates the experiment and blocks the milestone.
- Ruling: after the Task 2 dispatch blocked in a foreground launch, no agent may execute `roslaunch`, `roscore`, `rostest`, Gazebo, PX4 SITL, RViz, rosbag recording, or long topic-wait loops in this OpenCode environment. Dynamic acceptance commands remain documented for external/manual execution; local gates use pure unit tests, Python/XML checks, and bounded catkin builds. Cost if wrong: dynamic milestone thresholds cannot be claimed from this environment and remain explicitly pending external evidence.
- Debugging root cause: Task 2 production/test files were created, no report was written, and no ROS/Gazebo/PX4 process remained after tool abort. The dispatch brief required a foreground cold-start launch, so the implementation agent reached a process that intentionally does not terminate. Recovery uses a fresh implementer, preserves partial files, and forbids all long-running launch validation.

## Plan Conflict And Interface Scan

| Scope | Producer / requested change | Consumer / test | Finding and ruling |
|---|---|---|---|
| Task 1 | `se2.py`, `odom_buffer.py`, package setup | SE(2) and interpolation unit tests | Internally consistent. `OdomBuffer` numeric `append` plus ROS `append_odometry` is authoritative. |
| Task 2 | Robust batch estimator and one-shot ROS adapter | Unit test, rostest, legacy Demo | Internally consistent. Legacy topics remain and new covariance/revision topics are additive. |
| Task 3 | Body-relative registration waypoint and spawn yaw | Three cold-start geometry cases | Internally consistent. Compatibility launch retains fixed yaw; research launch later enables visual yaw. |
| Task 4 | Experiment-only frame perturbation, observation gate, recorder | Determinism and metric tests | Internally consistent after adding the observation gate. Recorder is read-only with respect to autonomy. |
| Task 5 | One-shot matrix runner and integration launch | 30-seed M1 acceptance | Internally consistent. Physical UGV remains at the configured registration geometry; independent odom offsets test frame registration. |
| Task 6 | Random-walk SE(2) filter | Seeded filter tests | Internally consistent. This is empirical relative-drift estimation, not SLAM. |
| Task 7 | Repeated windows, periodic/opportunistic revisions | Node rostest and visibility schedule | Internally consistent after explicitly adding prediction from elapsed time and odometric travel. |
| Task 8 | Uncertainty policy and UAV return-to-UGV states | Policy and state-transition tests | Ruling: a post-target re-registration does not erase target-sensing error. Preserve the target but allow Task 9 policy to require `REOBSERVE`; do not claim registration alone corrects stale UAV target estimates. Cost if wrong: handoff covariance can be overconfident. |
| Task 9 | UAV anomaly covariance and common-frame goal | Handoff unit/integration tests | Internally consistent. Origin-target covariance is sensing/pose only; executable-goal covariance additionally includes registration uncertainty. |
| Task 10 | Common-frame goal tracker | Dynamic revision and jump tests | Internally consistent. Common goal remains immutable while resolved UGV odom goal changes by revision. |
| Task 11 | UGV camera confirmation and mission completion | Image tests and relay rostest | Internally consistent. `ARRIVED` alone never completes the task. |
| Task 12 | Frozen matrices, direct CSV/JSON metrics | Aggregate tests and complete suite | Internally consistent. Oracle is offline evaluation only and never publishes autonomy inputs. |
| Tasks 1→2 | SE(2), covariance, odom interpolation | Robust estimator and ROS adapter | Interfaces agree; Task 2 must import Task 1 modules rather than duplicate functions. |
| Tasks 2↔3 | Registration config/launch and legacy policy | Compatibility vs research yaw policy | No conflict: fixed yaw is explicit only in legacy; research uses visual yaw. |
| Tasks 2→7 | One-shot estimator topics and revision 1 | Online repeated revisions | Additive evolution; `/frozen` changes semantics to initialized but stays true after first estimate. |
| Tasks 3→5 | Spawn geometry and registration waypoint | Seeded integration launch | Interfaces agree; runner supplies explicit launch arguments. |
| Tasks 3→8 | Mission waypoint/state code | Return-to-UGV states | Shared file is sequential; Task 8 must retain Task 3 body-relative geometry helper. |
| Tasks 4→5 | Experiment package and metrics | Matrix runner | Interfaces agree through `TrialSpec`, perturbation topics, and result schema. |
| Tasks 4→12 | `metrics.py`, observation gate, recorder | Final disturbance matrix | Sequential extension; Task 12 must preserve seed determinism and truth isolation. |
| Tasks 5→11 | Inspection integration launch | UGV inspector integration | Sequential extension; Task 11 adds the inspector without bypassing existing experiment routing. |
| Tasks 5→12 | Trial runner | Full matrices and resume | Sequential extension; Task 12 adds grouping and failure retention. |
| Tasks 6→7 | `RegistrationFilter` | ROS prediction/update lifecycle | Interfaces agree on `FilterState` and `UpdateResult`. |
| Tasks 7→8 | Estimate covariance and revision | Re-registration policy/state transition | Interfaces agree; mission waits for revision increment before dispatch. |
| Tasks 7→10 | Registration revision and TF | Dynamic goal resolution | Interfaces agree; exactly one registration TF broadcaster remains. |
| Tasks 8→9 | `target_handoff.py` and mission state | Full covariance handoff node | Task 8 creates policy core; Task 9 extends it without changing action names. |
| Tasks 8↔11 | Mission state machine | Inspection completion states | Sequential and compatible; re-registration occurs before dispatch, inspection after arrival. |
| Tasks 9→10 | Common-frame `inspection_goal` | Goal tracker | Exact topic and message type agree: `PoseWithCovarianceStamped` in `air_ground_origin`. |
| Tasks 9↔11 | Mission and relay rostest | Close inspection | Sequential; Task 11 must preserve Task 9 target publication. |
| Tasks 10↔11 | `inspection_relay.test` and controller | Camera-confirmed terminal state | Shared test intentionally accumulates integration coverage. |
| Tasks 11→12 | `INSPECTION_CONFIRMED` terminal state | Trial result classification | Exact terminal state is the M3/M4 success criterion. |

## Task Progress

- Task 1: fix round 1/5 (6 addressed, 0 open — NumPy dependency; compose direction test; full covariance test; following bracket test; ROS import isolation; maxlen/non-exact distance coverage; no commits because workspace is unversioned)
- Task 1: complete (review clean after fix round 1; 20 focused tests pass; package build passes)
- Task 2: recovery after aborted foreground launch (original RED chronology unavailable; recovery RED/GREEN and mutation evidence recorded)
- Task 2: fix round 1/5 (6 Important and 2 Minor addressed; full 3-D interpolation/chain, covariance propagation, fixed-yaw consistency, frame validation, atomic freeze, direct tf dependency, stronger tests/report)
- Task 2: fix round 2/5 (3 Minor addressed; validator mutation sensitivity, independent covariance assertions, legacy frame-parameter fallback, report correction)
- Task 2: complete (bounded-scope final review clean; 34 focused pure tests pass; package build passes; dynamic ROS evidence pending externally)
- M0 bounded verification: fresh 34-test run passed, Python compilation passed, 4 XML files parsed, and `catkin_make --pkg air_ground_coordinate_transform air_ground_bringup` passed. M0-A/M0-B rostest, launch-resolution, TF timing, and cold-start compatibility evidence remain pending external execution under the no-long-process ruling; no dynamic pass is claimed.
- Task 3: minor (deferred): static launch test proves UGV yaw only to the include boundary, not through `spawn_ugv.launch` leaf `-Y` wiring.
- Task 3: minor (deferred): fixed-yaw static test does not assert node-local override occurs after YAML load.
- Task 3: minor (deferred): Task 3 pure tests are not registered with catkin and `python3-yaml` test dependency is undeclared.
- Task 3: fix round 1/5 (1 Important addressed, 0 Important open — rotated-waypoint reach transitions now mutation-sensitive; no commits because workspace is unversioned)
- Task 3: complete (spec and code-quality review pass; 14 Task 3 pure tests and 34 Task 2 regressions pass; bounded package build passes; 3 deferred Minor; dynamic M1-A evidence pending externally)
- Task 4: minor (deferred): position-command adapter does not validate an explicit source frame before inverse transformation.
- Task 4: minor (deferred): package manifest does not explicitly declare `python3-setuptools` build metadata and NumPy test-stage metadata.
- Task 4: fix round 1/5 (9 of 10 original ADDRESSED, C2 partial; fix diff introduced NC1 Critical + NI1/NI2 Important + 3 Minor)
- Task 4: minor (deferred): new test modules not registered under catkin testing; round-0 report bullet stale.
- Task 4: minor (deferred): legacy mission terminal `COMPLETE` classifies PENDING; document or map for research scope.
- Task 4: minor (deferred): epoch hardening defaults missing (`epoch_seconds=0.0`, no `maximum_elapsed_seconds`) plus small nits.
- Task 4: fix round 2/5 (NC1/NI1/NI2 addressed; 0 new Critical/Important; 1 new Minor NM4)
- Task 4: minor (deferred): anchor parameters not wired into launch, defaults inverted vs research config, stale docstring.
- Task 4: complete (spec and code-quality review pass after fix round 2; 78 pure/static tests pass; Tasks 1-3 regressions 34+14 pass; bounded package build passes; dynamic M1-B pending externally)
- Task 5: minor (deferred): runner emits `drift_step_seconds` that the experiment launch neither declares nor forwards; silent divergence risk if constants change independently.
- Task 5: minor (deferred): runner duplicates canonical statuses/failure codes/file-stem as literals instead of importing from `metrics`.
- Task 5: minor (deferred): kill-escalation path untested; a second `TimeoutError` in `_reap` would abort the matrix without classification.
- Task 5: minor (deferred): "request recorder flush" is wired to an undocumented no-op; needs honest comment or minimal ack.
- Task 5: minor (deferred): `registration_mode` reaches the node as an unconsumed parameter; consume or warn in Task 7+.
- Task 5: minor (deferred): report calls `TrialSpec` immutable while attributes are mutable; doc correction needed.
- Task 5: minor (deferred): `--force` rmtree uses ignore_errors and still reports cleared on silent failure.
- Task 5: fix round 1/5 (I1 timeout forwarding chain and I2 stale-output preflight addressed; re-review found pre-existing Critical C1 missed in round 0)
- Task 5: minor (deferred) update: deferred M1 (`drift_step_seconds`) substantively resolved during fix round 2 via glue-audit test; remaining deferred minors are N1/M2-M6.
- Task 5: fix round 2/5 (pre-existing Critical C1 output_directory argv fixed; glue audit also closed deferred M1)
- Task 5: fix round 3/5 (IA1 --output-root argv/reader divergence fixed; single source of truth restored)
- Task 5: complete (spec and code-quality review pass; experiments 115 + bringup 14 + coordinate-transform 34 tests pass; bounded package build passes; dynamic M1-C smoke/full matrix execution pending externally)
- Task 6: Ruling: represent empirical process noise with six nonnegative scalar variance-growth rates: isotropic XY translation and yaw, each split by elapsed time, UAV travel, and UGV travel. This follows the plan's separate translation/yaw requirement while keeping units calibratable; cost if wrong: anisotropic drift would require extending the frozen parameterization before Task 7 experiments.
- Task 6: Ruling: use the 99% chi-square threshold for three innovation degrees of freedom, `11.344866730144373`, as the default statistical gate. Calibrate with held-out NIS coverage rather than outcome tuning; cost if wrong: non-Gaussian residuals may require a predeclared robust calibration before M2 trials.
- Task 6: Ruling: allow `(initial_mean, initial_covariance) == (None, None)` to represent an uninitialized filter so Task 7 can initialize from its first valid batch at revision 1; initialized construction starts at revision 1 and later accepted batches increment it. Cost if wrong: Task 7 integration semantics would need a separate initialization API.
- Task 6: minor (deferred): supplied prior yaw is not canonicalized to `[-pi, pi)` at construction.
- Task 6: minor (deferred): finite but extreme prediction products can overflow and commit nonfinite covariance.
- Task 6: minor (deferred): covariance symmetry/PSD tolerances are absolute rather than scale-aware for mixed-unit states.
- Task 6: minor (deferred): YAML numerical rates are not explicitly labeled provisional/not field-calibrated with dataset/version/date metadata requirements.
- Task 6: fix round 1/5 (I1 state snapshots, I2 causal stamps, I4 edge coverage, I5 auditable TDD reset addressed; I3 partial with one innovation-overflow rejection path open; prediction-overflow portion of deferred M2 resolved)
- Task 6: fix round 2/5 (innovation-overflow rejection path addressed; 0 new Critical/Important; deferred M2 fully resolved)
- Task 6: complete (spec and code-quality review pass; focused 41 and full pure 62 tests pass; seeded Monte Carlo 99/100 improved; bounded package build passes; real coefficient calibration and Task 7 ROS integration remain pending)
- Task 7: Ruling: one visual frame can only append one candidate sample; a revision is created only by one accepted robust-window batch. Every consumed accepted/rejected batch clears its window so frame count and registration-event count cannot alias. Cost if wrong: overlapping windows could correlate updates or inflate revisions.
- Task 7: Ruling: preserve legacy one-shot status `FROZEN` and revision 1, while periodic/opportunistic modes use `TRACKING`, `DEGRADED`, `UPDATING`, and `REJECTED`; `/frozen=True` means initialized in all modes. Cost if wrong: changing one-shot status would break the compatibility baseline.
- Task 7: Ruling: odometry prediction uses each vehicle's monotonic incremental distance and a single global monotonic prediction timestamp; callback-order lag within the configured odometry bracket may be coalesced to current filter time, while older batches are rejected as `stale_batch`. Cost if wrong: a larger real observation delay may require fixed-lag replay rather than bounded coalescing before M2 experiments.
- Task 7: Ruling: exact rejection reasons remain the filter/window result and are emitted in throttled logs while `/status` remains the stable coarse state `REJECTED`; no new control-facing reason topic is added. Cost if wrong: external tooling needing machine-readable reasons would require an additive diagnostic topic later.
- Task 7: minor (deferred): initial pure/stub tests were helper-focused and missed research launch frame contracts, monitor callback ordering, and node publication sequencing; required Critical/Important fixes add scoped integration coverage, but final broad review should reassess remaining adapter gaps.
- Task 7: minor (deferred): the four auxiliary registration nodes in `registration_node.test` do not remap the latched `/air_ground/registration/accepted_update` output, so future late-subscriber assertions could receive a latch from the wrong fixture; production single-node launches are unaffected.
- Task 7: complete after two review-fix rounds. Final independent verdict: Spec Compliance PASS, no Critical/Important, one deferred Minor. Coordinator fresh evidence: 113/113 bounded pure/stub/real-message tests passed, `py_compile` passed, five XML plus one YAML parsed, and bounded two-package catkin build exited 0. Written rostest and dynamic M2-B remain external-only under the no-long-process ruling.
- Task 8: preflight shared-interface scan: `target_handoff.py` pure policy is consumed by `uav_sphere_mission.py`; mission registration inputs consume Task 7 `RegistrationUpdate` revision and continuous legacy estimate covariance; research/final launch parameters feed mission defaults; Task 9 later extends target covariance and must not be implemented now. No file/interface conflict after the constructor ruling below.
- Task 8: Ruling: extend the planned three-input `UncertaintyBudget` constructor with required `inspection_yaw` because Task 8 simultaneously requires explicit meter and radian thresholds. Cost if wrong: callers expecting the incomplete three-argument sketch must provide the fourth explicit safety threshold.
- Task 8: Ruling: uncertainty-aware handoff is opt-in and uses `registration_mode=opportunistic`; the final-demo and research-launch defaults remain disabled. Cost if wrong: manual M2-C must pass one extra launch argument, but no legacy mission silently changes behavior.
- Task 8: Ruling: until Task 9 supplies unbiased target covariance, preserve an isotropic covariance from `max(final_spread, 0.02 m)^2`; Task 9 remains owner of sensor/sample covariance. Cost if wrong: M2-C action choice uses a provisional conservative target model and must be reported as such.
- Task 8: round-0 review found 1 Critical and 7 Important: research mission experimental-frame mismatch; stale HOLD policy; WAIT missing-data advance; deadline precedence; callback/timer and selected-window races; numerical PSD false rejection; missing `tf` dependency; and insufficient full-constructor integration coverage. All enter fix round 1.
- Task 8: minor (deferred): CMake registers the two new Task 8 suites but not the modified launch-wiring and legacy waypoint regressions, so package-level `run_tests` can miss those manually run suites.
- Task 8: fix round 1/5 (C1 and I1-I3/I6-I7 addressed; I4/I5 substantially addressed; 2 Important open: preserve stable final before disagreement-error transition, and reset all public confidence properties after a late numerical failure; M1 remains deferred).
- Task 8: fix round 2/5 (stable final preservation ordering addressed; staged validation/radius/yaw failures addressed; 1 Important open: exception from final finite-check operation still escapes instead of yielding HOLD plus four NaNs; M1 remains deferred).
- Task 8: fix round 3/5 (final finite-check exception boundary addressed; 0 Critical/Important open; one CMake test-discovery Minor remains deferred).
- Task 8: complete (independent Spec Compliance PASS and Code Quality PASS with one deferred Minor; coordinator fresh evidence: Task 8/Task 3 55/55, Task 7 bounded regressions 34/34, `py_compile`, three XML parses, static no-truth/no-header-seq/no-broadcaster/one-DISPATCH-call audit, and bounded bringup build all passed). Dynamic M2-C remains external-only under the no-long-process ruling. Stop before Task 9.

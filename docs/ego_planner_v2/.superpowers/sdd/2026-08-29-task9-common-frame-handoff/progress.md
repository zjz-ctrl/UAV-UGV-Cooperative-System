# SDD ledger — plan: docs/superpowers/plans/2026-08-29-task9-common-frame-handoff.md

## Preflight

- Baseline: bringup Task 8 policy/mission tests passed 35/35.
- Baseline: coordinate-transform adapter/serialization tests passed 12/12 after sourcing `devel/setup.bash`.
- Ruling: execute in the current workspace because it has no Git metadata and the existing project ledger forbids initializing Git. Reviews use task-scoped changed-file reads and reports instead of commit diffs. Cost if wrong: reviewers must reconstruct exact changes from current files and report manifests.
- Ruling: the target remains the planned `PoseWithCovarianceStamped` and is registration-independent in UAV odom. Atomicity means one relay lock snapshot freezes one target generation plus a continuous `RegistrationUpdate` and matching accepted `RegistrationUpdate`; the two registration revisions must match. Cost if wrong: a transport-level target revision would require changing the original message interface or adding a custom message.
- Ruling: use the registration transform translation as the Task 9 standoff anchor, matching the original plan's "registration anchor" and the fact that the UGV remains stationary before dispatch. Cost if wrong: a mission that moves the UGV before handoff would need a fresh UGV-pose input before Task 10.
- Ruling: target sample scatter and the mean projected UAV pose covariance are added once; front range variance is an additional sensing-axis term. Registration covariance is not included in this product. Cost if wrong: sample scatter may already contain some empirical pose jitter, making the sensing covariance conservative.
- Ruling: the goal sensing covariance is the origin target covariance propagated through the standoff Jacobian; the registration-only covariance is projected through the lever arm and added once. `UncertaintyBudget` receives those two separate products. Cost if wrong: first-order linearization can be inaccurate for very large uncertainty or near-zero target-anchor distance, which is rejected.

## Plan Conflict And Interface Scan

| Scope | Producer / requested change | Consumer / test | Finding and ruling |
|---|---|---|---|
| Task 1 internal | Covariance functions and `HandoffResult` | New pure tests | Internally consistent; invalid geometry/covariance returns no result and maps to HOLD in Task 4. |
| Task 1 -> Task 3 | `sample_target_covariance` | Mission final estimate | Interface agrees; mission supplies selected XY samples and per-sample projected UAV pose covariance. |
| Task 1 -> Task 4 | Standoff and split covariance propagation | Relay evaluation | Interface agrees; origin and registration covariance products remain separate until the final goal sum. |
| Task 2 internal | Revision-bearing continuous `RegistrationUpdate` | Adapter/serialization tests | Reuses the existing message and preserves all legacy topics. |
| Task 2 -> Task 4 | `/air_ground/registration/state` | Atomic relay snapshot | Exact message type agrees; continuous and accepted revisions must match. |
| Task 3 internal | Mission publishes anomaly and consumes relay action | Task 8 state-machine regressions | Potential ownership conflict resolved by retaining legacy local policy only when uncertainty-aware mode is disabled; enabled mode has one relay action publisher. |
| Task 3 -> Task 4 | `/air_ground/anomaly/uav_estimate` | Relay target input | Exact planned type/frame agree; target has a local generation but no application revision field. |
| Task 4 -> Task 3 | Action/confidence outputs | Mission handoff state | Preserve action strings and `/air_ground/handoff/confidence_radius`; mission accepts decisions only while waiting for the current target generation. |
| Task 4 internal | Prior DIRECT goal followed by later HOLD | No-goal-on-invalid tests | Task 9 suppresses new goals but cannot retract a prior latched goal; Task 10 is responsible for controller-side stale revision safety. Record as a scope limitation, not Task 10 implementation. |
| Tasks 3/4 -> Task 5 | New nodes/topics/parameters | Research launch and integration test | Wire only the uncertainty-aware research path; legacy final demo remains disabled and unchanged. |
| Task 5 -> Task 6 | Installed/wired system | Verification/report | Tests and audits cover Task 9 only; Task 10 files are out of scope. |

## Task Progress

- Task 1: Ruling: the initial brief's `evaluate_handoff` and `registration_execution_covariance` signatures could not represent distinct `origin<-UAV` and `origin<-UGV-odom` transforms. Add explicit registration mean/yaw inputs; compute the origin target only with `origin<-UAV`, use registration translation as the standoff anchor, inverse-transform the origin target into the registration frame for the lever arm, and rotate the yaw Jacobian by registration yaw. Cost if wrong: goal mean, execution covariance, confidence, and action are wrong whenever UAV and UGV frame transforms differ.
- Task 1: minor (deferred): NumPy result arrays marked read-only can restore writability because they own their buffers; frozen dataclass prevents ordinary assignment but is not cryptographically/deeply immutable.
- Task 1: fix round 1/5 (5 addressed, 0 Critical/Important open — distinct transforms/anchor, rotated registration-yaw Jacobian, nonzero UAV yaw, split radii, and multi-axis range tests; no commits because workspace is unversioned).
- Task 1: complete (27/27 focused/full tests pass; spec PASS and code quality APPROVED; one deferred Minor).
- Task 2: minor (deferred): the new constructor test protects the continuous state publisher but does not independently reassert every legacy publisher type/latch or count TF broadcasters; existing broader adapter/static tests retain partial coverage.
- Task 2: fix round 1/5 (2 addressed, 0 Critical/Important open — complete nonzero-yaw accepted pose and full timer state/legacy covariance identity; no commits because workspace is unversioned).
- Task 2: complete (13/13 focused and 105/105 bounded non-ROS tests pass; spec PASS and code quality APPROVED; live ROS integration deferred to Task 9 verification).

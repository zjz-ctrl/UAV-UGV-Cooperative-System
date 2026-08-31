# Task 2 Brief: Revision-Bearing Continuous Registration State

Read this first. It is the complete requirement for this task.

## Context

The Task 9 relay cannot atomically pair the existing continuous `PoseWithCovarianceStamped` estimate with a separate headerless `UInt32` revision. Reuse the existing `air_ground_coordinate_transform/RegistrationUpdate` message to publish a revision-bearing continuous state while preserving every legacy topic and accepted-update behavior.

## Files

- Modify `src/air_ground_coordinate_transform/scripts/takeoff_registration.py`.
- Modify `src/air_ground_coordinate_transform/test/test_registration_node_adapter.py`.
- Modify `src/air_ground_coordinate_transform/test/test_registration_update_serialization.py` only if needed for real-message identity coverage.
- Do not edit bringup files, messages, CMake, package manifests, launch files, or generated `build/`/`devel/` content.

## Required Interface

```text
/air_ground/registration/state
air_ground_coordinate_transform/RegistrationUpdate
latched
```

Each initialized continuous state publication must carry:

```text
header.stamp = filter-state stamp
header.frame_id = origin_frame
revision = explicit filter-state revision
pose = the same mean/full planar covariance as /air_ground/registration/estimate
```

Populate all nine `(x,y,yaw)` covariance entries at ROS axes `(0,1,5)`, including cross-covariances.

## Required Behavior

- Preserve `/air_ground/registration/estimate` as `PoseWithCovarianceStamped`.
- Preserve `/air_ground/registration/accepted_update` as the atomic accepted-event message.
- Preserve `/air_ground/registration/revision` and all status/TF behavior.
- On accepted update, publication order is continuous legacy estimate/state, then accepted update, then legacy revision/status. The continuous state and accepted update must have identical stamp, frame, pose covariance, and revision.
- On timer prediction, publish the continuous state with the current unchanged revision even when covariance has grown.
- Never use or depend on `Header.seq` as application identity.
- Do not create a new message type.

Use one serialization helper so legacy estimate and `RegistrationUpdate.pose` cannot diverge. Avoid aliasing mutable pose objects between messages if a later mutation could change already-published test fixtures.

## TDD Steps

1. Add failing constructor/publisher tests for the exact topic, type, and latch behavior.
2. Add failing accepted-publication tests for identical state/accepted revision, stamp, frame, mean, full covariance, and ordering.
3. Add failing timer-prediction test proving covariance may grow while revision remains unchanged in `/registration/state`.
4. Run the focused adapter test and record RED caused by missing state publication.
5. Implement the smallest publisher/serialization change.
6. Run focused GREEN and then all bounded coordinate-transform tests with the catkin environment sourced.
7. Self-review that all old topics remain and no TF broadcaster was added.

## Commands

```bash
source devel/setup.bash
python3 -m unittest \
  src.air_ground_coordinate_transform.test.test_registration_node_adapter -v
python3 -m unittest \
  src.air_ground_coordinate_transform.test.test_registration_update_serialization -v
python3 -m unittest discover \
  -s src/air_ground_coordinate_transform/test -p 'test_*.py' -v
```

## Global Constraints

- This topic is the continuous half of the relay's matched revision snapshot; accepted update remains the acceptance authority.
- Invalid/mismatched handling belongs to the relay task, not this producer.
- No truth inputs, no new TF broadcaster, no `Header.seq`, and no Task 10 work.
- This workspace has no Git metadata. Do not initialize Git or attempt commits.

## Report

Write `.superpowers/sdd/2026-08-29-task9-common-frame-handoff/task-2-report.md` with status, files changed, genuine RED evidence, GREEN/full-suite commands and exact counts, publication ordering, message-copy semantics, and concerns. Return only status, one-line tests, and concerns. Do not dispatch subagents.

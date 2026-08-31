# Task 2 Report: Revision-Bearing Continuous Registration State

## Status

Implemented the continuous `/air_ground/registration/state` output as a latched `air_ground_coordinate_transform/RegistrationUpdate` publisher. The focused adapter tests, real generated-message serialization test, and every non-ROS coordinate-transform unit test pass. The exact discovery command from the brief cannot complete without a live ROS graph because it discovers `test_registration_node.py`; running ROS was explicitly prohibited.

## Files Changed

- `src/air_ground_coordinate_transform/scripts/takeoff_registration.py`
- `src/air_ground_coordinate_transform/test/test_registration_node_adapter.py`
- `.superpowers/sdd/2026-08-29-task9-common-frame-handoff/task-2-report.md`

`test_registration_update_serialization.py` did not require modification because its existing real generated-message test already proves that transport `Header.seq` replacement cannot change the explicit revision or serialized pose.

## Implementation

- Added `/air_ground/registration/state` with type `RegistrationUpdate`, `queue_size=1`, and `latch=True`.
- Preserved `/air_ground/registration/estimate` as `PoseWithCovarianceStamped`.
- Preserved `/air_ground/registration/accepted_update`, `/air_ground/registration/revision`, all status publishers, and existing TF behavior.
- Added `_serialize_pose`, the single mean/covariance serialization helper used by the legacy estimate, continuous state, and accepted update.
- Populated all nine planar covariance entries at ROS axes `(0, 1, 5)`, including every cross-covariance.
- Used the filter-state stamp and `origin_frame` for both continuous messages.
- Used the accepted decision's explicit revision during accepted publication and the snapshot state's current revision during timer publication.
- Did not read or write `Header.seq` and did not add a message type or TF broadcaster.

## Genuine RED Evidence

After adding the constructor, accepted-event, and timer-prediction tests, the first focused run exposed a test-runtime compatibility error from `str.removeprefix` under the sourced ROS Python. That test-only issue was replaced with compatible slicing and RED was rerun before production code changed.

Command:

```bash
source devel/setup.bash && python3 -m unittest \
  src.air_ground_coordinate_transform.test.test_registration_node_adapter -v
```

Clean RED result: `Ran 13 tests`; `FAILED (failures=3)`.

All three failures were the required missing behavior:

- `test_constructor_creates_latched_continuous_registration_state_publisher`: expected the exact state publisher, found none.
- `test_accepted_event_publishes_matching_independent_state_before_event`: expected one continuous state, found none.
- `test_timer_publishes_grown_covariance_with_unchanged_state_revision`: expected revisions `[4, 4]`, found no state messages.

## GREEN Evidence

Focused adapter command:

```bash
source devel/setup.bash && python3 -m unittest \
  src.air_ground_coordinate_transform.test.test_registration_node_adapter -v
```

Result: `Ran 13 tests in 0.041s`; `OK`.

Real generated-message command:

```bash
source devel/setup.bash && python3 -m unittest \
  src.air_ground_coordinate_transform.test.test_registration_update_serialization -v
```

Result: `Ran 1 test in 0.000s`; `OK`.

Exact full-discovery command from the brief:

```bash
source devel/setup.bash && python3 -m unittest discover \
  -s src/air_ground_coordinate_transform/test -p 'test_*.py' -v
```

This was attempted twice, first with a 120-second limit and then with a 600-second limit. Both attempts passed the first 79 tests and then timed out before a final unittest result. Discovery next loads `test_registration_node.py`, whose `RegistrationNodeTest.setUpClass` calls `rospy.init_node` and waits for a live ROS graph. No ROS master or node was started because the task forbids running ROS.

Complete non-ROS bounded-suite command:

```bash
source devel/setup.bash && python3 -m unittest \
  src.air_ground_coordinate_transform.test.test_acquisition_diagnostics \
  src.air_ground_coordinate_transform.test.test_odom_buffer \
  src.air_ground_coordinate_transform.test.test_registration_coordinator \
  src.air_ground_coordinate_transform.test.test_registration_estimator \
  src.air_ground_coordinate_transform.test.test_registration_node_adapter \
  src.air_ground_coordinate_transform.test.test_registration_update_serialization \
  src.air_ground_coordinate_transform.test.test_se2 \
  src.air_ground_coordinate_transform.test.test_ugv_coordinate_monitor -v
```

Results: initial run `Ran 105 tests in 9.384s`; final verification run `Ran 105 tests in 11.537s`; both `OK`.

## Publication Ordering

For an accepted decision, the relevant publication sequence is:

```text
/air_ground/registration/estimate
/air_ground/registration/state
/air_ground/registration/accepted_update
/air_ground/registration/revision
/air_ground/registration/status
```

The adapter test observes and asserts this sequence. It also proves that state and accepted update have identical explicit revision, stamp, frame, mean, and complete 36-slot covariance representation.

On timer prediction, `publish_estimate(state)` publishes the legacy estimate and continuous state from the same snapshot. The continuous state takes `state.revision`, so covariance and stamp can advance while the revision remains unchanged.

## Message-Copy Semantics

Each call to `_serialize_pose` creates a fresh `PoseWithCovariance` object through a fresh `PoseWithCovarianceStamped`. Legacy estimate, continuous state, and accepted update therefore do not alias mutable pose or covariance objects. The accepted-event test mutates the already-published continuous fixture and proves the legacy estimate and accepted-event fixtures remain unchanged.

## Self-Review

- Every pre-existing registration publisher remains present.
- `/air_ground/registration/estimate` retains its original type and latch behavior.
- `/air_ground/registration/accepted_update` retains its original atomic-event role.
- The source still constructs exactly one pre-existing `TransformBroadcaster`; no broadcaster was added.
- The production source has no `header.seq` access.
- No bringup, message, CMake, manifest, launch, generated `build/`, or generated `devel/` file was modified.
- Git was not initialized or used, and no subagent was dispatched.
- ROS, Gazebo, and PX4 were not started.

## Concerns

- The literal discovery command cannot provide a final full-suite count under the no-ROS constraint because it includes the ROS integration test. All 105 non-ROS tests pass, but `test_registration_node.py` remains unexecuted in this task environment.

## Review Round 1/5

### Status

Addressed both Important review findings with test-only changes in `test_registration_node_adapter.py`. No strengthened test exposed a production defect, so `takeoff_registration.py` was not changed in this round.

### Coverage Changes

- Replaced the identity `quaternion_from_euler` adapter stub with a planar yaw-sensitive quaternion implementation.
- Strengthened accepted-publication coverage to assert the complete `(x, y, z, qx, qy, qz, qw)` pose with nonzero yaw for legacy estimate, continuous state, and accepted update.
- Retained the complete 36-slot covariance assertions and added explicit assertions for all nine planar ROS slots `(0, 1, 5, 6, 7, 11, 30, 31, 35)`.
- Strengthened timer coverage with two full, correlated planar covariance matrices whose diagonal covariance grows while revision remains `4`.
- For both timer publications, asserted stamp, frame, complete nonzero-yaw pose, all nine planar covariance slots, complete 36-slot covariance, and equality between continuous state and its paired legacy estimate.

### RED And Mutation Evidence

Before making the quaternion stub yaw-sensitive, the strengthened focused suite was run with:

```bash
source devel/setup.bash && python3 -m unittest \
  src.air_ground_coordinate_transform.test.test_registration_node_adapter -v
```

Result: `Ran 13 tests in 0.050s`; `FAILED (failures=2)`. The accepted test expected quaternion `z=0.14943813247359922` and got `0.0`; the timer test expected `z=0.09983341664682815` and got `0.0`.

After restoring yaw-sensitive test serialization, a temporary test-only mutation changed only the accepted update quaternion `z` to `0.0`. Command:

```bash
source devel/setup.bash && python3 -m unittest \
  src.air_ground_coordinate_transform.test.test_registration_node_adapter.RegistrationNodeAdapterTest.test_accepted_event_publishes_matching_independent_state_before_event -v
```

Mutation result: `Ran 1 test in 0.003s`; `FAILED (failures=1)`, expected `0.14943813247359922`, got `0.0`. The mutation was removed.

A second temporary test-only mutation changed the second timer state covariance at non-`[0]` ROS slot `31` from `0.06` to `999.0`. Command:

```bash
source devel/setup.bash && python3 -m unittest \
  src.air_ground_coordinate_transform.test.test_registration_node_adapter.RegistrationNodeAdapterTest.test_timer_publishes_grown_covariance_with_unchanged_state_revision -v
```

Mutation result: `Ran 1 test in 0.004s`; `FAILED (failures=1)`, with the first differing full-covariance element at slot `31`: expected `0.06`, got `999.0`. The mutation was removed.

### GREEN Evidence

Focused adapter command after removing all mutations:

```bash
source devel/setup.bash && python3 -m unittest \
  src.air_ground_coordinate_transform.test.test_registration_node_adapter -v
```

Result: `Ran 13 tests in 0.046s`; `OK`.

Bounded non-ROS suite command:

```bash
source devel/setup.bash && python3 -m unittest \
  src.air_ground_coordinate_transform.test.test_acquisition_diagnostics \
  src.air_ground_coordinate_transform.test.test_odom_buffer \
  src.air_ground_coordinate_transform.test.test_registration_coordinator \
  src.air_ground_coordinate_transform.test.test_registration_estimator \
  src.air_ground_coordinate_transform.test.test_registration_node_adapter \
  src.air_ground_coordinate_transform.test.test_registration_update_serialization \
  src.air_ground_coordinate_transform.test.test_se2 \
  src.air_ground_coordinate_transform.test.test_ugv_coordinate_monitor -v
```

Result: `Ran 105 tests in 10.378s`; `OK`.

### Concerns

- No new concerns. The existing no-ROS limitation on `test_registration_node.py` remains unchanged.

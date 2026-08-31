# Task 2 Review Package

The workspace has no Git metadata. Review these complete task-scoped files:

- Brief: `.superpowers/sdd/2026-08-29-task9-common-frame-handoff/task-2-brief.md`
- Report: `.superpowers/sdd/2026-08-29-task9-common-frame-handoff/task-2-report.md`
- Production: `src/air_ground_coordinate_transform/scripts/takeoff_registration.py`
- Tests: `src/air_ground_coordinate_transform/test/test_registration_node_adapter.py`
- Existing identity test: `src/air_ground_coordinate_transform/test/test_registration_update_serialization.py`

Task 2 changes are the new `state_pub`, shared pose serialization, state publication in `publish_estimate`, and the new adapter tests. Binding requirements: the state is latched `RegistrationUpdate`; state/accepted match revision/stamp/frame/pose on accepted updates; timer prediction carries unchanged explicit revision with current covariance; legacy topics/order semantics and single TF broadcaster remain; no `Header.seq`, custom message, truth, or bringup edits.

The report records 13/13 adapter, 1/1 serialization, and 105/105 bounded non-ROS tests passing. Full discovery includes a live ROS test and is not a valid pure-suite command in this environment. Review without editing or starting ROS.

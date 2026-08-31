# Task 1 Review Package

The workspace has no Git metadata. Review the complete current contents of these task-scoped files against the brief and report:

- Requirements: `.superpowers/sdd/2026-08-29-task9-common-frame-handoff/task-1-brief.md`
- Implementer report: `.superpowers/sdd/2026-08-29-task9-common-frame-handoff/task-1-report.md`
- Production: `src/air_ground_bringup/src/air_ground_bringup/target_handoff.py`
- Tests: `src/air_ground_bringup/test/test_target_handoff.py`

Baseline before Task 1 was the first 13 Task 8 tests in `test_target_handoff.py`; those tests must retain their behavior. All code after the original 96-line `UncertaintyBudget` module and all new Task 9 test classes are Task 1 scope.

Binding constraints:

- Origin covariance contains sensing/UAV-pose uncertainty only.
- Goal covariance adds registration translation/yaw lever-arm uncertainty exactly once.
- Registration transform translation is the standoff anchor; the origin-from-UAV transform and registration transform are distinct physical transforms.
- Invalid/nonfinite covariance must lead to an invalid/HOLD result without raising.
- Preserve Task 8 actions and policy behavior.
- No ROS code, truth input, Task 10 behavior, Git initialization, or unrelated edits.

The implementer reports 24/24 tests passing. Review code and tests; do not merely trust the test count and do not edit files.

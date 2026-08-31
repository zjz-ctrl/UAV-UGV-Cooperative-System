# Task 6 Review Package

## Repository State

No Git metadata exists. Review complete current Task 6 files against the brief,
specification, and report. Task 7 integration has not started.

## Files In Scope

- `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py`
- `src/air_ground_coordinate_transform/config/registration.yaml`
- `src/air_ground_coordinate_transform/test/test_registration_estimator.py`

## Binding Constraints

- State is `x=[tx,ty,yaw]^T = ^O T_G`, with full 3x3 covariance and wrapped yaw.
- Prediction leaves the mean unchanged and adds the exact six-rate empirical
  random-walk variance model from `task-6-brief.md`.
- Update uses full 3x3 identity-measurement EKF, wrapped yaw innovation, NIS
  `nu.T solve(P+R,nu)`, a three-DOF chi-square gate, and Joseph covariance.
- Default gate is `chi2_3(0.99)=11.344866730144373`; no SciPy runtime dependency.
- Invalid covariances are rejected, not silently repaired or diagonalized.
- First valid batch initializes revision 1; only accepted updates increment it.
- Process parameters have explicit physical units and a held-out calibration
  method. Numerical values are provisional until real no-update calibration.
- Monte Carlo must compare against prediction-only on the same seeded random
  walk/noise realizations and may not tune per seed.
- No Gazebo/experiment truth or offline alignment may enter production filtering
  or decisions. Synthetic test truth is test-only.
- One-shot node behavior and integration files remain unchanged.
- Do not run ROS launch/core/test, simulation, PX4, RViz, rosbag, topic waits, or
  any long process in this environment.

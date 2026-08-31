### Task 2: Refactor current one-shot registration behind a pure robust estimator

**Files:**
- Create: `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/registration_estimator.py`
- Modify: `src/air_ground_coordinate_transform/scripts/takeoff_registration.py:17-216`
- Test: `src/air_ground_coordinate_transform/test/test_registration_estimator.py`
- Test: `src/air_ground_coordinate_transform/test/registration_node.test`
- Test: `src/air_ground_coordinate_transform/test/test_registration_node.py`

**Interfaces:**
- Consumes: SE(2) helpers and `OdomBuffer` from Task 1.
- Produces: `RegistrationSample(mean: np.ndarray, anchor: np.ndarray, covariance: np.ndarray, stamp: float)`.
- Produces: `RobustBatchEstimator(min_samples, max_translation_residual, max_yaw_residual).estimate(samples) -> BatchEstimate | None`.
- Produces: `BatchEstimate(mean, covariance, inlier_count, stamp)`.
- Preserves: `/air_ground/registration/frozen`, `/status`, `/valid`, `/inlier_count`, and existing TF edges in `one_shot` mode.

- [ ] **Step 1: Write failing randomized robust-estimation tests**

Generate 20 inliers around `[2.0, -1.0, 0.35]` and 8 gross outliers. Require correct wrapped yaw, at least 18 inliers, symmetric positive-semidefinite covariance, and no estimate when fewer than `min_samples` survive.

```python
estimate = RobustBatchEstimator(20, 0.12, 0.03).estimate(samples)
np.testing.assert_allclose(estimate.mean[:2], [2.0, -1.0], atol=0.03)
self.assertLess(abs(wrap_angle(estimate.mean[2] - 0.35)), 0.01)
self.assertGreaterEqual(estimate.inlier_count, 18)
np.testing.assert_allclose(estimate.covariance, estimate.covariance.T, atol=1e-12)
```

- [ ] **Step 2: Run the focused estimator test and verify failure**

Run:

```bash
python3 -m unittest src/air_ground_coordinate_transform/test/test_registration_estimator.py -v
```

Expected: import failure for `RobustBatchEstimator`.

- [ ] **Step 3: Implement robust median/circular-mean estimation and covariance floors**

Use median translation, circular yaw mean, the current translation/yaw gates, and sample covariance divided by effective inlier count. Enforce configurable floors:

```python
covariance += np.diag([
    minimum_translation_sigma ** 2,
    minimum_translation_sigma ** 2,
    minimum_yaw_sigma ** 2,
])
```

- [ ] **Step 4: Replace duplicated matrix/interpolation code in the ROS node**

Keep observation construction equivalent to the existing equation:

```python
sample = origin_to_uav_odom.dot(uav).dot(base_camera).dot(
    observation_matrix).dot(inverse_matrix(ugv.dot(base_board)))
```

Convert the planar result to `RegistrationSample`. In `one_shot` mode, stop accepting observations after the first estimate exactly as today.

- [ ] **Step 5: Publish a covariance-bearing estimate without breaking old topics**

Add latched:

```text
/air_ground/registration/estimate  geometry_msgs/PoseWithCovarianceStamped
/air_ground/registration/revision  std_msgs/UInt32
```

The pose represents `ugv_0/odom` origin in `air_ground_origin`. Populate `(x,y,yaw)` covariance slots and set revision to `1` on the initial freeze.

- [ ] **Step 6: Add a rostest for initial status, sample rejection, and one-shot freeze**

Publish synthetic UAV odom, UGV odom, and board observations. Assert:

```python
self.assertEqual(wait_for_status(), "FROZEN")
self.assertTrue(rospy.wait_for_message("/air_ground/registration/frozen", Bool).data)
self.assertEqual(rospy.wait_for_message("/air_ground/registration/revision", UInt32).data, 1)
self.assertLess(abs(received.pose.pose.position.x - expected_x), 0.03)
```

Then publish a contradictory second batch and assert revision remains `1` in `one_shot` mode.

- [ ] **Step 7: Run unit tests, rostest, and current launch XML validation**

Run:

```bash
python3 -m unittest src/air_ground_coordinate_transform/test/test_registration_estimator.py -v
rostest air_ground_coordinate_transform registration_node.test
roslaunch --check air_ground_bringup air_ground_final_demo.launch
catkin_make --pkg air_ground_coordinate_transform air_ground_bringup
```

Expected: tests pass and launch resolves.

- [ ] **Step 8: Cold-start the compatibility Demo**

Run:

```bash
source src/air_ground_bringup/scripts/setup_mvp_env.sh
roslaunch air_ground_bringup air_ground_final_demo.launch separate_terminals:=false
```

Expected: `FROZEN -> OVERWATCH`, UGV `ARRIVED`, revision remains `1`, and the final stop remains approximately `0.76 m` from the anomaly.

- [ ] **Step 9: Verification checkpoint M0-B**

Retain the cold-start status, target coordinate, final UGV coordinate, and shutdown-process check as the compatibility record.

---

## Milestone 1: Multi-Initial-Pose One-Shot Registration Baseline


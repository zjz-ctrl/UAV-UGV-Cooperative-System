### Task 1: Extract SE(2) and odometry interpolation primitives

**Files:**
- Create: `src/air_ground_coordinate_transform/setup.py`
- Create: `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/__init__.py`
- Create: `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/se2.py`
- Create: `src/air_ground_coordinate_transform/src/air_ground_coordinate_transform/odom_buffer.py`
- Modify: `src/air_ground_coordinate_transform/CMakeLists.txt:1-31`
- Modify: `src/air_ground_coordinate_transform/package.xml:8-30`
- Test: `src/air_ground_coordinate_transform/test/test_se2.py`
- Test: `src/air_ground_coordinate_transform/test/test_odom_buffer.py`

**Interfaces:**
- Produces: `wrap_angle(float) -> float`.
- Produces: `wrap_xyyaw(np.ndarray) -> np.ndarray`, normalizing element `2` only.
- Produces: `matrix_from_xyyaw(float, float, float) -> np.ndarray` with shape `(3, 3)`.
- Produces: `xyyaw_from_matrix(np.ndarray) -> np.ndarray` with shape `(3,)`.
- Produces: `compose(np.ndarray, np.ndarray) -> np.ndarray` and `inverse(np.ndarray) -> np.ndarray` for planar transforms.
- Produces: `transform_pose_covariance(mean, covariance, transform_mean, transform_covariance) -> (mean, covariance)` for first-order SE(2) propagation.
- Produces: `OdomBuffer(maxlen: int, max_bracket: float)` with `append(stamp, x, y, z, yaw)`, `append_odometry(message)`, `interpolate(stamp)`, and `distance_since(stamp)`.

- [ ] **Step 1: Add failing SE(2) identity, inverse, wrap, and covariance tests**

```python
def test_compose_with_inverse_returns_identity(self):
    value = np.array([1.2, -0.4, 0.7])
    matrix = matrix_from_xyyaw(*value)
    np.testing.assert_allclose(compose(matrix, inverse(matrix)), np.eye(3), atol=1e-9)

def test_transform_covariance_includes_heading_lever_arm(self):
    point = np.array([10.0, 0.0, 0.0])
    point_cov = np.zeros((3, 3))
    tf_mean = np.zeros(3)
    tf_cov = np.diag([0.01, 0.01, math.radians(1.0) ** 2])
    _, covariance = transform_pose_covariance(point, point_cov, tf_mean, tf_cov)
    self.assertGreater(covariance[1, 1], 0.03)
```

- [ ] **Step 2: Run the SE(2) tests and verify the missing module failure**

Run:

```bash
source devel/setup.bash
python3 -m unittest discover -s src/air_ground_coordinate_transform/test -p 'test_se2.py' -v
```

Expected: `ModuleNotFoundError: No module named 'air_ground_coordinate_transform.se2'`.

- [ ] **Step 3: Implement the minimal SE(2) functions**

Use the state convention `[x, y, yaw]`. Normalize yaw after every composition. For a point transformed by uncertain yaw, use Jacobians:

```python
j_point = np.array([
    [c, -s, -s * px - c * py],
    [s,  c,  c * px - s * py],
    [0.0, 0.0, 1.0],
])
j_transform = np.array([
    [1.0, 0.0, -s * px - c * py],
    [0.0, 1.0,  c * px - s * py],
    [0.0, 0.0, 1.0],
])
covariance = j_point @ point_covariance @ j_point.T + j_transform @ transform_covariance @ j_transform.T
```

- [ ] **Step 4: Add failing interpolation tests**

Cover exact timestamp, linear translation, shortest-path yaw interpolation across `+pi/-pi`, a preceding sample within `max_bracket`, rejection outside `max_bracket`, and accumulated planar distance.

```python
def test_interpolates_yaw_across_pi_by_shortest_path(self):
    buffer = OdomBuffer(maxlen=10, max_bracket=0.08)
    buffer.append(make_odom(1.00, 0.0, 0.0, math.radians(179.0)))
    buffer.append(make_odom(1.10, 1.0, 0.0, math.radians(-179.0)))
    result = buffer.interpolate(rospy.Time.from_sec(1.05))
    self.assertAlmostEqual(abs(result[2]), math.pi, places=5)
```

- [ ] **Step 5: Implement `OdomBuffer` without ROS side effects at import time**

Store `(stamp_sec, x, y, z, yaw)` samples. Keep ROS message conversion in a small `append_odometry(message)` adapter so pure tests can append numeric samples.

- [ ] **Step 6: Register the Python package and tests with catkin**

Add `catkin_python_setup()` and guarded test declarations:

```cmake
catkin_python_setup()
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_se2.py)
  catkin_add_nosetests(test/test_odom_buffer.py)
endif()
```

Add `<test_depend>python3-nose</test_depend>` to `package.xml`.

- [ ] **Step 7: Run the focused tests and package build**

Run:

```bash
python3 -m unittest discover -s src/air_ground_coordinate_transform/test -p 'test_*.py' -v
catkin_make --pkg air_ground_coordinate_transform
```

Expected: all tests pass; package builds with no new error.

- [ ] **Step 8: Verification checkpoint M0-A**

Record the command output and confirm no runtime script or launch behavior changed.

---


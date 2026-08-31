# Ego_Planner_v2 — 空地协同 (Air-Ground Cooperation)

> 基于 Ego-Planner 的无人机-地面车协同系统，包含鲁棒的跨载体坐标对齐、视觉感知、轨迹规划与 PX4 飞控桥接。运行于 ROS Noetic + Gazebo + PX4 SITL 仿真环境。

---

## 目录

- [项目简介](#项目简介)
- [系统架构](#系统架构)
- [坐标对齐原理](#坐标对齐原理)
- [包一览](#包一览)
- [环境依赖](#环境依赖)
- [安装与编译](#安装与编译)
- [快速开始](#快速开始)
- [配置说明](#配置说明)
- [话题与 TF 树](#话题与-tf-树)
- [目录结构](#目录结构)
- [开发与测试](#开发与测试)

---

## 项目简介

本项目实现 **UAV（Iris 四旋翼）+ UGV（差速小车）** 的空地协同：

- UAV 搭载下视相机（nadir camera），通过检测 UGV 顶部的 **ChArUco 标定板** 建立两个载体独立里程计之间的平面变换关系。
- 对齐后的统一坐标系下，UAV 可使用 Ego-Planner 进行避障轨迹规划，跟踪/伴飞 UGV。
- 支持三种配准模式：`one_shot`（一次标定后冻结）、`periodic`（定期更新）、`opportunistic`（机会式更新），配合卡尔曼滤波与鲁棒批估计保证野外鲁棒性。

核心特点：

- **视觉-里程计紧耦合**：将 6-DoF 相机观测经外参链传播为 SE(2) 平面样本，协方差通过数值雅可比一阶传播。
- **鲁棒估计**：中位数/圆均值初值 + 残差门限剔除离群点 + Mahalanobis χ² 门限（3 DoF, 99% 分位数 11.34）。
- **时变不确定性建模**：配准协方差随时间与 UAV/UGV 行驶距离线性增长，反映里程计漂移。

---

## 系统架构

```
                    ┌─────────────────────────────────────────────┐
                    │           Gazebo + PX4 SITL                  │
                    │  ┌──────────┐    ┌──────────┐  ┌────────┐  │
                    │  │ Iris UAV │    │ UGV (MVP)│  │ ego.world│  │
                    │  │ PX4 EKF2 │    │ diff-drive│  │  walls   │  │
                    │  └────┬─────┘    └────┬─────┘  └────────┘  │
                    └───────┼───────────────┼────────────────────┘
                            │ odom          │ odom
              ┌─────────────┼───────────────┼────────────────────┐
              │             ▼               ▼                    │
              │   mavros/local_position/odom   /ugv_0/odom       │
              │             │               │                    │
              │  ┌──────────┴──┐  ┌────────┴────────┐            │
              │  │ Ego-Planner │  │  px4ctrl /       │            │
              │  │ plan_manage │  │  cxr_egoctrl_v1  │            │
              │  │ plan_env    │  │  (MAVROS 速度控) │            │
              │  │ traj_opt    │  └────────┬────────┘            │
              │  └──────┬──────┘           │                     │
              │         │ position_cmd     │ mavros/setpoint_raw  │
              │         └──────────────────┘                     │
              │                                                  │
              │  ┌─────────────────────────────────┐             │
              │  │  air_ground_perception          │             │
              │  │  charuco_detector.py            │◄── nadir image
              │  │  red_sphere_detector            │             │
              │  └──────────────┬──────────────────┘             │
              │                 │ /air_ground/charuco/observation│
              │  ┌──────────────▼──────────────────┐             │
              │  │ air_ground_coordinate_transform │             │
              │  │  takeoff_registration.py        │             │
              │  │  ├─ RobustBatchEstimator        │             │
              │  │  ├─ RegistrationFilter (KF)     │             │
              │  │  └─ RegistrationCoordinator     │             │
              │  └──────────────┬──────────────────┘             │
              │                 │ TF: air_ground_origin           │
              │                 │     → iris_0/odom               │
              │                 │     → ugv_0/odom                │
              │  ┌──────────────▼──────────────────┐             │
              │  │  air_ground_bringup             │             │
              │  │  mvp_system.launch              │             │
              │  │  uav_follow_mission / ugv_patrol│             │
              │  └─────────────────────────────────┘             │
              └──────────────────────────────────────────────────┘
```

---

## 坐标对齐原理

### 问题定义

UAV 与 UGV 各自维护独立的里程计坐标系（`iris_0/odom` 与 `ugv_0/odom`），二者无先验关联。需估计平面变换 `T ∈ SE(2)`，使得：

```
p_origin = T_origin→uav_odom  · p_uav_odom
p_origin = T_origin→ugv_odom  · p_ugv_odom     (待求)
```

等价于求 `iris_0/odom → ugv_0/odom` 的 `[tx, ty, yaw]`。

### 变换链

单次视觉观测产生一个 SE(2) 样本（`registration_estimator.py:491`）：

```python
# takeoff_registration.py:311 — registration_sample_from_observation()
prefix = origin_to_uav_odom @ pose_matrix(uav_pose) @ base_camera
suffix = inv(pose_matrix(ugv_pose) @ base_board)
mean   = planar_mean(prefix @ xyzrpy_matrix(observation) @ suffix)
```

| 符号 | 含义 | 来源 |
|------|------|------|
| `origin_to_uav_odom` | `air_ground_origin → iris_0/odom` | 起飞前 30 帧 UAV 里程计平均位姿求得 |
| `uav_pose` | UAV 在 `iris_0/odom` 中的 6-DoF 位姿 | `OdomBuffer` 插值到观测时刻 |
| `base_camera` | `iris_0/base_link → nadir_camera_optical_frame` | 外参 `[0, 0, -0.17]`, rpy `[π, 0, -π/2]` |
| `observation` | `camera → board` 的 6-DoF 位姿 | ChArUco PnP 解算 |
| `ugv_pose` | UGV 在 `ugv_0/odom` 中的位姿 | `OdomBuffer` 插值 |
| `base_board` | `ugv_0/base_link → ChArUco board` | 外参 `[-0.3125, -0.1875, 0.115]` |

协方差通过对 `observation`（6 维）的数值雅可比传播：

```python
jacobian[:, k] = (evaluate(mean + ε·e_k) - evaluate(mean - ε·e_k)) / 2ε
cov_sample = J @ cov_observation @ Jᵀ
```

### 两套实现

#### 1) C++ 节点 — `coordinate_transform_node.cpp`（轻量实时版）

- 订阅 `uav_odom` / `ugv_odom` / `observation`，时间同步门限 `sync_slop_sec=0.10 s`。
- 单样本公式：`sample = uav_odom_to_base · base_to_camera · camera_to_marker · ugv_odom_to_marker⁻¹`
- 滑动窗口（≤100 样本），四元数平均聚合，样本数 ≥ `minimum_observations`（20）即发布 `iris_0/odom → ugv_0/odom` TF。

#### 2) Python 节点 — `takeoff_registration.py`（鲁棒滤波版）

完整流水线分四阶段：

**阶段 1 — Origin 标定**

收集 `minimum_origin_samples`（30）帧 UAV 里程计，计算平均位置与圆均值 yaw，建立 `air_ground_origin`。若 `align_origin_to_uav_heading=true` 则令 origin yaw = -yaw_uav，使 UAV 起飞点为原点。

**阶段 2 — 观测门控**

每帧观测需同时满足：

- 帧 ID 校验、时间戳非零、与最近 UAV/UGV 里程计时间差 ≤ `max_odom_bracket`（0.08 s）
- UAV 高度 ≥ `minimum_uav_height`（1.2 m），线速度 ≤ 0.10 m/s，角速度 ≤ 0.10 rad/s，UGV 速度 ≤ 0.03 m/s
- 经 `OdomBuffer`（线性插值 + SLERP）精确插值到观测时刻

**阶段 3 — 鲁棒批估计** (`RobustBatchEstimator`)

```
1. 计算所有样本的 translation 中位数与 yaw 圆均值作为中心
2. 剔除 translation 残差 > max_translation_residual (0.12 m) 的样本
3. 在剩余样本上计算 yaw 圆均值，再剔除 yaw 残差 > max_yaw_residual (0.03 rad) 的样本
4. 剩余 inliers 的均值 = median(tx), median(ty), circular_mean(yaw)
   协方差 = (经验协方差 + 平均输入协方差) / N + diag(σₜ², σₜ², σ_yaw²)
```

**阶段 4 — 卡尔曼滤波** (`RegistrationFilter` + `RegistrationCoordinator`)

- 状态 `x = [tx, ty, yaw]`，协方差 `P ∈ ℝ³ˣ³`。
- **预测**：`P ← P + diag(Qₜ·dt + Qₜᵤₐᵥ·d_uav + Qₜᵤgᵥ·d_ugv,  ...,  Q_yaw·dt + ...)`，由 6 个可标定方差率驱动。
- **更新**：标准 KF 更新 + Mahalanobis 门限 `d² = innovationᵀ·S⁻¹·innovation ≤ 11.34`（χ²₀.₉₉, df=3）。支持 `one_shot` / `periodic` / `opportunistic` 三种协调策略。

TF 发布：`air_ground_origin → iris_0/odom`（静态/低频）与 `air_ground_origin → ugv_0/odom`（滤波结果），下游通过 `compose(frozen, ugv_pose)` 得到 UGV 在统一坐标系中的位姿。

### SE(2) 数学库 — `se2.py`

- `wrap_angle` / `wrap_xyyaw`：yaw 归一化到 [-π, π)
- `matrix_from_xyyaw` / `xyyaw_from_matrix`：SE(2) 齐次矩阵与向量互转
- `compose` / `inverse`：SE(2) 复合与求逆
- `transform_pose_covariance`：一阶雅可比传播不确定性

### 简化版 — `relative_target_estimator.py`

`air_ground_bringup` 中的轻量实现：维护 `alignment = visual_target · inv(ugv_pose)`，后续观测以 `correction_gain=0.30` 做 SLERP/线性插值融合，发布 `/air_ground/relative_target`。

---

## 包一览

| 包 | 路径 | 功能 |
|----|------|------|
| **air_ground_coordinate_transform** | `src/air_ground_coordinate_transform/` | 坐标对齐核心（C++ 节点 + Python 鲁棒滤波） |
| **air_ground_perception** | `src/air_ground_perception/` | ChArUco 标定板检测、红球检测（前视/下视） |
| **air_ground_bringup** | `src/air_ground_bringup/` | 一键启动 launch、任务脚本（跟随/巡逻/监控） |
| **air_ground_ugv_gazebo** | `src/air_ground_ugv_gazebo/` | UGV Gazebo 模型（MVP / TurtleBot3 / Polaris） |
| **air_ground_experiments** | `src/air_ground_experiments/` | 实验脚本与配置 |
| **px4ctrl** | `src/px4ctrl/` | PX4 控制器（基于 Fast-Drone-250，支持角速度控制与动态调参） |
| **cxr_egoctrl_v1** | `src/cxr_egoctrl_v1/` | Ego-Planner → MAVROS 速度控制桥接 |
| **main_ws** | `src/main_ws/` | Ego-Planner 主算法（`plan_manage` / `plan_env` / `traj_opt` / `path_searching` 等） |
| **apace_yaw_wrapper** | `src/apace_yaw_wrapper/` | 偏航角包装 |
| **light_gcs** | `src/light_gcs/` | 轻量地面站 |

---

## 环境依赖

| 依赖 | 版本/说明 |
|------|-----------|
| Ubuntu | 20.04 (推荐) |
| ROS | Noetic |
| PX4-Autopilot | 随仓库 `PX4-Autopilot/`，SITL 模型 `iris` + EKF2 |
| Gazebo | 11 + `gazebo_ros` / `mavlink_sitl_gazebo` |
| MAVROS | `ros-noetic-mavros` + `mavros_extras` |
| OpenCV | ≥ 4.x（含 `aruco` 模块） |
| Python | 3.8 + `numpy` / `cv_bridge` / `rospy` |
| Eigen / PCL | Ego-Planner 编译依赖 |

---

## 安装与编译

```bash
# 1. 克隆并准备工作区
git clone <repo_url> ~/Ego_Planner_v2
cd ~/Ego_Planner_v2

# 2. 安装 ROS 依赖
rosdep install --from-paths src --ignore-src -r -y

# 3. 编译（首次编译建议单线程，避免内存不足）
catkin_make -j1
# 或
catkin build

# 4. 配置完整 MVP 环境（必须是 launch 前最后一次 setup source）
source ~/Ego_Planner_v2/src/air_ground_bringup/scripts/setup_mvp_env.sh

# 5. （可选）安装 PX4 SITL 依赖
# 参考 PX4-Autopilot/Tools/setup/ubuntu.sh
```

> **注意**：`main_ws` 内含独立的 `src/` 子工作区，其 `Utils/quadrotor_msgs` 等为 Ego-Planner 内部依赖，已随主工作区一并编译，无需单独处理。
>
> `setup_mvp_env.sh` 会依次加载 ROS、当前工作区和 PX4 Gazebo 环境，并在最后固定 `px4` 与 `mavlink_sitl_gazebo` 的 package lookup 顺序。执行后不要再次 source `/opt/ros/noetic/setup.bash`、`devel/setup.bash` 或其他 catkin setup；这些脚本会重建 `ROS_PACKAGE_PATH` 并移除 PX4 package entries。新终端应直接重新 source `setup_mvp_env.sh`。

---

## 快速开始

### 一键启动完整 MVP 系统

```bash
source ~/Ego_Planner_v2/src/air_ground_bringup/scripts/setup_mvp_env.sh
roslaunch air_ground_bringup mvp_system.launch
```

该 launch 将依次启动：Gazebo 空世界 → PX4 SITL (Iris) → UGV 模型 → Ego-Planner → cxr_egoctrl → 坐标对齐 → RViz。

常用参数：

```bash
roslaunch air_ground_bringup mvp_system.launch \
  gui:=true start_ego:=true start_cxr:=true \
  auto_takeoff:=true start_coordinate_transform:=true \
  uav_x:=-6.0 uav_y:=0.0 ugv_x:=-5.3 ugv_y:=0.0
```

### 仅启动坐标对齐

```bash
roslaunch air_ground_coordinate_transform coordinate_transform.launch \
  registration_mode:=one_shot use_visual_frame_yaw:=true
```

### 启动感知

```bash
roslaunch air_ground_perception perception.launch
```

### RViz 指点飞行

1. 等待 UAV 自动起飞悬停（`auto_takeoff_trigger.py`）。
2. 在 RViz 中使用 `2D Nav Goal` 指点目标，Ego-Planner 规划轨迹后由 `cxr_egoctrl_v1` 跟踪。

### 任务脚本

```bash
# UAV 跟随 UGV
rosrun air_ground_bringup uav_follow_mission.py

# UGV 巡逻
rosrun air_ground_bringup ugv_patrol.py

# UGV 坐标监控
rosrun air_ground_bringup ugv_coordinate_monitor.py
```

---

## 配置说明

### 坐标对齐 — `registration.yaml`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `registration_mode` | `one_shot` | `one_shot` / `periodic` / `opportunistic` |
| `minimum_origin_samples` | 30 | Origin 标定所需 UAV 里程计帧数 |
| `minimum_samples` | 20 | 批估计所需最小样本数 |
| `registration_window_seconds` | 3.0 | 滑动窗口时长 |
| `registration_window_max_samples` | 60 | 滑动窗口最大样本数 |
| `max_translation_residual` | 0.12 m | 平移离群点门限 |
| `max_yaw_residual` | 0.03 rad | 偏航离群点门限 |
| `innovation_mahalanobis_threshold` | 11.34 | KF 新息 χ² 门限（df=3, 99%） |
| `translation_time_variance_rate` | 0.0004 m²/s | 平移过程噪声（时间） |
| `translation_uav_distance_variance_rate` | 0.0009 m²/m | 平移过程噪声（UAV 里程） |
| `translation_ugv_distance_variance_rate` | 0.0016 m²/m | 平移过程噪声（UGV 里程） |
| `yaw_*_variance_rate` | 见文件 | 偏航过程噪声（同上三项） |
| `uav_base_to_camera_translation` | `[0, 0, -0.17]` | UAV 机体系 → 下视相机光心 |
| `ugv_base_to_board_translation` | `[-0.3125, -0.1875, 0.115]` | UGV 机体系 → ChArUco 板原点 |

### 感知 — `charuco.yaml`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `dictionary` | `DICT_5X5_100` | ArUco 字典 |
| `squares_x` / `squares_y` | 7 / 5 | 棋盘格数 |
| `square_length` | 0.075 m | 格子边长 |
| `marker_length` | 0.055 m | Marker 边长 |
| `minimum_markers` | 4 | 最小检测 Marker 数 |
| `minimum_corners` | 12 | 最小 ChArUco 角点数 |
| `maximum_reprojection_error_px` | 0.8 | 最大重投影 RMSE |

### 世界边界 — `world_boundaries.yaml`

定义 Gazebo 墙体在 `iris_0/odom` 系中的位姿（已补偿 PX4 起飞点 `x=-6` 的偏移）。

---

## 话题与 TF 树

### 关键话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/iris_0/mavros/local_position/odom` | `nav_msgs/Odometry` | UAV 里程计（PX4 EKF2） |
| `/ugv_0/odom` | `nav_msgs/Odometry` | UGV 里程计 |
| `/iris_0/nadir_camera/image_raw` | `sensor_msgs/Image` | 下视相机图像 |
| `/air_ground/charuco/observation` | `PoseWithCovarianceStamped` | ChArUco 板位姿观测 |
| `/air_ground/charuco/valid` | `Bool` | 观测有效标志 |
| `/air_ground/registration/estimate` | `PoseWithCovarianceStamped` | 配准估计 `[tx, ty, yaw]` |
| `/air_ground/registration/accepted_update` | `RegistrationUpdate` | latched accepted event，含显式 revision 与对应 covariance |
| `/air_ground/registration/revision` | `UInt32` | latched 当前 revision；首次成功配准为 1 |
| `/air_ground/registration/valid` | `Bool` | 配准有效标志 |
| `/air_ground/registration/status` | `String` | latched 状态机（`CAPTURING_ORIGIN` / `ACQUIRING_*` / `TRACKING` / `FROZEN` / `REJECTED`） |
| `/air_ground/ugv/pose_takeoff` | `PoseWithCovarianceStamped` | UGV 在统一系中的位姿 |
| `/coordinate_transform/uav_to_ugv` | `TransformStamped` | C++ 节点的 `iris_0/odom → ugv_0/odom` |
| `iris_0/position_cmd` | `quadrotor_msgs/PositionCommand` | Ego-Planner 轨迹指令 |
| `iris_0/mavros/setpoint_raw/local` | `PositionTarget` | 飞控速度指令 |

### TF 树

```
map ──→ air_ground_origin ──→ iris_0/odom ──→ iris_0/base_link ──→ iris_0/nadir_camera_optical_frame
                             └─────────────→ ugv_0/odom ──→ ugv_0/base_link ──→ ugv_0/fiducial (board)
```

- `air_ground_origin → iris_0/odom`：起飞前标定，静态或低频更新
- `air_ground_origin → ugv_0/odom`：滤波估计结果
- `iris_0/base_link → nadir_camera_optical_frame`：静态外参发布

Research experiment 使用 `air_ground_experiment/uav_odom` 和 `air_ground_experiment/ugv_odom` 替代两个 raw odom aliases。`air_ground_origin → air_ground_experiment/ugv_odom` 仅在 accepted revision 1 后由 registration 发布；系统不会广播 injected `air_ground_experiment/ugv_odom ↔ ugv_0/odom` bridge。Research UGV controller 直接消费 experimental UGV odometry，避免通过 TF 暴露注入扰动。

配准成功与否应以 latched `/air_ground/registration/accepted_update`、`/air_ground/registration/revision` 和 `/air_ground/registration/status` 为准。`/rosout` 只用于读取 accepted/rejected 诊断原因，不提供历史事件重放，晚启动监听可能看不到早期日志。

---

## 目录结构

```
Ego_Planner_v2/
├── src/
│   ├── main_ws/src/
│   │   ├── planner/          # Ego-Planner 核心
│   │   │   ├── plan_manage/  # 规划管理
│   │   │   ├── plan_env/     # 环境感知 / 占据栅格
│   │   │   ├── path_searching/ # A* / Kinodynamic 搜索
│   │   │   ├── traj_opt/     # 轨迹优化
│   │   │   ├── traj_utils/   # 轨迹工具
│   │   │   └── swarm_bridge/ # 多机桥接
│   │   └── Utils/            # 通用工具（quadrotor_msgs 等）
│   ├── air_ground_coordinate_transform/
│   │   ├── src/
│   │   │   ├── coordinate_transform_node.cpp
│   │   │   └── air_ground_coordinate_transform/
│   │   │       ├── registration_estimator.py  # 批估计 + KF
│   │   │       ├── registration_coordinator.py # 窗口协调
│   │   │       ├── odom_buffer.py            # 里程计插值
│   │   │       └── se2.py                    # SE(2) 数学
│   │   ├── scripts/takeoff_registration.py
│   │   ├── config/{registration,coordinate_transform}.yaml
│   │   └── msg/RegistrationUpdate.msg
│   ├── air_ground_perception/
│   │   ├── scripts/{charuco_detector,red_sphere_detector}.py
│   │   └── config/charuco.yaml
│   ├── air_ground_bringup/
│   │   ├── launch/{mvp_system,air_ground_demo,uav_sitl}.launch
│   │   ├── scripts/{uav_follow_mission,ugv_patrol,...}.py
│   │   └── config/world_boundaries.yaml
│   ├── air_ground_ugv_gazebo/  # UGV 模型与 Gazebo 插件
│   ├── px4ctrl/                # PX4 控制器
│   └── cxr_egoctrl_v1/         # Ego → MAVROS 桥接
├── docs/
├── script/
└── test_bags/
```

---

## 开发与测试

### 运行单元测试

```bash
# 坐标对齐模块测试
catkin_make run_tests_air_ground_coordinate_transform
rostest air_ground_coordinate_transform registration_node.test

# 单测（pytest）
python3 -m pytest src/air_ground_coordinate_transform/test/ -v
```

测试覆盖：`test_se2` / `test_odom_buffer` / `test_registration_estimator` / `test_registration_coordinator` / `test_registration_node` 等。

### 常见问题

| 现象 | 排查 |
|------|------|
| 配准状态卡在 `CAPTURING_ORIGIN` | 检查 UAV 是否已起飞且高度 > 1.2 m，里程计话题是否正常 |
| `REJECTED: insufficient_inliers` | 增大 `registration_window_seconds` 或检查 ChArUco 检测有效性 (`/air_ground/charuco/valid`) |
| TF 抖动 / 跳变 | 检查 `max_odom_bracket` 是否过小，或降低 `maximum_uav_speed` 门限 |
| Ego-Planner 不规划 | 确认深度图话题 `/iris_0/realsense/depth_camera/depth/image_raw` 正常，`pointcloud` 有数据 |
| Gazebo 中 UAV 漂移 | 检查 PX4 EKF2 是否收敛，`mavros/state` 是否 `connected` |

---

## 致谢

- Ego-Planner：[ZJU-FAST-Lab/Ego-Planner](https://github.com/ZJU-FAST-Lab/Ego-Planner)
- PX4 控制器修改自 [ZJU-FAST-Lab/Fast-Drone-250](https://github.com/ZJU-FAST-Lab/Fast-Drone-250)

---

## 许可证

各子包见对应 `package.xml` 中的 `license` 字段（BSD-3-Clause / GPLv3 等）。

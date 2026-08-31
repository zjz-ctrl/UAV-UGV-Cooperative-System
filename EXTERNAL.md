# EXTERNAL — 独立/外部项目管理台账

顶层主仓**不跟踪**下列目录（见 `.gitignore`）。它们各自维护独立 Git 或作为参考存档。
对任何一项做升级/迁移前，先读本文件并核对其 pinned commit。

---

## 1. PX4-Autopilot

| 项 | 值 |
|---|---|
| 用途 | PX4 v1.13 SITL 仿真基座 + 真机固件（全系统唯一飞控栈） |
| 路径 | `PX4-Autopilot/` |
| Git remote | https://github.com/PX4/PX4-Autopilot.git（官方） |
| branch | `sdu/uav-ugv-overlay`（工作分支）；`main` 保持与官方一致 `8cc6d02af3`，**永不 pull/merge 官方新 main**（v1.14+ 结构不兼容） |
| commit | `4e7059a25d` — SDU/UAV-UGV overlay：13 个自定义 launch、10023_SDU_UAV 机架注册、mybot config、.vscode、sitl_gazebo 子模块资产指针 |
| 子模块 | `Tools/sitl_gazebo` @ `bff292d`（分支 `sdu/uav-ugv-overlay`，含 SDU_UAV / iris_downcam / iris_D435i / r1_rover_tag / ego*.world / yopo / circle 赛道等全部自定义模型与世界） |
| KNOWN DIRTY | ① `Tools/flightgear_bridge`：嵌套 FlightGear 模型子模块先存文件缺失（本工程不用 FlightGear，不修）；② `Tools/sitl_gazebo/.gitignore`：官方 jinja 生成机制追加 2 行（iris_downcam / r1_rover_tag 的 SDF 生成登记）。两者均只记录、不 reset/clean/restore |
| 参与主工程编译 | **否** — 独立 `make px4_sitl_default` + `make px4_sitl_default sitl_gazebo`（插件是 EXCLUDE_FROM_ALL 的 ExternalProject，必须带 sitl_gazebo 目标） |
| 使用方式 | `scripts/env.sh` 注入 ROS_PACKAGE_PATH / GAZEBO_MODEL_PATH / GAZEBO_PLUGIN_PATH，`$(find px4)`、`$(find mavlink_sitl_gazebo)` 可用 |
| 自定义资产母本 | `px4_overlay/`（改模型/world/launch/机架一律改母本，再 `./px4_overlay/deploy_overlay.sh` 下发；先 `--dry-run`） |

## 2. CarMavlink

| 项 | 值 |
|---|---|
| 用途 | 无人车端 QGC MAVLink 网关（状态上报/摇杆→cmd_vel/失联保护）+ ROS 图像→GStreamer RTP 视频推流 |
| 路径 | `deploy/CarMavlink/` |
| Git remote | https://github.com/77bbq/CarMavlink.git（自研独立仓） |
| branch / commit | `main` @ `e0dd539` |
| 本地修改 | 2 个未提交文件（ros_video_bridge 的默认图像话题与 udp_host 调整） |
| 参与主工程编译 | **否** — 部署在无人车工控机（另一台机器），不进 main_ws、不 catkin_make |
| 部署位置 | 无人车工控机；QGC IP 等部署配置在其 `src/car_mavlink/config/gateway.yaml` |

## 3. YOPO（yopo_ws）

| 项 | 值 |
|---|---|
| 用途 | 学习型局部规划器（参考/备选），输出与 EGO 同接口的 `/position_cmd`（quadrotor_msgs/PositionCommand），与 EGO 二选一 |
| 路径 | `reference/yopo_ws/`（git 仓在内层 `reference/yopo_ws/src/YOPO/`） |
| Git remote | https://github.com/TJU-Aerial-Robotics/YOPO.git |
| branch / commit | `YOPO-Simple` @ `59933f7` |
| 本地修改 | 138 处未提交定制（Controller/so3_control、mavros_interface、话题对接等）——**一旦误 checkout/重置即丢失，操作前务必先处理** |
| 参与主工程编译 | **否**；不进入主 ROS_PACKAGE_PATH |
| 备注 | 运行需 conda env `yopo`（PyTorch 2.4.1+cu118）+ CUDA 仿真器；`main_ws/src/cxr_egoctrl_v1` 内已内置 `yopo_cxr_bridge.py` 适配 |

## 4. ColAG

| 项 | 值 |
|---|---|
| 用途 | ICRA2024 空地协同引导论文参考实现（浙大 FAST-Lab：UGV 盲区预测 + UAV VRPTW 引导分配 + EKF 位姿融合） |
| 路径 | `reference/ColAG_ws/`（git 仓在内层 `reference/ColAG_ws/ColAG/`，内含 Air_ws/Ground_ws/MARSIM_ws 三套独立 catkin ws） |
| Git remote | https://github.com/FAST-FIRE/ColAG.git |
| branch / commit | `main` @ `274786c` |
| 本地修改 | 1 处（未核实，原样保留） |
| 参与主工程编译 | **否**；不进入主 ROS_PACKAGE_PATH（其 quadrotor_msgs/px4ctrl/custom_msgs 与主工程隔离） |

## 5. reference/ego_upstream_extra

EGO-Planner 上游**非主链** demo 包 14 个（drone_detect、swarm_bridge、fake_drone(poscmd_2_odom)、local_sensing、map_generator、mockamap、so3_control、so3_quadrotor_simulator、assign_goals、manual_take_over、moving_obstacles、random_goals、rviz_plugins、selected_points_publisher）。无独立 git；仅供查阅上游 demo 用法，**不编译、不入 ROS_PACKAGE_PATH**。

## 6. 内嵌于 main_ws 的 vendored 包溯源

| 包 | 上游 | 说明 |
|---|---|---|
| `main_ws/src/px4ctrl` | https://github.com/A-ppIes/px4ctrl @ `18b1c15`（Fast-Drone-250 系控制器，含本地修改） | 原 `.git` 已摘除并保存于 `archive/legacy/px4ctrl_nested_git/git`（含 15 处未提交差异的历史现场）；源码现由顶层主仓统一跟踪 |
| `main_ws/src/ego_planner/*` | ZJU-FAST-Lab ego-planner 系 | 按依赖闭包选取 9 包，vendor 锁定，由顶层主仓跟踪 |
| `main_ws/src/uav_ugv_coord` | 自研（原 uav_ugv_project） | 顶层主仓跟踪 |

## 7. archive/legacy

迁移前历史快照（含旧 build/devel、旧版 SDU 机架/SDF、旧脚本）：`Ego_Planner_v2/`、`uav_ugv_project/`、`SDU_UAV_package/`。只读存档，任何 env/build/run 脚本不得引用。

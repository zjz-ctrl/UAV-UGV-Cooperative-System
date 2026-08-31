# UAV-UGV Cooperative System

无人机–地面车空地协同统一工程（ROS1 Noetic + Gazebo Classic 11 + PX4 v1.13 SITL）。
所有路径均相对于工程根 `~/UAV-UGV_ws`（脚本内为 `$UAV_UGV_ROOT`）。

## 1. Project Structure

| 目录 | 作用 |
|---|---|
| `main_ws/` | **主 ROS workspace**（唯一参与 catkin_make 的工程）：10 个自研包 + `src/ego_planner/` 内 9 个 EGO 闭包包，共 19 包 |
| `PX4-Autopilot/` | PX4 v1.13 独立固件/SITL 基座（独立 git，不进 catkin_make），自定义资产见 `px4_overlay/` |
| `px4_overlay/` | **我方 PX4 自定义资产维护母本**：SDU_UAV 机型、iris_downcam、r1_rover_tag、iris_D435i、ego/yopo/circle worlds、自定义 launch、config；`deploy_overlay.sh` 下发到 PX4 |
| `deploy/` | 实机部署工程：`deploy/CarMavlink/`（车端 QGC MAVLink 网关 + 视频推流，部署于无人车工控机） |
| `reference/` | 参考项目（不参与主工程编译，不入 ROS_PACKAGE_PATH）：`ColAG_ws`、`yopo_ws`、`ego_upstream_extra`（EGO 上游非主链 demo 包 ×14） |
| `scripts/` | `env.sh`（统一环境）、`build_all.sh`、`clean_build.sh`、`run_sim.sh`、`smoke_test.sh`、`legacy/`（旧脚本存档） |
| `tools/` | 标定/诊断工具（`calibration/`：fine_calib、multi_calib 等） |
| `docs/` | 工程文档（`ego_planner_v2/`：原主工程文档与 SDD 开发记录） |
| `logs/` | 编译与运行日志（不入库） |
| `archive/` | 历史归档（`legacy/`：迁移前快照，只读） |

外部/独立项目管理与版本锁定见 **[EXTERNAL.md](EXTERNAL.md)**。

## 2. Environment

每个新终端：

```bash
source ~/UAV-UGV_ws/scripts/env.sh
```

该脚本幂等（重复 source 不产生重复路径），注入 UAV_UGV_ROOT、ROS noetic、PX4 SITL 路径、
Gazebo 模型/插件路径，并 source `main_ws/devel/setup.bash`。**不修改 ~/.bashrc，与旧工程共存。**

## 3. Build

```bash
./scripts/build_all.sh      # 一键全量编译
./scripts/clean_build.sh    # 只清 build/devel 等可重建产物，绝不触碰源码
```

实际执行：
- PX4：`make px4_sitl_default`（固件）+ `make px4_sitl_default sitl_gazebo`（Gazebo 插件，v1.13 为独立 ExternalProject，缺它则无插件）
- ROS：`cd main_ws && catkin_make`

## 4. Run

```bash
./scripts/run_sim.sh                          # 默认参数（GUI + RViz + 自动起飞）
./scripts/run_sim.sh gui:=false start_rviz:=false auto_takeoff:=false   # 常用冒烟组合
```

等价于 `roslaunch air_ground_bringup mvp_system.launch <args>`（唯一主入口，不另起炉灶）。
参数透传 launch 已有定义：`gui / start_ego / start_cxr / start_coordinate_transform / start_rviz / auto_takeoff / uav_depth_topic / uav_x ...`。

## 5. Core Modules

- **UAV / PX4 / MAVROS**：`iris_0` SITL + MAVROS（odom/state/setpoint），机型 SDU_UAV 或 iris_D435i 由 overlay 提供
- **UGV**：Gazebo 差速车（`air_ground_ugv_gazebo`），顶置 ChArUco 板；实车控制走 `deploy/CarMavlink`
- **EGO Planner**：`main_ws/src/ego_planner`（`/position_cmd` → `cxr_egoctrl_v1` → mavros setpoint）
- **coordinate registration**：`air_ground_coordinate_transform`（起飞配准 SE(2)+KF，TF `air_ground_origin`）
- **perception**：`air_ground_perception`（ChArUco / 红球检测）
- **UAV-UGV coordination**：`uav_ugv_coord`（AprilTag 相对定位，map_uav→map_ugv 合并，可单独 `roslaunch uav_ugv_coord coord.launch` 启动）

## 6. External Projects

PX4 / CarMavlink / YOPO / ColAG 的 remote、branch、commit、本地修改状态、是否参与编译：
见 **[EXTERNAL.md](EXTERNAL.md)**。

## 7. Important Rules

1. `reference/`（ColAG、yopo_ws、ego_upstream_extra）**绝不参与主工程编译**，绝不加入 ROS_PACKAGE_PATH。
2. `px4_overlay/` 是 PX4 自定义资产的**唯一母本**——不要在多个副本里同时改同一个模型；
   修改流程：改 overlay → `./px4_overlay/deploy_overlay.sh --dry-run` 核对 → 部署 → 重新 `make px4_sitl_default sitl_gazebo`（如涉及插件）。
3. PX4-Autopilot 保持 v1.13 独立 git；`main` 分支不 pull 官方更新；本地资产在 `sdu/uav-ugv-overlay` 分支。
4. 顶层仓不跟踪 `PX4-Autopilot/`、`reference/`、`deploy/CarMavlink/`、`archive/`、`logs/`（见 `.gitignore`）。

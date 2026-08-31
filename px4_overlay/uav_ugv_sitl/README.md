# uav_ugv_sitl（空地双车 SITL 场景）

本目录只保留说明，**不存放资源副本**，避免形成第三份资产。

UAV-UGV 双车 SITL 的实际维护母本统一在：

- 双车联合 launch：`../launch/uav_ugv.launch`
  （empty_world + 两组命名空间：`uav0`=iris_downcam 下视相机机、`ugv1`=r1_rover_tag 顶置 AprilTag 车；
  内部以 `$(find px4)/launch/spawn_vehicle.launch` + `$(find mavros)/launch/px4.launch` 组装）
- UAV 模型：`../models/iris_downcam`（iris + 下视单目，机体系 -0.17 m 挂架）
- UGV 模型：`../models/r1_rover_tag`（车顶 AprilTag id0）

## 部署方式

在 `px4_overlay/` 下执行：

```bash
./deploy_overlay.sh --dry-run   # 先校验与 PX4 当前版本一致
./deploy_overlay.sh             # 实际部署
```

## PX4 内的已部署副本

- `PX4-Autopilot/launch/uav_ugv.launch`
- `PX4-Autopilot/Tools/sitl_gazebo/models/{iris_downcam, r1_rover_tag}`

二者由 PX4 本地分支 `sdu/uav-ugv-overlay` 保护。今后修改一律改 `px4_overlay/` 母本后用脚本下发。

## 历史来源

原 `uav_ugv_project/px4/`（install.sh 拷贝式部署），与 PX4 部署版逐字节一致后并入本 overlay；
原目录保留在 `uav_ugv_project/` 残留 husk 中作为备份。
配套坐标对齐节点为 `main_ws/src/uav_ugv_coord`（map_uav → map_ugv）。

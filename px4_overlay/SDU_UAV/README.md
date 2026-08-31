# SDU_UAV 自定义无人机 PX4 SITL 移植包

将自定义四旋翼(SDU_UAV)替换 PX4 SITL 默认机型(iris),支持 MAVROS/控制器闭环仿真。

## 环境要求

- Ubuntu 20.04 + ROS Noetic + Gazebo 11
- PX4-Autopilot v1.13.x(已在 ~/PX4-Autopilot 编译过 px4_sitl)
- mavros
- QGroundControl(可选)

## 包内容

```
SDU_UAV_package/
├── deploy.sh                 # 一键部署脚本
├── README.md                 # 本说明
├── models/SDU_UAV/           # 模型文件(由 SolidWorks URDF 转换)
│   ├── SDU_UAV.sdf           # 模型定义(含 PX4 插件)
│   └── meshes/               # STL 网格
└── airframes/10023_SDU_UAV   # PX4 机架文件(混控 + 调校参数)
```

## 部署步骤

```bash
cd SDU_UAV_package
bash deploy.sh ~/PX4-Autopilot     # 参数为对方机器的 PX4 路径
```

脚本会:
1. 复制模型到 `Tools/sitl_gazebo/models/SDU_UAV/`
2. 复制机架文件到 `ROMFS/.../airframes/`
3. 登记进 `airframes/CMakeLists.txt`(自动备份原文件)
4. 修改 `launch/posix_sitl.launch`,使 `PX4_SIM_MODEL` 跟随 `vehicle` 参数(自动备份)

之后重新编译:

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

## 运行

```bash
# 终端1: 启动仿真(等同原来的 fast_test.launch + iris)
source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
roslaunch px4 fast_test.launch vehicle:=SDU_UAV

# 终端2: 你的控制器(话题 /mavros/*, 无前缀)
roslaunch cxr_egoctrl_v1 cxr_egoctrl_v1.launch

# 或 QGC: 连接 UDP 14550
```

切 OFFBOARD 前,控制器需持续发布 setpoint 至少 15 秒(PX4 v1.13 机制,否则被拒)。

## 模型文件说明

### SDU_UAV.sdf 关键设计

| 项目 | 说明 |
|------|------|
| 格式 | SDF 1.7(关节 `<pose>` 带变换,与 gazebo 的 URDF 转换一致) |
| 坐标 | 模型居中于原点,机头 +x,相机朝前(网格内容已修正 SolidWorks 全局偏移) |
| 碰撞体 | 机体=盒体 0.21×0.21×0.1,桨=圆柱 r=0.035,相机=盒体(避免 STL 碰撞卡物理) |
| 质量 | 机体 0.94kg,整机 1.0kg(模拟加重 jetson+电池),惯量同步缩放 |
| 电机 | motorConstant 1.36e-05, maxRotVelocity 600, **时间常数 0.05/0.1s**(关键) |
| 锁步 | `enable_lockstep=0`(规避 PX4 v1.13 gazebo 锁步死锁) |
| 传感器 | IMU/GPS/磁力计/气压计 + mavlink 接口(TCP 4560) |

### 电机插件转向(与混控对应)

```
M1 properller_front_right  ccw   (motorNumber 0)
M2 properller_back_left    ccw   (motorNumber 1)
M3 properller_front_left   cw    (motorNumber 2)
M4 properller_back_right   cw    (motorNumber 3)
混控: quad_w(与 iris 相同)
```

### 网格修改说明

`meshes/base_link.STL` 顶点已平移 (-0.0408, -1.2523, 0.2053):
SolidWorks 导出的机体网格内容偏离文件原点 1.25m,不修正时"机体消失"。
其余 5 个 STL 未修改。**如需换用其他机架网格,请确保网格内容以自身原点为中心。**

## 机架文件参数(10023_SDU_UAV)

为 1.0kg 轻小型机调校(默认值是 1.5kg 的 iris 用的):

```
姿态环:   MC_ROLL_P / MC_PITCH_P = 3.0,  MC_YAW_P = 1.5
角速率环: MC_*RATE_P = 0.04, I = 0.05, D = 0.003, YAW I = 0.02
位置环:   MPC_XY_P = 1.2, MPC_XY_VEL_P_ACC = 2.8, MPC_XY_VEL_I_ACC = 0.6
          MPC_Z_P = 1.2, MPC_Z_VEL_P_ACC = 5.0
起飞:     MPC_TKO_SPEED = 0.8
```

如需重新调参,修改机架文件后重新编译。

## 验证结果(本机)

- 悬停姿态抖动: ±0.08°(roll/pitch)
- 位置漂移: 厘米级(x 0.009, y 0.027)
- 位置阶跃: 超调 10-17%,稳定 2-5s

## 注意事项

1. `make px4_sitl gazebo` 编译完成后会自动启动一次仿真,直接 Ctrl-C 即可
2. 磁盘空间:ROS 日志目录(~/.ros/log)会快速增长,注意清理(`rosclean`)
3. 两个 gazebo 实例不能同时运行
4. 若对方机器 PX4 版本不同(v1.14+),机架文件格式可能有差异,需参考对应版本的 iris 机架文件
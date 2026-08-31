#!/usr/bin/env bash
# =============================================================================
# 主仿真入口: 唯一起点 = roslaunch air_ground_bringup mvp_system.launch
# 所有 launch 已有参数原样透传, 不另造第二套启动逻辑。
# 常用:
#   ./scripts/run_sim.sh                                            # 默认(GUI+RViz+自动起飞)
#   ./scripts/run_sim.sh gui:=false start_rviz:=false auto_takeoff:=false   # 冒烟
#   ./scripts/run_sim.sh start_ego:=false start_cxr:=false          # 只起仿真与 MAVROS
# =============================================================================
set -euo pipefail
ROOT="${UAV_UGV_ROOT:-$HOME/UAV-UGV_ws}"
source "$ROOT/scripts/env.sh"
exec roslaunch air_ground_bringup mvp_system.launch "$@"

#!/usr/bin/env bash
# =============================================================================
# UAV-UGV 一键编译: [PX4 BUILD] -> [ROS BUILD] -> [DONE]
#   PX4: make px4_sitl_default + make px4_sitl_default sitl_gazebo
#        (v1.13 中 Gazebo 插件是 EXCLUDE_FROM_ALL 的 ExternalProject,
#         只跑第一个目标不会有插件)
#   ROS: cd main_ws && catkin_make
# 任何一步失败立即停止(set -e + pipefail), 日志写入 logs/build_all_<时间>.log
# =============================================================================
set -euo pipefail

ROOT="${UAV_UGV_ROOT:-$HOME/UAV-UGV_ws}"
LOG_DIR="$ROOT/logs"; mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/build_all_$(date +%Y%m%d_%H%M%S).log"

echo "[PX4 BUILD] start (log: $LOG)"
( cd "$ROOT/PX4-Autopilot" \
  && make px4_sitl_default \
  && make px4_sitl_default sitl_gazebo ) 2>&1 | tee -a "$LOG"

echo "[ROS BUILD] start"
(
  source /opt/ros/noetic/setup.bash
  # main_ws 已编译过时环境更完整; 未编译也不影响本次 catkin_make
  [ -f "$ROOT/main_ws/devel/setup.bash" ] && source "$ROOT/main_ws/devel/setup.bash" || true
  cd "$ROOT/main_ws"
  catkin_make
) 2>&1 | tee -a "$LOG"

echo "[DONE] all builds finished"

#!/bin/bash
# ============================================================
# SDU_UAV 机型补丁部署 —— 兼容入口/包装器
#
# 原 SDU_UAV_package/deploy.sh 的独立部署逻辑已并入统一脚本:
#   px4_overlay/deploy_overlay.sh
# (一次性部署 overlay 全部资产: models/worlds/launch/config/airframe,
#  幂等、无 .bak、不删除 PX4 内文件。)
#
# 用法:
#   ./deploy.sh [--dry-run]
#   PX4_DIR=/path/to/PX4 ./deploy.sh [--dry-run]
#
# 目标 PX4 目录优先级: $PX4_DIR > $UAV_UGV_ROOT/PX4-Autopilot > ~/UAV-UGV_ws/PX4-Autopilot
# ============================================================
set -u
OVERLAY_TOP="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PX4_DIR="${PX4_DIR:-${UAV_UGV_ROOT:-$HOME/UAV-UGV_ws}/PX4-Autopilot}"
exec bash "$OVERLAY_TOP/deploy_overlay.sh" "$@"

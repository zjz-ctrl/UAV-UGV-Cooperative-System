#!/usr/bin/env bash
# =============================================================================
# 清理可重建构建产物(白名单式; 绝不触碰 src/launch/config/models 等源码资源)
#   - main_ws: build/ devel/ install/ log/
#   - PX4    : build/px4_sitl_default (含 build_gazebo 插件, 重建由 build_all.sh 覆盖)
# =============================================================================
set -euo pipefail
ROOT="${UAV_UGV_ROOT:-$HOME/UAV-UGV_ws}"

echo "[CLEAN] main_ws/build devel install log"
rm -rf "$ROOT/main_ws/build" "$ROOT/main_ws/devel" "$ROOT/main_ws/install" "$ROOT/main_ws/log"

echo "[CLEAN] PX4-Autopilot/build/px4_sitl_default"
rm -rf "$ROOT/PX4-Autopilot/build/px4_sitl_default"

echo "[DONE] clean finished (源码未触碰; 下一步: ./scripts/build_all.sh)"

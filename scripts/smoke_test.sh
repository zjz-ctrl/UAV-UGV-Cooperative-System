#!/usr/bin/env bash
# =============================================================================
# 静态冒烟检查(只读, 不启动仿真):
#   1) 关键包 rospack 解析且路径属于 $UAV_UGV_ROOT, 不含 archive/reference/旧工程路径
#   2) main_ws/devel/setup.bash 存在
#   3) PX4 bin/px4 存在
#   4) Gazebo 插件目录有产物
#   5) ROS_PACKAGE_PATH / GAZEBO_* 无禁用路径
# =============================================================================
set -u
ROOT="${UAV_UGV_ROOT:-$HOME/UAV-UGV_ws}"
FAIL=0

source "$ROOT/scripts/env.sh" >/dev/null 2>&1 || { echo "[FAIL] env.sh source 失败"; exit 1; }

FORBIDDEN_RE="archive|reference|/home/zjz/Ego_Planner_v2|/home/zjz/PX4-Autopilot|/home/zjz/air_ground_cooperation|/home/ymy"

check_pkg() {
  local p="$1" r
  if ! r=$(rospack find "$p" 2>/dev/null); then
    echo "[FAIL] rospack find $p (未找到)"; FAIL=1; return
  fi
  case "$r" in
    "$ROOT"/*) echo "[OK]   $p -> $r" ;;
    *) echo "[FAIL] $p -> $r (不在 $ROOT 下)"; FAIL=1; return ;;
  esac
  if printf '%s' "$r" | grep -qE "$FORBIDDEN_RE"; then
    echo "[FAIL] $p 命中禁用路径"; FAIL=1
  fi
}

check_pkg px4
check_pkg air_ground_bringup
check_pkg ego_planner
check_pkg uav_ugv_coord

if [ -f "$ROOT/main_ws/devel/setup.bash" ]; then
  echo "[OK]   main_ws/devel/setup.bash"
else
  echo "[FAIL] main_ws/devel/setup.bash 缺失 (先运行 ./scripts/build_all.sh)"; FAIL=1
fi

if [ -f "$ROOT/PX4-Autopilot/build/px4_sitl_default/bin/px4" ]; then
  echo "[OK]   PX4 build/px4_sitl_default/bin/px4"
else
  echo "[FAIL] PX4 bin/px4 缺失"; FAIL=1
fi

NSO=$(ls "$ROOT/PX4-Autopilot/build/px4_sitl_default/build_gazebo"/*.so 2>/dev/null | wc -l)
if [ "$NSO" -ge 10 ]; then
  echo "[OK]   Gazebo 插件 $NSO 个"
else
  echo "[FAIL] Gazebo 插件不足 ($NSO 个) — 需要 make px4_sitl_default sitl_gazebo"; FAIL=1
fi

for v in ROS_PACKAGE_PATH GAZEBO_MODEL_PATH GAZEBO_PLUGIN_PATH; do
  val="${!v}"
  if printf '%s' "$val" | grep -qE "$FORBIDDEN_RE"; then
    echo "[FAIL] \$$v 含禁用路径"; FAIL=1
  fi
done

if [ "$FAIL" -eq 0 ]; then
  echo "==== SMOKE TEST PASS ===="
  exit 0
else
  echo "==== SMOKE TEST FAIL ===="
  exit 1
fi

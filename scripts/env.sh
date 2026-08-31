#!/usr/bin/env bash
# =============================================================================
# UAV-UGV 统一环境脚本（必须 source，勿直接执行）
#
#   source ~/UAV-UGV_ws/scripts/env.sh
#
# 特性:
#   - 幂等: 重复 source 不会让 ROS_PACKAGE_PATH / GAZEBO_MODEL_PATH 增长重复项;
#   - 自包含: 不引用任何旧工程路径(/home/zjz/Ego_Planner_v2 等, 一律不出现);
#   - 不写 ~/.bashrc, 与旧工程共存(旧工程在干净终端按原方式 source)。
# =============================================================================

# 兼容调用方 set -u
_ugv_nounset=false
case $- in *u*) _ugv_nounset=true; set +u ;; esac

# ---- 1. 根目录 --------------------------------------------------------------
export UAV_UGV_ROOT="${UAV_UGV_ROOT:-$HOME/UAV-UGV_ws}"
export UAV_UGV_PX4="$UAV_UGV_ROOT/PX4-Autopilot"
export UAV_UGV_ROS_WS="$UAV_UGV_ROOT/main_ws"

# ---- 2. 路径工具 ------------------------------------------------------------
# 追加(已存在则跳过)
_ugv_add_path() {
  local var="$1" val="$2"
  local cur="${!var}"
  case ":$cur:" in *":$val:"*) return 0 ;; esac
  if [ -z "$cur" ]; then printf -v "$var" '%s' "$val"
  else printf -v "$var" '%s:%s' "$cur" "$val"
  fi
  export "$var"
}
# 去重(按字符串保序去重, 收敛各 source 累积的重复项)
_ugv_dedupe() {
  local var="$1"
  local val="${!var}" out="" seen="|" p
  local _ifs="$IFS"
  IFS=':'
  for p in $val; do
    [ -n "$p" ] || continue
    case "$seen" in *"|$p|"*) continue ;; esac
    seen="$seen$p|"
    if [ -z "$out" ]; then out="$p"; else out="$out:$p"; fi
  done
  IFS="$_ifs"
  printf -v "$var" '%s' "$out"
  export "$var"
}

# ---- 3. ROS -----------------------------------------------------------------
if [ -z "${ROS_DISTRO:-}" ]; then
  source /opt/ros/noetic/setup.bash
fi

# ---- 4. PX4 -----------------------------------------------------------------
if [ ! -d "$UAV_UGV_PX4" ]; then
  echo "[ERROR] PX4 not found: $UAV_UGV_PX4" >&2
  if [ "$_ugv_nounset" = true ]; then set -u; fi
  return 1 2>/dev/null || exit 1
fi
# PX4 v1.13 约定: source setup_gazebo.bash <px4_root> <build_dir>
# (该脚本追加 GAZEBO_PLUGIN_PATH/GAZEBO_MODEL_PATH/LD_LIBRARY_PATH, 不碰 ROS_PACKAGE_PATH)
if [ -f "$UAV_UGV_PX4/Tools/setup_gazebo.bash" ]; then
  source "$UAV_UGV_PX4/Tools/setup_gazebo.bash" \
         "$UAV_UGV_PX4" "$UAV_UGV_PX4/build/px4_sitl_default"
fi

# ---- 5. main_ws devel -------------------------------------------------------
# 注意: catkin 的 devel/setup.bash 会整体重写 ROS_PACKAGE_PATH,
# 因此 PX4 路径注入必须放在它之后(见第 6 节)。
if [ -f "$UAV_UGV_ROS_WS/devel/setup.bash" ]; then
  source "$UAV_UGV_ROS_WS/devel/setup.bash"
else
  echo "[INFO] main_ws has not been built yet"
fi

# ---- 6. ROS_PACKAGE_PATH: 保证可解析 px4 与 mavlink_sitl_gazebo -------------
_ugv_add_path ROS_PACKAGE_PATH "$UAV_UGV_PX4"
_ugv_add_path ROS_PACKAGE_PATH "$UAV_UGV_PX4/Tools/sitl_gazebo"

# ---- 6b. GAZEBO_MODEL_PATH: PX4 模型 + 主工程 UGV 模型 -----------------------
_ugv_add_path GAZEBO_MODEL_PATH "$UAV_UGV_PX4/Tools/sitl_gazebo/models"
if [ -d "$UAV_UGV_ROS_WS/src/air_ground_ugv_gazebo/models" ]; then
  _ugv_add_path GAZEBO_MODEL_PATH "$UAV_UGV_ROS_WS/src/air_ground_ugv_gazebo/models"
fi

# ---- 8. 收敛去重 ------------------------------------------------------------
_ugv_dedupe ROS_PACKAGE_PATH
_ugv_dedupe GAZEBO_MODEL_PATH
_ugv_dedupe GAZEBO_PLUGIN_PATH
_ugv_dedupe LD_LIBRARY_PATH

# ---- 9. 汇报 ----------------------------------------------------------------
echo "========================================"
echo " UAV-UGV NEW ENVIRONMENT"
echo "========================================"
echo "ROOT   : $UAV_UGV_ROOT"
echo "ROS_WS : $UAV_UGV_ROS_WS"
echo "PX4    : $UAV_UGV_PX4"
echo "ROS    : ${ROS_DISTRO:-unknown}"
echo "========================================"
echo "ROS_PACKAGE_PATH  = $ROS_PACKAGE_PATH"
echo "GAZEBO_MODEL_PATH = $GAZEBO_MODEL_PATH"

if [ "$_ugv_nounset" = true ]; then set -u; fi

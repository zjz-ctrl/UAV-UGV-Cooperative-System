#!/usr/bin/env bash
# =============================================================================
# px4_overlay 统一部署脚本：把"我方 PX4 自定义资产的维护母本"下发到 PX4-Autopilot。
#
# 用法:
#   ./deploy_overlay.sh [--dry-run]
#   PX4_DIR=/path/to/PX4 ./deploy_overlay.sh [--dry-run]
#
# 缺省目标: $UAV_UGV_ROOT/PX4-Autopilot (未定义时 ~/UAV-UGV_ws/PX4-Autopilot)
#
# 特性:
#   - 幂等: 目标与母本内容一致时零写入(不触碰 mtime);
#   - 不产生 .bak; 不删除 PX4 内任何既有文件; 不使用 rsync --delete;
#   - CMakeLists 机架注册幂等(已注册则跳过);
#   - 部署前检查目标目录存在。
# =============================================================================
set -u

OVERLAY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-${UAV_UGV_ROOT:-$HOME/UAV-UGV_ws}/PX4-Autopilot}"
DRY_RUN=0
[ "${1:-}" = "--dry-run" ] && DRY_RUN=1

SG_DIR="$PX4_DIR/Tools/sitl_gazebo"

if [ ! -d "$PX4_DIR" ]; then
  echo "[ERROR] PX4 dir not found: $PX4_DIR" >&2
  exit 1
fi
for d in "$SG_DIR/models" "$SG_DIR/worlds" "$PX4_DIR/launch" \
         "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"; do
  if [ ! -d "$d" ]; then
    echo "[ERROR] required target dir missing: $d" >&2
    exit 1
  fi
done

# 单文件/目录树同步: 一致则跳过, 不同则 ADD/UPDATE (cp -a, 无 .bak, 不删除)
deploy_entry() { # $1=src $2=dst $3=tag
  local src="$1" dst="$2" tag="$3" act
  if [ ! -e "$dst" ]; then act="ADD"
  elif diff -rq "$src" "$dst" >/dev/null 2>&1; then return 0
  else act="UPDATE"
  fi
  echo "  [$tag] $act $dst"
  [ "$DRY_RUN" -eq 1 ] && return 0
  mkdir -p "$(dirname "$dst")"
  cp -a "$src" "$dst"
  return 0
}

echo "PX4_DIR = $PX4_DIR   (dry-run: $([ "$DRY_RUN" -eq 1 ] && echo yes || echo no))"

echo "[MODEL]"
for m in "$OVERLAY_DIR"/models/*; do
  [ -e "$m" ] || continue
  deploy_entry "$m" "$SG_DIR/models/$(basename "$m")" MODEL
done

echo "[WORLD]"
for w in "$OVERLAY_DIR"/worlds/*; do
  [ -e "$w" ] || continue
  deploy_entry "$w" "$SG_DIR/worlds/$(basename "$w")" WORLD
done

echo "[LAUNCH]"
for l in "$OVERLAY_DIR"/launch/*; do
  [ -f "$l" ] || continue   # deprecated/ 子目录不部署
  deploy_entry "$l" "$PX4_DIR/launch/$(basename "$l")" LAUNCH
done

echo "[CONFIG]"
if [ ! -d "$PX4_DIR/config" ]; then
  echo "  [CONFIG] create $PX4_DIR/config"
  [ "$DRY_RUN" -eq 0 ] && mkdir -p "$PX4_DIR/config"
fi
for c in "$OVERLAY_DIR"/config/*; do
  [ -e "$c" ] || continue
  deploy_entry "$c" "$PX4_DIR/config/$(basename "$c")" CONFIG
done

echo "[AIRFRAME]"
AF_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
deploy_entry "$OVERLAY_DIR/SDU_UAV/airframes/10023_SDU_UAV" \
             "$AF_DIR/10023_SDU_UAV" AIRFRAME
CMAKE="$AF_DIR/CMakeLists.txt"
if grep -q '10023_SDU_UAV' "$CMAKE"; then
  echo "  [AIRFRAME] CMakeLists registration: already present"
else
  echo "  [AIRFRAME] CMakeLists registration: WOULD ADD (after 10020_if750a)"
  if [ "$DRY_RUN" -eq 0 ]; then
    awk '{print} /10020_if750a/{print "\t10023_SDU_UAV"}' "$CMAKE" > "$CMAKE.tmp" \
      && mv "$CMAKE.tmp" "$CMAKE"
    grep -q '10023_SDU_UAV' "$CMAKE" \
      && echo "  [AIRFRAME] CMakeLists registration: added" \
      || { echo "  [ERROR] failed to register airframe, do it manually" >&2; exit 1; }
  fi
fi

if [ "$DRY_RUN" -eq 1 ]; then
  echo "[DONE] PX4 overlay dry-run passed (no changes written)"
else
  echo "[DONE] PX4 overlay deployed"
fi

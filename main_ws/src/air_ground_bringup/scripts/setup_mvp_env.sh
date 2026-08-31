#!/usr/bin/env bash
# Source this file as the final setup action before launching the MVP.
#
# It rebuilds every search path from a canonical, alias-collapsed baseline so
# one physical workspace resolves exactly once:
#   - legacy spellings (/home/zjz/Ego_Planner_v2, /home/zjz/PX4-Autopilot,
#     /home/zjz/air_ground_cooperation/*) are rewritten onto the UAV-UGV_ws
#     trees;
#   - entries are de-duplicated by PHYSICAL directory (readlink -m), matching
#     how rospack/roslaunch actually resolve through symlinks;
#   - sourcing repeatedly converges to the same environment (idempotent);
#   - the ~/.ros/rospack_cache is purged and re-profiled on every source so a
#     stale dual-root listing can never resurface to roslaunch.

_mvp_nounset_enabled=false
case $- in
  *u*) _mvp_nounset_enabled=true; set +u ;;
esac

MVP_CANONICAL_WORKSPACE="${UAV_UGV_ROOT:-$HOME/UAV-UGV_ws}/main_ws"
MVP_CANONICAL_PX4="${UAV_UGV_ROOT:-$HOME/UAV-UGV_ws}/PX4-Autopilot"

# Logical paths: pwd without -P preserves the canonical symlink spelling,
# while every existence check below traverses the links transparently.
_mvp_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_mvp_workspace="$(cd "${_mvp_script_dir}/../../.." && pwd)"
_mvp_px4_dir="$(cd "${_mvp_workspace}/.." && pwd)/PX4-Autopilot"

_mvp_abort() {
  local message="$1"
  local restore_nounset="${_mvp_nounset_enabled}"
  printf '%s\n' "${message}" >&2
  unset _mvp_script_dir _mvp_workspace _mvp_px4_dir _mvp_nounset_enabled
  unset _mvp_required_paths _mvp_required_path
  unset _mvp_catkin_marker
  unset _mvp_forbidden_prefixes _mvp_forbidden
  unset _mvp_search_vars _mvp_var
  unset -f _mvp_abort _mvp_rewrite_alias _mvp_export_merged
  if [ "${restore_nounset}" = true ]; then
    set -u
  fi
  return 1
}

# Canonical-path policy: reject direct references to the legacy trees before
# anything else runs.
_mvp_forbidden_prefixes=(
  "/home/zjz/Ego_Planner_v2"
  "/home/zjz/PX4-Autopilot"
  "/home/zjz/air_ground_cooperation"
)
for _mvp_forbidden in "${_mvp_forbidden_prefixes[@]}"; do
  case "${_mvp_workspace}" in
    "${_mvp_forbidden}"|"${_mvp_forbidden}"/*)
      _mvp_abort "Forbidden legacy path '${_mvp_workspace}'. Source this wrapper only via ${MVP_CANONICAL_WORKSPACE} so the environment references ${MVP_CANONICAL_PX4}."
      return 1 2>/dev/null || exit 1
      ;;
  esac
  case "${_mvp_px4_dir}" in
    "${_mvp_forbidden}"|"${_mvp_forbidden}"/*)
      _mvp_abort "Forbidden legacy path '${_mvp_px4_dir}'. Source this wrapper only via ${MVP_CANONICAL_WORKSPACE} so PX4 resolves as ${MVP_CANONICAL_PX4}."
      return 1 2>/dev/null || exit 1
      ;;
  esac
done
unset _mvp_forbidden_prefixes _mvp_forbidden

_mvp_required_paths=(
  /opt/ros/noetic/setup.bash
  "${_mvp_workspace}/devel/setup.bash"
  "${_mvp_workspace}/devel/.catkin"
  "${_mvp_workspace}/src/air_ground_bringup/package.xml"
  "${_mvp_px4_dir}/package.xml"
  "${_mvp_px4_dir}/Tools/setup_gazebo.bash"
  "${_mvp_px4_dir}/Tools/sitl_gazebo/package.xml"
  "${_mvp_px4_dir}/build/px4_sitl_default"
)
for _mvp_required_path in "${_mvp_required_paths[@]}"; do
  if [ ! -e "${_mvp_required_path}" ]; then
    _mvp_abort "Missing required MVP setup path: ${_mvp_required_path}"
    return 1 2>/dev/null || exit 1
  fi
done
unset _mvp_required_paths _mvp_required_path

if [ "${_mvp_workspace}" != "${MVP_CANONICAL_WORKSPACE}" ]; then
  _mvp_abort "MVP setup must be sourced through the canonical workspace path ${MVP_CANONICAL_WORKSPACE}; got ${_mvp_workspace}."
  return 1 2>/dev/null || exit 1
fi
if [ "${_mvp_px4_dir}" != "${MVP_CANONICAL_PX4}" ]; then
  _mvp_abort "MVP setup must resolve PX4 through the canonical workspace path ${MVP_CANONICAL_PX4}; got ${_mvp_px4_dir}."
  return 1 2>/dev/null || exit 1
fi

# catkin_find searches every source space recorded in this marker. Builds made
# through both symlink spellings accumulate two paths to the same packages,
# which roslaunch reports as duplicate resources after resolving symlinks.
_mvp_catkin_marker="${_mvp_workspace}/devel/.catkin"
if ! printf '%s' "${_mvp_workspace}/src" > "${_mvp_catkin_marker}"; then
  _mvp_abort "Failed to normalize catkin workspace marker: ${_mvp_catkin_marker}"
  return 1 2>/dev/null || exit 1
fi

# ---- alias-aware path machinery -------------------------------------------

# Rewrite a single path entry onto canonical spellings when it names the
# legacy physical trees through their well-known prefixes.
_mvp_rewrite_alias() {
  local entry="$1"
  case "${entry}" in
    "/home/zjz/Ego_Planner_v2"|"/home/zjz/Ego_Planner_v2"/*)
      entry="${MVP_CANONICAL_WORKSPACE}${entry#/home/zjz/Ego_Planner_v2}" ;;
    "/home/zjz/PX4-Autopilot"|"/home/zjz/PX4-Autopilot"/*)
      entry="${MVP_CANONICAL_PX4}${entry#/home/zjz/PX4-Autopilot}" ;;
    "/home/zjz/air_ground_cooperation/Ego_Planner_v2"|"/home/zjz/air_ground_cooperation/Ego_Planner_v2"/*)
      entry="${MVP_CANONICAL_WORKSPACE}${entry#/home/zjz/air_ground_cooperation/Ego_Planner_v2}" ;;
    "/home/zjz/air_ground_cooperation/PX4-Autopilot"|"/home/zjz/air_ground_cooperation/PX4-Autopilot"/*)
      entry="${MVP_CANONICAL_PX4}${entry#/home/zjz/air_ground_cooperation/PX4-Autopilot}" ;;
  esac
  printf '%s' "${entry}"
}

# Rebuild one PATH-like variable from ordered head entries followed by what is
# already stored there. Every item passes through alias rewriting and de-dup-
# lication keyed on its resolved physical directory, so symlink aliases of one
# workspace can never appear twice. Called without heads it simply re-canoni-
# calizes the current value in place.
_mvp_export_merged() {
  local variable_name="$1"
  shift
  local item real out=""
  local -a items=()
  local -A seen_head=() seen_rest=()
  for item in "$@"; do
    item="$(_mvp_rewrite_alias "${item}")"
    [ -n "${item}" ] || continue
    real="$(readlink -m "${item}" 2>/dev/null || printf '%s' "${item}")"
    [ -n "${seen_head[${real}]+x}" ] && continue
    seen_head["${real}"]=1
    if [ -n "${out}" ]; then out="${out}:${item}"; else out="${item}"; fi
  done
  IFS=':' read -r -a items <<< "${!variable_name-}"
  for item in "${items[@]}"; do
    [ -n "${item}" ] || continue
    item="$(_mvp_rewrite_alias "${item}")"
    real="$(readlink -m "${item}" 2>/dev/null || printf '%s' "${item}")"
    { [ -n "${seen_head[${real}]+x}" ] || [ -n "${seen_rest[${real}]+x}" ]; } && continue
    seen_rest["${real}"]=1
    if [ -n "${out}" ]; then out="${out}:${item}"; else out="${item}"; fi
  done
  printf -v "${variable_name}" '%s' "${out}"
  export "${variable_name}"
}

# Collapse mixed spellings accumulated by the calling shell BEFORE any ROS or
# catkin setup runs; each variable is rebuilt again after sourcing too.
_mvp_search_vars=(
  PATH
  ROS_PACKAGE_PATH
  CMAKE_PREFIX_PATH
  PKG_CONFIG_PATH
  ROSLISP_PACKAGE_DIRECTORIES
  PYTHONPATH
  GAZEBO_PLUGIN_PATH
  GAZEBO_MODEL_PATH
  LD_LIBRARY_PATH
)
for _mvp_var in "${_mvp_search_vars[@]}"; do
  _mvp_export_merged "${_mvp_var}"
done

# PX4's setup script expands these directly; they must exist even when empty.
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH-}"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH-}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH-}"

if ! source /opt/ros/noetic/setup.bash; then
  _mvp_abort 'Failed to source ROS setup: /opt/ros/noetic/setup.bash'
  return 1 2>/dev/null || exit 1
fi
if ! source "${_mvp_workspace}/devel/setup.bash"; then
  _mvp_abort "Failed to source workspace setup: ${_mvp_workspace}/devel/setup.bash"
  return 1 2>/dev/null || exit 1
fi
if ! source "${_mvp_px4_dir}/Tools/setup_gazebo.bash" \
    "${_mvp_px4_dir}" "${_mvp_px4_dir}/build/px4_sitl_default"; then
  _mvp_abort "Failed to source PX4 Gazebo setup: ${_mvp_px4_dir}/Tools/setup_gazebo.bash"
  return 1 2>/dev/null || exit 1
fi

# Catkin/ROS/PX4 sources may have reintroduced alias spellings or duplicates;
# re-collapse everything with deterministic workspace-first ordering.
_mvp_export_merged GAZEBO_PLUGIN_PATH \
  "${_mvp_px4_dir}/build/px4_sitl_default/build_gazebo" \
  "${_mvp_workspace}/devel/lib"
_mvp_export_merged GAZEBO_MODEL_PATH \
  "${_mvp_px4_dir}/Tools/sitl_gazebo/models" \
  "${_mvp_workspace}/src/air_ground_ugv_gazebo/models"
_mvp_export_merged LD_LIBRARY_PATH \
  "${_mvp_workspace}/devel/lib" \
  "${_mvp_px4_dir}/build/px4_sitl_default/build_gazebo"
_mvp_export_merged PATH
_mvp_export_merged PKG_CONFIG_PATH

# Catkin's devel records embed build-time absolute paths; normalize CMAKE /
# PYTHON / LISP lookups without adding new heads.
_mvp_export_merged CMAKE_PREFIX_PATH
_mvp_export_merged PYTHONPATH
_mvp_export_merged ROSLISP_PACKAGE_DIRECTORIES

# The package lookup contract stays deterministic: exactly four roots, each
# workspace-related directory expressed exactly once, canonically spelled.
export ROS_PACKAGE_PATH="${_mvp_px4_dir}:${_mvp_px4_dir}/Tools/sitl_gazebo:${_mvp_workspace}/src:/opt/ros/noetic/share"

# Keep the devel prefix so roslaunch can resolve generated Python wrappers and
# compiled nodes under devel/lib. The normalized .catkin marker above makes
# source resources appear once even though the workspace is reached by a
# symlink alias.
export CMAKE_PREFIX_PATH="${_mvp_workspace}/devel:/opt/ros/noetic"
unset ROSLISP_PACKAGE_DIRECTORIES

# A cache written under an aliased/polluted path set feeds rospack's index and
# can resurface duplicate package roots to roslaunch; force a fresh profile.
mkdir -p "${HOME}/.ros"
rm -f "${HOME}/.ros/rospack_cache" "${HOME}/.ros/rosstack_cache"
if command -v rospack >/dev/null 2>&1; then
  rospack profile >/dev/null 2>&1 || true
fi

_mvp_restore_nounset="${_mvp_nounset_enabled}"
unset -f _mvp_abort _mvp_rewrite_alias _mvp_export_merged
unset _mvp_script_dir _mvp_workspace _mvp_px4_dir _mvp_nounset_enabled _mvp_catkin_marker
unset _mvp_search_vars _mvp_var
if [ "${_mvp_restore_nounset}" = true ]; then
  set -u
fi
unset _mvp_restore_nounset

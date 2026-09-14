#!/bin/bash
# 机载层顺序启动：MAVROS + ego_swarm executor（每个仿真无人机一个独立 ROS Master）
#
# 用法（在 swarm_uav_executor 所在工作空间 root 运行）：
#   source devel/setup.bash
#   bash src/swarm_uav_executor/scripts/startup_offboard_ego.sh 1 2 3        # 顺序启动 UAV1/UAV2/UAV3 (tgt_system 1/2/3)
#   bash src/swarm_uav_executor/scripts/startup_offboard_ego.sh               # 默认启动 UAV1 (tgt_system 1)
#
# 每个 UAV 一个独立 Master：UAV$idx -> 11310+idx
# 每机正式订阅其余 14 机经 topology bridge 重发布的 trajectory_intent。
# 测试时可用 EGO_REBOUND_UAVS="1 15" 仅为指定机显式开启主动 rebound；默认全关。
# 端口公式（与 uav_offboard_sim.launch 一致）：
#   udp_port = 24540 - 1 + tgt_system
#   gcs_port = 34580 - 1 + tgt_system
set -u

# 将 ROS 运行日志重定向到当前工作空间 .ros_home
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
mkdir -p "$ROS_LOG_DIR"

WS=/home/ub20tg/catkin_swarm6-2
# 本脚本运行日志/pid 统一放工作空间 .tmp/logs（不使用 /tmp）
mkdir -p "$WS/.tmp/logs"
PX4_ROOT=/home/ub20tg/PX4_Firmware
PX4_BUILD="$PX4_ROOT/build/px4_sitl_default"
# source 期间临时关掉 -u（L15 的契约不变）：ROS noetic 的
# profile.d/1.ros_distro.sh:3 直接读未定义的 $ROS_DISTRO，nounset 下是**致命错** →
# 整个非交互 shell 当场中止（2026-09-14 实测）。本地先 source 过 ROS 才不复现，
# 从 ROS-less 的 shell 链进来（ssh/cron/包装 bash/新终端）必现，且 L105/106 的
# 日志与 pid 文件都不会生成（现象 = 「什么都没发生」）。
set +u
# shellcheck source=/dev/null
source /opt/ros/noetic/setup.bash
# shellcheck source=/dev/null
source "$WS/devel/setup.bash"
# The PX4 launch files used by uav_offboard_ego.launch are not in the catkin
# workspace.  Load the same Gazebo/PX4 environment used for the simulation
# layer after sourcing catkin, otherwise each background roslaunch exits before
# creating its ROS master and leaves an empty log.
# （该脚本读 $ROS_PACKAGE_PATH，故一并放在关掉 -u 的这段里。）
# shellcheck source=/dev/null
source "$PX4_ROOT/Tools/setup_gazebo.bash" "$PX4_ROOT" "$PX4_BUILD"
set -u
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4_ROOT:$PX4_ROOT/Tools/sitl_gazebo"

STARTUP_TIMEOUT_S=${STARTUP_TIMEOUT_S:-120}
EGO_REBOUND_UAVS=${EGO_REBOUND_UAVS:-}
EGO_SAFETY_SUPERVISOR_MODE=${EGO_SAFETY_SUPERVISOR_MODE:-active}

case "$EGO_SAFETY_SUPERVISOR_MODE" in
  off|shadow|active) ;;
  *)
    echo "ERROR: EGO_SAFETY_SUPERVISOR_MODE must be off, shadow, or active" >&2
    exit 2
    ;;
esac

rebound_enabled() {
  local wanted=$1
  local configured
  for configured in $EGO_REBOUND_UAVS; do
    if [ "$configured" = "$wanted" ]; then
      echo true
      return
    fi
  done
  echo false
}

neighbor_topics() {
  local idx=$1
  local suffix=$2
  local topics=""
  local neighbor_idx
  for neighbor_idx in $(seq 1 15); do
    if [ "$neighbor_idx" -eq "$idx" ]; then
      continue
    fi
    if [ -n "$topics" ]; then
      topics+=","
    fi
    topics+="/UAV${neighbor_idx}/${suffix}"
  done
  echo "$topics"
}

print_config() {
  local idx
  for idx in "$@"; do
    printf 'UAV%s enable_rebound=%s safety_supervisor_mode=%s neighbor_intents=%s\n' \
      "$idx" "$(rebound_enabled "$idx")" "$EGO_SAFETY_SUPERVISOR_MODE" \
      "$(neighbor_topics "$idx" trajectory_intent)"
  done
}

start_uav() {
  local idx=$1
  local master_port=$((11310 + idx))
  local uav_name="UAV$idx"
  local uav_id="UAV$idx"
  local neighbor_odom_topics
  local neighbor_intents
  local enable_rebound
  neighbor_odom_topics=$(neighbor_topics "$idx" "mavros/local_position/odom")
  neighbor_intents=$(neighbor_topics "$idx" "trajectory_intent")
  enable_rebound=$(rebound_enabled "$idx")
  export ROS_MASTER_URI="http://localhost:$master_port"
  export ROS_HOSTNAME=localhost
  nohup roslaunch swarm_uav_executor uav_offboard_ego.launch \
    uav_name:=$uav_name uav_id:=$uav_id tgt_system:=$idx \
    neighbor_odom_topics:=$neighbor_odom_topics \
    neighbor_intents:=$neighbor_intents enable_rebound:=$enable_rebound \
    safety_supervisor_mode:=$EGO_SAFETY_SUPERVISOR_MODE \
    interfaces_version:=sitl-ego-combined \
    > "$WS/.tmp/logs/${uav_name}_offboard_ego.log" 2>&1 &
  echo "$!" > "$WS/.tmp/logs/${uav_name}_offboard_ego.pid"
  echo "started $uav_name (master=$master_port, tgt_system=$idx, rebound=$enable_rebound, safety=$EGO_SAFETY_SUPERVISOR_MODE, pid=$!)"

  echo "waiting $uav_name ROS Master and MAVROS state (timeout=${STARTUP_TIMEOUT_S}s)"
  if ! timeout "$STARTUP_TIMEOUT_S" bash -c \
    "until ROS_MASTER_URI=http://localhost:$master_port rosnode list >/dev/null 2>&1; do sleep 1; done"; then
    echo "ERROR: $uav_name ROS Master not ready within ${STARTUP_TIMEOUT_S}s" >&2
    return 1
  fi
  if ! timeout "$STARTUP_TIMEOUT_S" bash -c \
    "until ROS_MASTER_URI=http://localhost:$master_port rostopic echo -n 1 /mavros/state >/dev/null 2>&1; do sleep 1; done"; then
    echo "ERROR: $uav_name /mavros/state not ready within ${STARTUP_TIMEOUT_S}s" >&2
    return 1
  fi
  echo "ready $uav_name (master=$master_port, MAVROS state available)"
}

stop_all() {
  for idx in "${@:-}"; do
    if [ -f "$WS/.tmp/logs/UAV${idx}_offboard_ego.pid" ]; then
      kill "$(cat "$WS/.tmp/logs/UAV${idx}_offboard_ego.pid")" 2>/dev/null
      rm -f "$WS/.tmp/logs/UAV${idx}_offboard_ego.pid"
    fi
  done
  sleep 1
  ps aux | grep -E '[u]av_offboard_ego|[m]avros|[u]av_executor|[e]go_planner_driver_node' \
    | awk '{print $2}' | xargs -r kill 2>/dev/null || true
}

if [ "$#" -eq 0 ]; then
  set -- 1
fi

case "$1" in
  config)
    shift
    print_config "$@"
    ;;
  stop)
    shift
    stop_all "${@:-}"
    echo "offboard ego stopped"
    ;;
  *)
    for idx in "$@"; do
      if ! start_uav "$idx"; then
        echo "startup stopped before UAV$idx; inspect $WS/.tmp/logs/UAV${idx}_offboard_ego.log" >&2
        exit 1
      fi
    done
    echo "offboard ego started sequentially: $*"
    ;;
esac

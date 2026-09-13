#!/bin/bash
# start_onboard.sh —— 手动启动本机机载栈（在机载电脑上执行；从 GCS ssh 过来执行同样的命令）
#
#   等价手工步骤：real_setup.md §5.1.1 + §5.1.2 方式 B。本脚本**不做检查**，启动条件自己先看：
#     - 板钟：date +%s 应 ~1.79e9；仍是 1970 就先修时钟（要早于启动栈）
#     - 本机身份：hostname -s 应为 UAVn（bridge 按它匹配 topology.yaml）
#     - 没在跑：pgrep -f uav_offboard_ego_real.launch 为空（有则先 stop_onboard.sh）
#   也不做开机自启（自启走 services/swarm-uav-onboard.service）。
#
# 用法：
#   bash src/swarm_uav_executor/bashs/start_onboard.sh                          # 启动
#   bash src/swarm_uav_executor/bashs/start_onboard.sh interfaces_version:=xxx  # 参数原样透传 roslaunch
#   ssh ubuntu@192.168.5.71 'bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh'
# 停止：Ctrl-C；或另开终端 bash .../bashs/stop_onboard.sh
#
# 输出：终端实时显示，同时追加转录到 <ws>/.tmp/logs/${UAV}_offboard_ego.log（log_fetch.sh 采集此名）。
#   追加不覆盖：本轮从最后一行 `===== UAVn 启动 <时间> =====` 往下读。
#   ROS 运行日志落 <ws>/.ros_home/log/<session>/，planner 诊断落 <ws>/runtime_logs/ego_planner/。
# 退出码：roslaunch 的退出码（script -e 透传）。
# =============================================================================
set -eu

WS=$(cd "$(dirname "$0")/../../.." && pwd)
UAV=$(hostname -s)
LOG="$WS/.tmp/logs/${UAV}_offboard_ego.log"

cd "$WS"
# shellcheck source=/dev/null
source /opt/ros/noetic/setup.bash
# shellcheck source=/dev/null
source devel/setup.bash
export ROS_HOME="$WS/.ros_home" ROS_LOG_DIR="$WS/.ros_home/log"   # 运行数据留工作区，不落 ~/.ros
mkdir -p "$ROS_LOG_DIR" "$(dirname "$LOG")"

CMD="roslaunch swarm_uav_executor uav_offboard_ego_real.launch $*"
echo "===== $UAV 启动 $(date '+%F %T %z') =====" >> "$LOG"
echo "$UAV: $CMD"
echo "转录（追加）：$LOG"
exec script -q -e -f -a -c "$CMD" "$LOG"
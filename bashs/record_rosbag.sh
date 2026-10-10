#!/bin/bash
# record_rosbag.sh —— 机载 rosbag 记录（只录关键数据）
#
# 为什么要有这个：2026-09-22/09-23 两轮 coverage 失败排查时**没有 rosbag**，只剩"日志尾部 + ulog"，
# 无法回答"断的那几秒到底有没有帧、到达间隔多大"（两轮报告均把 rosbag 列为下轮必做，仍未做）。
# 健康观测 JSONL 只能定层（发送侧/电台/接收侧哪一段），**连续量真值必须靠 rosbag**。
#
# 记录范围（2026-10-10 按"leader→follower odom 为什么中断"重定；不录 100 Hz 的 IMU 等噪音）：
#   每台机载各录一份，三份按时间轴对齐即可把断点钉在"发送侧→桥→链路→接收侧"的哪一段：
#   ① 发送侧：本机 mavros 的状态/位姿/odom/rc/time_reference（领机是否一直在发）；
#   ② 接收侧：桥接来的**邻机同名话题**（如 UAV10 上的 /UAV8/mavros/local_position/odom
#      —— 跟随环的真正输入，见 topology.yaml 桥接清单）；①与②对减＝中间段丢了什么；
#   ③ 链路哨兵：与 odom 同源的低速流（mavros/state、time_reference、rc/in、gps_bias）
#      —— 一起停＝整条链路断；只有 odom 停＝该话题的发布/桥接端问题；
#   ④ 消费侧：跟随环 setpoint、exec_state、intent、task_state —— 断链时本机在做什么。
#   只录当前**存在**的话题：缺席对端不会产生空话题或刷告警。
#
# 用法（在机载电脑上执行；或从 GCS ssh 过去执行）：
#   bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/record_rosbag.sh          # 录到 Ctrl-C
#   bash .../record_rosbag.sh -d 120                                             # 录 120 s 自动停
#   bash .../record_rosbag.sh -n mission1                                        # 文件名加后缀
#   bash .../record_rosbag.sh -u 8,10,13                                        # 只录这三台的桥接话题（收紧体积）
# 输出：<ws>/.ros_home/log/rosbag/<UAV>_<YYYYmmdd_HHMMSS>[_<name>].bag（lz4 压缩）
# 停止：Ctrl-C（rosbag 会正常收尾并写索引）；-d 到时自动停。
# 注意：log_fetch.sh 目前**不采集** rosbag（体积大）——收工后按需手工 scp。
# =============================================================================
set -u

WS=$(cd "$(dirname "$0")/../../.." && pwd)
UAV=$(hostname -s)
DURATION=""
NAME=""
UAVS=""
while [ $# -gt 0 ]; do
  case "$1" in
    -d|--duration) DURATION="${2:-}"; shift 2 ;;
    -n|--name)     NAME="${2:-}";     shift 2 ;;
    -u|--uavs)     UAVS="${2:-}";     shift 2 ;;
    -h|--help)     sed -n '2,26p' "$0"; exit 0 ;;
    *) echo "未知参数：$1（-h 看用法）" >&2; exit 2 ;;
  esac
done
if [ -n "$DURATION" ] && ! [[ "$DURATION" =~ ^[0-9]+$ ]]; then
  echo "-d 需要整数秒：$DURATION" >&2; exit 2
fi

set +u
# shellcheck source=/dev/null
source /opt/ros/noetic/setup.bash
# shellcheck source=/dev/null
source "$WS/devel/setup.bash"
set -u

OUTDIR="$WS/.ros_home/log/rosbag"
mkdir -p "$OUTDIR"
STAMP=$(date +%Y%m%d_%H%M%S)
SUFFIX=$([ -n "$NAME" ] && printf '_%s' "$NAME")
BAG="$OUTDIR/${UAV}_${STAMP}${SUFFIX}.bag"

# 本机关键话题（存在才录）
LOCAL_CANDIDATES=(
  /mavros/state
  /mavros/local_position/odom
  /mavros/local_position/pose
  /mavros/rc/in
  /mavros/time_reference         # 链路哨兵：与 odom 同源的低速流
  /mavros/global_position/global # GPS（桥上限频 5 Hz；判"是否只有 odom 这一路停"）
  /mavros/battery
  /mavros/setpoint_raw/local     # relay 真正发往飞控的那一路
  /setpoint/ego                  # 规划器 setpoint
  /setpoint/follower             # 跟随环 setpoint
  /local_pose
  /exec_state
  /uav_task_state
  /goal
  /waypoints
  /hold
  /brake_hold
  /direct_control_active
  /trajectory_intent
  /neighbor_intent
  /gps_bias
  /gp_origin_confirmed
  /rosout                        # 文本证据回退：10-09 出现"日志文件静默但进程在跑"，bag 内 /rosout 不受影响
)

TOPICS=()
EXISTING=$(rostopic list 2>/dev/null || true)
if [ -z "$EXISTING" ]; then
  echo "rosmaster 连不上（机载栈没起？）" >&2; exit 1
fi
for t in "${LOCAL_CANDIDATES[@]}"; do
  if grep -qxF "$t" <<<"$EXISTING"; then TOPICS+=("$t"); fi
done
# 邻机 / GCS 的桥接话题（存在才录）——跟随环输入与链路证据都在这里。
# 清单＝topology.yaml 里 bridge 实际转发的话题全集（mavros 状态族 + 时钟 + 任务/intent）。
# 默认收录全部"存在"的对端；板上 bridge 会给 16 台都注册发布者（哪怕没在飞），
# 想收紧就 `-u 8,10,13`（必须带 -u，避免把没在飞的对端也录进包）。
RELAY_PATTERN='/(UAV[0-9]+|GCS_A)/(mavros/(state|time_reference|rc/in|global_position/global|local_position/(odom|pose))|trajectory_intent|uav_task_state|gps_bias|gp_origin_confirmed|group_a/gp_origin)'
if [ -n "$UAVS" ]; then
  peer_re="UAV($(tr ',' '|' <<<"$UAVS"))"
  mapfile -t RELAYED < <(grep -xE "$RELAY_PATTERN" <<<"$EXISTING" | grep -E "/(${peer_re}|GCS_A)/")
else
  mapfile -t RELAYED < <(grep -xE "$RELAY_PATTERN" <<<"$EXISTING")
fi
TOPICS+=("${RELAYED[@]}")

if [ "${#TOPICS[@]}" -eq 0 ]; then
  echo "没有匹配到任何要录的话题" >&2; exit 1
fi

echo "UAV=$UAV  输出=$BAG  话题 ${#TOPICS[@]} 个："
printf '  %s\n' "${TOPICS[@]}"
[ -n "$DURATION" ] && echo "时长：${DURATION}s（到时自动停）" || echo "时长：Ctrl-C 停止"

ARGS=(record -O "$BAG" --lz4)
[ -n "$DURATION" ] && ARGS+=(--duration="$DURATION")
rosbag "${ARGS[@]}" "${TOPICS[@]}"
rc=$?

if [ -f "$BAG" ]; then
  echo "rosbag 已写入：$BAG（$(du -h "$BAG" | cut -f1)）"
else
  echo "rosbag 未生成（rc=$rc）" >&2
fi
exit "$rc"

#!/bin/bash
# stop_onboard.sh —— 停止本机机载栈（等价在启动终端 Ctrl-C）
#
# 给本机 roslaunch 发 SIGINT（它会关掉自己拉起的全部节点）→ 10 s 不退发 SIGTERM → 再 2 s 不退发
# SIGKILL；最后列残留。只认 roslaunch 本体，不会碰 ssh/本脚本自身。
#
# 用法：
#   bash src/swarm_uav_executor/bashs/stop_onboard.sh
#   ssh ubuntu@192.168.5.71 'bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/stop_onboard.sh'
# 注意：开机自启在跑时用 sudo systemctl stop swarm-uav-onboard.service，不要硬杀（unit 会重启它）。
# 退出码：0 = 已停干净（或本来没跑）；2 = 还有残留。
# =============================================================================
set -u

PATTERN='bin/roslaunch .*uav_offboard_ego_real\.launch'

mapfile -t pids < <(pgrep -f "$PATTERN" || true)
if [ "${#pids[@]}" -eq 0 ]; then
  echo "机载栈没在跑（没有 roslaunch 进程）"
  exit 0
fi

echo "命中："
ps -o pid=,etime=,args= -p "${pids[@]}"

kill -INT "${pids[@]}" 2>/dev/null || true
for _ in $(seq 10); do
  kill -0 "${pids[@]}" 2>/dev/null || break
  sleep 1
done
if kill -0 "${pids[@]}" 2>/dev/null; then
  echo "SIGINT 10 s 没退，发 SIGTERM"
  kill -TERM "${pids[@]}" 2>/dev/null || true
  sleep 2
fi
if kill -0 "${pids[@]}" 2>/dev/null; then
  echo "仍不退，发 SIGKILL"
  kill -KILL "${pids[@]}" 2>/dev/null || true
  sleep 1
fi

if kill -0 "${pids[@]}" 2>/dev/null; then
  echo "WARN: 还有残留进程：" >&2
  ps -o pid=,etime=,args= -p "${pids[@]}" >&2
  exit 2
fi
echo "已停止"
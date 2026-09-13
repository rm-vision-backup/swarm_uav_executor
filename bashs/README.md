# swarm_uav_executor/bashs：机载栈手动启动/停止

机载电脑上手动启动/停止**本机**机载栈。等价于 `../../swarm_uav_interfaces/real_setup.md`
§5.1.1 + §5.1.2 方式 B 的几条命令，只是不用每次手打、也不会漏设 `UAV`。
**只做这两件事，不做任何检查**——启动条件自己先看（§2）。开机自启是另一条路径：
`../services/swarm-uav-onboard.service`。

## 1. 用法

```bash
# 在机载电脑上执行；或从 GCS/本机 ssh 过去执行（脚本跑在机载电脑上）
bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh

# 参数原样透传给 roslaunch
bash .../bashs/start_onboard.sh interfaces_version:=deployment-260914

# 停止（等价在启动终端 Ctrl-C）；远程：
bash .../bashs/stop_onboard.sh
ssh ubuntu@192.168.5.71 'bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/stop_onboard.sh'
```

| 文件 | 作用 |
|---|---|
| `start_onboard.sh` | `cd` 工作空间、source ROS + `devel/setup.bash`、导出 `ROS_HOME`/`ROS_LOG_DIR`，然后 `script` 起 `roslaunch swarm_uav_executor uav_offboard_ego_real.launch`（终端输出同时落转录） |
| `stop_onboard.sh` | 给本机 `roslaunch` 发 SIGINT → SIGTERM → SIGKILL，最后列残留 |

## 2. 启动前自己看（脚本不检查）

- **板钟**：`date +%s` 应 ~1.79e9；仍是 1970 就先做时钟修正
  （`../../tcp_to_ros/tools/onboard_clock_fix.sh`，须在启动栈**之前**，否则本轮 ROS 日志时间戳不可用）。
- **本机身份**：`hostname -s` 应为 `UAVn`（bridge 按它匹配 `topology.yaml`）。
- **没在跑**：`pgrep -f uav_offboard_ego_real.launch` 为空；有则先 `stop_onboard.sh`。
- 其它现场条件（`fcu_url`/`tgt_system` 由现场 MAVROS launch 配、GCS_A 已就绪、就绪检查）：

## 3. 日志

| 内容 | 位置 |
|---|---|
| 终端全量转录 | `~/catkin_swarm6-2/.tmp/logs/${UAV}_offboard_ego.log`（`log_fetch.sh` 采集此名） |
| ROS 运行日志 | `~/catkin_swarm6-2/.ros_home/log/<session>/` |
| planner 诊断 | `~/catkin_swarm6-2/runtime_logs/ego_planner/` |

转录是**追加**（`script -a`）：重跑同一台不会丢旧内容，要看本轮就从最后一行的
`===== UAVn 启动 <时间> =====` 分隔行往下读。

## 4. 注意

- 不 arm、不 OFFBOARD、不起飞、不发任务 `START`，也不启动其它 UAV。
- 停止/重跑顺序：先停，确认 `pgrep -f uav_offboard_ego_real.launch` 为空，再启动；不要就地重启单机子集。
- systemd 自启在跑时（`systemctl is-active swarm-uav-onboard.service`）用
  `sudo systemctl stop swarm-uav-onboard.service`，不要用 `stop_onboard.sh` 硬杀。
- 相关：`../../swarm_uav_interfaces/real_setup.md` §5、`../services/README.md`、`../../tcp_to_ros/tools/README.md`。
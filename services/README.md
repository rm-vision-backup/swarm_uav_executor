# swarm_uav_executor/services：机载开机自启动（systemd）

> implementation_plan_26091101 §6。每台机载电脑部署本 service，只启动**本机**机载程序；
> 不循环启动其他 UAV，不自动 arm / OFFBOARD / 起飞 / 发送任务 START。
> 实机 arm/OFFBOARD 由现场遥控器完成；实机 Startup Applications 仍由现场手动设置，
> 本目录不新增重复配置。

## 文件

| 文件 | 作用 |
|---|---|
| `swarm-uav-onboard.service` | 单机机载栈：MAVROS(`px4.launch`) + gp_origin_receiver + gps_bias + bridge(`topology.yaml`) + planner + relay + executor，入口 `uav_offboard_ego_real.launch` |

## 前置条件

1. 机载电脑已安装 ROS Noetic、MAVROS、`python3-zmq`，且工作空间已 `catkin build`（含
   `swarm_uav_interfaces` / `swarm_topology_bridge` / `swarm_uav_executor` / `ego_planner_driver` / `tcp_to_ros`）。
2. 机载电脑 **hostname 固定为 `UAVn`**（如 `UAV3`），与 `swarm_topology_bridge/config/topology.yaml`
   及 `tcp_to_ros/config/uav_identity_map.yaml` 一致（部署环境条件，已实机验证）。
3. 现场确认 `SWARM_FCU_URL` 与实际 FCU 链路一致（默认 `serial:///dev/ttyACM0:921600`）。
4. 若无 `%h/catkin_swarm6-2` 布局，用 drop-in 覆盖 `SWARM_WS`：

   ```bash
   sudo systemctl edit swarm-uav-onboard.service   # 写入 [Service] Environment=SWARM_WS=...
   ```

## 安装与操作

```bash
# 安装（机载电脑本机）
sudo cp swarm-uav-onboard.service /etc/systemd/system/
sudo systemctl daemon-reload

sudo systemctl enable swarm-uav-onboard.service    # 开机自启
sudo systemctl start swarm-uav-onboard.service     # 立即启动
systemctl status swarm-uav-onboard.service         # 状态（含最近日志）
journalctl -u swarm-uav-onboard.service -f         # 跟踪日志
sudo systemctl stop swarm-uav-onboard.service      # 停止
sudo systemctl disable swarm-uav-onboard.service   # 取消开机自启
```

- 失败重启：unit 已设 `Restart=on-failure` / `RestartSec=5`；连续失败可用
  `systemctl reset-failed` 后重新 `start`。
- ROS 运行日志写入工作区 `.ros_home/log`（`ROS_HOME`/`ROS_LOG_DIR` 已在 unit 中设置，
  不使用 `/tmp`）；planner 诊断日志写入 `runtime_logs/ego_planner/`。
- 停止/重跑：先 `stop`（或现场 `cleanup`），确认进程与端口 clean 后再 `start`；不要就地
  重启单机子集。

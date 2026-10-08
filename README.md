# swarm_uav_executor

ROS Noetic 单机任务执行器。每个实例绑定一个不可变的 `(uav_id, exec_target)`，暴露本机 `UavTask`、`UavTaskControl`、`UavHold` 和 `UavTaskState` 接口。`HOLD` 为高优先级安全入口。

## 安全边界

- 实例只接受同时匹配 `uav_id` 与 `exec_target` 的请求，不能代其他 UAV 执行动作。
- 同一时刻最多一个正常任务；同键同内容幂等，同键不同内容拒绝，HOLD 可抢占。
- `UavTask` 执行 PREPARE，`UavTaskControl START` 才允许动作线程推进；整批未通过时可在 START 前 ABORT。
- `mavros_position` 只连接配置的本机 MAVROS namespace，持续发布本地位置 setpoint。
- arm/OFFBOARD 由外部起飞脚本（`offboard_takeoff_15_sitl.py`：HOLD → arm → OFFBOARD）先行完成；
  外部起飞完成后再提交 `UavTask` 执行 PREPARE，整批 PREPARE 成功后才发送 `UavTaskControl START`。
  `ego_swarm` driver 的 `prepare()` 要求本机**已 armed + OFFBOARD**，不再自行按需 arm；
  软起飞由 `ego_planner_driver` 的 `TAKEOFF` 状态自动完成（`takeoff_height_m=5.0`）。
  `mavros_position` 兼容 driver 仍不自动 arm。
- `MOVE_TO`、`FOLLOW_ROUTE`、`FAULT_EXIT` 分别使用 15/12/8 m 动作层；严格垂直段冻结 x/y/yaw，其他非 follower 航段使用 EGO，followers 使用 PI+编队偏置。
- EGO launch 的 `safety_supervisor_mode` 默认 `active`（方案 Y：EMERGENCY 只置位，
  制动由 replan 线程提交）；预测诊断写入工作区
  `.ros_home/log/ego_planner/UAVn-ego-planner.log`。监督层细节见
  `ego_planner_driver/README.md`。
- EGO 巡航、垂直段与 follower PI 对邻机实施半径 1.0 m 的三维球形中心距运行时门禁；
  三维 Euclidean 中心距小于 1.0 m 时返回失败并触发整批 HOLD（恰好 1.0 m 安全，
  EGO 巡航与 follower 共用门禁）。
- 坐标系、yaw 约定、阈值、A01-A15 映射及 MAVROS namespace 未经现场冻结前，只允许 mock、SITL 或不上桨验证。

## setpoint_relay（MAVROS setpoint 唯一出口）

- EGO 轨迹发布 `/setpoint/ego`（`mavros_msgs/PositionTarget`，mask 2048），follower PI 发布
  `/setpoint/follower`（mask 2496）；两者都只是候选，不直接接触 MAVROS 输出话题。
- `setpoint_relay_node.py` 订阅 `/setpoint/ego`、`/setpoint/follower`、`/direct_control_active`
  与 `/mavros/local_position/pose`，以固定频率（默认 30 Hz）独占发布
  `/mavros/setpoint_raw/local`（PositionTarget）。
- 选源：`direct_control_active=false` 选 EGO、`true` 选 follower；模式切换递增 generation 并作废
  新选中源旧缓存，收到切换后新候选前持续 HOLD。
- 校验：frame 必须 `FRAME_LOCAL_NED`、mask 必须属于契约、启用字段必须 finite；选中源过期或
  无效立即进入固定 position/yaw HOLD，不自动回退未选中源。
- HOLD：从 fresh local pose 锁存一次固定 position/yaw；pose stale 时复用上个已验证 HOLD，
  不生成 (0,0,0)。业务字段按 ROS ENU 填写，MAVROS 完成 ENU→NED。
- **禁止**任何其他节点在 EGO 运行链路上发布 `/mavros/setpoint_raw/local` 或
  `/mavros/setpoint_position/local`；已删除的 `safe_valley_exp` flock 链路
  （implementation_plan_26083018 拆包删除）与 `mavros_position` 兼容 driver 属
  独立 legacy/兼容场景，不得与 EGO launch 同时启动。
- 仲裁逻辑为纯类 `swarm_uav_executor.setpoint_relay.SetpointRelay`（无 ROS 依赖，可单测）。

## 实机部署（implementation_plan_26091101）

机载实机入口与仿真入口分离：

| 入口 | 用途 |
|---|---|
| `launch/uav_offboard_ego.launch` | **仿真**单机机载层（每机独立 ROS Master 11311–11325、SITL UDP FCU、仿真 topology） |
| `launch/uav_offboard_ego_real.launch` | **实机**单机机载层：身份取本机 hostname（机载电脑 hostname 固定为 `UAVn`），桥接加载 `swarm_topology_bridge/config/topology.yaml`（port_offset=0），FCU 连接（`fcu_url` / `tgt_system`）沿用机载 MAVROS launch 的现场配置、本入口不传（不沿用 SITL localhost/仿真 UDP） |

- 启动/停止：`bashs/start_onboard.sh` / `bashs/stop_onboard.sh`（现场或 ssh 手操），
  等价 `real_setup.md` §5.1.1 + §5.1.2 方式 B；见 `bashs/README.md`。多机批量从地面站用
  `tcp_to_ros/tools/onboard_ctl.sh`（见其 `README.md` §5.1）。只启动本机程序，不循环启动其他 UAV；
  **不自动 arm / OFFBOARD / 起飞 / 任务 START**，实机由现场遥控器完成 arm/OFFBOARD。
  **不设开机自启**：原 `services/`（systemd unit）已于 2026-09-14 删除（未在实机安装过）。
- 身份边界：机载 ROS 运行时统一使用执行层 `UAVn`（`uav_id`、`exec_target`、hostname、
  topic/service 路径）；任务层 `Axx` 仅由 `tcp_to_ros` 处理，机载不做 Axx 反向映射。
- 诊断日志：`diagnostic_log_dir` 默认 `~/catkin_swarm6-2/.ros_home/log/ego_planner`，不硬编码固定 home。

## 构建与测试

```bash
cd "$HOME/catkin_swarm6-2"
catkin build swarm_uav_interfaces swarm_uav_executor
source devel/setup.bash
catkin test swarm_uav_executor
catkin_test_results build/swarm_uav_executor/test_results
```

## 环境变量（ROS 日志重定向）

为避免 ROS 运行日志散落在 `~/.ros/log`，本工作空间统一将运行日志重定向到 `.ros_home/`：

```bash
export ROS_HOME="$HOME/catkin_swarm6-2/.ros_home"
export ROS_LOG_DIR="$HOME/catkin_swarm6-2/.ros_home/log"
mkdir -p "$ROS_LOG_DIR"
```

手动启动前请先在终端执行以上命令，确保当前工作空间所有 ROS 运行日志均写入 `.ros_home/log`。

## 事件日志（2026-10-08 增）

两轮实飞（2026-09-22 三机 / 09-23 四机）coverage 失败时，机载侧最关键的 20 s **没有任何节点日志**——
executor / `state_reporter` / `setpoint_relay` 运行期零日志语句，跟随环为何落 `LEADER_LOST`、relay 何时
切源或进 HOLD，只能事后从 ulog 反推。为下一次实飞能直接定因，新增两个**独立文件**事件日志：

| 文件（默认 `<ws>/.ros_home/log/health/`） | 内容 |
|---|---|
| `<UAV>_follower_events.jsonl` | 跟随环：`follower_start`（领机 id/话题/偏移）、`leader_odom_first_frame(wait_s)`、`leader_odom_gap`、`own_pose_stale`（只记录，不改控制行为）、`follower_exit(reason=…, leader_age_s)` |
| `<UAV>_relay_events.jsonl` | relay：`mode_switch`（选源切换）、`hold_enter`（锚点来源/位姿/年龄）、`hold_exit`、`no_safe_hold`（连 HOLD 锚点都拿不到，本 tick 不发 setpoint） |

**归置约定**：落在**工作区运行日志根 `.ros_home/log/health/`**，与 planner 诊断
（`diagnostic_log_dir` 默认 `~/catkin_swarm6-2/.ros_home/log/ego_planner`，见上）同根——现场只记一个日志根。
**刻意不经 rosout**：任务窗口 rosout 本就无输出（这正是要修的黑箱），诊断类文件日志一律直接用文件写。
目录由代码按需 `makedirs`，无需改启动脚本。

**采集**：`src/tcp_to_ros/tools/log_fetch.sh` 第 ④ 段会把 `.ros_home/log/health/*.jsonl` 收进采集包的
`onboard/health/`（固定文件名追加写，机上超 32 MB 时只留尾部＝最近一次任务的数据）。
GCS 侧同名文件（在 GCS 自己的工作区里）不在该脚本范围，需随 GCS 日志手工归档。

**体积**：跟随环事件只有"起/首帧/空档/退出"几条，relay 事件是边沿触发（选源切换/HOLD 进出一共几条），
整轮四机仿真实测每机 <2 KB——与 bridge 健康文件一样按需增长，不需要轮转。

**契约（改这里请保持）**：`EventLog.emit()` 只入队、写盘在独立线程、队列满丢最旧并计数——**绝不能把
文件 IO 时延加进跟随环（10/30 Hz）或 relay（30 Hz 独占 setpoint）的实时路径**；relay 事件一律**边沿触发**，
不逐帧刷盘；关闭目录（`event_log_dir` 为空）= 不写 param、不建文件。

相关参数：跟随环 `~ego_swarm/event_log_dir`、relay `~event_log_dir`；上层 launch `event_log_dir`
（`uav_executor_ego.launch` 默认空、仅非空时条件写入；实机/仿真入口传 `.ros_home/log/health`）。
背景与定层口径见 `quality_reports/2026-10-08_two_rounds_coverage_failure_root_cause.md`。

## Mock 启动

```bash
roslaunch swarm_uav_executor uav_executor_mock.launch \
  uav_id:=UAV1 exec_target:=UAV1 service_namespace:=UAV1
```

## Direct MAVROS 启动

先由外部安全流程启动并检查本机 MAVROS；此兼容 driver 不会 arm 或切换模式：

```bash
roslaunch swarm_uav_executor uav_executor_mavros.launch \
  uav_id:=UAV1 exec_target:=UAV1 service_namespace:=UAV1 \
  mavros_namespace:=/uav1/mavros frame_id:=map \
  interfaces_version:=<deployment-version>
```

上线前必须逐机设置唯一 `mavros_namespace`，并复核 `config/executor_defaults.yaml` 中：

- `setpoint_rate_hz`
- `pose_timeout_s` / `state_timeout_s`
- `position_tolerance_m` / `yaw_tolerance_rad`
- `settle_duration_s`
- `require_connected` / `require_armed` / `require_offboard`

建议兼容 direct driver 的实机门禁将 `require_armed`、`require_offboard` 设为 `true`。

## SITL / 不上桨验收门禁

启动 direct driver 前必须先确认：

```bash
rostopic echo -n 1 /uav1/mavros/state
rostopic echo -n 1 /uav1/mavros/local_position/pose
rostopic hz /uav1/mavros/setpoint_position/local
```

- `/state.connected`、pose 新鲜度和配置 frame 必须满足现场约定；
- 对正式 `ego_swarm` driver，先完成外部起飞并确认本机 armed + OFFBOARD，再确认冻结原点回读、逐机提交 PREPARE；整批 PREPARE 全部通过后，最后由 GCS_A 整批发送 START；
- 验证 setpoint 频率、软起飞（TAKEOFF）、分层轨迹、整批 timeout、pose stale、MAVROS 断连和 HOLD；
- 保存 rosbag/ROS 日志与 PX4 failsafe 配置。没有 PX4 SITL 或现场签字时，只能声明 driver 自动化测试通过，不能声明飞行验收通过。

## 后续扩展

规划器、航线、编队和机间避碰应通过 `MotionDriver` 或上层协调器扩展；不得改变单实例只控制本机的权限边界，也不得把其他 UAV 的感知输入变成跨机控制权。

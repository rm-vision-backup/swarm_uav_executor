# swarm_uav_executor

ROS Noetic 单机任务执行器。每个实例绑定一个不可变的 `(uav_id, exec_target)`，暴露本机 `UavTask`、`UavTaskControl`、`UavHold` 和 `UavTaskState` 接口。`HOLD` 为高优先级安全入口。

## 安全边界

- 实例只接受同时匹配 `uav_id` 与 `exec_target` 的请求，不能代其他 UAV 执行动作。
- 同一时刻最多一个正常任务；同键同内容幂等，同键不同内容拒绝，HOLD 可抢占。
- `UavTask` 执行 PREPARE，`UavTaskControl START` 才允许动作线程推进；整批未通过时可在 START 前 ABORT。
- `mavros_position` 只连接配置的本机 MAVROS namespace，持续发布本地位置 setpoint。
- arm/OFFBOARD 由外部起飞脚本（`offboard_takeoff_15.py`：HOLD → arm → OFFBOARD）先行完成；
  外部起飞完成后再提交 `UavTask` 执行 PREPARE，整批 PREPARE 成功后才发送 `UavTaskControl START`。
  `ego_swarm` driver 的 `prepare()` 要求本机**已 armed + OFFBOARD**，不再自行按需 arm；
  软起飞由 `ego_planner_driver` 的 `TAKEOFF` 状态自动完成（`takeoff_height_m=5.0`）。
  `mavros_position` 兼容 driver 仍不自动 arm。
- `MOVE_TO`、`FOLLOW_ROUTE`、`FAULT_EXIT` 分别使用 15/12/8 m 动作层；严格垂直段冻结 x/y/yaw，其他非 follower 航段使用 EGO，followers 使用 PI+编队偏置。
- EGO launch 的 `safety_supervisor_mode` 默认 `active`（方案 Y：EMERGENCY 只置位，
  制动由 replan 线程提交）；预测诊断写入工作区
  `runtime_logs/ego_planner/UAVn-ego-planner.log`。监督层细节见
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
  `/mavros/setpoint_position/local`；`safe_valley_exp` 的 `flock_comm.py`/`safe_flock_*` 与
  `mavros_position` 兼容 driver 属独立 legacy/兼容场景，不得与 EGO launch 同时启动。
- 仲裁逻辑为纯类 `swarm_uav_executor.setpoint_relay.SetpointRelay`（无 ROS 依赖，可单测）。

## 构建与测试

```bash
cd /home/ub20tg/catkin_swarm6-2
catkin build swarm_uav_interfaces swarm_uav_executor
source devel/setup.bash
catkin test swarm_uav_executor
catkin_test_results build/swarm_uav_executor/test_results
```

## 环境变量（ROS 日志重定向）

为避免 ROS 运行日志散落在 `~/.ros/log`，本工作空间统一将运行日志重定向到 `.ros_home/`：

```bash
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
mkdir -p "$ROS_LOG_DIR"
```

手动启动前请先在终端执行以上命令，确保当前工作空间所有 ROS 运行日志均写入 `.ros_home/log`。

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

# swarm_uav_executor

ROS Noetic 单机任务执行器。每个实例绑定一个不可变的 `(uav_id, exec_target)`，暴露本机 `UavTask`、`UavTaskControl`、`UavHold` 和 `UavTaskState` 接口。`HOLD` 为高优先级安全入口。

## 安全边界

- 实例只接受同时匹配 `uav_id` 与 `exec_target` 的请求，不能代其他 UAV 执行动作。
- 同一时刻最多一个正常任务；同键同内容幂等，同键不同内容拒绝，HOLD 可抢占。
- `UavTask` 执行 PREPARE，`UavTaskControl START` 才允许动作线程推进；整批未通过时可在 START 前 ABORT。
- `mavros_position` 只连接配置的本机 MAVROS namespace，持续发布本地位置 setpoint。
- 正式 `ego_swarm` driver 是本机 arm/OFFBOARD 的唯一软件所有者：仅未 arm 的 `MOVE_TO` 在 START 后按需 arm，完成 5 m 垂直检查点并稳定 2 s；已 arm 飞机跳过该过程。`mavros_position` 兼容 driver 仍不自动 arm。
- `MOVE_TO`、`FOLLOW_ROUTE`、`FAULT_EXIT` 分别使用 15/12/8 m 动作层；严格垂直段冻结 x/y/yaw，其他非 follower 航段使用 EGO，followers 使用 PI+编队偏置。
- 垂直段与 follower PI 对邻机实施 1.0 m 水平、2.0 m 垂直中心距运行时门禁，突破门槛返回失败并触发整批 HOLD。
- 坐标系、yaw 约定、阈值、A01-A15 映射及 MAVROS namespace 未经现场冻结前，只允许 mock、SITL 或不上桨验证。

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
  uav_id:=A01 exec_target:=UAV1 service_namespace:=UAV1
```

## Direct MAVROS 启动

先由外部安全流程启动并检查本机 MAVROS；此兼容 driver 不会 arm 或切换模式：

```bash
roslaunch swarm_uav_executor uav_executor_mavros.launch \
  uav_id:=A01 exec_target:=UAV1 service_namespace:=UAV1 \
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
- 对正式 `ego_swarm` driver，先确认冻结原点回读和 PREPARE 全部通过，再由 GCS_A 整批 START 触发按需起飞；
- 验证 setpoint 频率、5 m 检查点、分层轨迹、整批 timeout、pose stale、MAVROS 断连和 HOLD；
- 保存 rosbag/ROS 日志与 PX4 failsafe 配置。没有 PX4 SITL 或现场签字时，只能声明 driver 自动化测试通过，不能声明飞行验收通过。

## 后续扩展

规划器、航线、编队和机间避碰应通过 `MotionDriver` 或上层协调器扩展；不得改变单实例只控制本机的权限边界，也不得把其他 UAV 的感知输入变成跨机控制权。

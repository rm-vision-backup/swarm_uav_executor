# swarm_uav_executor

ROS Noetic 单机任务执行器。每个实例绑定一个不可变的 `(uav_id, exec_target)`，只暴露本机 `UavTask`、`UavHold` 和 `UavTaskState` 接口。首版支持 `MOVE_TO`，`HOLD` 为高优先级安全入口。

## 安全边界

- 实例只接受同时匹配 `uav_id` 与 `exec_target` 的请求，不能代其他 UAV 执行动作。
- 同一时刻最多一个正常任务；同键同内容幂等，同键不同内容拒绝，HOLD 可抢占。
- Service 仅完成校验、登记与接管；动作终态由状态 Topic 发布。
- `mavros_position` 只连接配置的本机 MAVROS namespace，持续发布本地位置 setpoint。
- 执行器**永不自动 arm，也不自动切换 OFFBOARD**。PX4 mode/arm、failsafe 与急停由独立现场安全流程负责。
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

先由外部安全流程启动并检查本机 MAVROS；示例不会 arm 或切换模式：

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

建议实机门禁将 `require_armed`、`require_offboard` 设为 `true`，但 arm/mode 仍必须由外部操作者或安全控制器完成。

## SITL / 不上桨验收门禁

启动 direct driver 前必须先确认：

```bash
rostopic echo -n 1 /uav1/mavros/state
rostopic echo -n 1 /uav1/mavros/local_position/pose
rostopic hz /uav1/mavros/setpoint_position/local
```

- `/state.connected`、pose 新鲜度和配置 frame 必须满足现场约定；
- 先在未 arm 状态确认节点不会调用 arm/mode Service；
- 再由外部安全流程完成 mode/arm，并验证 setpoint 频率、稳定窗口、timeout、pose stale、MAVROS 断连和 HOLD；
- 保存 rosbag/ROS 日志与 PX4 failsafe 配置。没有 PX4 SITL 或现场签字时，只能声明 driver 自动化测试通过，不能声明飞行验收通过。

## 后续扩展

规划器、航线、编队和机间避碰应通过 `MotionDriver` 或上层协调器扩展；不得改变单实例只控制本机的权限边界，也不得把其他 UAV 的感知输入变成跨机控制权。
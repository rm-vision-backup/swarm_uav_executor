#!/usr/bin/env python3
"""setpoint_relay：/mavros/setpoint_raw/local 的唯一 MAVROS setpoint 发布者。

EGO 轨迹（/setpoint/ego）与 follower PI（/setpoint/follower）只发布
PositionTarget 候选；本节点仲裁、校验、失效 HOLD，并以固定频率独占输出：

    /setpoint/ego        (PositionTarget, EGO mask 2048 / 2552)
    /setpoint/follower   (PositionTarget, follower mask 2496)
    /direct_control_active (Bool: false=EGO, true=follower)
    /mavros/local_position/pose (安全 HOLD 捕获源)
        -->  /mavros/setpoint_raw/local (PositionTarget, 30 Hz 唯一发布者)

选源规则：
- 模式切换递增 generation 并作废新选中源旧缓存，收到切换后新候选前持续 HOLD；
- 选中源过期或含 NaN/Inf 进入 HOLD；不自动回退未选中源；
- HOLD 锁存一次 fresh local pose 的固定 position/yaw；pose stale 时复用上一个
  已验证 HOLD，不生成 (0,0,0)。

业务字段按 ROS ENU 填写，MAVROS setpoint_raw 插件完成 ENU→NED 转换；
本节点不做坐标换轴。
"""

from __future__ import annotations

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Bool

from swarm_uav_executor.setpoint_relay import SetpointRelay


def main() -> None:
    rospy.init_node("setpoint_relay")
    ego_topic = rospy.get_param("~ego_topic", "/setpoint/ego")
    follower_topic = rospy.get_param("~follower_topic", "/setpoint/follower")
    mode_topic = rospy.get_param("~mode_topic", "/direct_control_active")
    pose_topic = rospy.get_param("~local_pose_topic", "/mavros/local_position/pose")
    output_topic = rospy.get_param("~output_topic", "/mavros/setpoint_raw/local")
    rate_hz = rospy.get_param("~rate_hz", 30.0)
    candidate_timeout_s = rospy.get_param("~candidate_timeout_s", 0.2)
    local_pose_timeout_s = rospy.get_param("~local_pose_timeout_s", 1.0)

    relay = SetpointRelay(candidate_timeout_s, local_pose_timeout_s)
    pub = rospy.Publisher(output_topic, PositionTarget, queue_size=1)

    rospy.Subscriber(ego_topic, PositionTarget,
                     lambda msg: relay.on_candidate("ego", msg), queue_size=1)
    rospy.Subscriber(follower_topic, PositionTarget,
                     lambda msg: relay.on_candidate("follower", msg), queue_size=1)
    rospy.Subscriber(mode_topic, Bool,
                     lambda msg: relay.on_mode(bool(msg.data)), queue_size=1)
    rospy.Subscriber(pose_topic, PoseStamped, relay.on_pose, queue_size=1)

    rospy.loginfo(
        "setpoint_relay: sole owner of %s (rate=%s Hz, candidate_timeout=%s s, "
        "pose_timeout=%s s)", output_topic, rate_hz, candidate_timeout_s,
        local_pose_timeout_s)

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        out = relay.tick(stamp=rospy.Time.now())
        if out is not None:
            pub.publish(out)
        rate.sleep()


if __name__ == "__main__":
    main()

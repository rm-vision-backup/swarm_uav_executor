#!/usr/bin/env python3
"""把 ego 层 /setpoint 转发为 MAVROS 期望的 /mavros/setpoint_position/local。

EgoSwarmDriver 与 ego_planner_driver 把本机位置 setpoint 发布在
`/setpoint`（geometry_msgs/PoseStamped），而本机 MAVROS 的
`setpoint_position` 插件订阅 `/mavros/setpoint_position/local`（同类型）。
本中继节点把两者桥接：

    /setpoint  -->  /mavros/setpoint_position/local

在机载多 Master 隔离仿真中，MAVROS 与 ego/executor 运行在同一机载
Master，本节点直接订阅/发布即可；跨 Master 的机间转发（leader odom /
intent）由 topology bridge 处理，不需要本节点参与。

默认话题可经私有参数覆盖：
  ~source_topic  (默认 /setpoint)
  ~target_topic  (默认 /mavros/setpoint_position/local)
"""

from __future__ import annotations

import rospy
from geometry_msgs.msg import PoseStamped


def _handle(source: str, target: str) -> None:
    pub = rospy.Publisher(target, PoseStamped, queue_size=1)

    def on_setpoint(msg):
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose
        pub.publish(out)

    rospy.Subscriber(source, PoseStamped, on_setpoint, queue_size=1)
    rospy.loginfo(
        "setpoint_relay: %s -> %s",
        source,
        target,
    )


def main() -> None:
    rospy.init_node("setpoint_relay")
    source = rospy.get_param("~source_topic", "/setpoint")
    target = rospy.get_param(
        "~target_topic", "/mavros/setpoint_position/local"
    )
    _handle(source, target)
    rospy.spin()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""把 MAVROS 本机位姿转发为 ego 层期望的 /local_pose。

ego_planner_driver_node 与 EgoSwarmDriver 都订阅 `local_pose`
（geometry_msgs/PoseStamped），而本机 MAVROS 发布的是
`/mavros/local_position/pose`（同类型）。本中继节点把两者桥接：

    /mavros/local_position/pose  -->  /local_pose

在机载多 Master 隔离仿真中，MAVROS 与 ego/executor 运行在同一机载
Master，本节点直接订阅/发布即可；跨 Master 的机间转发由 topology bridge
处理（只转发 /UAV{n}/mavros/local_position/odom），不需要本节点参与。

默认话题可经私有参数覆盖：
  ~source_topic  (默认 /mavros/local_position/pose)
  ~target_topic  (默认 /local_pose)
"""

from __future__ import annotations

import rospy
from geometry_msgs.msg import PoseStamped


def _handle(source: str, target: str) -> None:
    pub = rospy.Publisher(target, PoseStamped, queue_size=1)

    def on_pose(msg):
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose
        pub.publish(out)

    rospy.Subscriber(source, PoseStamped, on_pose, queue_size=1)
    rospy.loginfo(
        "pose_relay: %s -> %s",
        source,
        target,
    )


def main() -> None:
    rospy.init_node("pose_relay")
    source = rospy.get_param("~source_topic", "/mavros/local_position/pose")
    target = rospy.get_param("~target_topic", "/local_pose")
    _handle(source, target)
    rospy.spin()


if __name__ == "__main__":
    main()
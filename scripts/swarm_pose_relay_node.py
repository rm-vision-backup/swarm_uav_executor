#!/usr/bin/env python3
"""把 MAVROS 本机 odom 收敛为轻量机间位姿流 /swarm_pose。

机间桥（swarm_topology_bridge）转发的是**整条 ROS 序列化消息**；本机 MAVROS 发的是
`nav_msgs/Odometry`（~712 B），其中 twist 与两个 36 元协方差无人消费——跟随器只读
`pose.pose.position`（见 ego_swarm.py）。本节点把位置抽成 `geometry_msgs/PointStamped`
（~45 B）交桥转发，把机间位姿流从 ~68 kbps/流压到 ~4 kbps/流
（quality_reports/2026-10-10_three_uav_airlink_stall_analysis.md）。

    /mavros/local_position/odom  -->  /swarm_pose

只复制位置与 header（原样带回 stamp/frame_id）；两端队列 queue_size=1（只保留最新样本，
不缓存补发旧数据）。跨机坐标系契约由调用方核对：位置与 frame_id 原样透传，不在此变换。

默认话题可经私有参数覆盖：
  ~source_topic  (默认 /mavros/local_position/odom)
  ~target_topic  (默认 /swarm_pose)
"""

from __future__ import annotations

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry


def _handle(source: str, target: str) -> None:
    pub = rospy.Publisher(target, PointStamped, queue_size=1)

    def on_odom(msg):
        out = PointStamped()
        out.header = msg.header
        out.point = msg.pose.pose.position
        pub.publish(out)

    rospy.Subscriber(source, Odometry, on_odom, queue_size=1)
    rospy.loginfo("swarm_pose_relay: %s -> %s", source, target)


def main() -> None:
    rospy.init_node("swarm_pose_relay")
    source = rospy.get_param("~source_topic", "/mavros/local_position/odom")
    target = rospy.get_param("~target_topic", "/swarm_pose")
    _handle(source, target)
    rospy.spin()


if __name__ == "__main__":
    main()

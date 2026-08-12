#!/usr/bin/env python3
"""机载全局原点接收器（每机一个实例）。

经 bridge 订阅 GCS_A 发布的 `/GCS_A/group_a/gp_origin`
（`geographic_msgs/GeoPointStamped`），收到后以 1Hz 周期转发到本机
`/mavros/global_position/set_gp_origin`（该处是 MAVROS 的 topic 插件，
非 service），使本机 local ENU 坐标系与 GCS_A 统一原点对齐。

forward_origin 1Hz 循环设置次数上限为 max_origin_sets（默认 5 次）：
  - 起飞前持续设置以规避 RTK 定位完成较晚导致的坐标突变；
  - 达到上限后停止改写飞控原点，避免无谓持续干扰飞控坐标状态；
  - 本机 armed 后也停止周期转发（最后一次原点仍生效）。

用法（由 uav_offboard_ego.launch 装配，也可单独运行）:
    rosrun swarm_uav_executor gp_origin_receiver.py
"""
import rospy
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import State

DEFAULT_MAX_ORIGIN_SETS = 5


class GpOriginReceiver:
    def __init__(self):
        self.origin = None
        self.armed = False
        self.origin_sent = 0
        self.max_origin_sets = rospy.get_param("~max_origin_sets",
                                               DEFAULT_MAX_ORIGIN_SETS)
        self.origin_pub = rospy.Publisher(
            "/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=1)
        rospy.Subscriber("/GCS_A/group_a/gp_origin", GeoPointStamped,
                         self.on_gp_origin, queue_size=1)
        rospy.Subscriber("/mavros/state", State, self.on_state, queue_size=1)
        rospy.Timer(rospy.Duration(1.0), self.forward_origin)
        rospy.loginfo(
            "gp_origin_receiver: subscribe /GCS_A/group_a/gp_origin, "
            "forward to /mavros/global_position/set_gp_origin "
            "(max_origin_sets=%d)" % self.max_origin_sets)

    def on_gp_origin(self, msg):
        self.origin = msg
        # 收到即立即设置一次：让原点尽快生效
        if not self.armed and self.origin_sent < self.max_origin_sets:
            self.origin_pub.publish(msg)
            self.origin_sent += 1

    def on_state(self, msg):
        self.armed = msg.armed

    def forward_origin(self, _event):
        # 起飞前 1Hz 重复设置（规避 RTK 延迟坐标突变）；
        # armed 后或达到次数上限后停止，避免干扰飞行中的坐标系
        if self.armed:
            return
        if self.origin_sent >= self.max_origin_sets:
            return
        if self.origin is not None:
            self.origin_pub.publish(self.origin)
            self.origin_sent += 1


def main():
    rospy.init_node("gp_origin_receiver")
    GpOriginReceiver()
    rospy.spin()


if __name__ == "__main__":
    main()
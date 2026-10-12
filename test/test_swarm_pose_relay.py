#!/usr/bin/env python3
"""swarm_pose_relay 纯单元测试（无 ROS Master 依赖）。

覆盖：订阅 Odometry、发布 PointStamped；位置与 header（stamp/frame_id）逐字段
透传；两端队列 queue_size=1（只保留最新样本）；无输入时不产生伪位置。
"""
import importlib.util
import pathlib
import unittest

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

_SCRIPT = (pathlib.Path(__file__).resolve().parents[1]
           / "scripts" / "swarm_pose_relay_node.py")


def _load_module():
    spec = importlib.util.spec_from_file_location("swarm_pose_relay_node", _SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _FakePub:
    def __init__(self, topic, typ, queue_size):
        self.topic, self.typ, self.queue_size = topic, typ, queue_size
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class _FakeRos:
    def __init__(self):
        self.pub = None
        self.sub = None

    def Publisher(self, topic, typ, queue_size=1):
        self.pub = _FakePub(topic, typ, queue_size)
        return self.pub

    def Subscriber(self, topic, typ, cb, queue_size=1):
        self.sub = {"topic": topic, "type": typ, "cb": cb, "queue_size": queue_size}
        return object()

    def loginfo(self, *args, **kwargs):
        pass


def _odom(x, y, z, frame="map", stamp=12.5):
    od = Odometry()
    od.header.frame_id = frame
    od.header.stamp.secs = int(stamp)
    od.header.stamp.nsecs = int((stamp - int(stamp)) * 1e9)
    od.pose.pose.position.x = x
    od.pose.pose.position.y = y
    od.pose.pose.position.z = z
    # 塞入 twist/协方差噪声，确认不会透传到 PointStamped。
    od.twist.twist.linear.x = 9.9
    od.pose.covariance[0] = 9.9
    return od


class SwarmPoseRelayTest(unittest.TestCase):
    def setUp(self):
        self.mod = _load_module()
        self.fake = _FakeRos()
        self.mod.rospy = self.fake

    def test_wires_odom_to_point_stamped_with_queue_one(self):
        self.mod._handle("/mavros/local_position/odom", "/swarm_pose")
        self.assertEqual(self.fake.pub.topic, "/swarm_pose")
        self.assertIs(self.fake.pub.typ, PointStamped)
        self.assertEqual(self.fake.pub.queue_size, 1)
        self.assertEqual(self.fake.sub["topic"], "/mavros/local_position/odom")
        self.assertIs(self.fake.sub["type"], Odometry)
        self.assertEqual(self.fake.sub["queue_size"], 1)

    def test_position_and_header_pass_through(self):
        self.mod._handle("/mavros/local_position/odom", "/swarm_pose")
        self.fake.sub["cb"](_odom(1.5, -2.0, 12.0, frame="map", stamp=12.5))
        self.assertEqual(len(self.fake.pub.msgs), 1)
        out = self.fake.pub.msgs[0]
        self.assertIsInstance(out, PointStamped)
        self.assertEqual((out.point.x, out.point.y, out.point.z), (1.5, -2.0, 12.0))
        self.assertEqual(out.header.frame_id, "map")
        self.assertAlmostEqual(out.header.stamp.to_sec(), 12.5)

    def test_no_input_produces_no_message(self):
        self.mod._handle("/mavros/local_position/odom", "/swarm_pose")
        self.assertEqual(self.fake.pub.msgs, [])


if __name__ == "__main__":
    unittest.main()

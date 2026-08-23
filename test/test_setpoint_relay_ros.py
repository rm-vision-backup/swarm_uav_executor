#!/usr/bin/env python3
"""relay rostest：验证 /mavros/setpoint_raw/local 唯一发布者、30Hz 输出、
候选停止后 HOLD、模式切换选 follower、旧 /mavros/setpoint_position/local 无发布者。

测试节点同时充当 fake pose / 两个候选源发布者与输出收集者。
"""
import threading
import unittest

import rosgraph
import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Bool

EGO_MASK = PositionTarget.IGNORE_YAW_RATE  # 2048
FOL_MASK = (PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE)  # 2496
HOLD_MASK = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
             PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
             PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
             PositionTarget.IGNORE_YAW_RATE)  # 2552
STREAM_S = 6.0
STOP_EGO_AT = 2.0
RESUME_EGO_AT = 3.0
FOLLOWER_AT = 4.0


def make_target(x, y, z, mask, vx=0.5, vy=0.0, vz=0.0, stamp=None):
    m = PositionTarget()
    m.header.frame_id = "map"
    m.header.stamp = stamp if stamp is not None else rospy.Time.now()
    m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    m.type_mask = mask
    m.position.x, m.position.y, m.position.z = x, y, z
    m.velocity.x, m.velocity.y, m.velocity.z = vx, vy, vz
    m.acceleration_or_force.x = 0.0
    m.acceleration_or_force.y = 0.0
    m.acceleration_or_force.z = 0.0
    m.yaw = 0.0
    return m


class RelayRosTest(unittest.TestCase):
    def setUp(self):
        self.outputs = []
        self.lock = threading.Lock()
        self.sub = rospy.Subscriber("/mavros/setpoint_raw/local", PositionTarget,
                                    self._on_output, queue_size=100)

    def _on_output(self, msg):
        with self.lock:
            self.outputs.append((rospy.Time.now().to_sec(), msg))

    def _publish_stream(self):
        rate = rospy.Rate(30.0)
        pose_pub = rospy.Publisher("/mavros/local_position/pose", PoseStamped, queue_size=1)
        ego_pub = rospy.Publisher("/setpoint/ego", PositionTarget, queue_size=1)
        fol_pub = rospy.Publisher("/setpoint/follower", PositionTarget, queue_size=1)
        mode_pub = rospy.Publisher("/direct_control_active", Bool, queue_size=1)
        t0 = rospy.Time.now()
        ego_stopped = False
        follower_on = False
        while not rospy.is_shutdown() and \
                (rospy.Time.now() - t0).to_sec() < STREAM_S:
            now = rospy.Time.now()
            t = (now - t0).to_sec()
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = "map"
            pose.pose.position.x = 1.0
            pose.pose.position.y = 2.0
            pose.pose.position.z = 3.0
            pose.pose.orientation.w = 1.0
            pose_pub.publish(pose)
            if not ego_stopped and t >= STOP_EGO_AT:
                ego_stopped = True
            if ego_stopped and t >= RESUME_EGO_AT:
                ego_stopped = False
            if not follower_on and t >= FOLLOWER_AT:
                follower_on = True
                mode_pub.publish(Bool(True))
            if not ego_stopped:
                ego_pub.publish(make_target(5.0 + t * 0.1, 2.0, 3.0, EGO_MASK,
                                            stamp=now))
            if follower_on:
                fol_pub.publish(make_target(10.0, 2.0, 3.0, FOL_MASK, vx=0.5,
                                            stamp=now))
            rate.sleep()

    def test_relay_contract(self):
        thread = threading.Thread(target=self._publish_stream, daemon=True)
        thread.start()
        thread.join(STREAM_S + 3.0)
        with self.lock:
            outputs = list(self.outputs)
        self.assertGreater(len(outputs), 100)

        # 唯一发布者：/mavros/setpoint_raw/local 只有 relay；旧 position 话题无发布者。
        master = rosgraph.Master("/rostest_relay_pubcheck")
        state = master.getSystemState()
        pubs = dict(state[0])
        self.assertEqual(len(pubs.get("/mavros/setpoint_raw/local", [])), 1)
        self.assertNotIn("/mavros/setpoint_position/local", pubs)

        # 契约掩码都出现过，且 follower 只出现在模式切换后。
        masks = [o[1].type_mask for o in outputs]
        self.assertIn(EGO_MASK, masks)
        self.assertIn(HOLD_MASK, masks)
        self.assertIn(FOL_MASK, masks)
        t_hold = next(o[0] for o in outputs if o[1].type_mask == HOLD_MASK)
        t_fol = next(o[0] for o in outputs if o[1].type_mask == FOL_MASK)
        self.assertLess(t_hold, t_fol)

        # 输出始终是 PositionTarget 且 frame/mask 合法。
        for _, msg in outputs:
            self.assertIsInstance(msg, PositionTarget)
            self.assertEqual(msg.coordinate_frame, PositionTarget.FRAME_LOCAL_NED)


if __name__ == "__main__":
    rospy.init_node("test_setpoint_relay_ros")
    rostest.rosrun("swarm_uav_executor", "setpoint_relay_ros", RelayRosTest)

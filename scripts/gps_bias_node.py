#!/usr/bin/env python3
"""GPS 时钟偏置节点。

订阅 /mavros/time_reference（~1Hz，GPS PPS 秒脉冲，source=fcu），计算
    bias = header.stamp - time_ref
即本机 ROS 系统时钟与 GPS 绝对时间（GPST）的偏差——本质是常数（晶振固定偏差）
叠加缓慢漂移。本节点在滑动窗口（默认 10s）内取均值平滑 PPS/网络抖动，以 1Hz
发布 std_msgs/Float64（秒）到 `gps_bias`（latch）。

GPS 失锁（time_reference 超过 lockout_s 未更新）时停止发布新值，接收端用
"最后一次接收时刻"判断新鲜度并降级（回退相对时钟，不修正）。

邻机经 topology bridge 把 /UAVn/gps_bias 转发到本机 master，接收端用
    T_own = T_other - bias_other + bias_own
把邻机话题时间戳修正到本机时间轴（消除机间时钟漂移对 intent 时间对齐的影响）。

用法：rosrun swarm_uav_executor gps_bias_node.py
参数（私有）：~window_s(10.0) ~publish_rate_hz(1.0) ~lockout_s(3.0)
"""
from collections import deque

import math

import rospy
from sensor_msgs.msg import TimeReference
from std_msgs.msg import Float64


def _finite_value(value, name, min_value):
    value = float(value)
    if not math.isfinite(value) or value < min_value:
        rospy.logfatal("%s=%r invalid (must be finite and >=%s); refusing to start",
                       name, value, min_value)
        raise ValueError(f"{name} must be finite and >= {min_value}")
    return value


class BiasEstimator:
    """ROS 无关的 GPS 时钟偏置滑动窗口估计器。"""

    def __init__(self, window_s=10.0, lockout_s=3.0):
        self.window_s = float(window_s)
        self.lockout_s = float(lockout_s)
        self.samples = deque()          # [(received_at_s, bias_s), ...]
        self.last_bias_s = None         # 最近窗口均值
        self.last_update_s = None       # 最近一次参考到达时刻

    def observe(self, received_at_s, ros_stamp_s, reference_s):
        """接收一次时间参考，并返回当前窗口估计值（所有参数单位为秒）。"""
        received_at_s = float(received_at_s)
        bias_s = float(ros_stamp_s) - float(reference_s)
        self.samples.append((received_at_s, bias_s))
        cutoff = received_at_s - self.window_s
        # 保留恰好位于窗口左边界的样本，与原节点行为一致。
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()
        self.last_bias_s = sum(bias for _, bias in self.samples) / len(self.samples)
        self.last_update_s = received_at_s
        return self.last_bias_s

    def value_at(self, now_s):
        """返回当前估计；超过 lockout（严格大于）或尚无样本时返回 None。"""
        if self.last_bias_s is None:
            return None
        if float(now_s) - self.last_update_s > self.lockout_s:
            return None
        return self.last_bias_s


class GpsBiasNode:
    def __init__(self):
        # 参数唯一来源 gps_bias_defaults.yaml（implementation_plan_26082916 §6）；
        # finite/范围校验 fail-fast：window_s/publish_rate_hz >0、lockout_s >=0。
        window_s = _finite_value(rospy.get_param("~window_s", 10.0), "window_s", 1e-9)
        self.publish_rate_hz = _finite_value(
            rospy.get_param("~publish_rate_hz", 1.0), "publish_rate_hz", 1e-9)
        lockout_s = _finite_value(rospy.get_param("~lockout_s", 3.0), "lockout_s", 0.0)
        self.estimator = BiasEstimator(window_s=window_s, lockout_s=lockout_s)

        self.pub = rospy.Publisher("gps_bias", Float64, queue_size=1, latch=True)
        rospy.Subscriber("/mavros/time_reference", TimeReference,
                         self.on_time_ref, queue_size=10, tcp_nodelay=True)
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate_hz), self.publish)

    def on_time_ref(self, msg):
        self.estimator.observe(
            rospy.Time.now().to_sec(),
            msg.header.stamp.to_sec(),
            msg.time_ref.to_sec(),
        )

    def publish(self, event):
        bias_s = self.estimator.value_at(rospy.Time.now().to_sec())
        if bias_s is None:
            # 尚无参考或 GPS 失锁：不发新值；latch 保留最后一个。
            return
        self.pub.publish(Float64(bias_s))


if __name__ == "__main__":
    rospy.init_node("gps_bias_node", anonymous=False)
    GpsBiasNode()
    rospy.spin()

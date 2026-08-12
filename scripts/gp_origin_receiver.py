#!/usr/bin/env python3
"""Apply one frozen GCS origin through MAVROS and verify its ROS readback."""
import math
import threading

import rospy
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import State
from std_msgs.msg import Bool


def horizontal_error_m(a, b):
    radius = 6371008.8
    lat1 = math.radians(a.latitude)
    lat2 = math.radians(b.latitude)
    dlat = lat2 - lat1
    dlon = math.radians(b.longitude - a.longitude)
    h = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    return radius * 2.0 * math.asin(min(1.0, math.sqrt(h)))


class GpOriginReceiver:
    def __init__(self):
        self._lock = threading.RLock()
        self.origin = None
        self.state_received = False
        self.connected = False
        self.armed = False
        self.confirmed = False
        self.last_send = rospy.Time(0)
        self.attempts = 0
        self.max_attempts = int(rospy.get_param("~max_attempts", 10))
        self.retry_s = float(rospy.get_param("~retry_s", 1.0))
        self.horizontal_tolerance_m = float(rospy.get_param("~horizontal_tolerance_m", 0.1))
        self.altitude_tolerance_m = float(rospy.get_param("~altitude_tolerance_m", 0.1))
        self.origin_pub = rospy.Publisher(
            "/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=1)
        self.confirmed_pub = rospy.Publisher("/gp_origin_confirmed", Bool, queue_size=1, latch=True)
        rospy.Subscriber("/GCS_A/group_a/gp_origin", GeoPointStamped,
                         self.on_origin, queue_size=1)
        rospy.Subscriber("/mavros/global_position/gp_origin", GeoPointStamped,
                         self.on_readback, queue_size=1)
        rospy.Subscriber("/mavros/state", State, self.on_state, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.on_timer)
        self.confirmed_pub.publish(Bool(False))

    @staticmethod
    def _same_origin(a, b):
        return (a is not None and b is not None and
                a.position.latitude == b.position.latitude and
                a.position.longitude == b.position.longitude and
                a.position.altitude == b.position.altitude)

    def on_origin(self, msg):
        with self._lock:
            if self.confirmed and self._same_origin(self.origin, msg):
                return
            if self.state_received and self.armed:
                rospy.logerr("refusing to change global origin while armed")
                return
            self.origin = msg
            self.confirmed = False
            self.attempts = 0
            self.last_send = rospy.Time(0)
            self.confirmed_pub.publish(Bool(False))
        self._send()

    def on_state(self, msg):
        with self._lock:
            self.state_received = True
            self.connected = bool(msg.connected)
            self.armed = bool(msg.armed)
            should_send = (self.connected and not self.armed and self.origin is not None
                           and not self.confirmed and self.attempts == 0)
        if should_send:
            self._send()

    def _send(self):
        with self._lock:
            if (self.origin is None or self.confirmed or not self.state_received or
                    not self.connected or self.armed or self.attempts >= self.max_attempts):
                return
            self.last_send = rospy.Time.now()
            self.attempts += 1
            msg = self.origin
        self.origin_pub.publish(msg)

    def on_readback(self, msg):
        with self._lock:
            origin = self.origin
            sent_at = self.last_send
            if origin is None or self.confirmed or sent_at == rospy.Time(0):
                return
            # MAVROS stamps the received event. Ignore a queued event that
            # predates this receiver's most recent set request.
            if msg.header.stamp != rospy.Time(0) and msg.header.stamp < sent_at:
                return
            horizontal = horizontal_error_m(origin.position, msg.position)
            altitude = abs(origin.position.altitude - msg.position.altitude)
            if horizontal > self.horizontal_tolerance_m or altitude > self.altitude_tolerance_m:
                rospy.logwarn("gp_origin readback mismatch horizontal=%.3fm altitude=%.3fm", horizontal, altitude)
                return
            self.confirmed = True
        self.confirmed_pub.publish(Bool(True))
        rospy.loginfo("gp_origin confirmed horizontal=%.3fm altitude=%.3fm", horizontal, altitude)

    def on_timer(self, _event):
        with self._lock:
            if (self.confirmed or self.origin is None or not self.state_received or
                    not self.connected or self.armed):
                return
            elapsed = (rospy.Time.now() - self.last_send).to_sec() if self.last_send != rospy.Time(0) else self.retry_s
            exhausted = self.attempts >= self.max_attempts
        if exhausted:
            rospy.logerr_throttle(5.0, "gp_origin confirmation exhausted %d attempts", self.max_attempts)
        elif elapsed >= self.retry_s:
            self._send()


def main():
    rospy.init_node("gp_origin_receiver")
    GpOriginReceiver()
    rospy.spin()


if __name__ == "__main__":
    main()

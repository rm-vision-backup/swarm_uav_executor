#!/usr/bin/env python3
import threading
import time
import rospy
from swarm_uav_interfaces.msg import UavTaskState
from swarm_uav_interfaces.srv import UavHold, UavSafetyLease, UavTask
from swarm_uav_executor.executor import UavTaskExecutor
from swarm_uav_executor.models import ExecutorConfig, ExecutorIdentity, SafetyLeaseConfig
from swarm_uav_executor.safety_lease import SafetyLeaseWatchdog
from swarm_uav_executor.state_reporter import StateReporter
from swarm_uav_executor.task_store import TaskStore
from swarm_uav_executor.drivers.mock import MockMotionDriver


def load_config_from_ros_params():
    uav_id = rospy.get_param("~uav_id"); exec_target = rospy.get_param("~exec_target")
    if not uav_id or not exec_target: raise ValueError("uav_id and exec_target must be explicit")
    namespace = "/" + rospy.get_param("~service_namespace", exec_target).strip("/")
    identity = ExecutorIdentity(uav_id, exec_target)
    config = ExecutorConfig(rospy.get_param("~task_service", namespace + "/uav_task"),
        rospy.get_param("~hold_service", namespace + "/uav_hold"),
        rospy.get_param("~state_topic", namespace + "/uav_task_state"),
        tuple(rospy.get_param("~supported_commands", ["MOVE_TO"])), float(rospy.get_param("~store_ttl_s", 3600.0)),
        int(rospy.get_param("~store_max_records", 1024)), int(rospy.get_param("~terminal_republish_count", 3)),
        float(rospy.get_param("~terminal_republish_interval_s", 0.2)), float(rospy.get_param("~shutdown_hold_timeout_s", 2.0)))
    return identity, config


def build_driver(driver_name):
    if driver_name == "mock":
        return MockMotionDriver(rospy.get_param("~mock/result_delay_s", 0.5), rospy.get_param("~mock/final_success", True),
            rospy.get_param("~mock/error_code", ""), rospy.get_param("~mock/message", "mock UAV task finished"),
            rospy.get_param("~mock/ready", True), rospy.get_param("~mock/hold_success", True))
    if driver_name == "mavros_position":
        from swarm_uav_executor.drivers.mavros_position import MavrosPositionDriver
        return MavrosPositionDriver.from_ros_params()
    raise ValueError("unknown driver: %s" % driver_name)


class MonotonicWatchdogThread:
    def __init__(self, watchdog, frequency_hz):
        if frequency_hz <= 0.0: raise ValueError("watchdog_hz must be positive")
        self.watchdog = watchdog; self.period_s = 1.0 / float(frequency_hz)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.wait(self.period_s):
            event = self.watchdog.check_expiry()
            if event is not None:
                if event.hold_success:
                    rospy.logerr("safety lease expired and local HOLD latched mission=%s epoch=%s",
                                 event.mission_id, event.session_epoch)
                else:
                    rospy.logfatal("safety lease expired but local HOLD failed mission=%s epoch=%s error=%s message=%s",
                                   event.mission_id, event.session_epoch, event.error_code, event.message)

    def shutdown(self):
        self._stop.set(); self._thread.join(max(1.0, self.period_s * 2.0))


def main():
    rospy.init_node("uav_executor")
    identity, config = load_config_from_ros_params(); driver_name = rospy.get_param("~driver", "mock")
    driver = build_driver(driver_name); store = TaskStore(config.store_ttl_s, config.store_max_records)
    publisher = rospy.Publisher(config.state_topic, UavTaskState, queue_size=20)
    reporter = StateReporter(identity, store, publisher, config.terminal_republish_count, config.terminal_republish_interval_s)
    # Deadlines must use the same clock as the selected driver.  In particular,
    # MAVROS follows ROS time so SITL /use_sim_time cannot be mixed with wall time.
    executor = UavTaskExecutor(identity, config, store, reporter, driver,
                               clock=getattr(driver, "clock", None) or __import__("time").time)
    service_namespace = config.task_service.rsplit("/", 1)[0]
    lease_config = SafetyLeaseConfig(
        rospy.get_param("~safety_lease/service", service_namespace + "/uav_safety_lease"),
        float(rospy.get_param("~safety_lease/watchdog_hz", 10.0)),
        float(rospy.get_param("~safety_lease/default_ttl_s", 5.0)),
        float(rospy.get_param("~safety_lease/min_ttl_s", 1.0)),
        float(rospy.get_param("~safety_lease/max_ttl_s", 30.0)),
        float(rospy.get_param("~safety_lease/disarmed_stable_s", 3.0)),
        bool(rospy.get_param("~safety_lease/required_for_tasks", driver_name == "mavros_position")),
    )
    watchdog = SafetyLeaseWatchdog(
        identity, lease_config,
        lambda _event: executor.trigger_local_safety_hold("LEASE_EXPIRED", "GCS safety lease expired"),
        lambda: driver.can_end_safety_lease(lease_config.disarmed_stable_s), time.monotonic,
        executor.release_local_safety_latch)
    executor.lease_guard = watchdog
    task_service = rospy.Service(config.task_service, UavTask, executor.handle_task)
    hold_service = rospy.Service(config.hold_service, UavHold, executor.handle_hold)
    lease_service = rospy.Service(lease_config.service_name, UavSafetyLease, watchdog.handle_lease)
    watchdog_thread = MonotonicWatchdogThread(watchdog, lease_config.watchdog_hz)

    def shutdown():
        watchdog_thread.shutdown()
        for service in (lease_service, task_service, hold_service): service.shutdown("executor shutdown")
        executor.shutdown()

    rospy.on_shutdown(shutdown)
    rospy.loginfo("UAV executor ready identity=%s/%s driver=%s interfaces_version=%s", identity.uav_id, identity.exec_target,
                   driver_name, rospy.get_param("~interfaces_version", "deployment-unset"))
    rospy.spin()

if __name__ == "__main__": main()

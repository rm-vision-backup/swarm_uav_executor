#!/usr/bin/env python3
import threading
import time
import rospy
from swarm_uav_interfaces.msg import UavTaskState
from swarm_uav_interfaces.srv import UavHold, UavTask, UavTaskControl
from swarm_uav_executor.executor import UavTaskExecutor
from swarm_uav_executor.models import ExecutorConfig, ExecutorIdentity
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
        rospy.get_param("~task_control_service", namespace + "/uav_task_control"),
        bool(rospy.get_param("~require_explicit_start", False)),
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
    if driver_name == "ego_swarm":
        from swarm_uav_executor.drivers.ego_swarm import EgoSwarmDriver
        return EgoSwarmDriver.from_ros_params()
    raise ValueError("unknown driver: %s" % driver_name)


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
    task_service = rospy.Service(config.task_service, UavTask, executor.handle_task)
    task_control_service = rospy.Service(config.task_control_service, UavTaskControl, executor.handle_task_control)
    hold_service = rospy.Service(config.hold_service, UavHold, executor.handle_hold)

    def shutdown():
        for service in (task_control_service, task_service, hold_service): service.shutdown("executor shutdown")
        executor.shutdown()

    rospy.on_shutdown(shutdown)
    rospy.loginfo("UAV executor ready identity=%s/%s driver=%s interfaces_version=%s", identity.uav_id, identity.exec_target,
                   driver_name, rospy.get_param("~interfaces_version", "deployment-unset"))
    rospy.spin()

if __name__ == "__main__": main()

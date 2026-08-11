cmake_minimum_required(VERSION 3.0.2)
project(swarm_uav_executor)

find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  rostest
  swarm_uav_interfaces
  tf
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geographic_msgs geometry_msgs mavros_msgs nav_msgs rospy swarm_uav_interfaces tf
)

catkin_install_python(PROGRAMS
  scripts/pose_relay_node.py
  scripts/setpoint_relay_node.py
  scripts/uav_executor_node.py
  scripts/gp_origin_receiver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_validation.py)
  catkin_add_nosetests(test/test_task_store.py)
  catkin_add_nosetests(test/test_executor.py)
  catkin_add_nosetests(test/test_mock_driver.py)
  catkin_add_nosetests(test/test_mavros_position_driver.py)
  catkin_add_nosetests(test/test_safety_lease.py)
  catkin_add_nosetests(test/test_ego_swarm_driver.py)
  add_rostest(test/uav_executor_rostest.test)
endif()

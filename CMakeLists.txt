cmake_minimum_required(VERSION 3.0.2)
project(swarm_uav_executor)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  rospy
  rostest
  swarm_uav_interfaces
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geometry_msgs mavros_msgs rospy swarm_uav_interfaces
)

catkin_install_python(PROGRAMS
  scripts/uav_executor_node.py
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
  add_rostest(test/uav_executor_rostest.test)
endif()

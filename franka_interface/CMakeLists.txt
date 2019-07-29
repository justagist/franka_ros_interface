cmake_minimum_required(VERSION 2.8.3)
project(franka_interface)

find_package(catkin
  REQUIRED
  COMPONENTS
  cv_bridge
  rospy
  actionlib
  sensor_msgs
  std_msgs
  control_msgs
  trajectory_msgs
  dynamic_reconfigure
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  cv_bridge
  rospy
  actionlib
  sensor_msgs
  std_msgs
  control_msgs
  trajectory_msgs
  dynamic_reconfigure
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

cmake_minimum_required(VERSION 2.8.3)
project(sawyer_pykdl)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  tf
  intera_core_msgs
  intera_interface
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  geometry_msgs
  sensor_msgs
  urdfdom
  tf
  intera_interface
  intera_core_msgs
  DEPENDS
  python_orocos_kdl
  urdfdom
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

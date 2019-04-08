cmake_minimum_required(VERSION 2.8.3)
project(netft_rdt_driver)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  controller_manager
  diagnostic_msgs
  diagnostic_updater
  ##force_torque_sensor_controller
  geometry_msgs
  hardware_interface
  roscpp
  rtt_ros
  rtt_rosclock
)

find_package(Boost REQUIRED COMPONENTS system thread program_options)
find_package(CURL REQUIRED)
find_package(TinyXML REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES netft_rdt_driver netft_hardware_interface
  CATKIN_DEPENDS diagnostic_msgs diagnostic_updater geometry_msgs hardware_interface roscpp rtt_ros
)

include_directories(
 include ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(netft_rdt_driver src/netft_rdt_driver.cpp)
add_library(netft_hardware_interface src/netft_hardware_interface.cpp)
target_link_libraries(netft_rdt_driver 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${CURL_LIBRARIES} 
  ${TinyXML_LIBRARIES})

## Declare a cpp executable
add_executable(netft_node src/netft_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(netft_node netft_rdt_driver)

add_definitions(-DRTT_COMPONENT)
orocos_component(netft_rtt_controller src/netft_rtt_controller.cpp)
target_link_libraries(netft_rtt_controller netft_rdt_driver netft_hardware_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

orocos_generate_package()


#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS netft_rdt_driver netft_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

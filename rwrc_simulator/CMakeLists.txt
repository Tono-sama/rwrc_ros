cmake_minimum_required(VERSION 3.0.2)
project(rwrc_simulator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  tf2
  laser_geometry
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rwrc_simulator
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(get_sensors_node src/get_sensors_node.cpp src/sensors.cpp src/ros_util.cpp)

target_link_libraries(get_sensors_node
  ${catkin_LIBRARIES}
)

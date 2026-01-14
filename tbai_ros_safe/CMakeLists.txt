cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_safe)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  visualization_msgs
  tbai_ros_msgs
  tbai_ros_np3o
  tbai_ros_core
  tbai_ros_static
  tbai_ros_reference
)

find_package(tbai REQUIRED)
find_package(pinocchio REQUIRED)
find_package(spdlog REQUIRED)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs visualization_msgs tbai_ros_msgs tbai_ros_np3o
)

catkin_install_python(PROGRAMS
  src/tbai_safe_node_naive.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/virtual_joystick.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Build
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${pinocchio_INCLUDE_DIRS}
)

add_executable(simple
    src/simple.cpp
)
target_link_libraries(simple
    ${catkin_LIBRARIES}
    yaml-cpp
    spdlog::spdlog
    tbai::core
)
cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_gazebo)

## C++17 is a requirement
set (CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  roscpp
  realtime_tools
  tbai_ros_msgs
  tbai_ros_core
  std_msgs
)

find_package(tbai REQUIRED)
find_package(gazebo REQUIRED)

catkin_package(
 CATKIN_DEPENDS tbai_ros_msgs tbai_ros_core
 DEPENDS pinocchio
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

find_package(spdlog REQUIRED)

## Joint controller plugin
add_library(JointController src/JointController.cpp)
target_link_libraries(JointController ${catkin_LIBRARIES} tbai::core)

## RBD state plugin
# Robot state publisher plugin
add_library(StatePublisher src/StatePublisher.cpp)
target_link_libraries(StatePublisher ${gazebo_LIBRARIES} ${catkin_LIBRARIES} tbai::core)

## Robot state publisher plugin
add_library(RobotStatePublisher src/RobotStatePublisher.cpp)
target_link_libraries(RobotStatePublisher ${gazebo_LIBRARIES} ${catkin_LIBRARIES} tbai::core)

# Robot contact sensor
add_library(ContactSensor SHARED src/ContactSensor.cpp)
target_link_libraries(ContactSensor ${catkin_LIBRARIES} ${gazebo_LIBRARIES} tbai::core)

install(
  TARGETS
      JointController
      StatePublisher
      RobotStatePublisher
      ContactSensor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
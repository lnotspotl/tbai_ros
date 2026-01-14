cmake_minimum_required(VERSION 3.14)
project(tbai_ros_g1)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
  tbai_ros_core
  tbai_ros_msgs
  tbai_ros_static
  tbai_ros_reference
  kdl_parser
  robot_state_publisher
)

find_package(tbai REQUIRED)
find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib std_msgs sensor_msgs tbai_ros_core tbai_ros_msgs tbai_ros_static tbai_ros_reference
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# G1 RL Controller library
add_library(${PROJECT_NAME}
  src/G1RLController.cpp
  src/G1MimicController.cpp
  src/G1BeyondMimicController.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  spdlog::spdlog
  tbai::core
  tbai::reference
  tbai::deploy_g1
)

# Deployment executable
add_executable(deploy_g1_rl src/deploy_g1_rl.cpp)
target_link_libraries(deploy_g1_rl
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  tbai::deploy_g1
  tbai::static
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/g1_virtual_joystick.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(TARGETS ${PROJECT_NAME} deploy_g1_rl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

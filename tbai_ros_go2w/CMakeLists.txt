cmake_minimum_required(VERSION 3.14)
project(tbai_ros_go2w)

add_compile_options(-std=c++17)
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
  tbai_ros_core
  tbai_ros_msgs
  tbai_ros_static
  tbai_ros_reference
  tbai
  kdl_parser
  robot_state_publisher
)

find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Torch REQUIRED)
find_package(orocos_kdl REQUIRED)

# Find tbai_deploy_go2w library (built by tbai package)
find_library(TBAI_DEPLOY_GO2W_LIBRARY
  NAMES tbai_deploy_go2w
  PATHS ${catkin_LIBRARY_DIRS}
        ${CMAKE_INSTALL_PREFIX}/lib
        /usr/local/lib
)

if(NOT TBAI_DEPLOY_GO2W_LIBRARY)
  message(WARNING "tbai_deploy_go2w library not found, will attempt to link anyway")
  set(TBAI_DEPLOY_GO2W_LIBRARY "tbai_deploy_go2w")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib std_msgs sensor_msgs tbai_ros_core tbai_ros_msgs tbai_ros_static tbai_ros_reference tbai kdl_parser robot_state_publisher
  DEPENDS Eigen3 orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${TORCH_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

# Go2W DreamWaQ Controller library
add_library(${PROJECT_NAME}
  src/Go2WDreamWaQController.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${TORCH_LIBRARIES}
  spdlog::spdlog
)

# Deployment executable
add_executable(deploy_go2w_dreamwaq src/deploy_go2w_dreamwaq.cpp)
target_link_libraries(deploy_go2w_dreamwaq
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${TBAI_DEPLOY_GO2W_LIBRARY}
  ${orocos_kdl_LIBRARIES}
  yaml-cpp
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/go2w_virtual_joystick.py
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

install(TARGETS ${PROJECT_NAME} deploy_go2w_dreamwaq
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

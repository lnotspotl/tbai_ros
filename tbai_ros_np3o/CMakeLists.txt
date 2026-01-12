cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_np3o)

## C++17 is a requirement
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  tf
  urdf
  kdl_parser
  robot_state_publisher
  tbai_ros_msgs
  tbai_ros_core
  tbai_ros_static
  tbai_ros_gridmap
  tbai_ros_reference
  geometry_msgs
  visualization_msgs
)

find_package(Torch REQUIRED)
find_package(tbai REQUIRED)
find_package(pinocchio REQUIRED)
find_package(spdlog REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  ${PROJECT_NAME}
  CATKIN_DEPENDS tbai_ros_reference tbai_ros_gridmap
  DEPENDS
)

## Build
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${pinocchio_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/Visualizers.cpp
  src/Np3oController.cpp
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ${catkin_LIBRARIES}
    spdlog::spdlog
    ${pinocchio_LIBRARIES}
    tbai::core
    tbai::np3o
)

add_executable(simple
    src/simple.cpp
)
target_link_libraries(simple
    ${catkin_LIBRARIES} ${PROJECT_NAME} yaml-cpp
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
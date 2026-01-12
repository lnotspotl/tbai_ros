cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_static)

## C++17 is a requirement
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  tf
  urdf
  kdl_parser
  robot_state_publisher
  tbai_ros_msgs
  tbai_ros_core
  geometry_msgs
)

find_package(tbai REQUIRED)
find_package(spdlog REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  ${PROJECT_NAME}
  CATKIN_DEPENDS
  DEPENDS Eigen
)

## Build
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/StaticController.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    spdlog::spdlog
    tbai::core
    tbai::static
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
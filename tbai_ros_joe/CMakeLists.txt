cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_joe)

## C++17 is a requirement
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

find_package(Torch REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tbai_ros_msgs
  tbai_ros_core
  tbai_ros_static
  tbai_ros_reference
  tbai_ros_gridmap
  tbai_ros_ocs2
  tbai_ros_mpc
)

find_package(tbai REQUIRED)
find_package(spdlog REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  ${PROJECT_NAME}
  CATKIN_DEPENDS
  DEPENDS
)

find_package(pinocchio REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

## Build
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/RosJoeController.cpp
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ${catkin_LIBRARIES}
    spdlog::spdlog
    yaml-cpp
    ${pinocchio_LIBRARIES}
    ${TORCH_LIBRARIES}
    tbai::core
    tbai::joe
    tbai::quadruped_mpc
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(simple
    src/simple.cpp
)
target_link_libraries(simple
    ${catkin_LIBRARIES} ${PROJECT_NAME} yaml-cpp ${TORCH_LIBRARIES}
)
target_compile_options(simple PUBLIC ${OCS2_CXX_FLAGS})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_dtc)

## C++17 is a requirement
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

find_package(Torch REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tbai_ros_msgs
  tbai_ros_core
  tbai_ros_static
  tbai
  tbai_ros_ocs2
  tbai_ros_mpc
  tbai_ros_gridmap
  tbai_ros_reference
)

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
  src/RosDtcController.cpp
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ${catkin_LIBRARIES} spdlog::spdlog yaml-cpp
    ${pinocchio_LIBRARIES}
    ${TORCH_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(simple
    src/simple.cpp
)
target_link_libraries(simple
    ${catkin_LIBRARIES} ${PROJECT_NAME} yaml-cpp
)
target_compile_options(simple PUBLIC ${OCS2_CXX_FLAGS})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
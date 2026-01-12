cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_context_aware)

## C++17 is a requirement
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

find_package(Torch REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tbai_ros_core
    tbai_ros_mpc
    tbai_ros_static
    tbai_ros_bob
    tbai_ros_ocs2
)

find_package(tbai REQUIRED)
find_package(spdlog REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(simple
    src/simple.cpp
)
target_link_libraries(simple
    PUBLIC
    ${catkin_LIBRARIES}
    yaml-cpp
    spdlog::spdlog
    ${TORCH_LIBRARIES}
    tbai::core
)
target_compile_options(simple PUBLIC ${OCS2_CXX_FLAGS})

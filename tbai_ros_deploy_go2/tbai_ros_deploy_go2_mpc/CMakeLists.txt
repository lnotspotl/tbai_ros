cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_deploy_go2_mpc)

## C++17 is a requirement
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    tbai_ros_core
    tbai_ros_mpc
    tbai_ros_ocs2
    tbai_ros_static
    tbai_ros_reference
)

find_package(tbai REQUIRED)
find_package(spdlog REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS tbai_ros_mpc tbai_ros_static tbai_ros_reference
)


add_executable(go2_gait_node
    src/go2_gait_node.cpp
)
target_link_libraries(go2_gait_node
    PUBLIC
    ${catkin_LIBRARIES}
    tbai::core
)
target_include_directories(go2_gait_node
    PRIVATE
    "${catkin_INCLUDE_DIRS}" include
)

add_executable(deploy_go2_mpc
    src/deploy_go2_mpc.cpp
)
target_link_libraries(deploy_go2_mpc
    PUBLIC
    ${catkin_LIBRARIES}
    tbai::core
    tbai::deploy_go2
    tbai::reference
    tbai::static
)
target_include_directories(deploy_go2_mpc
    PRIVATE
    "${catkin_INCLUDE_DIRS}" include
)

add_executable(go2_joystick_node
    src/go2_joystick_node.cpp
)
target_link_libraries(go2_joystick_node
    PUBLIC
    ${catkin_LIBRARIES}
    tbai::core
    tbai::deploy_go2
    tbai::reference
)
target_include_directories(go2_joystick_node
    PRIVATE
    "${catkin_INCLUDE_DIRS}" include
)
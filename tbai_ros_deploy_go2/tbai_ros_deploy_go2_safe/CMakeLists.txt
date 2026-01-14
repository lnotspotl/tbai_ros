cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_deploy_go2_safe)

## C++17 is a requirement
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    tbai_ros_core
    tbai_ros_np3o
    tbai_ros_bob
    tbai_ros_static
    tbai_ros_reference
)

find_package(tbai REQUIRED)
find_package(spdlog REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS tbai_ros_np3o tbai_ros_bob tbai_ros_static tbai_ros_reference
)


add_executable(deploy_go2_safe
    src/deploy_go2_safe.cpp
)
target_link_libraries(deploy_go2_safe
    PUBLIC
    ${catkin_LIBRARIES}
    tbai::core
    tbai::deploy_go2
    tbai::reference
    tbai::static
)
target_include_directories(deploy_go2_safe
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

catkin_install_python(PROGRAMS
  src/tbai_safe_node_naive.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
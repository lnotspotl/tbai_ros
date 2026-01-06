cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_mpc)

## C++17 is a requirement
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    visualization_msgs
    tf
    urdf
    kdl_parser
    robot_state_publisher
    interactive_markers
    tbai_ros_core
    tbai_ros_msgs
    tbai_ros_ocs2
    grid_map_core
    grid_map_ros
    grid_map_filters_rsl
    tbai_ros_static
    tbai_ros_reference
    qpoases_catkin
    tbai
)

find_package(spdlog REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tbai_ros_mpc
  CATKIN_DEPENDS tbai_ros_reference tbai_ros_ocs2 tbai
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${pinocchio_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/quadruped_mpc/quadruped_reference/GridmapReferenceTrajectoryGenerator.cpp
    src/quadruped_mpc/utils/LoadConfig.cpp
    src/quadruped_mpc/RosMpcController.cpp
    src/quadruped_mpc/visualization/QuadrupedTfPublisher.cpp
    src/quadruped_mpc/visualization/QuadrupedVisualizer.cpp
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ${catkin_LIBRARIES} spdlog::spdlog
    ${pinocchio_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(mpc_node
  src/MpcNode.cpp
)
target_link_libraries(mpc_node
  ${catkin_LIBRARIES} yaml-cpp spdlog::spdlog ${PROJECT_NAME}
)
target_compile_options(mpc_node PRIVATE ${OCS2_CXX_FLAGS})

add_executable(simple_blind
    src/simple_blind.cpp
)
target_link_libraries(simple_blind
    ${catkin_LIBRARIES} yaml-cpp ${PROJECT_NAME}
)

add_executable(simple_perceptive
    src/simple_perceptive.cpp
)
target_link_libraries(simple_perceptive
    ${catkin_LIBRARIES} yaml-cpp ${PROJECT_NAME}
)

## Franka MPC visualization library
add_library(tbai_ros_franka_mpc
    src/franka_mpc/MobileManipulatorDummyVisualization.cpp
)
target_link_libraries(tbai_ros_franka_mpc
    PUBLIC
    ${catkin_LIBRARIES}
    ${pinocchio_LIBRARIES}
)
target_compile_options(tbai_ros_franka_mpc PUBLIC ${OCS2_CXX_FLAGS})

## Franka MPC node
add_executable(franka_mpc_node
    src/franka_mpc/MobileManipulatorMpcNode.cpp
)
target_link_libraries(franka_mpc_node
    ${catkin_LIBRARIES}
    tbai_ros_franka_mpc
)
target_compile_options(franka_mpc_node PRIVATE ${OCS2_CXX_FLAGS})

## Franka Dummy MRT node
add_executable(franka_dummy_mrt_node
    src/franka_mpc/MobileManipulatorDummyMRT.cpp
)
target_link_libraries(franka_dummy_mrt_node
    ${catkin_LIBRARIES}
    tbai_ros_franka_mpc
)
target_compile_options(franka_dummy_mrt_node PRIVATE ${OCS2_CXX_FLAGS})

## Franka target interactive marker node
add_executable(franka_target
    src/franka_mpc/MobileManipulatorTarget.cpp
)
target_link_libraries(franka_target
    ${catkin_LIBRARIES}
)
target_compile_options(franka_target PRIVATE ${OCS2_CXX_FLAGS})
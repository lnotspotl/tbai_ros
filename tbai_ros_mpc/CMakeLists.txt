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
)

find_package(tbai REQUIRED)
find_package(spdlog REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tbai_ros_mpc
  CATKIN_DEPENDS tbai_ros_reference tbai_ros_ocs2
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
    ${catkin_LIBRARIES}
    spdlog::spdlog
    ${pinocchio_LIBRARIES}
    tbai::core
    tbai::quadruped_mpc
    tbai::ocs2_core
    tbai::ocs2_mpc
    tbai::ocs2_ddp
    tbai::ocs2_pinocchio_interface
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(quadruped_mpc_node
  src/quadruped_mpc/QuadrupedMpcNode.cpp
)
target_link_libraries(quadruped_mpc_node
  ${catkin_LIBRARIES} yaml-cpp spdlog::spdlog ${PROJECT_NAME}
)
target_compile_options(quadruped_mpc_node PRIVATE ${OCS2_CXX_FLAGS})

add_executable(simple_blind
    src/quadruped_mpc/simple_blind.cpp
)
target_link_libraries(simple_blind
    ${catkin_LIBRARIES} yaml-cpp ${PROJECT_NAME}
)

add_executable(simple_perceptive
    src/quadruped_mpc/simple_perceptive.cpp
)
target_link_libraries(simple_perceptive
    ${catkin_LIBRARIES} yaml-cpp ${PROJECT_NAME}
)

## Quadruped Dummy MRT node
add_executable(quadruped_dummy_mrt_node
    src/quadruped_mpc/QuadrupedDummyMRT.cpp
)
target_link_libraries(quadruped_dummy_mrt_node
    ${catkin_LIBRARIES} yaml-cpp spdlog::spdlog ${PROJECT_NAME}
)
target_compile_options(quadruped_dummy_mrt_node PRIVATE ${OCS2_CXX_FLAGS})

## Quadruped ARM MPC library (for spot_arm with 6-DOF arm)
add_library(tbai_ros_quadruped_arm_mpc
    src/quadruped_arm_mpc/quadruped_reference/GridmapReferenceTrajectoryGenerator.cpp
    src/quadruped_arm_mpc/utils/LoadConfig.cpp
    src/quadruped_arm_mpc/RosMpcController.cpp
    src/quadruped_arm_mpc/visualization/QuadrupedTfPublisher.cpp
    src/quadruped_arm_mpc/visualization/QuadrupedVisualizer.cpp
    src/quadruped_arm_mpc/logic/GaitReceiver.cpp
    src/quadruped_arm_mpc/quadruped_interface/LocalTerrainReceiver.cpp
    src/quadruped_arm_mpc/quadruped_interface/TerrainReceiver.cpp
    src/quadruped_arm_mpc/quadruped_interface/SwingPlanningVisualizer.cpp
    src/quadruped_arm_mpc/quadruped_interface/TerrainPlaneVisualizer.cpp
    src/quadruped_arm_mpc/terrain/TerrainPlane.cpp
    src/quadruped_arm_mpc/segmented_planes/SegmentedPlanesTerrainModel.cpp
    src/quadruped_arm_mpc/segmented_planes/SegmentedPlanesTerrainModelRos.cpp
    src/quadruped_arm_mpc/common/GaitMsgConversions.cpp
)
target_link_libraries(tbai_ros_quadruped_arm_mpc
    PUBLIC
    ${catkin_LIBRARIES}
    spdlog::spdlog
    ${pinocchio_LIBRARIES}
    tbai::core
    tbai::quadruped_arm_mpc
    tbai::ocs2_core
    tbai::ocs2_mpc
    tbai::ocs2_oc
    tbai::ocs2_ddp
    tbai::ocs2_pinocchio_interface
)
target_compile_options(tbai_ros_quadruped_arm_mpc PUBLIC ${OCS2_CXX_FLAGS})

## Quadruped ARM MPC node
add_executable(quadruped_arm_mpc_node
    src/quadruped_arm_mpc/QuadrupedMpcNode.cpp
)
target_link_libraries(quadruped_arm_mpc_node
    ${catkin_LIBRARIES} yaml-cpp spdlog::spdlog tbai_ros_quadruped_arm_mpc
)
target_compile_options(quadruped_arm_mpc_node PRIVATE ${OCS2_CXX_FLAGS})

## Quadruped ARM Dummy MRT node
add_executable(quadruped_arm_dummy_mrt_node
    src/quadruped_arm_mpc/QuadrupedDummyMRT.cpp
)
target_link_libraries(quadruped_arm_dummy_mrt_node
    ${catkin_LIBRARIES} yaml-cpp spdlog::spdlog tbai_ros_quadruped_arm_mpc
)
target_compile_options(quadruped_arm_dummy_mrt_node PRIVATE ${OCS2_CXX_FLAGS})

## Quadruped ARM simple_blind node (MPC + WBC)
add_executable(simple_arm_blind
    src/quadruped_arm_mpc/simple_blind.cpp
)
target_link_libraries(simple_arm_blind
    ${catkin_LIBRARIES} yaml-cpp tbai_ros_quadruped_arm_mpc
)
target_compile_options(simple_arm_blind PRIVATE ${OCS2_CXX_FLAGS})

## Arm MPC visualization library (generic arm manipulator)
add_library(tbai_ros_arm_mpc
    src/arm_mpc/ArmDummyVisualization.cpp
    src/arm_mpc/ArmMpcController.cpp
    src/arm_mpc/ArmVisualizer.cpp
    src/arm_mpc/InteractiveMarkerTarget.cpp
)
target_link_libraries(tbai_ros_arm_mpc
    PUBLIC
    ${catkin_LIBRARIES}
    ${pinocchio_LIBRARIES}
    spdlog::spdlog
    tbai::core
    tbai::arm_mpc
    tbai::ocs2_core
    tbai::ocs2_mpc
    tbai::ocs2_ddp
    tbai::ocs2_pinocchio_interface
)
target_compile_options(tbai_ros_arm_mpc PUBLIC ${OCS2_CXX_FLAGS})

## Arm MPC node
add_executable(arm_mpc_node
    src/arm_mpc/ArmMpcNode.cpp
)
target_link_libraries(arm_mpc_node
    ${catkin_LIBRARIES}
    tbai_ros_arm_mpc
)
target_compile_options(arm_mpc_node PRIVATE ${OCS2_CXX_FLAGS})

## Arm Dummy MRT node
add_executable(arm_dummy_mrt_node
    src/arm_mpc/ArmDummyMRT.cpp
)
target_link_libraries(arm_dummy_mrt_node
    ${catkin_LIBRARIES}
    tbai_ros_arm_mpc
)
target_compile_options(arm_dummy_mrt_node PRIVATE ${OCS2_CXX_FLAGS})

## Arm target interactive marker node
add_executable(arm_target
    src/arm_mpc/ArmTarget.cpp
)
target_link_libraries(arm_target
    ${catkin_LIBRARIES}
    tbai::ocs2_core
)
target_compile_options(arm_target PRIVATE ${OCS2_CXX_FLAGS})

## Arm WBC simple node
add_executable(simple_arm_wbc
    src/arm_mpc/simple_arm_wbc.cpp
)
target_link_libraries(simple_arm_wbc
    ${catkin_LIBRARIES}
    tbai_ros_arm_mpc
    spdlog::spdlog
    yaml-cpp
)
target_compile_options(simple_arm_wbc PRIVATE ${OCS2_CXX_FLAGS})
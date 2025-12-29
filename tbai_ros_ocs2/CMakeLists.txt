cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_ocs2)

## C++17 is a requirement
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  message_generation
  tbai
  convex_plane_decomposition
  convex_plane_decomposition_ros
  convex_plane_decomposition_msgs
  grid_map_core
  grid_map_ros
  grid_map_sdf
  grid_map_filters_rsl
)

find_package(fmt REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    mpc_state.msg
    mpc_input.msg
    mode_schedule.msg
    mpc_observation.msg
    mpc_performance_indices.msg
    mpc_target_trajectories.msg
    controller_data.msg
    mpc_flattened_controller.msg
    lagrangian_metrics.msg
    multiplier.msg
    constraint.msg
    gait.msg
    gait_sequence.msg
    local_terrain.msg
    scheduled_gait_sequence.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
    reset.srv
)

## Generate added messages and services with any dependencies
generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs visualization_msgs message_runtime tbai
    convex_plane_decomposition convex_plane_decomposition_ros convex_plane_decomposition_msgs
    grid_map_core grid_map_ros grid_map_sdf grid_map_filters_rsl
)

## Build
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/MRT_ROS_Interface.cpp
  src/MPC_ROS_Interface.cpp
  src/RosReferenceManager.cpp
  src/RosMsgConversions.cpp
  src/RosMsgHelpers.cpp
  src/common/GaitMsgConversions.cpp
  src/visualization/VisualizationColors.cpp
  src/visualization/VisualizationHelpers.cpp
  src/logic/GaitReceiver.cpp
  src/logic/GaitCommandNode.cpp
  src/terrain/TerrainPlane.cpp
  src/quadruped_interface/SwingPlanningVisualizer.cpp
  src/quadruped_interface/TerrainPlaneVisualizer.cpp
  src/quadruped_interface/TerrainReceiver.cpp
  src/quadruped_interface/LocalTerrainReceiver.cpp
  src/segmented_planes/SegmentedPlanesTerrainModel.cpp
  src/segmented_planes/SegmentedPlanesTerrainModelRos.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  fmt::fmt
)

# Gait command node executable
add_executable(gait_command_node
  src/gait_command_node.cpp
)
add_dependencies(gait_command_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gait_command_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS gait_command_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

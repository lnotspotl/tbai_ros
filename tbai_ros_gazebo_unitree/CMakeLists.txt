cmake_minimum_required(VERSION 3.16.0)
project(tbai_ros_gazebo_unitree)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    pluginlib
)

find_package(gazebo REQUIRED)
find_package(tbai REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES UnitreeGazeboInterface G1GazeboInterface
    CATKIN_DEPENDS roscpp gazebo_ros pluginlib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# UnitreeGazeboInterface plugin
add_library(UnitreeGazeboInterface SHARED
    src/UnitreeGazeboInterface.cpp
)

target_link_libraries(UnitreeGazeboInterface
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    unitree_sdk2
    ddscxx
)

target_include_directories(UnitreeGazeboInterface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# G1GazeboInterface plugin (for G1 humanoid using unitree_hg messages)
add_library(G1GazeboInterface SHARED
    src/G1GazeboInterface.cpp
)

target_link_libraries(G1GazeboInterface
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    unitree_sdk2
    ddscxx
)

target_include_directories(G1GazeboInterface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS UnitreeGazeboInterface G1GazeboInterface
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY urdf/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

install(DIRECTORY worlds/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)

install(FILES controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

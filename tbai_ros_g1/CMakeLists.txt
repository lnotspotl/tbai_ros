cmake_minimum_required(VERSION 3.14)
project(tbai_ros_g1)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
  tbai_ros_core
  tbai_ros_msgs
  tbai_ros_static
  tbai_ros_reference
  tbai
  kdl_parser
  robot_state_publisher
)

find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)

# Find tbai_deploy_g1 library (built by tbai package)
find_library(TBAI_DEPLOY_G1_LIBRARY
  NAMES tbai_deploy_g1
  PATHS ${catkin_LIBRARY_DIRS}
        ${CMAKE_INSTALL_PREFIX}/lib
        /usr/local/lib
)

if(NOT TBAI_DEPLOY_G1_LIBRARY)
  message(WARNING "tbai_deploy_g1 library not found, will attempt to link anyway")
  set(TBAI_DEPLOY_G1_LIBRARY "tbai_deploy_g1")
endif()

# Fetch ONNX Runtime automatically
include(FetchContent)

if(NOT UNIX OR APPLE)
    message(FATAL_ERROR "tbai_ros_g1 currently only supports Linux platforms")
endif()

set(ONNXRUNTIME_VERSION "1.22.0")
set(ONNXRUNTIME_URL "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz")

message(STATUS "Fetching ONNX Runtime v${ONNXRUNTIME_VERSION}...")

FetchContent_Declare(
    onnxruntime
    URL ${ONNXRUNTIME_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(onnxruntime)

# Get the extracted directory path
FetchContent_GetProperties(onnxruntime SOURCE_DIR ONNXRUNTIME_SOURCE_DIR)

# Set up include and library directories
set(ONNXRUNTIME_INCLUDE_DIR "${onnxruntime_SOURCE_DIR}/include")
set(ONNXRUNTIME_LIB_DIR "${onnxruntime_SOURCE_DIR}/lib")

# Create imported target for ONNX Runtime
add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES
    IMPORTED_LOCATION "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so"
    INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}"
)

message(STATUS "ONNX Runtime configured: ${onnxruntime_SOURCE_DIR}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib std_msgs sensor_msgs tbai_ros_core tbai_ros_msgs tbai_ros_static tbai_ros_reference tbai
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# G1 RL Controller library
add_library(${PROJECT_NAME}
  src/G1RLController.cpp
  src/G1MimicController.cpp
  src/MotionLoader.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${ONNXRUNTIME_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  onnxruntime
  spdlog::spdlog
)

# Deployment executable
add_executable(deploy_g1_rl src/deploy_g1_rl.cpp)
target_link_libraries(deploy_g1_rl
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${TBAI_DEPLOY_G1_LIBRARY}
  yaml-cpp
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/g1_virtual_joystick.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(TARGETS ${PROJECT_NAME} deploy_g1_rl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install ONNX Runtime library for runtime
install(FILES "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so.${ONNXRUNTIME_VERSION}"
              "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so.1"
              "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so"
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

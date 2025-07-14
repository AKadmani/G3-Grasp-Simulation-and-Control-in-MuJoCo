cmake_minimum_required(VERSION 3.8)
project(grasp_simulation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glfw3 REQUIRED)

# Find MuJoCo - check multiple possible locations
find_path(MUJOCO_INCLUDE_DIRS mujoco/mujoco.h 
    PATHS 
    ${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco/include
    ~/grasp_ws/src/mujoco/include
    ~/mujoco/include 
    /usr/local/include
    /usr/include
)

find_library(MUJOCO_LIBRARIES 
    NAMES mujoco
    PATHS 
    ${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco/lib
    ~/grasp_ws/src/mujoco/lib
    ~/mujoco/lib 
    /usr/local/lib
    /usr/lib
)

if(NOT MUJOCO_INCLUDE_DIRS OR NOT MUJOCO_LIBRARIES)
  message(FATAL_ERROR "MuJoCo not found. Please install MuJoCo or set MUJOCO_INCLUDE_DIRS and MUJOCO_LIBRARIES")
else()
  message(STATUS "Found MuJoCo include: ${MUJOCO_INCLUDE_DIRS}")
  message(STATUS "Found MuJoCo library: ${MUJOCO_LIBRARIES}")
endif()

# Add RPATH for MuJoCo library
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
get_filename_component(MUJOCO_LIB_DIR ${MUJOCO_LIBRARIES} DIRECTORY)
set(CMAKE_INSTALL_RPATH ${MUJOCO_LIB_DIR})

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${MUJOCO_INCLUDE_DIRS}
)

# Grasp controller executable
add_executable(grasp_controller src/grasp_controller.cpp)
target_link_libraries(grasp_controller 
  ${MUJOCO_LIBRARIES}
  Eigen3::Eigen
)
ament_target_dependencies(grasp_controller
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  ament_index_cpp
)

# Visualizer executable
add_executable(mujoco_visualizer src/mujoco_visualizer.cpp)
target_link_libraries(mujoco_visualizer
  ${MUJOCO_LIBRARIES}
  glfw
  GL
)
ament_target_dependencies(mujoco_visualizer
  rclcpp
  sensor_msgs
  ament_index_cpp
)

# Minimal test executable
add_executable(minimal_test src/minimal_test.cpp)
ament_target_dependencies(minimal_test
  rclcpp
)

# Install executables
install(TARGETS
  grasp_controller
  mujoco_visualizer
  minimal_test
  DESTINATION lib/${PROJECT_NAME}
)

# Install models
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
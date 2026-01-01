cmake_minimum_required(VERSION 3.8)
project(sensor_fusion_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# ============================================================================
# BUILD ALL 4 NODES
# ============================================================================

# 1. IMU Sensor Node
add_executable(imu_sensor_node 
  src/sensors/imu_sensor_node.cpp
)
ament_target_dependencies(imu_sensor_node
  rclcpp
  sensor_msgs
)

# 2. Lidar Sensor Node
add_executable(lidar_sensor_node 
  src/sensors/lidar_sensor_node.cpp
)
ament_target_dependencies(lidar_sensor_node
  rclcpp
  sensor_msgs
)

# 3. Fusion Node
add_executable(fusion_node 
  src/fusion/fusion_node.cpp
)
ament_target_dependencies(fusion_node
  rclcpp
  sensor_msgs
  geometry_msgs
)
target_link_libraries(fusion_node
  Eigen3::Eigen
)

# 4. Safety Monitor Node
add_executable(safety_monitor_node 
  src/safety/safety_monitor_node.cpp
)
ament_target_dependencies(safety_monitor_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_srvs
)

# ============================================================================
# INSTALL ALL 4 NODES
# ============================================================================

install(TARGETS
  imu_sensor_node
  lidar_sensor_node
  fusion_node
  safety_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================================
# TESTING
# ============================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
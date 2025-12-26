cmake_minimum_required(VERSION 3.8)
project(sensor_fusion_system)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(imu_sensor_node src/sensors/imu_sensor_node.cpp)
ament_target_dependencies(imu_sensor_node rclcpp sensor_msgs)

install(TARGETS imu_sensor_node DESTINATION lib/${PROJECT_NAME})

ament_package()

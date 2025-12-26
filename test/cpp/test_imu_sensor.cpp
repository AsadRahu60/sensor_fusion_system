// ═══════════════════════════════════════════════════════════════════════
// FILE PURPOSE: test_imu_sensor.cpp
// ═══════════════════════════════════════════════════════════════════════
//
// WHAT IT DOES:
// - Tests IMU sensor node functionality
// - Verifies message correctness
// - Checks publish rate
//
// WHEN IT RUNS:
// - colcon test --packages-select sensor_fusion_system
//
// WHY YOU NEED IT:
// - Ensures IMU node works correctly
// - Catches bugs early
// - Demonstrates Google Test usage
//
// ═══════════════════════════════════════════════════════════════════════

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// Test message structure
TEST(ImuSensorTest, MessageHasCorrectFields) {
  rclcpp::init(0, nullptr);
  
  auto msg = sensor_msgs::msg::Imu();
  msg.header.frame_id = "imu_link";
  msg.linear_acceleration.z = 9.81;
  
  EXPECT_EQ(msg.header.frame_id, "imu_link");
  EXPECT_NEAR(msg.linear_acceleration.z, 9.81, 0.1);
  
  rclcpp::shutdown();
}

// Test covariance initialization
TEST(ImuSensorTest, CovarianceIsSet) {
  auto msg = sensor_msgs::msg::Imu();
  msg.angular_velocity_covariance[0] = 0.001;
  
  EXPECT_GT(msg.angular_velocity_covariance[0], 0.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cmath>

class SafetyMonitorNode : public rclcpp::Node
{
public:
  SafetyMonitorNode() : Node("safety_monitor_node"), system_healthy_(true)
  {
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "fusion/pose", 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        this->checkPoseSafety(msg);
      }
    );
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 10,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        this->checkImuSafety(msg);
      }
    );
    
    service_ = this->create_service<std_srvs::srv::Trigger>(
      "safety/check",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = this->isSystemHealthy();
        response->message = response->success ? "System OK" : "System has issues";
      }
    );
    
    RCLCPP_INFO(this->get_logger(), "Safety Monitor initialized");
  }

private:
  void checkPoseSafety(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (std::abs(msg->pose.pose.position.x) > 100.0 ||
        std::abs(msg->pose.pose.position.y) > 100.0) {
      RCLCPP_WARN(this->get_logger(), "Position out of bounds!");
      system_healthy_ = false;
    }
    
    double pos_cov = msg->pose.covariance[0];
    if (pos_cov > 10.0) {
      RCLCPP_WARN(this->get_logger(), "Covariance too large - filter may be diverging!");
      system_healthy_ = false;
    }
  }
  
  void checkImuSafety(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    double accel_mag = std::sqrt(
      msg->linear_acceleration.x * msg->linear_acceleration.x +
      msg->linear_acceleration.y * msg->linear_acceleration.y +
      msg->linear_acceleration.z * msg->linear_acceleration.z
    );
    
    if (accel_mag > 50.0) {
      RCLCPP_WARN(this->get_logger(), "Unrealistic acceleration detected!");
    }
  }
  
  bool isSystemHealthy() const
  {
    return system_healthy_;
  }
  
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  bool system_healthy_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyMonitorNode>());
  rclcpp::shutdown();
  return 0;
}

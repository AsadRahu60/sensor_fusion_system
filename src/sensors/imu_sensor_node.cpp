#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace std::chrono_literals;

class ImuNode : public rclcpp::Node
{
public:
  ImuNode() : Node("imu_sensor_node")
  {
    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    timer_ = this->create_wall_timer(10ms, [this]() { this->publish(); });
    RCLCPP_INFO(this->get_logger(), "IMU Node started at 100 Hz");
  }

private:
  void publish()
  {
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";
    msg.linear_acceleration.z = 9.81;
    pub_->publish(msg);
  }
  
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuNode>());
  rclcpp::shutdown();
  return 0;
}

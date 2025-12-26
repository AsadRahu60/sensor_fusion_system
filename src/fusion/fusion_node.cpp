#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <Eigen/Dense>
#include <memory>

using namespace std::chrono_literals;

constexpr size_t STATE_DIM = 6;
constexpr size_t IMU_MEAS_DIM = 3;
constexpr size_t LIDAR_MEAS_DIM = 2;

using StateVector = Eigen::Matrix<double, STATE_DIM, 1>;
using StateMatrix = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;

class FusionNode : public rclcpp::Node
{
public:
  FusionNode()
  : Node("fusion_node"),
    state_(StateVector::Zero()),
    covariance_(StateMatrix::Identity())
  {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 10,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        this->imuCallback(msg);
      }
    );
    
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "lidar/scan", 10,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        this->lidarCallback(msg);
      }
    );
    
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "fusion/pose", 10
    );
    
    timer_ = this->create_wall_timer(
      20ms,
      [this]() { this->predictionStep(); }
    );
    
    initializeKalmanFilter();
    
    RCLCPP_INFO(this->get_logger(), "Fusion Node initialized - running at 50 Hz");
  }

private:
  void initializeKalmanFilter()
  {
    process_noise_ = StateMatrix::Identity();
    process_noise_(0, 0) = 0.01;
    process_noise_(1, 1) = 0.01;
    process_noise_(2, 2) = 0.02;
    process_noise_(3, 3) = 0.05;
    process_noise_(4, 4) = 0.05;
    process_noise_(5, 5) = 0.03;
    
    imu_noise_ = Eigen::Matrix<double, IMU_MEAS_DIM, IMU_MEAS_DIM>::Identity();
    imu_noise_(0, 0) = 0.1;
    imu_noise_(1, 1) = 0.1;
    imu_noise_(2, 2) = 0.05;
    
    lidar_noise_ = Eigen::Matrix<double, LIDAR_MEAS_DIM, LIDAR_MEAS_DIM>::Identity();
    lidar_noise_(0, 0) = 0.02;
    lidar_noise_(1, 1) = 0.02;
    
    covariance_ = StateMatrix::Identity() * 1.0;
    last_update_time_ = this->now();
  }
  
  void predictionStep()
  {
    auto current_time = this->now();
    double dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;
    
    if (dt <= 0.0 || dt > 1.0) return;
    
    StateMatrix F = StateMatrix::Identity();
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;
    
    state_ = F * state_;
    state_(2) = normalizeAngle(state_(2));
    
    covariance_ = F * covariance_ * F.transpose() + process_noise_;
    
    publishPose();
  }
  
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    Eigen::Vector3d measurement;
    measurement(0) = msg->linear_acceleration.x;
    measurement(1) = msg->linear_acceleration.y;
    measurement(2) = msg->angular_velocity.z;
    
    Eigen::Matrix<double, IMU_MEAS_DIM, STATE_DIM> H;
    H.setZero();
    H(2, 5) = 1.0;
    
    auto S = H * covariance_ * H.transpose() + imu_noise_;
    auto K = covariance_ * H.transpose() * S.inverse();
    
    Eigen::Vector3d predicted_measurement = H * state_;
    Eigen::Vector3d innovation = measurement - predicted_measurement;
    
    state_ = state_ + K * innovation;
    state_(2) = normalizeAngle(state_(2));
    
    StateMatrix I = StateMatrix::Identity();
    covariance_ = (I - K * H) * covariance_;
  }
  
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    Eigen::Vector2d measurement;
    measurement(0) = state_(0) + 0.01 * (rand() % 100 - 50) / 50.0;
    measurement(1) = state_(1) + 0.01 * (rand() % 100 - 50) / 50.0;
    
    Eigen::Matrix<double, LIDAR_MEAS_DIM, STATE_DIM> H;
    H.setZero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    
    auto S = H * covariance_ * H.transpose() + lidar_noise_;
    auto K = covariance_ * H.transpose() * S.inverse();
    
    Eigen::Vector2d predicted_measurement = H * state_;
    Eigen::Vector2d innovation = measurement - predicted_measurement;
    
    state_ = state_ + K * innovation;
    state_(2) = normalizeAngle(state_(2));
    
    StateMatrix I = StateMatrix::Identity();
    covariance_ = (I - K * H) * covariance_;
  }
  
  void publishPose()
  {
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    
    msg->header.stamp = this->now();
    msg->header.frame_id = "map";
    
    msg->pose.pose.position.x = state_(0);
    msg->pose.pose.position.y = state_(1);
    msg->pose.pose.position.z = 0.0;
    
    double theta = state_(2);
    msg->pose.pose.orientation.z = sin(theta / 2.0);
    msg->pose.pose.orientation.w = cos(theta / 2.0);
    
    for (size_t i = 0; i < 6; ++i) {
      for (size_t j = 0; j < 6; ++j) {
        msg->pose.covariance[i * 6 + j] = covariance_(i, j);
      }
    }
    
    pose_pub_->publish(*msg);
  }
  
  double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }
  
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  StateVector state_;
  StateMatrix covariance_;
  StateMatrix process_noise_;
  Eigen::Matrix<double, IMU_MEAS_DIM, IMU_MEAS_DIM> imu_noise_;
  Eigen::Matrix<double, LIDAR_MEAS_DIM, LIDAR_MEAS_DIM> lidar_noise_;
  
  rclcpp::Time last_update_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionNode>());
  rclcpp::shutdown();
  return 0;
}

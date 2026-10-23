
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

struct Scanner {
  double angle_min{0.0};
  double angle_max{0.0};
  double angle_increments{0.0};
  double range_max{0.0};
  double range_min{0.0};
  size_t ray_count{0};
};

class Patrol : public rclcpp::Node {

public:
  Patrol(const std::string node_name) : Node(node_name), scan_received_(false) {

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", qos,
        std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1));

    auto timer_period = std::chrono::milliseconds(100);

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", 10);

    timer_ = this->create_wall_timer(timer_period,
                                     std::bind(&Patrol::timer_callback, this));
  }

private:
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    scan_data_ = msg;
    if (!scan_received_) {

      scan_info_.angle_min = msg->angle_min;
      scan_info_.angle_max = msg->angle_max;
      scan_info_.angle_increments = msg->angle_increment;
      scan_info_.ray_count = msg->ranges.size();
      scan_received_ = true;

      RCLCPP_INFO(this->get_logger(),
                  "Angle min :%0.3f ,Angle max : %0.3f  angle increment :%0.3f "
                  " Ray count: %zu ",
                  scan_info_.angle_min, scan_info_.angle_max,
                  scan_info_.angle_increments, scan_info_.ray_count);
    }

    // Need to get teh laser scan use teh fron 180
    // ignore inf and Nan values below range_min and range_max
  }
  void timer_callback() {}

  geometry_msgs::msg::Twist t_msg;
  sensor_msgs::msg::LaserScan::SharedPtr scan_data_;
  Scanner scan_info_;
  bool scan_received_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>("PatrolNode");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
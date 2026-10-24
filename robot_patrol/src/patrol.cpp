#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

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
        "/scan", qos,
        std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1));

    auto timer_period = std::chrono::milliseconds(100);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(timer_period,
                                     std::bind(&Patrol::timer_callback, this));
  }

private:
  int angle_to_index(double angle_deg) {

    double angle_rad = angle_deg * M_PI / 180.0;
    double nth_ray =
        (angle_rad - scan_info_.angle_min) / scan_info_.angle_increments;

    // nearest whole number
    int index = static_cast<int>(std::round(nth_ray));
    int total_rays = static_cast<int>(scan_info_.ray_count);

    return (index + total_rays) % total_rays;
  }

  bool is_valid(float r) {
    if (!std::isfinite(r))
      return false;
    if (r < scan_info_.range_min)
      return false;
    if (r > scan_info_.range_max)
      return false;
    return true;
  }

  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // getting the pointer data so we can use teh timer callback as a control
    // loop.
    scan_data_ = msg;

    if (!scan_received_) {

      scan_info_.angle_min = msg->angle_min;
      scan_info_.angle_max = msg->angle_max;
      scan_info_.range_max = msg->range_max;
      scan_info_.range_min = msg->range_min;
      scan_info_.angle_increments = msg->angle_increment;
      scan_info_.ray_count = msg->ranges.size();
      scan_received_ = true;

      RCLCPP_INFO(this->get_logger(),
                  "Angle min :%0.3f ,Angle max : %0.3f  angle increment :%0.3f "
                  " Ray count: %zu ",
                  scan_info_.angle_min, scan_info_.angle_max,
                  scan_info_.angle_increments, scan_info_.ray_count);
      int a = angle_to_index(0);
      int b = angle_to_index(90);
      int c = angle_to_index(-90);
      int d = angle_to_index(360);
      RCLCPP_INFO(this->get_logger(), " 0:%d ,90 : %d , -90: %d , 360 : %d", a,
                  b, c, d);
    }

    front_ranges_left_.clear();
    front_ranges_right_.clear();

    int n = static_cast<int>(msg->ranges.size());

    //-90 - 0
    for (int i = angle_to_index(-90); i < n; i++) {
      front_ranges_right_.push_back(msg->ranges[i]);
    }

    // 0 - 90
    for (int i = 0; i <= angle_to_index(90); i++) {
      front_ranges_left_.push_back(msg->ranges[i]);
    }
  }
  void timer_callback() {
    if (!scan_received_)
      return;

    int w = angle_to_index(20);
    float front_min_left = scan_info_.range_max;
    float front_min_right = scan_info_.range_max;
    float front_max_left = scan_info_.range_min;
    float front_max_right = scan_info_.range_min;

    int left_size = static_cast<int>(front_ranges_left_.size());
    for (int i = 0; i < left_size; i++) {
      float r = front_ranges_left_[i];
      if (is_valid(r) && r < front_min_left && i <= w)
        front_min_left = r;

      if (is_valid(r) && r > front_max_left)
        front_max_left = r;
    }

    int right_size = static_cast<int>(front_ranges_right_.size());
    for (int i = 0; i < right_size; i++) {
      float r = front_ranges_right_[i];
      if (is_valid(r) && r < front_min_right && i >= (right_size - w))
        front_min_right = r;
      if (is_valid(r) && r > front_max_right)
        front_max_right = r;
    }

    float front_min = std::min(front_min_left, front_min_right);
    geometry_msgs::msg::Twist cmd;
    if (front_min > 0.35) {
      cmd.linear.x = 0.1;
      cmd.angular.z = 0.0;
    } else {
      cmd.linear.x = 0.05;
      cmd.angular.z = (front_max_left > front_max_right) ? 0.5 : -0.5;
    }
    publisher_->publish(cmd);
  }

  sensor_msgs::msg::LaserScan::SharedPtr scan_data_;
  Scanner scan_info_;
  bool scan_received_;
  std::vector<float> front_ranges_left_;
  std::vector<float> front_ranges_right_;
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
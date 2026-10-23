
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

class Patrol : public rclcpp::Node {

public:
  Patrol(const std::string node_name) : Node(node_name) {

    auto timer_period = std::chrono::milliseconds(100);

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", 10);

    timer_ = this->create_wall_timer(timer_period,
                                     std::bind(&Patrol::timer_callback, this));

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", qos,
        std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1));
  }

private:
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // Need to get teh laser scan use teh fron 180
    // ignore inf and Nan values below range_min and range_max
  }
  void timer_callback() {}

  geometry_msgs::msg::Twist t_msg;

  double values_min_;
  double values_max_;

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
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class CmdVelSubscriber : public rclcpp::Node
{
public:
  CmdVelSubscriber() : Node("cmd_vel_subscriber")
  {
    // Create a subscriber to the "/cmd_vel" topic with message type geometry_msgs/msg/Twist
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&CmdVelSubscriber::cmd_vel_callback, this, std::placeholders::_1));
  }

private:
  // Callback function to process received cmd_vel messages
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Print the received cmd_vel message values to the console
    RCLCPP_INFO(this->get_logger(), "Received cmd_vel message:");
    RCLCPP_INFO(this->get_logger(), "Linear velocity: x = %f, y = %f, z = %f", msg->linear.x, msg->linear.y, msg->linear.z);
    RCLCPP_INFO(this->get_logger(), "Angular velocity: x = %f, y = %f, z = %f", msg->angular.x, msg->angular.y, msg->angular.z);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);

  // Create the CmdVelSubscriber node and spin it
  rclcpp::spin(std::make_shared<CmdVelSubscriber>());

  // Shutdown ROS 2 when finished
  rclcpp::shutdown();
  return 0;
}

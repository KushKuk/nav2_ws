#include "differential_steering/ackermann_cmd_vel_converter.hpp"
#include <cmath>
#include <algorithm>

namespace differential_steering
{

AckermannCmdVelConverter::AckermannCmdVelConverter()
: Node("ackermann_cmd_vel_converter")
{
    // Declare parameters
    this->declare_parameter<double>("wheel_radius", 0.1125);  // From URDF wheel size
    this->declare_parameter<double>("max_steering_angle", M_PI/3);  // 60 degrees max
    this->declare_parameter<double>("robot_length", 1.0);  // Distance from front to rear axle
    this->declare_parameter<double>("robot_width", 0.54);   // Distance between left/right wheels

    // Get parameters
    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("max_steering_angle", max_steering_angle_);
    this->get_parameter("robot_length", robot_length_);
    this->get_parameter("robot_width", robot_width_);

    // Initialize wheel positions based on URDF (x, y coordinates relative to base_link)
    // Order: [wheel_1, wheel_2, wheel_3, wheel_4, wheel_5, wheel_6]
    wheel_positions_ = {
        {-0.52014, -0.27},  // Wheel 1: Front Left
        {0.042287, -0.27},  // Wheel 2: Front Right
        {0.47299, -0.225},  // Wheel 3: Middle Left
        {-0.51862, 0.27},   // Wheel 4: Middle Right
        {0.0438, 0.27},     // Wheel 5: Rear Left
        {0.47715, 0.225}    // Wheel 6: Rear Right
    };

    // Publishers for individual wheel control
    drive_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/drive_controller/commands", 10);

    // Subscriber for velocity commands
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&AckermannCmdVelConverter::cmdVelCallback, this, std::placeholders::_1));
}

void AckermannCmdVelConverter::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;

    // Calculate linear speeds of left and right wheels for skid steering
    double left_linear = linear_x - (angular_z * robot_width_ / 2.0);
    double right_linear = linear_x + (angular_z * robot_width_ / 2.0);

    // Convert to angular velocity (rad/s) for wheels
    double left_speed = left_linear / wheel_radius_;
    double right_speed = right_linear / wheel_radius_;

    // Wheel rotational speeds (rad/s)
    std::vector<double> wheel_speeds(6, 0.0);

    // Assign speeds to the wheels based on their side.
    // Assuming wheels 4, 5, 6 are left and 1, 2, 3 are right based on y-coordinates in constructor.
    // Left wheels
    wheel_speeds[3] = left_speed;
    wheel_speeds[4] = left_speed;
    wheel_speeds[5] = left_speed;

    // Right wheels
    wheel_speeds[0] = right_speed;
    wheel_speeds[1] = right_speed;
    wheel_speeds[2] = right_speed;

    // Publish drive commands
    std_msgs::msg::Float64MultiArray drive_msg;
    drive_msg.data = wheel_speeds;
    drive_pub_->publish(drive_msg);

    // Debug logging
    RCLCPP_INFO(this->get_logger(),
                "Cmd: vx=%.2f, wz=%.2f -> Left Speed: %.2f rad/s, Right Speed: %.2f rad/s",
                linear_x, angular_z, left_speed, right_speed);
    RCLCPP_INFO(this->get_logger(),
                "Wheel Speeds (rad/s): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                wheel_speeds[0], wheel_speeds[1], wheel_speeds[2],
                wheel_speeds[3], wheel_speeds[4], wheel_speeds[5]);
}
}  
// namespace differential_steering

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<differential_steering::AckermannCmdVelConverter>());
    rclcpp::shutdown();
    return 0;
}

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
    steer_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/steer_controller/commands", 10);

    // Subscriber for velocity commands
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&AckermannCmdVelConverter::cmdVelCallback, this, std::placeholders::_1));
}

void AckermannCmdVelConverter::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;

    // Calculate individual wheel velocities and steering angles
    std::vector<double> wheel_speeds(6, 0.0);
    std::vector<double> steering_angles(6, 0.0);

    if (std::abs(angular_z) < 1e-6) {
        // Pure forward/backward motion - all wheels same speed, zero steering
        std::fill(wheel_speeds.begin(), wheel_speeds.end(), linear_x / wheel_radius_);
        std::fill(steering_angles.begin(), steering_angles.end(), 0.0);
    } else {
        // Calculate instantaneous center of rotation (ICR)
        double icr_x = 0.0;  // ICR on the y-axis for pure rotation about center
        double icr_y = linear_x / angular_z;  // Distance from robot center to ICR
        
        // Calculate individual wheel parameters
        for (size_t i = 0; i < 6; ++i) {
            double wheel_x = wheel_positions_[i].first;
            double wheel_y = wheel_positions_[i].second;
            
            // Vector from ICR to wheel
            double dx = wheel_x - icr_x;
            double dy = wheel_y - icr_y;
            
            // Distance from ICR to wheel
            double radius = std::sqrt(dx * dx + dy * dy);
            
            // Wheel velocity magnitude
            double wheel_velocity = angular_z * radius;
            
            // Convert to wheel rotational speed (rad/s)
            wheel_speeds[i] = wheel_velocity / wheel_radius_;
            
            // Steering angle - direction from wheel to ICR
            if (std::abs(radius) > 1e-6) {
                steering_angles[i] = std::atan2(-dx, -dy);
                
                // Limit steering angle
                steering_angles[i] = std::max(-max_steering_angle_, 
                                            std::min(max_steering_angle_, steering_angles[i]));
            } else {
                steering_angles[i] = 0.0;
            }
        }
    }

    // Publish commands
    std_msgs::msg::Float64MultiArray drive_msg;
    std_msgs::msg::Float64MultiArray steer_msg;
    
    drive_msg.data = wheel_speeds;
    steer_msg.data = steering_angles;

    drive_pub_->publish(drive_msg);
    steer_pub_->publish(steer_msg);

    // Log the commands for debugging
    RCLCPP_DEBUG(this->get_logger(), 
                "Cmd: vx=%.2f, wz=%.2f", linear_x, angular_z);
    RCLCPP_DEBUG(this->get_logger(),
                "Speeds: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                wheel_speeds[0], wheel_speeds[1], wheel_speeds[2],
                wheel_speeds[3], wheel_speeds[4], wheel_speeds[5]);
    RCLCPP_DEBUG(this->get_logger(),
                "Angles: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                steering_angles[0], steering_angles[1], steering_angles[2],
                steering_angles[3], steering_angles[4], steering_angles[5]);
}

}  // namespace differential_steering

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<differential_steering::AckermannCmdVelConverter>());
    rclcpp::shutdown();
    return 0;
}

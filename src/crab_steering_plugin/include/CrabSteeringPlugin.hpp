#ifndef CRAB_STEERING_PLUGIN_HPP
#define CRAB_STEERING_PLUGIN_HPP

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/Name.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace crab_steering
{
  // Simple PID struct
  struct PIDData
  {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};

    double integral{0.0};
    double previousError{0.0};
  };

  class CrabSteeringPlugin
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
  {
  public:
    // Constructor
    CrabSteeringPlugin() = default;

    // Destructor
    ~CrabSteeringPlugin() = default;

    // Configure method (called once at the beginning)
    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &eventMgr) override;

    // PreUpdate method (called every simulation step)
    void PreUpdate(const ignition::gazebo::UpdateInfo &info,
                   ignition::gazebo::EntityComponentManager &_ecm) override;

    // Callback for /cmd_vel messages
    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  private:
    // ROS 2 Node, subscriber, publisher, and transform broadcaster
    std::shared_ptr<rclcpp::Node> rosNode;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdSub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

    // Joint entities
    std::vector<ignition::gazebo::Entity> steeringJoints;
    std::vector<ignition::gazebo::Entity> wheelJoints;

    // PID controllers per steering joint
    std::unordered_map<ignition::gazebo::Entity, PIDData> pidControllers;

    // Motion commands
    double linearX{0.0};
    double linearY{0.0};

    // Odometry variables
    double x{0.0}, y{0.0}, theta{0.0};
    rclcpp::Time lastTime;

    // Model reference
    ignition::gazebo::Entity modelEntity;
  };
}

#endif  // CRAB_STEERING_PLUGIN_HPP

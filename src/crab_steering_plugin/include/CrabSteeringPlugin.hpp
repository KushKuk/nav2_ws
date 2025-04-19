#ifndef CRAB_STEERING_PLUGIN_HPP
#define CRAB_STEERING_PLUGIN_HPP

#include <memory>
#include <vector>
#include <string>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/Name.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace crab_steering
{
  class CrabSteeringPlugin
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
  {
  public:
    // Constructor
    CrabSteeringPlugin() = default;

    // Configure method (called once at the beginning)
    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &eventMgr) override;

    // PreUpdate method (called at every simulation step)
    void PreUpdate(const ignition::gazebo::UpdateInfo &info,
                   ignition::gazebo::EntityComponentManager &_ecm) override;

    // Callback for cmd_vel messages
    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  private:
    // ROS 2 Node and subscriber
    std::shared_ptr<rclcpp::Node> rosNode;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdSub;

    // Entities representing steering and drive wheel joints
    std::vector<ignition::gazebo::Entity> steeringJoints;
    std::vector<ignition::gazebo::Entity> wheelJoints;

    // Linear velocities from Twist messages
    double linearX{0.0};
    double linearY{0.0};

    // Model entity this plugin is attached to
    ignition::gazebo::Entity modelEntity;
  };
}

#endif  // CRAB_STEERING_PLUGIN_HPP

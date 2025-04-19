#include "CrabSteeringPlugin.hpp"

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/Name.hh>

using namespace crab_steering;

void CrabSteeringPlugin::Configure(const ignition::gazebo::Entity &_entity,
                                   const std::shared_ptr<const sdf::Element> & /*_sdf*/,
                                   ignition::gazebo::EntityComponentManager &_ecm,
                                   ignition::gazebo::EventManager & /*_eventMgr*/)
{
  this->modelEntity = _entity;

  // Find joints and categorize into steering and wheel joints
  _ecm.Each<ignition::gazebo::components::Joint, ignition::gazebo::components::Name>(
      [&](const ignition::gazebo::Entity &entity,
          const ignition::gazebo::components::Joint *,
          const ignition::gazebo::components::Name *name) {
        std::string jointName = name->Data();
        if (jointName.find("steer") != std::string::npos)
          this->steeringJoints.push_back(entity);
        else if (jointName.find("wheel") != std::string::npos)
          this->wheelJoints.push_back(entity);
        return true;
      });

  // Initialize ROS 2 node
  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  this->rosNode = std::make_shared<rclcpp::Node>("crab_steering_plugin");

  // Subscribe to /cmd_vel
  this->cmdSub = rosNode->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CrabSteeringPlugin::CmdVelCallback, this, std::placeholders::_1));
}

void CrabSteeringPlugin::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  this->linearX = msg->linear.x;
  this->linearY = msg->linear.y;
}

void CrabSteeringPlugin::PreUpdate(const ignition::gazebo::UpdateInfo & /*info*/,
                                   ignition::gazebo::EntityComponentManager &_ecm)
{
  // Calculate desired steering direction as angular velocity
  double steerAngle = std::atan2(this->linearY, std::abs(this->linearX) + 1e-5);

  // Scale angle to velocity (tunable gain)
  const double steeringGain = 1.0;
  double steerVelocity = steerAngle * steeringGain;

  for (auto &steerJoint : this->steeringJoints)
  {
    _ecm.SetComponentData<ignition::gazebo::components::JointVelocityCmd>(
        steerJoint, {steerVelocity});
  }

  double speed = std::sqrt(linearX * linearX + linearY * linearY);
  for (auto &wheelJoint : this->wheelJoints)
  {
    _ecm.SetComponentData<ignition::gazebo::components::JointVelocityCmd>(
        wheelJoint, {speed});
  }
}

// Register the plugin
IGNITION_ADD_PLUGIN(CrabSteeringPlugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(CrabSteeringPlugin, "crab_steering::CrabSteeringPlugin")

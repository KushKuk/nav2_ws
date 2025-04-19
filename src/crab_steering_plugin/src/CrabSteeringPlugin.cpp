#include "CrabSteeringPlugin.hpp"

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/Name.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <cmath>
#include <unordered_map>
#include <algorithm>

using namespace crab_steering;

void CrabSteeringPlugin::Configure(const ignition::gazebo::Entity &_entity,
                                   const std::shared_ptr<const sdf::Element> & /*_sdf*/,
                                   ignition::gazebo::EntityComponentManager &_ecm,
                                   ignition::gazebo::EventManager & /*_eventMgr*/)
{
  this->modelEntity = _entity;

  // Discover joints
  _ecm.Each<ignition::gazebo::components::Joint, ignition::gazebo::components::Name>(
      [&](const ignition::gazebo::Entity &entity,
          const ignition::gazebo::components::Joint *,
          const ignition::gazebo::components::Name *name) {
        std::string jointName = name->Data();
        if (jointName.find("steer") != std::string::npos)
        {
          this->steeringJoints.push_back(entity);
          _ecm.CreateComponent(entity, ignition::gazebo::components::JointPosition());

          // Initialize PID controller for this joint
          PIDData pid;
          pid.kp = 5.0;
          pid.ki = 0.0;
          pid.kd = 0.2;
          this->pidControllers[entity] = pid;
        }
        else if (jointName.find("wheel") != std::string::npos)
        {
          this->wheelJoints.push_back(entity);
        }
        return true;
      });

  // ROS 2 Node setup
  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  this->rosNode = std::make_shared<rclcpp::Node>("crab_steering_plugin");

  this->cmdSub = rosNode->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CrabSteeringPlugin::CmdVelCallback, this, std::placeholders::_1));
}

void CrabSteeringPlugin::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_INFO(this->rosNode->get_logger(), "Received cmd_vel message: linear x = %f, y = %f", msg->linear.x, msg->linear.y);
  this->linearX = msg->linear.x;
  this->linearY = msg->linear.y;
}

void CrabSteeringPlugin::PreUpdate(const ignition::gazebo::UpdateInfo & /*info*/,
                                   ignition::gazebo::EntityComponentManager &_ecm)
{
  double targetAngle = std::atan2(this->linearY, std::abs(this->linearX) + 1e-5);

  for (auto &steerJoint : this->steeringJoints)
  {
    auto posComp = _ecm.Component<ignition::gazebo::components::JointPosition>(steerJoint);
    if (!posComp || posComp->Data().empty())
      continue;

    double currentAngle = posComp->Data()[0];
    double error = targetAngle - currentAngle;

    PIDData &pid = this->pidControllers[steerJoint];

    // PID update
    pid.integral += error;
    double derivative = error - pid.previousError;
    pid.previousError = error;

    double velCmd = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;

    // Clamp the velocity if needed
    velCmd = std::clamp(velCmd, -1.0, 1.0);

    _ecm.SetComponentData<ignition::gazebo::components::JointVelocityCmd>(
        steerJoint, {velCmd});
  }

  // Control wheel joints
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

IGNITION_ADD_PLUGIN_ALIAS(CrabSteeringPlugin, "crab_steering")

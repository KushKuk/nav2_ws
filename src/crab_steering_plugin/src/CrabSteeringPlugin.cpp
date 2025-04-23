#include "CrabSteeringPlugin.hpp"

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/Name.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

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

    // Discover joints and initialize components
    _ecm.Each<ignition::gazebo::components::Joint, ignition::gazebo::components::Name>(
        [&](const ignition::gazebo::Entity &entity,
            const ignition::gazebo::components::Joint * /*joint*/,
            const ignition::gazebo::components::Name *name) -> bool
        {
            const std::string jointName = name->Data();

            if (jointName.find("rotate") != std::string::npos)
            {
                this->steeringJoints.push_back(entity);

                if (!_ecm.Component<ignition::gazebo::components::JointPosition>(entity))
                {
                    _ecm.CreateComponent(entity, ignition::gazebo::components::JointPosition());
                }

                if (!_ecm.Component<ignition::gazebo::components::JointVelocityCmd>(entity))
                {
                    _ecm.CreateComponent(entity, ignition::gazebo::components::JointVelocityCmd({0.0}));
                }

                // Init PID
                PIDData pid;
                pid.kp = 5.0;
                pid.ki = 0.0;
                pid.kd = 0.2;
                this->pidControllers[entity] = pid;

                RCLCPP_INFO(this->rosNode->get_logger(), "Added rotate joint: %s", jointName.c_str());
            }
            else if (jointName.find("wheel") != std::string::npos)
            {
                this->wheelJoints.push_back(entity);

                if (!_ecm.Component<ignition::gazebo::components::JointVelocityCmd>(entity))
                {
                    _ecm.CreateComponent(entity, ignition::gazebo::components::JointVelocityCmd({0.0}));
                }
                RCLCPP_INFO(this->rosNode->get_logger(), "Added wheel joint: %s", jointName.c_str());
            }
            return true;
        });

    // Safe ROS 2 initialization
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    this->rosNode = std::make_shared<rclcpp::Node>("crab_steering_plugin");

    this->cmdSub = rosNode->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&CrabSteeringPlugin::CmdVelCallback, this, std::placeholders::_1));

    // Odometry publisher
    this->odomPub = this->rosNode->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    this->tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this->rosNode);
    this->lastTime = this->rosNode->now();
}

void CrabSteeringPlugin::PreUpdate(const ignition::gazebo::UpdateInfo & /*info*/,
    ignition::gazebo::EntityComponentManager &_ecm)
{
    double targetAngle = std::atan2(this->linearY, std::abs(this->linearX) + 1e-5);

    // Control steering joints
    for (auto &steerJoint : this->steeringJoints)
    {
        auto posComp = _ecm.Component<ignition::gazebo::components::JointPosition>(steerJoint);
        if (!posComp || posComp->Data().empty())
            continue;

        double currentAngle = posComp->Data()[0];
        double error = targetAngle - currentAngle;

        PIDData &pid = this->pidControllers[steerJoint];
        pid.integral += error;
        double derivative = error - pid.previousError;
        pid.previousError = error;

        double velCmd = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
        velCmd = std::clamp(velCmd, -1.0, 1.0);

        auto velCmdComp = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(steerJoint);
        if (velCmdComp)
        {
            _ecm.SetComponentData<ignition::gazebo::components::JointVelocityCmd>(
                steerJoint, {velCmd});
        }
    }

    // Control wheel joints
    double speed = std::sqrt(linearX * linearX + linearY * linearY);
    for (auto &wheelJoint : this->wheelJoints)
    {
        auto velCmdComp = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(wheelJoint);
        if (velCmdComp)
        {
            _ecm.SetComponentData<ignition::gazebo::components::JointVelocityCmd>(
                wheelJoint, {speed});
        }
    }

    // Odometry calculation
    auto currentTime = this->rosNode->now();
    double dt = (currentTime - this->lastTime).seconds();

    // Simple odometry calculation
    double vx = this->linearX;
    double vy = this->linearY;
    double delta_x = vx * std::cos(this->theta) * dt - vy * std::sin(this->theta) * dt;
    double delta_y = vx * std::sin(this->theta) * dt + vy * std::cos(this->theta) * dt;
    double delta_theta = 0.0; // assuming no yaw change

    this->x += delta_x;
    this->y += delta_y;
    this->theta += delta_theta;

    // Create odometry message
    geometry_msgs::msg::Quaternion odom_quat;
    odom_quat.z = std::sin(this->theta / 2.0);
    odom_quat.w = std::cos(this->theta / 2.0);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = currentTime;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = this->x;
    odom.pose.pose.position.y = this->y;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;

    this->odomPub->publish(odom);

    // Publish TF transform
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = currentTime;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = this->x;
    odom_tf.transform.translation.y = this->y;
    odom_tf.transform.rotation = odom_quat;
    this->tfBroadcaster->sendTransform(odom_tf);

    this->lastTime = currentTime;
}

void CrabSteeringPlugin::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    RCLCPP_INFO(this->rosNode->get_logger(),
                "Received cmd_vel: linear.x = %.2f, linear.y = %.2f",
                msg->linear.x, msg->linear.y);
    this->linearX = msg->linear.x;
    this->linearY = msg->linear.y;
}

// Register the plugin
IGNITION_ADD_PLUGIN(CrabSteeringPlugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(CrabSteeringPlugin, "crab_steering")

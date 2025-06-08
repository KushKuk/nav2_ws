#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from math import cos, sin
import numpy as np

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.declare_parameter('wheel_radius', 0.1125)
        self.declare_parameter('wheel_separation', 0.54)
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_time = self.get_clock().now()

    def joint_state_callback(self, msg):
        left_vel, right_vel = 0.0, 0.0
        for i, name in enumerate(msg.name):
            if name in ['joint_wheel_1', 'joint_wheel_2', 'joint_wheel_3']:
                left_vel += msg.velocity[i] * self.wheel_radius
            elif name in ['joint_wheel_4', 'joint_wheel_5', 'joint_wheel_6']:
                right_vel += msg.velocity[i] * self.wheel_radius
        left_vel /= 3.0
        right_vel /= 3.0

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        v = (left_vel + right_vel) / 2.0
        omega = (right_vel - left_vel) / self.wheel_separation

        self.x += v * cos(self.theta) * dt
        self.y += v * sin(self.theta) * dt
        self.theta += omega * dt

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = np.array([0.0, 0.0, np.sin(self.theta / 2), np.cos(self.theta / 2)])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        odom.pose.covariance[0] = 0.001
        odom.pose.covariance[7] = 0.001
        odom.pose.covariance[14] = 0.001
        odom.pose.covariance[21] = 0.001
        odom.pose.covariance[28] = 0.001
        odom.pose.covariance[35] = 0.01
        odom.twist.covariance = odom.pose.covariance
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

        self.last_time = current_time

    def imu_callback(self, msg):
        q = msg.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        self.theta = yaw

if __name__ == '__main__':
    rclpy.init()
    node = OdometryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
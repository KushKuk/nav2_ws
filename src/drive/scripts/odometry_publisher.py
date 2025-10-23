#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from math import sin, cos
import numpy as np
if not hasattr(np, 'float'):
    np.float = float
from tf_transformations import quaternion_from_euler


class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # Parameters
        self.declare_parameter('wheel_radius', 0.1125)
        self.declare_parameter('wheel_separation', 0.54)
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Publishers / Subscribers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("✅ OdometryPublisher initialized and running.")

    def joint_state_callback(self, msg: JointState):
        if not msg.velocity:
            self.get_logger().warn("Received JointState with no velocities.")
            return

        # Compute left/right wheel velocities
        left_vels, right_vels = [], []
        for i, name in enumerate(msg.name):
            if name in ['joint_wheel_1', 'joint_wheel_2', 'joint_wheel_3']:
                left_vels.append(msg.velocity[i] * self.wheel_radius)
            elif name in ['joint_wheel_4', 'joint_wheel_5', 'joint_wheel_6']:
                right_vels.append(msg.velocity[i] * self.wheel_radius)

        if not left_vels or not right_vels:
            self.get_logger().warn_throttle(5.0, "Wheel joint names not matching expected lists.")
            return

        left_vel = np.mean(left_vels)
        right_vel = np.mean(right_vels)

        # Integrate position
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        v = (left_vel + right_vel) / 2.0
        omega = (right_vel - left_vel) / self.wheel_separation

        self.x += v * cos(self.theta) * dt
        self.y += v * sin(self.theta) * dt
        self.theta += omega * dt

        # Build odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = quaternion_from_euler(0.0, 0.0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        # Covariances
        cov = [0.001, 0, 0, 0, 0, 0,
               0, 0.001, 0, 0, 0, 0,
               0, 0, 99999, 0, 0, 0,
               0, 0, 0, 99999, 0, 0,
               0, 0, 0, 0, 99999, 0,
               0, 0, 0, 0, 0, 0.01]
        odom.pose.covariance = cov
        odom.twist.covariance = cov

        # Publish odom
        self.odom_pub.publish(odom)

        # Publish TF
        tf = TransformStamped()
        tf.header.stamp = current_time.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(tf)

        self.last_time = current_time

    def imu_callback(self, msg: Imu):
        # Update heading from IMU yaw (optional)
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        self.theta = yaw


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

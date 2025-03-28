#!/usr/bin/env python3
import math
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.r = 0.045  # Wheel radius
        self.l = 0.3076  # Distance between wheels
        
        # Pose and velocity
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        # Previous wheel positions
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0
        self.first_message = True
        self.last_time = self.get_clock().now()

    def joint_state_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        left_pos = msg.position[0]  # Assuming left wheel is first
        right_pos = msg.position[1]  # Assuming right wheel is second

        if self.first_message:
            self.prev_left_pos = left_pos
            self.prev_right_pos = right_pos
            self.first_message = False
            return
        
        # Calculate wheel movements
        delta_left = left_pos - self.prev_left_pos
        delta_right = right_pos - self.prev_right_pos
        
        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos
        
        # Calculate linear and angular velocity
        linear = (delta_right + delta_left) * self.r / 2.0
        angular = (delta_right - delta_left) * self.r / self.l
        
        # Update velocities
        if dt > 0:
            self.vx = linear / dt
            self.vth = angular / dt
        
        # Update pose
        self.x += linear * math.cos(self.theta)
        self.y += linear * math.sin(self.theta)
        self.theta += angular
        
        # Normalize angle
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        self.publish_odometry(current_time.to_msg())
        self.publish_odometry_transform(current_time.to_msg())

    def publish_odometry_transform(self, time):
        t = TransformStamped()
        
        t.header.stamp = time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
        
    def publish_odometry(self, time):
        odom = Odometry()
        odom.header.stamp = time
        odom.header.frame_id = 'odom'
        
        # Set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Set the velocity
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        
        self.odom_pub.publish(odom)
        
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
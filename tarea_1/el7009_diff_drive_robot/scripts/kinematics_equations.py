#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import rosidl_parser
import threading
from std_msgs import msg
from std_msgs.msg import   Float64
import numpy as np

l = 0.35
r = 0.05

class Kinematics(Node):

    def __init__(self):
        super().__init__('Kinematics')
        self.publisher_left_ = self.create_publisher(Float64, 'left_wheel_cmd_vel', 10)
        self.publisher_right_ = self.create_publisher(Float64, 'right_wheel_cmd_vel', 10)
        self._loop_rate = self.create_rate(10.0, self.get_clock())

def rotate(theta: float, dt: int = 10):
    rclpy.init(args=None)
    dt = 5
    v = (theta*l)/(2*dt)
    w = v/r

    # Spin in a separate thread so that the clock is updated
    minimal_publisher = Kinematics()
    thread = threading.Thread(target=rclpy.spin, args=(minimal_publisher,))
    thread.start()

    # Publish speeds for dt seconds
    minimal_publisher.get_logger().info("Publishing speeds")
    for i in range(dt*10):
        msg = Float64()
        msg.data = -w
        minimal_publisher.publisher_left_.publish(msg)
        msg.data = w
        minimal_publisher.publisher_right_.publish(msg)
        minimal_publisher.get_logger().info("publishing speeds")
        minimal_publisher._loop_rate.sleep()

    # Stop the robot
    minimal_publisher.get_logger().info("Stopping the robot")
    msg = Float64()
    msg.data = 0.0
    minimal_publisher.publisher_left_.publish(msg)
    minimal_publisher.publisher_right_.publish(msg)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

def forward(x: float, dt: int = 15):
    rclpy.init(args=None)
    v = x/dt

    w = v/r

    # Spin in a separate thread so that the clock is updated
    minimal_publisher = Kinematics()
    thread = threading.Thread(target=rclpy.spin, args=(minimal_publisher,))
    thread.start()

    # Publish speeds for dt seconds
    minimal_publisher.get_logger().info("Publishing speeds")
    for i in range(dt*10):
        msg = Float64()
        msg.data = w
        minimal_publisher.publisher_left_.publish(msg)
        msg.data = w
        minimal_publisher.publisher_right_.publish(msg)
        minimal_publisher.get_logger().info("publishing speeds")
        minimal_publisher._loop_rate.sleep()

    # Stop the robot
    minimal_publisher.get_logger().info("Stopping the robot")
    msg = Float64()
    msg.data = 0.0
    minimal_publisher.publisher_left_.publish(msg)
    minimal_publisher.publisher_right_.publish(msg)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

def move(x: float, y: float, theta: float, dtx: int= 15, dtt: int = 10):
    if x == 0 and y == 0:
        target_angle = 0
    else:
        target_angle = np.arctan2(y, x)
    
    rotate(target_angle, dtt)
    
    distance = np.sqrt(x**2 + y**2)
    
    forward(distance, dtx)
    
    final_rotation = theta - target_angle
    rotate(final_rotation, dtt)

if __name__ == '__main__':
    # rotate(3/2, 10)
    # forward(3.5, 20)
    # rotate(3/2, 10)
    # forward(3.0, 20)
    # rotate(3/2, 10)
    # forward(3.5, 20)
    # rotate(3/2, 10)
    # forward(3.5, 20)
    move(3, -2, 3.14)

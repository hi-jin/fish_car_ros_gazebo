#!/usr/bin/env python3

import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
import termios
import tty

def euler_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class CarController(Node):
    def __init__(self):
        super().__init__('car_controller')
        self.current_x = 0.0
        self.current_y = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def odom_callback(self, odom_msg):
        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y
        orientation = odom_msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation)
        self.get_logger().info('Current Position: x={:.2f}, y={:.2f}, roll={:.2f}, pitch={:.2f}, yaw={:.2f}'.format(self.current_x, self.current_y, self.roll, self.pitch, self.yaw))

    def stop_car(self):
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

    def handle_key(self, key):
        cmd_vel = Twist()
        if key == 'w':
            cmd_vel.linear.x = 0.5
        elif key == 's':
            cmd_vel.linear.x = -0.5
        elif key == 'a':
            cmd_vel.angular.z = 0.5
        elif key == 'd':
            cmd_vel.angular.z = -0.5
        elif key == 'q':
            rclpy.shutdown()
            return
        self.cmd_vel_pub.publish(cmd_vel)

def get_key():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
    return key

def main(args=None):
    rclpy.init(args=args)
    car_controller = CarController()

    try:
        while rclpy.ok():
            rclpy.spin_once(car_controller, timeout_sec=0.1)
            key = get_key()
            car_controller.handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        car_controller.stop_car()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
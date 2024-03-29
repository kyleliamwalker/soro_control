#!/usr/bin/env python

from imu_ros2_driver.utils import quaternion_to_euler
from rclpy.node import Node
from dynamixel_sdk import *
from sensor_msgs.msg import Joy
from custom_interfaces.msg import ArcParam2
import rclpy
import numpy as np
from soro_control.util_funcs import *
import math

class JoyController( Node ):

    def __init__(self):

        super().__init__('joy_controller')

        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.angles_pub = self.create_publisher(ArcParam2, '/set_arc_parameters', 10)

    def joy_callback(self, data):

        seg1_joy_x = -data.axes[3]
        seg1_joy_y = data.axes[4]
        seg2_joy_x = -data.axes[0]
        seg2_joy_y = data.axes[1]

        # self.get_logger().info("X = %s" % joy_x)
        # self.get_logger().info("Y = %s" % joy_y)

        if seg1_joy_x == 0:
            seg1_joy_x = 0.01
        if seg1_joy_y == 0:
            seg1_joy_y == 0.01
        if seg2_joy_x == 0:
            seg2_joy_x = 0.01
        if seg2_joy_y == 0:
            seg2_joy_y == 0.01

        theta1 = math.sqrt(seg1_joy_x**2 + seg1_joy_y**2) * 90.0
        phi_rad1 = math.atan2(seg1_joy_y, seg1_joy_x)
        phi_deg1 = math.degrees(phi_rad1)

        theta2 = math.sqrt(seg2_joy_x**2 + seg2_joy_y**2) * 90.0
        phi_rad2 = math.atan2(seg2_joy_y, seg2_joy_x)
        phi_deg2 = math.degrees(phi_rad2)

        # self.get_logger().info("Phi = %s" % phi_deg)
        # self.get_logger().info("Theta = %s" % theta)
        
        msg = ArcParam2()

        if theta1 > -90 and theta1 < 90:
            msg.theta = theta1
        else:
            msg.theta = np.sign(theta1)*90
        msg.phi = phi_deg1

        if theta2 > -90 and theta2 < 90:
            msg.theta2 = theta2
        else:
            msg.theta2 = np.sign(theta2)*90
        msg.phi2 = phi_deg2
    
        self.angles_pub.publish(msg)

def main():
    rclpy.init()
    joy_controller = JoyController()
    try:
        rclpy.spin(joy_controller)
    except KeyboardInterrupt:
        print("Shutting down imu controller...")
    finally:
        joy_controller.destroy_node()

if __name__ == '__main__':
    main()
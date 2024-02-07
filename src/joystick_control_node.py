#!/usr/bin/env python

from imu_ros2_driver.utils import quaternion_to_euler
from rclpy.node import Node
from dynamixel_sdk import *
from sensor_msgs.msg import Joy
from custom_interfaces.msg import ArcParam
import rclpy
import numpy as np
from soro_control.util_funcs import *
import math

class JoyController( Node ):

    def __init__(self):

        super().__init__('imu_controller')

        self.imu_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.angles_pub = self.create_publisher(ArcParam, '/set_arc_parameters', 10)

    def joy_callback(self, data):

        # INSERT ANGLE CALCULATIONS FROM JOYSTICK READINGS HERE

        joy_x = data.axes[3]
        joy_y = data.axes[4]

        self.get_logger().info("X = %s" % joy_x)
        self.get_logger().info("Y = %s" % joy_y)

        if joy_x == 0:
            joy_x = 0.01
        
        if joy_y == 0:
            joy_y == 0.01

        theta = math.sqrt(joy_x**2 + joy_y**2) * 90.0
        phi_rad = math.atan2(joy_y, joy_x)
        phi_deg = math.degrees(phi_rad)


        self.get_logger().info("Phi = %s" % phi_deg)
        self.get_logger().info("Theta = %s" % theta)
        
        msg = ArcParam()
        if theta > -60 and theta < 60:
            
            msg.theta = theta
        else:
            msg.theta = np.sign(theta)*60

        # if theta < -10 and theta > 10:
        #     msg.theta = theta
        # else:
        #     msg.theta = np.sign(theta)*0

        msg.phi = phi_deg
        
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
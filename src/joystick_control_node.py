#!/usr/bin/env python

from imu_ros2_driver.utils import quaternion_to_euler
from rclpy.node import Node
from dynamixel_sdk import *
from sensor_msgs.msg import Joy
from custom_interfaces.msg import ArcParam
import rclpy
import numpy as np
from soro_control.util_funcs import *

class JoyController( Node ):

    def __init__(self):

        super().__init__('imu_controller')

        self.imu_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.angles_pub = self.create_publisher(ArcParam, '/set_arc_parameters', 10)

    def joy_callback(self, data):

        # INSERT ANGLE CALCULATIONS FROM JOYSTICK READINGS HERE

        self.euler = quaternion_to_euler(data.orientation)
        state = generate_phi_theta(self)
        phi = np.rad2deg(state[0].item())
        theta = np.rad2deg(state[1].item())

        # self.get_logger().info("Phi = %s" % phi)
        # self.get_logger().info("Theta = %s" % theta)
        
        msg = ArcParam()
        if theta > -60 and theta < 60:
            
            msg.theta = theta
        else:
            msg.theta = np.sign(theta)*60

        # if theta < -10 and theta > 10:
        #     msg.theta = theta
        # else:
        #     msg.theta = np.sign(theta)*0

        msg.phi = phi
        
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
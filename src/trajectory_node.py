#!/usr/bin/env python

from rclpy.node import Node
from dynamixel_sdk import *
from custom_interfaces.msg import ArcParam
import rclpy
import numpy as np
from soro_control.util_funcs import *

class JoyController( Node ):

    def __init__(self):

        super().__init__('imu_controller')

        # two arrays below must be same length.
        # please do not exceed 90.0
        self.theta_traj = np.array([ 90.0, 0.0, 90.0, 0.0, 90.0, 0.0, 90.0, 0.0])
        # must be between 0 and 360
        self.phi_traj = np.array([ 0.0, 0.0, 90.0, 90.0, 180.0, 180.0, 270.0, 270.0])

        self.counter = 0

        timer_period = 10.0 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angles_pub = self.create_publisher(ArcParam, '/set_arc_parameters', 10)

    def timer_callback(self):

        self.get_logger().info("Phi = %s" % self.phi_traj[self.counter])
        self.get_logger().info("Theta = %s" % self.theta_traj[self.counter])
        
        msg = ArcParam()
        
        msg.theta = self.theta_traj[self.counter]
        msg.phi = self.phi_traj[self.counter]
        
        self.angles_pub.publish(msg)

        if self.counter < len(self.theta_traj)-1:
            self.counter += 1
        else:
            self.counter = 0
            

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
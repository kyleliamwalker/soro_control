#!/usr/bin/env python

from soro_control.dyn_setup import DynamixelConfig
from soro_control.control_funcs import ControllerSpec
import rclpy


def main(motor):
    rclpy.init()
    controller = ControllerSpec(motor)
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()     

if __name__ == '__main__':

    dyn = DynamixelConfig()
    main(dyn)

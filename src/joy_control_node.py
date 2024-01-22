#!/usr/bin/env python

from soro_control.dyn_setup import DynamixelConfig
from soro_control.joy_control_class import JoyControl
import rclpy
from dynamixel_sdk import *

# import signal
 
def close_dxl(dyn):
    #disable torque, close port to reset for next run
    print("Disabling torque...")
    for id in range(len(dyn.DXL_ID)):
        dxl_comm_result, dxl_error = dyn.packetHandler.write1ByteTxRx(dyn.portHandler, dyn.DXL_ID[id], dyn.ADDR_TORQUE_ENABLE, dyn.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
                print("%s" % dyn.packetHandler.getTxRxResult(dxl_comm_result))
                
    try:
        dyn.portHandler.closePort()
        print("Succeeded to close the port")
    except:
        print("Failed to close the port")
 
def main(motor):
    rclpy.init()
    joy_controller = JoyControl(motor)
    try:
        rclpy.spin(joy_controller)
    except KeyboardInterrupt:
        print("Shutting down manual controller...")
    finally:
        joy_controller.destroy_node()
        close_dxl(motor)
        rclpy.shutdown()

if __name__ == '__main__':

    dyn = DynamixelConfig()
    # signal.signal(signal.SIGINT, handler)
    # ros2 node
    main(dyn)


    
#!/usr/bin/env python

# import math
import numpy as np
from rclpy.node import Node
from dynamixel_sdk import *
from sensor_msgs.msg import Joy

# basic functionality, steps motors planarly
class JoyControl( Node ):

    def __init__(self, dxl):
        super().__init__('joy_controller')

        # step set as 150 for now as initial test
        self.dxl = dxl
        self.motor_step = 150
        self.o_increment = np.array([self.motor_step, 0, -self.motor_step, 0])
        self.s_increment = np.array([-self.motor_step, 0, self.motor_step, 0])
        self.t_increment = np.array([0, self.motor_step, 0, -self.motor_step])
        self.x_increment = np.array([0, -self.motor_step, 0, self.motor_step])

        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)


    # manually defined incremental stepping of each button press
    def joy_callback(self, joy_data):

        if joy_data.buttons[0] == 1:
            for id in range(len(self.dxl.DXL_ID)):
                dxl_present_position, dxl_comm_result, dxl_error = self.dxl.packetHandler.read4ByteTxRx(self.dxl.portHandler, self.dxl.DXL_ID[id], self.dxl.ADDR_PRESENT_POSITION)
                dxl_goal_position = dxl_present_position + self.x_increment[id]
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
                dxl_addparam_result = self.dxl.groupSyncWrite.addParam(self.dxl.DXL_ID[id], param_goal_position)
                
                if dxl_addparam_result != 1:
                    self.get_logger().info("[ID:%03d] groupSyncWrite addparam failed" % (self.dxl.DXL_ID[id]))
                    quit()

        elif joy_data.buttons[1] == 1:
            for id in range(len(self.dxl.DXL_ID)):
                dxl_present_position, dxl_comm_result, dxl_error = self.dxl.packetHandler.read4ByteTxRx(self.dxl.portHandler, self.dxl.DXL_ID[id], self.dxl.ADDR_PRESENT_POSITION)
                dxl_goal_position = dxl_present_position + self.o_increment[id]
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
                dxl_addparam_result = self.dxl.groupSyncWrite.addParam(self.dxl.DXL_ID[id], param_goal_position)
                
                if dxl_addparam_result != 1:
                    self.get_logger().info("[ID:%03d] groupSyncWrite addparam failed" % (self.dxl.DXL_ID[id]))
                    quit()

        elif joy_data.buttons[2] == 1:
            for id in range(len(self.dxl.DXL_ID)):
                dxl_present_position, dxl_comm_result, dxl_error = self.dxl.packetHandler.read4ByteTxRx(self.dxl.portHandler, self.dxl.DXL_ID[id], self.dxl.ADDR_PRESENT_POSITION)
                dxl_goal_position = dxl_present_position + self.t_increment[id]
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
                dxl_addparam_result = self.dxl.groupSyncWrite.addParam(self.dxl.DXL_ID[id], param_goal_position)
                
                if dxl_addparam_result != 1:
                    self.get_logger().info("[ID:%03d] groupSyncWrite addparam failed" % (self.dxl.DXL_ID[id]))
                    quit()

        elif joy_data.buttons[3] == 1:
            for id in range(len(self.dxl.DXL_ID)):
                dxl_present_position, dxl_comm_result, dxl_error = self.dxl.packetHandler.read4ByteTxRx(self.dxl.portHandler, self.dxl.DXL_ID[id], self.dxl.ADDR_PRESENT_POSITION)
                dxl_goal_position = dxl_present_position + self.s_increment[id]
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
                dxl_addparam_result = self.dxl.groupSyncWrite.addParam(self.dxl.DXL_ID[id], param_goal_position)
                
                if dxl_addparam_result != 1:
                    self.get_logger().info("[ID:%03d] groupSyncWrite addparam failed" % (self.dxl.DXL_ID[id]))
                    quit()

        else:
            self.get_logger().info("Please press a button to move motors...")

        dxl_comm_result = self.dxl.groupSyncWrite.txPacket()
        
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().info("%s" % self.dxl.packetHandler.getTxRxResult(dxl_comm_result))#

        self.dxl.groupSyncWrite.clearParam()

            
        

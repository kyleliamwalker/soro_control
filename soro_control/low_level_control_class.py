#!/usr/bin/env python

# import math
import numpy as np
from rclpy.node import Node
from dynamixel_sdk import *
# from sensor_msgs.msg import Joy, Imu
from custom_interfaces.msg import Control, Encoder#, EulerAngles, Status

class LowLevelControl( Node ):

    def __init__(self, dxl):
        super().__init__('ll_controller')

        self.dxl = dxl
        
        self.pos = np.array([0,0,0,0])
        self.get_enc_pos()
        # self.pos_init = self.pos
        #self.step_change = np.array([0,0,0,0])

        self.encoder_publisher = self.create_publisher(Encoder, '/encoder_pos', 10)
        # subscriber to control message
        self.control_subscriber = self.create_subscription(Control, '/control_input', self.control_callback, 10)

        # init timer to publish encoder position
        timer_period = 0.25 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # publish encoder position, 100Hz
    def timer_callback(self):
        
        encoder_msg = Encoder()

        self.get_enc_pos()

        self.step_change = self.pos - self.dxl.dxl_home_position

        # self.get_logger().info("Previous Position %s" % self.pos)

        for id in range(len(self.dxl.DXL_ID)):
        #     self.step_change[id] = self.pos[id] - self.pos_prev[id]
            encoder_msg.position.append(self.pos[id])
            encoder_msg.step_change.append(self.step_change[id])

        self.encoder_publisher.publish(encoder_msg)

        # synthetic data, testing
        # dxl_present_position = 500
        # # create and publish encoder position msg
        # msg.data = dxl_present_position
        # self.encoder_publisher.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % encoder_msg.position)

    def get_enc_pos(self):

        for id in range(len(self.dxl.DXL_ID)):
            dxl_present_position, dxl_comm_result, dxl_error = self.dxl.packetHandler.read4ByteTxRx(self.dxl.portHandler, self.dxl.DXL_ID[id], self.dxl.ADDR_PRESENT_POSITION)
            self.pos[id] = dxl_present_position

    # control input is goal encoder position, based on tendon length changes
    def control_callback(self, input_msg):

        input = [input_msg.l1, 
                 input_msg.l2, 
                 input_msg.l3,
                 input_msg.l4]
        
        for id in range(len(self.dxl.DXL_ID)):
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(input[id])), DXL_HIBYTE(DXL_LOWORD(input[id])), DXL_LOBYTE(DXL_HIWORD(input[id])), DXL_HIBYTE(DXL_HIWORD(input[id]))]
            dxl_addparam_result = self.dxl.groupSyncWrite.addParam(self.dxl.DXL_ID[id], param_goal_position)
    
            if dxl_addparam_result != 1:
                self.get_logger().info("[ID:%03d] groupSyncWrite addparam failed" % (self.dxl.DXL_ID[id]))
                quit()

        dxl_comm_result = self.dxl.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.dxl.packetHandler.getTxRxResult(dxl_comm_result))#

        self.dxl.groupSyncWrite.clearParam()

    
            
        
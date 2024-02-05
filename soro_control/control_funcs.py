#!/usr/bin/env python

import math
from rclpy.node import Node
from dynamixel_sdk import *
from sensor_msgs.msg import Joy
from custom_interfaces.msg import ArcParam


class ControllerSpec( Node ):

    def __init__(self, dxl):

        super().__init__('controller')

        self.dxl = dxl
        self.L = 0.1
        self.alpha = [ 0, math.pi/2 ]
        self.r_d = 0.01
        self.L_init = [ self.L, self.L, self.L, self.L ]
        self.L_change = [ 0, 0, 0, 0 ]
        self.step_change = [ 0, 0, 0, 0 ]
        self.total_steps = 4096
        self.metres_per_step = 2*math.pi*self.r_d/self.total_steps

        self.angle_subscriber = self.create_subscription(ArcParam, '/move_tendons', self.set_ten_len_callback, 10)
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_input, 10) 

        self.test_counter = 1
        self.test_publisher = self.create_publisher(ArcParam, "/move_tendons", 10)
        self.test_timer = self.create_timer(3.0, self.timer_callback)

    def timer_callback(self):
    
        msg = ArcParam()
        if self.test_counter == 1:
            msg.theta = 60.0
            self.test_counter += 1
        elif self.test_counter == 2:
            msg.theta = 0.0
            self.test_counter += 1
        elif self.test_counter == 3:
            msg.theta = -60.0
            self.test_counter += 1
        else:
            msg.theta = 0.0
            self.test_counter = 1
        
        msg.phi = 0.0
        self.test_publisher.publish(msg)
        

    def set_ten_len_callback(self, data):

        l1, l2, l3, l4 = self.get_tendon_lengths(data)
        L_new = [ l1, l2, l3, l4 ]
        print("Set Goal Positions of Tendons to l1 = %s, l2 = %s, l3 = %s and l4 = %s" % (round(l1,4), round(l2,4), round(l3,4), round(l4,4)) )
        
        for i in range(len(L_new)):
            self.L_change[i] = L_new[i] - self.L_init[i]
            self.step_change[i] = self.L_change[i]/self.metres_per_step

        print("Length changes are L1 = %s, L2 = %s, L3 = %s and L4 = %s" % (self.L_change[0], self.L_change[1], self.L_change[2], self.L_change[3]) )
        print("Step changes are L1 = %s, L2 = %s, L3 = %s and L4 = %s" % (self.step_change[0], self.step_change[1], self.step_change[2], self.step_change[3]) )
            
        # self.get_logger().info('Length Change: %s' % self.L_change[0])
        # self.get_logger().info('Step Change: %s' % self.step_change[0])

        for id in range(len(self.dxl.DXL_ID)):
            dxl_present_position, dxl_comm_result, dxl_error = self.dxl.packetHandler.read4ByteTxRx(self.dxl.portHandler, self.dxl.DXL_ID[id], self.dxl.ADDR_PRESENT_POSITION)
            dxl_goal_position = dxl_present_position + round(self.step_change[id])

            print("Goal Position = %s" % dxl_goal_position)
            # Allocate goal position value into byte array
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))]
            dxl_addparam_result = self.dxl.groupSyncWrite.addParam(self.dxl.DXL_ID[id], param_goal_position)
    
            if dxl_addparam_result != 1:
                print("[ID:%03d] groupSyncWrite addparam failed" % (self.dxl.DXL_ID[id]))
                quit()

        dxl_comm_result = self.dxl.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.dxl.packetHandler.getTxRxResult(dxl_comm_result))#

        self.dxl.groupSyncWrite.clearParam()

        self.L_init = L_new

    def get_tendon_lengths(self, input): 
    
        theta = math.radians(input.theta)
        phi = math.radians(input.phi)
        # kappa = theta/L

        l1 = self.L - theta*self.r_d*math.cos(self.alpha[0]-phi)
        l2 = self.L - theta*self.r_d*math.cos(self.alpha[1]-phi)
        l3 = 2*self.L-l1
        l4 = 2*self.L-l2
        # print("Tendon lengths required are l1 = %s, l2 = %s, l3 = %s and l4 = %s" % (round(l1,4), round(l2,4), round(l3,4), round(l4,4)) )
        return l1, l2, l3, l4
    
    # joystick command to halt movement when X is pressed
    def joy_input(self, joy_data):
        # global dxl_home_position
        cross = joy_data.buttons[0]
        circle = joy_data.buttons[1]

        if cross == 1:
            # Allocate goal position value into byte array
            for id in range(len(self.dxl.DXL_ID)):
                dxl_present_position, dxl_comm_result, dxl_error = self.dxl.packetHandler.read4ByteTxRx(self.dxl.portHandler, self.dxl.DXL_ID[id], self.dxl.ADDR_PRESENT_POSITION)
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_present_position)), DXL_HIBYTE(DXL_LOWORD(dxl_present_position)), DXL_LOBYTE(DXL_HIWORD(dxl_present_position)), DXL_HIBYTE(DXL_HIWORD(dxl_present_position))]
                dxl_addparam_result = self.dxl.groupSyncWrite.addParam(self.dxl.DXL_ID[id], param_goal_position)
                
                if dxl_addparam_result != 1:
                    print("[ID:%03d] groupSyncWrite addparam failed" % (self.dxl.DXL_ID[id]))
                    quit()

        if circle == 1:
            for id in range(len(self.dxl.DXL_ID)):
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(self.dxl.dxl_home_position[id])), DXL_HIBYTE(DXL_LOWORD(self.dxl.dxl_home_position[id])), DXL_LOBYTE(DXL_HIWORD(self.dxl.dxl_home_position[id])), DXL_HIBYTE(DXL_HIWORD(self.dxl.dxl_home_position[id]))]
                dxl_addparam_result = self.dxl.groupSyncWrite.addParam(self.dxl.DXL_ID[id], param_goal_position)
                
                if dxl_addparam_result != 1:
                    print("[ID:%03d] groupSyncWrite addparam failed" % (self.dxl.DXL_ID[id]))
                    quit()

        
        dxl_comm_result = self.dxl.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.dxl.packetHandler.getTxRxResult(dxl_comm_result))#

        self.dxl.groupSyncWrite.clearParam()


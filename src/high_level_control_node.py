#!/usr/bin/env python

import math
from rclpy.node import Node
from dynamixel_sdk import *
from custom_interfaces.msg import ArcParam, Control, Encoder
import rclpy
import numpy as np

class HighLevelControl( Node ):

    def __init__(self):

        super().__init__('hl_controller')

        # self.L_init = 0.1
        self.L_init = 0.2   #SHELL
        self.alpha = [ 0, math.pi/2 ]
        self.r_d = 0.0125
        self.L = np.array([ self.L_init, self.L_init, self.L_init, self.L_init ])
        self.L_change = [ 0, 0, 0, 0 ]
        self.steps_to_goal = [ 0, 0, 0, 0 ]
        self.total_steps = 4096
        self.metres_per_step = 2*math.pi*self.r_d/self.total_steps
        self.pos = [0, 0, 0, 0]

        # subscribe to desired angle
        self.angle_subscriber = self.create_subscription(ArcParam, '/set_arc_parameters', self.set_ten_len_callback, 10)
        # assign encoder position
        self.encoder_subscriber = self.create_subscription(Encoder, '/encoder_pos', self.encoder_callback, 10)
        # publish control message
        self.control_publisher = self.create_publisher(Control, '/control_input', 10)

    # could be used in future for direct control
    def encoder_callback(self, data):
        self.pos = data.position
        self.step_change = np.array(data.step_change)
        self.L = self.L_init + self.step_change * self.metres_per_step
        self.get_logger().info("Length %s" % self.L)

    def set_ten_len_callback(self, data):

        l1, l2, l3, l4 = self.get_tendon_lengths(data)
        # self.get_logger().info("%s" % data)
        L_goal = [ l1, l2, l3, l4 ]

        for i in range(len(L_goal)):
            self.L_change[i] = L_goal[i] - self.L[i]
            if np.all(self.L_change[i] == 0):
                self.steps_to_goal[i] = 0
            else:
                self.steps_to_goal[i] = self.L_change[i]/self.metres_per_step

        # self.get_logger().info("Set Goal Positions of Tendons to l1 = %s, l2 = %s, l3 = %s and l4 = %s" 
        #                        % (round(l1,4), round(l2,4), round(l3,4), round(l4,4)) )
        
        # # print("Length changes are L1 = %s, L2 = %s, L3 = %s and L4 = %s" % (L_change[0], L_change[1], L_change[2], L_change[3]) )
        # # print("Step changes are L1 = %s, L2 = %s, L3 = %s and L4 = %s" % (step_change[0], step_change[1], step_change[2], step_change[3]) )
            
        # # self.get_logger().info('Length Change: %s' % self.L_change[0])
        # # self.get_logger().info('Step Change: %s' % self.step_change[0])

        # need to create message here for length change or some other control input
        control_msg = Control()
        control_msg.l1 = self.pos[0] + round(self.steps_to_goal[0])
        control_msg.l2 = self.pos[1] + round(self.steps_to_goal[1])
        control_msg.l3 = self.pos[2] + round(self.steps_to_goal[2])
        control_msg.l4 = self.pos[3] + round(self.steps_to_goal[3])
        #control_msg.l4 = round(self.step_change[3])
        self.control_publisher.publish(control_msg)
        
        # # need to link to tendon lengths to avoid too quick updating
        # time.sleep(2)
        # self.L_init = L_new

    def get_tendon_lengths(self, input): 
    
        theta = math.radians(input.theta)
        phi = math.radians(input.phi)
        # kappa = theta/L

        l1 = self.L_init - theta*self.r_d*math.cos(self.alpha[0]-phi)
        l2 = self.L_init - theta*self.r_d*math.cos(self.alpha[1]-phi)
        l3 = 2*self.L_init-l1
        l4 = 2*self.L_init-l2
        # print("Tendon lengths required are l1 = %s, l2 = %s, l3 = %s and l4 = %s" % (round(l1,4), round(l2,4), round(l3,4), round(l4,4)) )
        return l1, l2, l3, l4
    
def main():
    rclpy.init()
    hl_controller = HighLevelControl()
    try:
        rclpy.spin(hl_controller)
    except KeyboardInterrupt:
        print("Shutting down high level controller...")
    finally:
        hl_controller.destroy_node()

if __name__ == '__main__':
    main()
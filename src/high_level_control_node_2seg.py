#!/usr/bin/env python

import math
from rclpy.node import Node
from dynamixel_sdk import *
from custom_interfaces.msg import ArcParam2, Control2, Encoder
import rclpy
import numpy as np

class HighLevelControl( Node ):

    def __init__(self):

        super().__init__('hl_controller')

        #self.L_init = 0.1
        self.L_init = 0.35   # SHELL
        self.alpha = [ math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4 ]
        #self.r_d = 0.0125
        self.r_d = 0.03
        self.L = np.array([ self.L_init, self.L_init, self.L_init, self.L_init, self.L_init, self.L_init, self.L_init, self.L_init ])
        self.L_change = [ 0, 0, 0, 0, 0, 0, 0, 0 ]
        self.steps_to_goal = [ 0, 0, 0, 0, 0, 0, 0, 0 ]
        self.total_steps = 4096
        self.metres_per_step = 2*math.pi*self.r_d/self.total_steps
        self.pos = [0, 0, 0, 0, 0, 0, 0, 0]

        # subscribe to desired angle
        self.seg1_angle_subscriber = self.create_subscription(ArcParam2, '/set_arc_parameters', self.set_ten_len_callback, 10)
        # assign encoder position
        self.encoder_subscriber = self.create_subscription(Encoder, '/encoder_pos', self.encoder_callback, 10)
        # publish control message
        self.control_publisher = self.create_publisher(Control2, '/control_input', 10)

    # could be used in future for direct control
    def encoder_callback(self, data):
        self.pos = data.position
        self.step_change = np.array(data.step_change)
        self.L = self.L_init + self.step_change * self.metres_per_step
        # self.get_logger().info("Length %s" % self.L)

    def set_ten_len_callback(self, data):

        # segment 1
        s1_l1, s1_l2, s1_l3, s1_l4 = self.get_tendon_lengths(np.array([data.theta, data.phi]))
        # segment 2
        s2_l1, s2_l2, s2_l3, s2_l4 = self.get_tendon_lengths(np.array([data.theta2, data.phi2]))
        # self.get_logger().info("%s" % data)
        L_goal = [ s1_l1, s1_l2, s1_l3, s1_l4, s2_l1, s2_l2, s2_l3, s2_l4 ]

        for i in range(len(L_goal)):
            self.L_change[i] = L_goal[i] - self.L[i]
            if np.all(self.L_change[i] == 0):
                self.steps_to_goal[i] = 0
            else:
                self.steps_to_goal[i] = self.L_change[i]/self.metres_per_step

        # need to create message here for length change or some other control input
        control_msg = Control2()
        control_msg.l1 = self.pos[0] + round(self.steps_to_goal[0])
        control_msg.l2 = self.pos[1] + round(self.steps_to_goal[1])
        control_msg.l3 = self.pos[2] + round(self.steps_to_goal[2])
        control_msg.l4 = self.pos[3] + round(self.steps_to_goal[3])
        control_msg.l5 = self.pos[4] + round(self.steps_to_goal[0]+self.steps_to_goal[4])
        control_msg.l6 = self.pos[5] + round(self.steps_to_goal[1]+self.steps_to_goal[5])
        control_msg.l7 = self.pos[6] + round(self.steps_to_goal[2]+self.steps_to_goal[6])
        control_msg.l8 = self.pos[7] + round(self.steps_to_goal[3]+self.steps_to_goal[7])
        #control_msg.l4 = round(self.step_change[3])
        self.control_publisher.publish(control_msg)

    def get_tendon_lengths(self, input): 
    
        theta = math.radians(input[0])
        phi = math.radians(input[1])
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
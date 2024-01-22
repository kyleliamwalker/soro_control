#!/usr/bin/env python

from dynamixel_sdk import *
import sys

# create and configure dynamixel object, return error messages if unsuccessful
class DynamixelConfig():

    def __init__(self):
    
        # Control table address
        self.ADDR_TORQUE_ENABLE             = 64               # Control table address is different in Dynamixel model
        self.ADDR_GOAL_POSITION             = 116
        self.ADDR_PRESENT_POSITION          = 132
        self.ADDR_OPERATING_MODE            = 11
        self.ADDR_PRO_VELOCITY              = 112
        self.ADDR_PRO_ACCEL                 = 108
        self.ADDR_PRO_GOAL_VELOCITY         = 104
        # Protocol version
        self.PROTOCOL_VERSION               = 2.0               # See which protocol version is used in the Dynamixel
        # Default setting
        #self.DXL_ID                        = [ 9 ]
        self.DXL_ID                         = [ 2, 5, 8, 11 ]
        self.BAUDRATE                       = 57600             # Dynamixel default baudrate : 57600
        self.DEVICENAME                     = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
        self.TORQUE_ENABLE                  = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE                 = 0                 # Value for disabling the torque
        self.DXL_MOVING_STATUS_THRESHOLD    = 20                # Dynamixel moving status threshold
        self.EXT_POSITION_CONTROL_MODE      = 4
        self.VEL_CONTROL_MODE               = 1
        # Data Byte Length
        self.LEN_GOAL_POSITION              = 4

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Initialize Groupsyncwrite instance
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION)

        self.setup_dyn()
        # forces printing
        sys.stdout.flush()

    def setup_dyn(self):
        # Open port
        print("Opening the port.")
        try:
            self.portHandler.openPort()
            print("Succeeded to open the port")
        except:
            print("Failed to open the port")
            quit()

        # Set port baudrate
        try:
            self.portHandler.setBaudRate(self.BAUDRATE)
            print("Succeeded to change the baudrate")
        except:
            print("Failed to change the baudrate")
            quit()

        for id in range(len(self.DXL_ID)):
            
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID[id], self.ADDR_OPERATING_MODE, self.EXT_POSITION_CONTROL_MODE)
            # dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID[id], self.ADDR_OPERATING_MODE, self.VEL_CONTROL_MODE)#
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Operating mode of ID: %s changed to extended position control mode." % self.DXL_ID[id])
                
                
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID[id], self.ADDR_PRO_ACCEL, 40)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Acceleration of ID: %s changed." % self.DXL_ID[id])
                
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID[id], self.ADDR_PRO_VELOCITY, 50)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Velocity of ID: %s changed." % self.DXL_ID[id])
            
            # Enable Dynamixel Torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID[id], self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                quit()
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                quit()
            else:
                print("DYNAMIXEL ID: %s has been successfully connected" % self.DXL_ID[id])

        # self.dxl_home_position = [0, 0, 0, 0]
        # for id in range(len(self.DXL_ID)):
        #      self.dxl_home_position[id], dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID[id], self.ADDR_PRESENT_POSITION)
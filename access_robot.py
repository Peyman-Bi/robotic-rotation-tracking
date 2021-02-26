from dynamixel_sdk import *
import os
import numpy as np


if os.name == 'nt':
    import msvcrt

    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

position = 3
velocity = 1
# home_position = np.array([1277,1567,1403])
home_position = np.array([2047, 2047, 2047])
# home_position = np.array([2727, 2394, 2560])

full_rotation = 4095  # quaternion

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11

LEN_GOAL_POSITION = 4
LEN_PRESENT_POSITION = 4
LEN_GOAL_VELOCITY = 4
LEN_PRESENT_VELOCITY = 4

# Protocol version
PROTOCOL_VERSION = 2.0  # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID = 1  # Dynamixel#1 ID: 1
DXL2_ID = 2  # Dynamixel#2 ID: 2
DXL3_ID = 3  # Dynamixel#3 ID: 3
BAUDRATE = 57600  # Dynamixel default baudrate : 57600
DEVICENAME = '/dev/ttyUSB0'

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# --------------Group sync read and write-------------#
GSR_Position = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
GSR_Velocity = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)

GSW_Position = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
GSW_Velocity = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY)
# ---------------------------------------------------#

GSR_Position.addParam(DXL1_ID)
GSR_Position.addParam(DXL2_ID)
GSR_Position.addParam(DXL3_ID)

GSR_Velocity.addParam(DXL1_ID)
GSR_Velocity.addParam(DXL2_ID)
GSR_Velocity.addParam(DXL3_ID)

if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()


class Robot:
    def __init__(self):
        pass

    @staticmethod
    def allocate(inp):
        return [DXL_LOBYTE(DXL_LOWORD(inp)), DXL_HIBYTE(DXL_LOWORD(inp)), DXL_LOBYTE(DXL_HIWORD(inp)), DXL_HIBYTE(DXL_HIWORD(inp))]

    @staticmethod
    def goal_position(inp1,inp2,inp3):
        print(type(inp1), type(inp2), type(inp3))
        GSW_Position.addParam(DXL1_ID, Robot.allocate(inp1))
        GSW_Position.addParam(DXL2_ID, Robot.allocate(inp2))
        GSW_Position.addParam(DXL3_ID, Robot.allocate(inp3))
        GSW_Position.txPacket()
        GSW_Position.clearParam()

    @staticmethod
    def read_position():
        GSR_Position.txRxPacket()
        s1 = GSR_Position.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        s2 = GSR_Position.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        s3 = GSR_Position.getData(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        return s1, s2, s3

    @staticmethod
    def goal_velocity(inp1,inp2,inp3):
        # print('here in goal velocity', inp1,inp2,inp3)
        GSW_Velocity.addParam(DXL1_ID, Robot.allocate(inp1))
        GSW_Velocity.addParam(DXL2_ID, Robot.allocate(inp2))
        GSW_Velocity.addParam(DXL3_ID, Robot.allocate(inp3))
        GSW_Velocity.txPacket()
        GSW_Velocity.clearParam()

    @staticmethod
    def read_velocity():
        GSR_Velocity.txRxPacket()
        s1 = GSR_Velocity.getData(DXL1_ID, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)
        s2 = GSR_Velocity.getData(DXL2_ID, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)
        s3 = GSR_Velocity.getData(DXL3_ID, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)
        return s1 ,s2 ,s3

    @staticmethod
    def torque_enabling(inp):
        packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, inp)
        packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, inp)
        packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, inp)

    @staticmethod
    def change_operating_mode(inp):
        Robot.torque_enabling(TORQUE_DISABLE)
        packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_OPERATING_MODE, inp)
        packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_OPERATING_MODE, inp)
        packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_OPERATING_MODE, inp)
        Robot.torque_enabling(TORQUE_ENABLE)

    @staticmethod
    def homing():
        Robot.change_operating_mode(position)
        Robot.goal_position(home_position[0], home_position[1], home_position[2])
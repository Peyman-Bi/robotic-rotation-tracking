import serial
import numpy as np
from dynamixel_sdk import *
import time
import threading
import os
from access_robot import Robot, velocity
from control_robot import *

# SENSOR_PORT = '/dev/ttyUSB1'
SENSOR_PORT = '/dev/rfcomm8'
ROBOT_PORT = '/dev/ttyUSB0'
RESOLUTION = 4096
PULSE_PER_REV_RADIAN = RESOLUTION/(2*np.pi)

if __name__ == '__main__':
    serialPort = serial.Serial(port=SENSOR_PORT, baudrate=57600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
    stabilize_time = 3
    # home_pulse = 512
    home_pulse = 0
    calibration_trans = sensor_calib_rotation(serialPort, stabilize_time)@Euler2rotation(home_pulse/PULSE_PER_REV_RADIAN, home_pulse/PULSE_PER_REV_RADIAN, home_pulse/PULSE_PER_REV_RADIAN)
    k1, k2, k3 = 0.9, 0.9, 0.9
    # k1, k2, k3 = 0.01, 0.01, 0.01
    control_coefficients = np.array([k1, k2, k3])
    threshold = 50
    # run_event = threading.Event()
    # run_event.set()
    sensor_reader = SensorReader(serialPort, True)
    sensor_input = threading.Thread(target=sensor_reader.sensor_input_thread)
    sensor_input.start()
    # sensor_input.join()
    dt = 0.01
    Robot.homing()
    time.sleep(2)
    Robot.change_operating_mode(velocity)
    # Robot.torque_enabling(False)
    try:
        control(calibration_trans, sensor_reader, k1, k2, k3, threshold, dt)
    except KeyboardInterrupt as e:
        print('here')
        Robot.goal_velocity(0, 0, 0)
        # time.sleep(2)
        Robot.torque_enabling(False)
        print('after')
        sensor_input.join()
        sys.exit(e)

# Robot.torque_enabling(False)

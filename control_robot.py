import numpy as np
import time
from access_robot import Robot, velocity, position

RESOLUTION = 4096
PULSE_PER_REV_RADIAN = RESOLUTION/(2*np.pi)

def quat2rot(quaternion):
    cpm_Q = np.array([
        [0, -quaternion[3], quaternion[2]],
        [quaternion[3], 0, -quaternion[1]],
        [-quaternion[2], quaternion[1], 0]
    ])
    rotation_matrix = quaternion[0]*np.eye(3) + cpm_Q + \
    (np.multiply(quaternion[1:], quaternion[1:].transpose().reshape(-1, 1)))/(1+quaternion[0])
    return rotation_matrix

def sensor_calib_rotation(serial_port, stable_time):
    time.sleep(stable_time)
    if serial_port.in_waiting > 0:
        serialString = serial_port.readline()
        s = [float(x.split('=')[1]) for x in serialString.decode('utf8', 'ignore').strip('\r\n').split(' ')]
    init_rotation = quat2rot(np.array(s[:4]))
    return np.linalg.inv(init_rotation)

def rotation2Euler(rot_mat, order='ZYX'):
    # returns phi(alpha, yaw, z-axis), psi(gama, roll, x-axis), theta(beta, pitch, y-axis)
    if order == 'ZYX':
        phi = np.arctan2(rot_mat[1, 0], rot_mat[0, 0])
        psi = np.arctan2(rot_mat[2, 1], rot_mat[2, 2])
        theta = np.arctan2(-rot_mat[2, 0], np.sqrt(rot_mat[0, 0]**2+rot_mat[1, 0]**2))
    return phi, psi, theta

def Euler2rotation(phi, psi, theta):
    Qx = np.array([
        [1, 0, 0],
        [0, np.cos(psi), -np.sin(psi)],
        [0, np.sin(psi), np.cos(psi)]
    ])
    Qy = np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])
    Qz = np.array([
        [np.cos(phi), -np.sin(phi), 0],
        [np.sin(phi), np.cos(phi), 0],
        [0, 0, 1]
    ])
    return Qz@Qy@Qx

def IKP(phi, psi, theta):

    # compute Euler rotation angles base on ZYX order
    # u1 = np.array([1, 0, 0])
    # u2 = np.array([0, 1, 0])
    # u3 = np.array([0, 0, 1])
    # v1p = -u2
    # v2p = -u3
    # v3p = -u1
    # v1 = Euler2rotation(phi, psi, theta)@v1p
    # v2 = Euler2rotation(phi, psi, theta)@v2p
    # v3 = Euler2rotation(phi, psi, theta)@v3p

    v1 = [
        np.sin(phi)*np.cos(psi)-np.cos(phi)*np.sin(theta)*np.sin(psi),
        -np.cos(phi)*np.cos(psi)-np.sin(phi)*np.sin(theta)*np.sin(psi),
        -np.cos(theta)*np.sin(psi)
    ]
    v2 = [
        -np.sin(phi) * np.sin(psi) - np.cos(phi) * np.sin(theta) * np.cos(psi),
        np.cos(phi) * np.sin(psi) - np.sin(phi) * np.sin(theta) * np.cos(psi),
        -np.cos(theta) * np.cos(psi)
    ]
    v3 = [
        -np.cos(phi)*np.cos(theta),
        -np.sin(phi)*np.cos(theta),
        np.sin(theta)
    ]
    t1 = np.arctan2(v2[1], v2[2])
    t2 = np.arctan2(v3[2], v3[0])
    t3 = np.arctan2(v1[0], v1[1])
    return t1, t2, t3

class SensorReader():
    def __init__(self, serial_port, read_enable=True):
        self.last_value = ''
        self.serial_port = serial_port
        self.read_enable = read_enable

    def sensor_input_thread(self):
        while self.read_enable:
            if self.serial_port.in_waiting > 0:
                # print('whyyyyyyyyyyyyyyyyyyy')
                serialString = self.serial_port.readline()
                # serial output (s): [phi (Yaw, rotation over z), psi (Roll, rotation over x), theta (Pitch, rotation over y)]
                self.last_value = [float(x.split('=')[1]) for x in serialString.decode('utf8', 'ignore').strip('\r\n').split(' ')]
                print(self.last_value)


def control(calibration_trans, sensor_reader, k1=0.15, k2=0.15, k3=0.15, threshold=150, dt=0.1):
    control_coefficients = np.array([k1, k2, k3])
    # time.sleep(2)
    while True:
        time.sleep(dt)
        # print(sensor_reader.last_value)
        rotation_matrix = quat2rot(np.array(sensor_reader.last_value))
        calibrated_rotation = rotation_matrix @ calibration_trans
        phi, psi, theta = rotation2Euler(calibrated_rotation)
        # print('euler angles:', phi, psi, theta)
        desired_radians = IKP(phi, psi, theta)
        # print('desired radians:', desired_radians)
        desired_pulse = (np.array(desired_radians) * PULSE_PER_REV_RADIAN).astype(np.int)
        desired_pulse = desired_pulse % 4096
        # print('desired pulse:', desired_pulse)
        current = Robot.read_position()
        error = desired_pulse - current
        if np.linalg.norm(error) > threshold:
            next_position = current + error * dt
            # print(current)
            print(next_position)
            # print('threshold satisfied')
            # if all([1300 < t < 2800 for t in current]):
            if all([1300 < t < 2800 for t in next_position]):
                print('position satisfied')
                # print('error, threshold:', np.linalg.norm(error), threshold)
                control_signal = (error * control_coefficients).astype(np.int)
                # print('control signal:', control_signal)
                Robot.goal_velocity(*control_signal)
            else:
                print('*******position dissatisfied********')
                # Robot.goal_velocity(0, 0, 0)
                Robot.change_operating_mode(position)
                # time.sleep(2)
                Robot.homing()
                time.sleep(2)
                Robot.change_operating_mode(velocity)

        else:
            Robot.goal_velocity(0, 0, 0)
            # print('threshold passsssssed')
import re

import adafruit_vl53l0x
import dt_vl53l0x
import mpu6050
from dt_vl53l0x import VL53L0X, Vl53l0xAccuracyMode, Vl53l0xDeviceMode
import multiprocessing
import gpiozero
import numpy as np
import collections
import os
import dvg_pid_controller
import simple_pid
import math
import time
from time import sleep
from datetime import datetime as dt
import matplotlib.pyplot as plt
import serial
import matplotlib.animation as animation
from threading import Thread, Lock
from multiprocessing import Process, shared_memory


class DistanceSensor(dt_vl53l0x.VL53L0X):
    def __init__(self, i2c_bus=1, i2c_address=0x29, tca9548a_num=255, tca9548a_addr=0,
                 searchpath=None):
        super().__init__(i2c_bus=i2c_bus, i2c_address=i2c_address, tca9548a_num=tca9548a_num,
                         tca9548a_addr=tca9548a_addr, searchpath=searchpath)
        self.fil_value = 0

    def get_filtered_data(self):
        new_value = self.get_distance()
        if abs(new_value - self.fil_value) > 50:
            k = 0.9
        else:
            k = 0.03
        self.fil_value += (new_value - self.fil_value) * k
        return int(self.fil_value)


distance_limit = 0
def init_distance_sensors(mode=Vl53l0xAccuracyMode.GOOD) -> dict:
    global distance_limit
    if mode == Vl53l0xAccuracyMode.LONG_RANGE:
        distance_limit = 1700
    else:
        distance_limit = 1200
    sensors = {}
    x_shut_pins = {'left': gpiozero.LED(19),
                   'rear': gpiozero.LED(26),
                   'front': gpiozero.LED(16),
                   'right': gpiozero.LED(20)}

    initialised = []
    for x_shut_pin in x_shut_pins.values():
        x_shut_pin.off()
    for current_side, current_x_shut_pin in x_shut_pins.items():
        for side, x_shut_pin in x_shut_pins.items():
            if side in initialised:
                continue
            if side == current_side:
                x_shut_pin.on()
                continue
            x_shut_pin.off()
        sensors[current_side] = (DistanceSensor())
        new_address = 0x30 + len(initialised)
        sensors[current_side].change_address(new_address)
        sensors[current_side].open()
        sensors[current_side].start_ranging(mode=mode)
        initialised.append(current_side)

    return sensors


def format_imu_data(coordinates_dict):
    msg = ''
    for axis in ['x', 'y', 'z']:
        if coordinates_dict[axis] < 0:
            msg += axis.ljust(9) + str(round(coordinates_dict[axis], 2)).ljust(11)
        else:
            msg += axis.ljust(10) + str(round(coordinates_dict[axis], 2)).ljust(10)
    return msg


def format_lidar_data(lidar_dict):
    msg = ''
    for side, distance in lidar_dict.items():
        msg += side.ljust(10) + str(lidar_dict[side]).ljust(10)
    return msg


coordinates = {'x': 0, 'y': 0, 'theta': 0}
def update_coordinates():
    pass

last_received = ''
def receiving(ser):
    global last_received
    buffer_string = ''
    while True:
        received_bytes = ser.read(ser.inWaiting())
        # print(len(received_bytes))
        buffer_string = buffer_string + received_bytes.decode()
        if '\n' in buffer_string:
            lines = buffer_string.split(sep='\r\n')
            if lines[-2]:
                last_received = lines[-2]
            buffer_string = lines[-1]


actual_speed_data = {'left': 0.0, 'right': 0.0}
def update_actual_speed_data():
    global actual_speed_data
    global last_received
    speed_to_rad_coef = 0.010426758 / 2 * 1.123 * 0.98702765
    rad_to_deg_coef = 57.295779513
    while True:
        received_data = last_received.split(' ')
        if len(received_data) == 2:
            actual_speed_data['left'] = int(received_data[0]) * speed_to_rad_coef
            actual_speed_data['right'] = int(received_data[1]) * speed_to_rad_coef


odometry_coordinates = {'x': 0, 'y': 0}
def update_odometry_coordinates():
    global actual_speed_data
    global odometry_coordinates
    R = 0.070
    L = 0.244

    timer_x = time.time()
    timer_y = time.time()
    while True:

        linear_speed = R / 2 * (actual_speed_data['left'] + actual_speed_data['right'])
        x_dot = linear_speed * math.cos(get_gyro_angle())
        odometry_coordinates['x'] += x_dot * (time.time() - timer_x)
        timer_x = time.time()

        linear_speed = R / 2 * (actual_speed_data['left'] + actual_speed_data['right'])
        y_dot = linear_speed * math.sin(get_gyro_angle())
        odometry_coordinates['y'] += y_dot * (time.time() - timer_y)
        timer_y = time.time()


def update_gyro_angle(shm_name):
    mpu = mpu6050.MPU6050()
    mpu.dmpInitialize()
    mpu.setDMPEnabled(True)
    gyro_angle_shm = shared_memory.SharedMemory(shm_name)
    packetSize = mpu.dmpGetFIFOPacketSize()
    while True:
        mpuIntStatus = mpu.getIntStatus()
        if mpuIntStatus >= 2:
            fifoCount = mpu.getFIFOCount()
            if fifoCount == 1024:
                mpu.resetFIFO()
                print('FIFO overflow!')
            fifoCount = mpu.getFIFOCount()
            while fifoCount < packetSize:
                fifoCount = mpu.getFIFOCount()
            result = mpu.getFIFOBytes(packetSize)
            q = mpu.dmpGetQuaternion(result)
            g = mpu.dmpGetGravity(q)
            ypr = mpu.dmpGetYawPitchRoll(q, g)
            gyro_angle_bytes = str(ypr['yaw'] * 180 / math.pi).encode()
            for i in range(len(gyro_angle_bytes)):
                gyro_angle_shm.buf[i] = gyro_angle_bytes[i]
            fifoCount -= packetSize


actual_lidar_data = {'front': 0, 'rear': 0, 'left': 0, 'right': 0}
def update_actual_lidar_data(distance_sensors: dict):
    global actual_lidar_data
    while True:
        for side, distance_sensor in distance_sensors.items():
            actual_lidar_data[side] = distance_sensor.get_filtered_data()


control_rotation = 0
def send_control_rotation(control_rotation):
    command = str('$' + str(-control_rotation) + ' ' + str(control_rotation) + ';').encode()
    serial_port.write(command)


def rotate_to_course(target_course):
    pid = dvg_pid_controller.PID_Controller(4, 0, 0)
    pid.set_output_limits(-200, 200)
    pid.setpoint = target_course
    pid.in_auto = True
    global control_rotation
    start_timer = time.time()
    previous_angle = get_gyro_angle()
    while True:
        current_angle = get_gyro_angle()
        if abs(previous_angle - current_angle) > 180:
            if current_angle < 0:
                current_angle = 360 + current_angle
            else:
                current_angle = -360 + current_angle
        if pid.compute(current_angle) and (pid.output >= 0 or pid.output <= 0):
            control_rotation = int(pid.output)
            send_control_rotation(control_rotation)
        if abs(current_angle - target_course) < 1 and abs(control_rotation) < 50:
            control_rotation = 0
            send_control_rotation(control_rotation)
            time.sleep(0.2)
            print(get_gyro_angle())
            break
        previous_angle = current_angle


def absolute_angle_to_curse(absolute_angle):
    if absolute_angle > 180:
        curse = -360 + absolute_angle
    elif absolute_angle < -180:
        curse = 360 + absolute_angle
    else:
        curse = absolute_angle
    return curse


def get_initial_map():
    dots = []
    current_angle = get_gyro_angle()
    send_control_rotation(100)
    if get_gyro_angle() - current_angle >= 1:
        current_angle = get_gyro_angle()
        dot_front = (current_angle, actual_lidar_data['front'])
        dot_rear = (absolute_angle_to_curse(current_angle + 180), actual_lidar_data['rear'])
        dot_left = (absolute_angle_to_curse(current_angle + 90), actual_lidar_data['left'])
        dot_right = (absolute_angle_to_curse(current_angle - 90), actual_lidar_data['right'])
        for dot in (dot_front, dot_left, dot_rear, dot_right):
            dots.append(dot)


    return dots


gyro_shift = 0
def get_gyro_angle():
    angle = -float(re.sub(r'\x00', '0', bytes(shm.buf).decode())) - gyro_shift
    if angle > 180:
        angle = -180 + angle % 180
    if angle < -180:
        angle = 180 + angle % -180
    return angle



if __name__ == '__main__':

    distance_sensors = init_distance_sensors(mode=Vl53l0xAccuracyMode.LONG_RANGE)
    shm = shared_memory.SharedMemory(create=True, size=25)
    gyro_angle_process = Process(target=update_gyro_angle, args=(shm.name,))
    gyro_angle_process.start()

    serial_port = serial.Serial(port="/dev/ttyUSB0", baudrate=115200)
    serial_port.close()
    serial_port.open()

    receive_thread = Thread(target=receiving, args=(serial_port,))
    receive_thread.start()

    speed_data_thread = Thread(target=update_actual_speed_data)
    odometry_thread = Thread(target=update_odometry_coordinates)
    speed_data_thread.start()
    odometry_thread.start()

    lidar_data_thread = Thread(target=update_actual_lidar_data, args=(distance_sensors,))
    lidar_data_thread.start()

    print('calculating gyro shift')
    time.sleep(20)
    gyro_shift = get_gyro_angle()
    print('gyro_shift =', gyro_shift)

    # while True:
        # print(odometry_coordinates, get_gyro_angle())

    # map = get_initial_map()


    rotate_to_angle_thread = Thread(target=rotate_to_course, args=(90,))
    rotate_to_angle_thread.start()
    if rotate_to_angle_thread.is_alive():
        print(format_lidar_data(actual_lidar_data))
    # rotate_to_course(0)
    # rotate_to_course(90)
    # rotate_to_course(0)
    # rotate_to_course(180)
    # rotate_to_course(0)
    # rotate_to_course(-90)
    # rotate_to_course(0)
    # rotate_to_course(-180)
    # rotate_to_course(0)


    gyro_angle_process.join()
    receive_thread.join()
    speed_data_thread.join()
    odometry_thread.join()
    # lidar_data_thread.join()
    shm.unlink()
    shm.close()


shm.unlink()
shm.close()


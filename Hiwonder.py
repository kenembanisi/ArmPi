import os, sys
print(os.getcwd())
sys.path.append(os.getcwd()+'/HiwonderSDK/')
from HiwonderSDK import Board
from smbus2 import SMBus
import time
import numpy as np
from threading import Thread
import struct

# Define constants
MOTOR_TYPE_JGB37_520_12V_110RPM = 3 # the magnetic ring generates 44 pulses per revolution, combined with a gear reduction ratio of 90 Default

I2C_PORT = 1
ENCODER_MOTOR_MODULE_ADDR = 0x34
MOTOR_TYPE_ADDR = 20 #0x20
MOTOR_ENCODER_POLARITY_ADDR = 21 #0x21
ADC_BAT_ADDR = 0x00
MOTOR_ENCODER_TOTAL_ADDR = 60 #0x60
MOTOR_FIXED_PWM_ADDR = 31 #0x31 
MOTOR_FIXED_SPEED_ADDR = 51 #0x51 
MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM
MotorEncoderPolarity = 0


class HiwonderRobot():

    def __init__(self):
        # initialize the i2c bus & module board
        self.bus = SMBus(I2C_PORT) # assuming the first (and perhaps only) i2c device
        self.board = Board

        # initialize chassis motors
        self.initialize_motors()
        
        self.speed_control_delay = 1.0
        self.joint_control_delay = 0.25

        self._batt_vol = None
        self._encoder_data = None

        # create a thread to handle reading data from the board
        self.thread = Thread(target=self.read_data())
        self.thread.daemon = True
        self.thread.start()
        # self.thread.join()

    def initialize_motors(self):
        # initialize chassis motors
        self.bus.write_byte_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_TYPE_ADDR, MotorType) # Set motor type
        time.sleep(0.5)
        self.bus.write_byte_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_ENCODER_POLARITY_ADDR, MotorEncoderPolarity)  # Set encoding polarity

        print('Encoder motor driver module has been initialized!! \n')

    
    def set_fixed_speed(self, speed: list):
        # uses MOTOR_FIXED_SPEED_ADDR to set speed
        if len(speed) != 4:
            print('Please set correct size of speed...')
            raise ValueError
        else:
            self.bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_FIXED_SPEED_ADDR, speed)
            time.sleep(self.speed_control_delay)


    def stop_motors(self):
        stop_speed = [0]*4
        self.bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_FIXED_SPEED_ADDR, stop_speed)
        print('Stopping Motors!!')


    def set_joint_value(self, joint_id: int, theta: float, radians = False):
        if joint_id > 6 or joint_id < 1:
            print('Please set correct joint id within range (1-6)...')
            raise ValueError
        if radians:
            theta = np.rad2deg(theta)
        self.board.setBusServoPulse(joint_id, self.map_value(theta), 500)
        time.sleep(self.joint_control_delay)
    
    
    def get_joint_value(self, joint_id: int):
        if joint_id > 6 or joint_id < 1:
            print('Please set correct joint id within range (1-6)...')
            raise ValueError
        return self.board.getBusServoPulse(joint_id)
    

    def set_joint_values(self, thetalist: list, radians = False):
        if len(thetalist) != 6:
            print('Please supply 6 joint angles for the robot...')
            raise ValueError
        if radians:
            for i in range(len(thetalist)):
                thetalist[i] = np.rad2deg(thetalist[i])
        for id, th in enumerate(thetalist):
            self.board.setBusServoPulse(id, self.map_value(th), 500)
            time.sleep(self.joint_control_delay)
    

    def map_value(self, x: float):
        hw_min, hw_max = 0, 1000 # defined by the driver
        joint_min, joint_max = -150, 150
        return (x - joint_min) * (hw_max - hw_min) / (joint_max - joint_min) + hw_min


    def read_data(self):
        while True:
            # read battery data
            self._batt_vol = self.bus.read_i2c_block_data(ENCODER_MOTOR_MODULE_ADDR, ADC_BAT_ADDR, 2)
            # print("V = {0}mV".format(self._batt_vol[0]+(self._batt_vol[1]<<8)))
            
            # read encoder data
            self._encoder_data = struct.unpack('iiii',bytes(self.bus.read_i2c_block_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_ENCODER_TOTAL_ADDR,16)))
            # print("Encode1 = {0}  Encode2 = {1}  Encode3 = {2}  Encode4 = {3}".format(Encode[0],Encode[1],Encode[2],Encode[3]))
            time.sleep(0.5)

        

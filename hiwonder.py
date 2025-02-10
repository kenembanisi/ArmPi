import os, sys
sys.path.append(os.getcwd()+'/HiwonderSDK/')
from HiwonderSDK import Board
import time
import numpy as np
import struct
import threading
from smbus2 import SMBus
from hiwonder_servo_serialproxy import SerialProxy
from arm_models import FiveDOFRobot
import utils as ut


# Define constants
MOTOR_TYPE_JGB37_520_12V_110RPM = 3 # the magnetic ring generates 44 pulses per revolution, combined with a gear reduction ratio of 90 Default

I2C_PORT = 1
SERIAL_PORT = '/dev/ttyS0'
ENCODER_MOTOR_MODULE_ADDR = 0x34
MOTOR_TYPE_ADDR = 20 #0x20
MOTOR_ENCODER_POLARITY_ADDR = 21 #0x21
ADC_BAT_ADDR = 0x00
MOTOR_ENCODER_TOTAL_ADDR = 60 #0x60
MOTOR_FIXED_PWM_ADDR = 31 #0x31 
MOTOR_FIXED_SPEED_ADDR = 51 #0x51 
MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM
MotorEncoderPolarity = 0

WHEEL_RADIUS = 0.047 # m
BASE_LENGTH_X = 0.096 # m
BASE_LENGTH_Y = 0.105 # m



class HiwonderRobot():

    def __init__(self):
        # initialize the i2c bus & module board
        # self.bus = SMBus(I2C_PORT) # assuming the first (and perhaps only) i2c device
        self.serial_lock = threading.Lock()
        self.board = Board

        # initialize serial proxy
        # self.serial_proxy = SerialProxy(port_name=SERIAL_PORT)
        # self.serial_proxy.connect()

        # initialize chassis motors
        # self.initialize_motors()
        
        self.speed_control_delay = 0.25
        self.joint_control_delay = 0.25
        self._batt_vol = None
        self._encoder_data = None

        # create a thread to handle reading data from the board
        self.thread = threading.Thread(target=self.read_joint_data, daemon=True)
        self.thread.start()
        self.joint_values = [0]*6

        # initialize the five-dof robot model
        self.model = FiveDOFRobot()


    # -------------------------------------------------------------
    # methods for interfacing with the mobile base
    # -------------------------------------------------------------

    def initialize_motors(self):
        # initialize chassis motors
        time.sleep(1)
        self.bus.write_byte_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_TYPE_ADDR, MotorType) # Set motor type
        time.sleep(0.5)
        self.bus.write_byte_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_ENCODER_POLARITY_ADDR, MotorEncoderPolarity)  # Set encoding polarity

        print('Encoder motor driver module has been initialized!! \n')


    def set_fixed_speed(self, speed: list):
        # uses MOTOR_FIXED_SPEED_ADDR to set speed
        if len(speed) != 4:
            raise ValueError("Speed list must contain exactly 4 values.")
        # wheel speed range is -100 to 100
        np.clip(speed, -100, 100)
        speed_ = [int(s) for s in speed]
        self.bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_FIXED_SPEED_ADDR, speed_)
        time.sleep(self.speed_control_delay)


    def set_robot_velocity(self, cmd: ut.GamepadCmds):
        # set the base velocity
        u = ut.Controls()
        u.vx = cmd.base_vx
        u.vy = cmd.base_vy
        u.w = cmd.base_w
        # self.set_base_velocity(u)

        # set the arm end effector velocity
        # self.set_arm_velocity(cmd)
        self.set_arm_positions(cmd)

        


    def set_base_velocity(self, u: ut.Controls):
        """
        motor3 w0|  ↑  |w1 motor1
                 |     |
        motor4 w2|     |w3 motor2
        
        """
        # clip desired velocity to 0 if less than 0.009
        vx = u.vx if abs(u.vx) > 0.009 else 0.0
        vy = u.vy if abs(u.vy) > 0.009 else 0.0
        w = u.w if abs(u.w) > 0.009 else 0.0

        # compute wheel speeds
        w0 = (vx - vy - w * (BASE_LENGTH_X + BASE_LENGTH_Y)) / WHEEL_RADIUS
        w1 = (vx + vy + w * (BASE_LENGTH_X + BASE_LENGTH_Y)) / WHEEL_RADIUS
        w2 = (vx + vy - w * (BASE_LENGTH_X + BASE_LENGTH_Y)) / WHEEL_RADIUS
        w3 = (vx - vy + w * (BASE_LENGTH_X + BASE_LENGTH_Y)) / WHEEL_RADIUS

        # u.vx, u.vy are in m/s
        # u.w is in rad/s
        # wi is in rad/s

        # convert from rad/s to hw speed range (-100 to 100)
        speed_rad = [w1, w3, w0, w2] # rearrangement to map to the hw wheel mapping

        print(f'wheel speed in rad/s: {speed_rad}')

        speed = [w*8.4 for w in speed_rad] # where 8.4 is the rad/s to the speed scale factor
        print(f'Final speed to send: {speed}')

        self.set_fixed_speed(speed)

        
    def set_arm_velocity(self, vel: list):
        # calculate the joint velocities
        thetadot = self.model.calc_velocity_kinematics(vel)

        # get the joint values
        theta = self.get_joint_values()

        # Update joint angles
        theta[0] += 0.02 * thetadot[0]
        theta[1] += 0.02 * thetadot[1]
        theta[2] += 0.02 * thetadot[2]
        theta[3] += 0.02 * thetadot[3]
        theta[4] += 0.02 * thetadot[4]

        # set new joint angles
        self.set_joint_values(theta)


    def set_arm_positions(self, cmd):
        theta = self.joint_values.copy()

        # Update joint angles
        theta[0] += 800 * cmd.arm_j1
        theta[1] += 800 * cmd.arm_j2
        theta[2] += 800 * cmd.arm_j3
        theta[3] += 800 * cmd.arm_j4
        theta[4] += 800 * cmd.arm_j5

        # set new joint angles
        print(f'theta: {theta}')
        self.set_joint_values(theta)



    def read_data(self):
        while True:
            try:
                self._batt_vol = self.bus.read_i2c_block_data(ENCODER_MOTOR_MODULE_ADDR, ADC_BAT_ADDR, 2)
                self._encoder_data = struct.unpack('iiii', bytes(self.bus.read_i2c_block_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_ENCODER_TOTAL_ADDR, 16)))
                time.sleep(0.05)
            except Exception as e:
                print(f"Error reading data: {e}")


    def read_joint_data(self):
        while True:
            try:
                self.joint_values = self.get_joint_values()
                time.sleep(0.05)  # Slightly longer sleep to prevent excessive CPU usage
            except KeyboardInterrupt:
                print('Ending joint data reading thread')
                break
            
        
    def stop_motors(self):
        stop_speed = [0]*4
        self.bus.write_i2c_block_data(ENCODER_MOTOR_MODULE_ADDR, MOTOR_FIXED_SPEED_ADDR, stop_speed)
        self.bus.close()
        print('Stopping Motors!!')

    # -------------------------------------------------------------
    # methods for interfacing with the 5dof arm
    # -------------------------------------------------------------

    def set_joint_value(self, joint_id: int, theta: float, radians = False):
        if not (1 <= joint_id <= 6):
            raise ValueError("Joint ID must be between 1 and 6.")
        if radians:
            theta = np.rad2deg(theta)
        theta = np.clip(theta, -150, 150)

        # print("[DEBUG] Trying to acquire serial lock in set_joint_value")
        with self.serial_lock:
            # print("[DEBUG] Acquired serial lock in set_joint_value")
            self.board.setBusServoPulse(joint_id, self.angle_to_pulse(theta), 500)
        # print("[DEBUG] Released serial lock in set_joint_value")

        print(f'[DEBUG] Moving to [{theta}] deg or [{self.angle_to_pulse(theta)}] pulse')
        # self.serial_proxy.set_position(joint_id, self.map_value(theta), 500)
        time.sleep(self.joint_control_delay)


    def set_joint_values(self, thetalist: list, radians=False):
        if len(thetalist) != 6:
            raise ValueError("Please supply 6 joint angles for the robot.")
        
        if radians:
            thetalist = [np.rad2deg(theta) for theta in thetalist]
        
        # print("[DEBUG] Trying to acquire serial lock in set_joint_values")
        if self.serial_lock.acquire(timeout=1):  # Try for 1 second
            try:
                # print("[DEBUG] Acquired lock in set_joint_values")
                for id, theta in enumerate(thetalist):
                    self.board.setBusServoPulse(id+1, self.angle_to_pulse(theta), 200)
            finally:
                self.serial_lock.release()
                # print("[DEBUG] Released lock in set_joint_values")
        else:
            print("[WARNING] set_joint_values could not acquire lock!")
        
        time.sleep(self.joint_control_delay)
        
   
    
    def get_joint_value(self, joint_id: int):
        if not (1 <= joint_id <= 6):
            raise ValueError("Joint ID must be between 1 and 6.")
        
        # print("[DEBUG] Trying to acquire serial lock in get_joint_value")
        if self.serial_lock.acquire(timeout=0.5):  # Attempt to acquire lock
            try:
                # print("[DEBUG] Acquired lock in get_joint_value")
                return self.pulse_to_angle(self.board.getBusServoPulse(joint_id))
            finally:
                self.serial_lock.release()
                # print("[DEBUG] Released lock in get_joint_value")
        else:
            print("[WARNING] get_joint_value could not acquire lock!")
            return None  # Return None or handle appropriately
        
    
    
    def get_joint_values(self):
        # return [self.joints_state[id] for id in range(6)]
        return [self.get_joint_value(id) for id in range(1, 7)]


    def angle_to_pulse(self, x: float):
        hw_min, hw_max = 0, 1000 # defined by the driver
        joint_min, joint_max = -150, 150
        return int((x - joint_min) * (hw_max - hw_min) / (joint_max - joint_min) + hw_min)
    
    
    def pulse_to_angle(self, x: float):
        hw_min, hw_max = 0, 1000 # defined by the driver
        joint_min, joint_max = -150, 150
        return round((x - hw_min) * (joint_max - joint_min) / (hw_max - hw_min) + joint_min, 2)



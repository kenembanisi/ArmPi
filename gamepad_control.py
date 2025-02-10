import inputs
import time
from utils import GamepadCmds


class GamepadControl():

    def __init__(self):
        # initialize gamepad
        self.initialize_gamepad()

        # initialize gamepad cmds
        self.gamepad_cmds_prev = GamepadCmds()
        self.abs_x, self.abs_y, self.abs_z = 128, 128, 128

        # initialize variables
        self.MOBILE_BASE_FLAG = False
        self.ARM_FLAG = False
        self.ARM_J1_FLAG = False
        self.ARM_J2_FLAG = False
        self.ARM_J3_FLAG = False
        self.ARM_J4_FLAG = False
        self.ARM_J5_FLAG = False
        self.ARM_EE_FLAG = False



    
    def initialize_gamepad(self):
        i = 0
        while i < 10:
            # get a list of gamepads
            self.gamepads = inputs.devices.gamepads
            if not self.gamepads:
                print(f'No gamepads detected. Please check USB connection. Trying again [{i+1}/10]...')
            else:
                # use the first gamepad object
                self.gamepad = self.gamepads[0]
                print(f'Using gamepad: {self.gamepad}')
                return True
            time.sleep(1.0) # delay time
            i += 1
        print('Failed to detect gamepads. Please abort and try again!')
        return False
    

    def get_gamepad_cmds(self):
        gamepad_cmds = GamepadCmds()
        events = self.gamepad._do_iter()
        if events == None:
            return self.gamepad_cmds_prev
        else:
            for event in events:
                # print(f'\n number of events = {len(events)}')

                if event.ev_type != 'Sync':

                    if event.code == 'ABS_X':
                        self.abs_x = event.state
                    if event.code == 'ABS_Y':
                        self.abs_y = event.state
                    if event.code == 'ABS_Z':
                        self.abs_z = event.state
                    if event.code == 'BTN_WEST':
                        self.MOBILE_BASE_FLAG = bool(event.state)
                    if event.code == 'BTN_Z':
                        self.ARM_FLAG = bool(event.state)
                    if event.code == 'BTN_NORTH':
                        self.ARM_J1_FLAG = bool(event.state)
                    if event.code == 'BTN_C':
                        self.ARM_J2_FLAG = bool(event.state)
                    if event.code == 'BTN_EAST':
                        self.ARM_J3_FLAG = bool(event.state)
                    if event.code == 'BTN_SOUTH':
                        self.ARM_J4_FLAG = bool(event.state)
                    if event.code == 'BTN_TR':
                        self.ARM_J5_FLAG = bool(event.state)
                    if event.code == 'BTN_TL':
                        self.ARM_EE_FLAG = bool(event.state)
            
            if self.MOBILE_BASE_FLAG:
                # we set the range for vx, vy to -0.5 m/s to 0.5 m/s
                # w = -0.5 rad/s to 0.5 rad/s
                gamepad_cmds.base_vx = self.map_value(self.abs_x, 0.5, -0.5)
                gamepad_cmds.base_vy = self.map_value(self.abs_y, 0.5, -0.5)
                gamepad_cmds.base_w = self.map_value(self.abs_z, -0.5, 0.5)

                # print(f'vx = [{self.abs_x} -> {gamepad_cmds.base_vx}], \
                #        vy = [{self.abs_y} -> {gamepad_cmds.base_vy}] \
                #        w = [{self.abs_z} -> {gamepad_cmds.base_w}]')

            if self.ARM_FLAG:
                # we set the range for vx, vy, vz to -0.2 m/s to 0.2 m/s
                gamepad_cmds.arm_vx = self.map_value(self.abs_x, -0.2, 0.2)
                gamepad_cmds.arm_vy = self.map_value(self.abs_y, 0.2, -0.2)
                gamepad_cmds.arm_vz = self.map_value(self.abs_z, -0.2, 0.2)
            
            gamepad_cmds.arm_j1 = self.map_value(self.abs_x, -0.1, 0.1) if self.ARM_J1_FLAG else 0.0
            gamepad_cmds.arm_j2 = self.map_value(self.abs_x, -0.1, 0.1) if self.ARM_J2_FLAG else 0.0
            gamepad_cmds.arm_j3 = self.map_value(self.abs_x, -0.1, 0.1) if self.ARM_J3_FLAG else 0.0
            gamepad_cmds.arm_j4 = self.map_value(self.abs_x, -0.1, 0.1) if self.ARM_J4_FLAG else 0.0
            gamepad_cmds.arm_j5 = self.map_value(self.abs_x, -0.1, 0.1) if self.ARM_J5_FLAG else 0.0
            gamepad_cmds.arm_ee = 1.0 if self.ARM_EE_FLAG else 0.0



            self.gamepad_cmds_prev = gamepad_cmds
        
            return gamepad_cmds


    def map_value(self, x: float, hw_min=0, hw_max=1):
        # hw_min, hw_max = 0, 1000 # defined by the driver
        joint_min, joint_max = 0, 255
        val = (x - joint_min) * (hw_max - hw_min) / (joint_max - joint_min) + hw_min
        return val if abs(val) > 0.005 else 0.0

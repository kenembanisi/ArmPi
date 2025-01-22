import inputs
import time
from utils import GamepadCmds


class GamepadControl():

    def __init__(self):
        # initialize gamepad
        self.initialize_gamepad()

        # initialize gamepad cmds
        self.gamepad_cmds = GamepadCmds()
        self.abs_x, self.abs_y, self.abs_z = 0, 0, 0

        # initialize variables
        self.MOBILE_BASE_FLAG = False
        self.ARM_FLAG = False


    
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
        events = self.gamepad.read()
        for event in events:

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
        
        if self.MOBILE_BASE_FLAG:
            gamepad_cmds.base_vx = self.abs_x
            gamepad_cmds.base_vy = self.abs_y
            gamepad_cmds.base_w = self.abs_z

        if self.ARM_FLAG:
            gamepad_cmds.arm_vx = self.abs_x
            gamepad_cmds.arm_vy = self.abs_y
            gamepad_cmds.arm_vz = self.abs_z
        
        return gamepad_cmds
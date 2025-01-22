# functionalities I want:
# 1. Independent joint control using the gamepad
# 2. End-effector control using RRMC

import inputs
import time

# Get a list of gamepads
gamepads = inputs.devices.gamepads
if not gamepads:
    print("No gamepads detected.")
    exit()

# Use the first gamepad
gamepad = gamepads[0]
print(f"Using gamepad: {gamepad}")

# BTN_SOUTH -> A
# BTN_EAST -> B
# BTN_NORTH -> X
# BTN_WEST -> Y
# BTN_TR -> RB

i = 0
X, Y = 0, 0
vx0, vy0, w0, vx, vy, w = 0, 0, 0, 0, 0, 0
MOBILE_BASE_FLAG = False

try:
    while True:
        # events = inputs.get_gamepad()
        events = gamepad.read()
        # MOBILE_BASE_FLAG = False
        speed = None
        for i, event in enumerate(events):

            if event.ev_type != 'Sync':

                # print(f'{event.ev_type} \
                #         {event.code} \
                #         {event.state}')
                
                if event.code == 'ABS_X':
                    vy0 = event.state
                if event.code == 'ABS_Y':
                    vx0 = event.state
                if event.code == 'ABS_Z':
                    w0 = event.state
                if event.code == 'BTN_WEST':
                    MOBILE_BASE_FLAG = bool(event.state)
#       
        # print(MOBILE_BASE_FLAG)
        if MOBILE_BASE_FLAG:
            vx, vy, w = vx0, vy0, w0
        print(f'[vx, vy, w] = [{vx}, {vy}, {w}]')
        time.sleep(0.0005)  # Limit loop speed
except inputs.DeviceDisconnectedError:
    print("Gamepad disconnected.")
    # motor.set_speed(0) # Stop motor on disconnect
except KeyboardInterrupt:
  print("Exiting")
#   motor.set_speed(0)
finally:
    # Clean up resources (e.g., GPIO cleanup)
    print("Cleaning up")
    #GPIO.cleanup() # if using RPi.GPIO
    pass
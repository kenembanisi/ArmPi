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

try:
    while True:
        events = inputs.get_gamepad()
        CMD_PRESS = False
        speed = None
        for i, event in enumerate(events):

            print(f'{1} - {event.code}')
            # print(f'Event is {event.code}')
            # if event.code == 'ABS_Y':

            if event.code == 'ABS_HAT0X':
                # Y axis controls speed (inverted)
                # speed = -event.state / 32768.0  # Normalize to -1.0 to 1.0
                speed = event.state
                # motor.set_speed(speed)

                # print(f'cmd is {round(speed, 3)}')
            # elif event.code == 'BTN_SOUTH': 
            #   if event.state == 1:
            #     print("Button A pressed")
            #   else:
            #     print("Button A released")

            if speed is not None:
                if event.code == 'BTN_SOUTH':
                    print(f'Joint 1: {speed}')
                elif event.code == 'BTN_EAST':
                    print(f'Joint 2: {speed}')
                elif event.code == 'BTN_WEST':
                    print(f'Joint 3: {speed}')
                elif event.code == 'BTN_NORTH':
                    print(f'Joint 4: {speed}')
                elif event.code == 'BTN_TR':
                    print(f'Joint 5: {speed}')
            
            print(f'count {i}, speed is {speed}')
            i+=1
        
        print('end of events \n')
# 
            # Add more event handling for other buttons or axes

        time.sleep(0.005)  # Limit loop speed
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
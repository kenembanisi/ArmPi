from hiwonder import HiwonderRobot
from gamepad_control import GamepadControl
import time
import threading


cmdlist = []
gpc = GamepadControl()


def monitor_gampepad():
    while True:
        try:
            cmdlist.append(gpc.get_gamepad_cmds())
            time.sleep(0.001)
        except KeyboardInterrupt:
            print('Ending gamepad thread')


def main():
    try:
        # create a gamepad thread
        gamepad_thread = threading.Thread(target=monitor_gampepad)
        gamepad_thread.daemon = True
        gamepad_thread.start()

        # initialize hardware robot driver
        robot = HiwonderRobot()
        # gpc = GamepadControl()

        i = 0
        while True:
            
            cmds = GamepadControl()
            if len(cmdlist) > 1:
                cmds = cmdlist[-1]            

            print(f'MOBILE = [{i}][{round(cmds.base_vx,3)} {round(cmds.base_vy,3)} {round(cmds.base_w,3)}]')
            # print(f'ARM = [{cmds.arm_vx} {cmds.arm_vy} {cmds.arm_vz}] \n')
            
            robot.set_robot_velocity(cmds)

            # robot.read_data()

            i += 1
            time.sleep(0.1)  # Limit loop speed

    except KeyboardInterrupt:
        robot.stop_motors()
        print("done")
        


if __name__ == "__main__":
    main()



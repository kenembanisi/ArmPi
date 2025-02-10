from hiwonder import HiwonderRobot
from gamepad_control import GamepadControl
import time
import threading
import utils

# TODO: Fix the issue with mobile base directions not working right and turning in place not working
# TODO: Determine a mapping from -128 to 128 for EE velocity (xyz)
# TODO: Get RRMC working for the arm
# TODO: Implement a gamepad independent joint control

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
            
            # cmds = GamepadControl()
            if len(cmdlist) > 1:
                cmds = cmdlist[-1]            

                # print(f'MOBILE = [{i}][{round(cmds.base_vx,3)} {round(cmds.base_vy,3)} {round(cmds.base_w,3)}]')
                # print(f'ARM = [{i}][{round(cmds.arm_vx,3)} {round(cmds.arm_vy,3)} {round(cmds.arm_vz,3)}] \n')

                # utils.print_dataclass(cmds)
            
                robot.set_robot_velocity(cmds)
                print(f'Joint values = {robot.joint_values}')

                # values = robot.get_joint_values()
                # print(f"[MAIN] Direct Joint Values: {values}")

                # print(f"[CHECK] Thread Alive? {robot.thread.is_alive()}")

            # robot.read_data()

            i += 1
            time.sleep(0.05)  # Limit loop speed

    except KeyboardInterrupt:
        robot.stop_motors()
        print("done")
        


if __name__ == "__main__":
    main()



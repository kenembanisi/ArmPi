from hiwonder import HiwonderRobot
from gamepad_control import GamepadControl
import time


def main():
    try:
        robot = HiwonderRobot()
        gpc = GamepadControl()

        while True:
            
            cmds = gpc.get_gamepad_cmds()

            print(f'MOBILE = [{cmds.base_vx} {cmds.base_vy} {cmds.base_w}]')
            print(f'ARM = [{cmds.arm_vx} {cmds.arm_vy} {cmds.arm_vz}] \n')
            
            robot.set_robot_velocity(cmds)

            # robot.read_data()

            time.sleep(0.001)  # Limit loop speed

    except KeyboardInterrupt:
        robot.stop_motors()
        print("done")
        


if __name__ == "__main__":
    main()



from Hiwonder import HiwonderRobot



def main():
    try:
        robot = HiwonderRobot()

        while True:
            case = int(input('What control state do you want (1: motor, 2: inidivual joints, 3: all joints)? '))
            print('\n')

            if case == 1:
                val = input('Insert desired speeds: (e.g., 10, 20, 10, 20) ')
                speed = [int(s) for s in val.split(',')]
                robot.set_fixed_speed(speed)

            elif case == 2:
                val = input('Insert desired joint_id and joint angle: (e.g., 2, 30) ')
                joint_id, theta = [int(s) for s in val.split(',')]
                robot.move_joint(joint_id, theta)

            elif case == 3:
                val = input('Insert desired joint angles: (e.g., 2, 30, 20, 20, 30, 10) ')
                thetalist = [int(s) for s in val.split(',')]
                robot.move_joints(thetalist)

            else:
                print('Please check entry!')
                raise ValueError
            
            robot.read_data()

    except KeyboardInterrupt:
        robot.stop_motors()
        


if __name__ == "__main__":
    main()



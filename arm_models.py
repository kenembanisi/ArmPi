from math import sqrt, sin, cos, atan, atan2
import numpy as np
# from matplotlib.figure import Figure
from utils import EndEffector, rotm_to_euler, euler_to_rotm, check_joint_limits, dh_to_matrix, near_zero

PI = 3.1415926535897932384
np.set_printoptions(precision=3)


class FiveDOFRobot:
    """
    A class to represent a 5-DOF robotic arm with kinematics calculations, including
    forward kinematics, inverse kinematics, velocity kinematics, and Jacobian computation.

    Attributes:
        l1, l2, l3, l4, l5: Link lengths of the robotic arm.
        theta: List of joint angles in radians.
        theta_limits: Joint limits for each joint.
        ee: End-effector object for storing the position and orientation of the end-effector.
        num_dof: Number of degrees of freedom (5 in this case).
        points: List storing the positions of the robot joints.
        DH: Denavit-Hartenberg parameters for each joint.
        T: Transformation matrices for each joint.
    """
    
    def __init__(self):
        """Initialize the robot parameters and joint limits."""
        # Link lengths
        self.l1, self.l2, self.l3, self.l4, self.l5 = 0.30, 0.15, 0.18, 0.15, 0.12
        
        # Joint angles (initialized to zero)
        self.theta = [0, 0, 0, 0, 0]
        
        # Joint limits (in radians)
        self.theta_limits = [
            [-np.pi, np.pi], 
            [-np.pi/3, np.pi], 
            [-np.pi+np.pi/12, np.pi-np.pi/4], 
            [-np.pi+np.pi/12, np.pi-np.pi/12], 
            [-np.pi, np.pi]
        ]
        
        # End-effector object
        self.ee = EndEffector()
        
        # Robot's points
        self.num_dof = 5
        self.points = [None] * (self.num_dof + 1)
        
        # Denavit-Hartenberg parameters and transformation matrices
        self.DH = np.zeros((5, 4))
        self.T = np.zeros((self.num_dof, 4, 4))

    
    def calc_forward_kinematics(self, theta: list, radians=False):
        """
        Calculate forward kinematics based on the provided joint angles.
        
        Args:
            theta: List of joint angles (in degrees or radians).
            radians: Boolean flag to indicate if input angles are in radians.
        """
        if not radians:
            # Convert degrees to radians
            self.theta = np.deg2rad(theta)
        else:
            self.theta = theta
        
        # Apply joint limits
        self.theta = [np.clip(th, self.theta_limits[i][0], self.theta_limits[i][1]) 
                      for i, th in enumerate(self.theta)]

        # Set the Denavit-Hartenberg parameters for each joint
        self.DH[0] = [self.theta[0], self.l1, 0, np.pi/2]
        self.DH[1] = [self.theta[1], 0, self.l2, 0]
        self.DH[2] = [self.theta[2], 0, self.l3, 0]
        self.DH[3] = [self.theta[3] - np.pi/2, 0, 0, -np.pi/2]
        self.DH[4] = [self.theta[4], self.l4 + self.l5, 0, 0]

        # Compute the transformation matrices
        for i in range(self.num_dof):
            self.T[i] = dh_to_matrix(self.DH[i])
        
        # Calculate robot points (positions of joints)
        self.calc_robot_points()


    def calc_inverse_kinematics(self, EE: EndEffector, soln=0):
        """
        Calculate inverse kinematics to determine the joint angles based on end-effector position.
        
        Args:
            EE: EndEffector object containing desired position and orientation.
            soln: Optional parameter for multiple solutions (not implemented).
        """
        # Extract position and orientation of the end-effector
        l1, l2, l3, l4, l5 = self.l1, self.l2, self.l3, self.l4, self.l5

        # move robot slightly out of zeros singularity
        if all(th == 0.0 for th in self.theta):
            self.theta = [self.theta[i] + np.random.rand()*0.01 for i in range(self.num_dof)]
        
        # Desired position and rotation matrix
        p = np.array([EE.x, EE.y, EE.z])
        rpy = np.array([EE.rotz, EE.roty, EE.rotx])
        R = euler_to_rotm(rpy)
        
        # Calculate wrist position
        wrist_pos = p - R @ np.array([0, 0, 1]) * (l4 + l5)
        # print(f"Wrist pose: \n {wrist_pos} \n")


        try: 
            # Solve for theta_1 using trigonometry
            x_wrist, y_wrist, z_wrist = wrist_pos
            self.theta[0] = atan2(y_wrist, x_wrist)
            c1, s1 = np.cos(self.theta[0]), np.sin(self.theta[0])

            # using cosine rule, find theta_3
            L = sqrt(x_wrist**2 + y_wrist**2)
            s = abs(z_wrist - l1)
            cbeta = (l2**2 + l3**2 - L**2 - s**2) / (2 * l2 * l3)
            cbeta = np.clip(cbeta, -1.0, 1.0)
            beta = np.arccos(cbeta)
            self.theta[2] = PI - beta
            c3, s3 = np.cos(self.theta[2]), np.sin(self.theta[2])

            # Find theta_2
            alpha = atan2(l3 * s3, l2 + l3 * c3)
            gamma = atan2(s, L)
            self.theta[1] = gamma - alpha
            c2, s2 = np.cos(self.theta[1]), np.sin(self.theta[1])

            # calculate R0_3
            c23 = c2*c3 - s2*s3
            s23 = s2*c3 + c2*s3
            R0_3 = np.array([[c1 * c23, -c1 * s23, s1],
                            [s1 * c23, -s1 * s23, -c1],
                            [s23, c23, 0]])

            # Calculate R4_5
            R4_5 = R0_3.T @ R

            self.theta[3] = atan2(R4_5[1,2], R4_5[0,2])
            self.theta[4] = atan2(R4_5[2,0], R4_5[2,1])

            if not check_joint_limits(self.theta, self.theta_limits):
                print(f"\n [ERROR] Joint limits exceeded! \n \
                      Desired joints are {self.theta} \n \
                      Joint limits are {self.theta_limits}")
                raise ValueError
            
        except RuntimeWarning:
            print("\n [ERROR] (Runtime) Joint limits exceeded! \n")
        
        # Calculate forward kinematics with the new joint angles
        self.calc_forward_kinematics(self.theta, radians=True)


        print(f"theta: [{self.theta}]")

    
    def calc_numerical_ik(self, EE: EndEffector, tol=0.01, ilimit=50):
        """ Calculate numerical inverse kinematics based on input coordinates. """
        
        Te_d = [EE.x, EE.y, EE.z]

        # move robot slightly out of zeros singularity
        if all(th == 0.0 for th in self.theta):
            self.theta = [self.theta[i] + np.random.rand()*0.01 for i in range(self.num_dof)]
        
        # Iteration count
        i = 0
        q = [self.theta[i] for i in range(self.num_dof)]
        
        while i < ilimit:
            i += 1

            # compute current EE position based on q
            Te = self.solve_forward_kinematics(q, radians=True)

            print(f"curr position: {Te}")
            # calculate the EE position error
            e = [0, 0, 0]
            e[0] = Te_d[0] - Te[0]
            e[1] = Te_d[1] - Te[1]
            e[2] = Te_d[2] - Te[2]
            # e, E = self.error(Te, Tep)

            print(f"error: {e}")

            # calculate the jacobian matrix
            J = self.jacobian(q)

            # update q
            q += np.linalg.pinv(J) @ e

            # check for joint limits
            for j, th in enumerate(q):
                q[j] = np.clip(th, self.theta_limits[j][0], self.theta_limits[j][1])

            # if self.pinv:
            #     q += np.linalg.pinv(J) @ e
            # else:
            #     q += np.linalg.inv(J) @ e

            print(f"iterating... {i}/{ilimit} | q = {q} \n")


            # Check if we have arrived
            if abs(max(e, key=abs)) < tol:
                break 
        
        if abs(max(e, key=abs)) > tol:
            print("\n [ERROR] Numerical IK solution failed to converge... \n \
                  Possible causes: \n \
                  1. joint limits may have been exceeded! \n \
                  2. desired joint configuration is very close to OR at a singularity \n")
            raise ValueError

        self.theta = q

        # calculate robot points
        # self.calc_robot_points()
        self.calc_forward_kinematics(self.theta, radians=True)


    def calc_velocity_kinematics(self, vel: list):
        """
        Calculate the joint velocities required to achieve the given end-effector velocity.
        
        Args:
            vel: Desired end-effector velocity (3x1 vector).
        """
        # Avoid singularity by perturbing joint angles slightly
        if all(th == 0.0 for th in self.theta):
            self.theta = [th + np.random.rand() * 0.01 for th in self.theta]

        # Calculate the joint velocity using the inverse Jacobian
        thetadot = self.inverse_jacobian(pseudo=True) @ vel

        # Update joint angles
        self.theta[0] += 0.02 * thetadot[0]
        self.theta[1] += 0.02 * thetadot[1]
        self.theta[2] += 0.02 * thetadot[2]
        self.theta[3] += 0.02 * thetadot[3]
        self.theta[4] += 0.02 * thetadot[4]

        # Recompute robot points based on updated joint angles
        self.calc_forward_kinematics(self.theta, radians=True)

        return thetadot


    def jacobian(self, theta: list = None):
        """
        Compute the Jacobian matrix for the current robot configuration.

        Args:
            theta (list, optional): The joint angles for the robot. Defaults to self.theta.
        
        Returns:
            Jacobian matrix (3x5).
        """
        # Use default values if arguments are not provided
        if theta is None:
            theta = self.theta

        # Define DH parameters
        self.DH[0] = [theta[0], self.l1, 0, np.pi/2]
        self.DH[1] = [theta[1], 0, self.l2, 0]
        self.DH[2] = [theta[2], 0, self.l3, 0]
        self.DH[3] = [theta[3] - np.pi/2, 0, 0, -np.pi/2]
        self.DH[4] = [theta[4], self.l4 + self.l5, 0, 0]

        # Compute transformation matrices
        for i in range(self.num_dof):
            self.T[i] = dh_to_matrix(self.DH[i])

        # Precompute transformation matrices for efficiency
        T_cumulative = [np.eye(4)]
        for i in range(self.num_dof):
            T_cumulative.append(T_cumulative[-1] @ self.T[i])

        # Define O0 for calculations
        O0 = np.array([0, 0, 0, 1])
        
        # Initialize the Jacobian matrix
        jacobian = np.zeros((3, self.num_dof))

        # Calculate the Jacobian columns
        for i in range(self.num_dof):
            T_curr = T_cumulative[i]
            T_final = T_cumulative[-1]
            
            # Calculate position vector r
            r = (T_final @ O0 - T_curr @ O0)[:3]

            # Compute the rotation axis z
            z = T_curr[:3, :3] @ np.array([0, 0, 1])

            # Compute linear velocity part of the Jacobian
            jacobian[:, i] = np.cross(z, r)

        # Replace near-zero values with zero, primarily for debugging purposes
        return near_zero(jacobian)
  

    def inverse_jacobian(self, pseudo=False):
        """
        Compute the inverse of the Jacobian matrix using either pseudo-inverse or regular inverse.
        
        Args:
            pseudo: Boolean flag to use pseudo-inverse (default is False).
        
        Returns:
            The inverse (or pseudo-inverse) of the Jacobian matrix.
        """
        if pseudo:
            return np.linalg.pinv(self.jacobian())
        else:
            return np.linalg.inv(self.jacobian())
        

    def solve_forward_kinematics(self, theta: list, radians=False):

        # Convert degrees to radians
        if not radians:
            for i in range(len(theta)):
                theta[i] = np.deg2rad(theta[i])

        # DH parameters = [theta, d, a, alpha]
        DH = np.zeros((5, 4))
        DH[0] = [theta[0],   self.l1,    0,       np.pi/2]
        DH[1] = [theta[1],   0,          self.l2, 0]
        DH[2] = [theta[2],   0,          self.l3, 0]
        DH[3] = [theta[3]-np.pi/2,   0,          0,       -np.pi/2]
        DH[4] = [theta[4],   self.l4+self.l5, 0, 0]

        T = np.zeros((self.num_dof,4,4))
        for i in range(self.num_dof):
            T[i] = dh_to_matrix(DH[i])

        return T[0] @ T[1] @ T[2] @ T[3] @ T[4] @ np.array([0, 0, 0, 1])


    def calc_robot_points(self):
        """ Calculates the main arm points using the current joint angles """

        # Initialize points[0] to the base (origin)
        self.points[0] = np.array([0, 0, 0, 1])

        # Precompute cumulative transformations to avoid redundant calculations
        T_cumulative = [np.eye(4)]
        for i in range(self.num_dof):
            T_cumulative.append(T_cumulative[-1] @ self.T[i])

        # Calculate the robot points by applying the cumulative transformations
        for i in range(1, 6):
            self.points[i] = T_cumulative[i] @ self.points[0]

        # Calculate EE position and rotation
        self.EE_axes = T_cumulative[-1] @ np.array([0.075, 0.075, 0.075, 1])  # End-effector axes
        self.T_ee = T_cumulative[-1]  # Final transformation matrix for EE

        # Set the end effector (EE) position
        self.ee.x, self.ee.y, self.ee.z = self.points[-1][:3]
        
        # Extract and assign the RPY (roll, pitch, yaw) from the rotation matrix
        rpy = rotm_to_euler(self.T_ee[:3, :3])
        self.ee.rotx, self.ee.roty, self.ee.rotz = rpy[2], rpy[1], rpy[0]

        # Calculate the EE axes in space (in the base frame)
        self.EE = [self.ee.x, self.ee.y, self.ee.z]
        self.EE_axes = np.array([self.T_ee[:3, i] * 0.075 + self.points[-1][:3] for i in range(3)])
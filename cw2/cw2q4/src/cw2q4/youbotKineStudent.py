#!/usr/bin/env python

import numpy as np
from youbotKineBase import YoubotKinematicBase


class YoubotKinematicStudent(YoubotKinematicBase):
    def __init__(self):
        super(YoubotKinematicStudent, self).__init__(tf_suffix='student')

        # Set the offset for theta --> This was updated on 20/11/2021. Feel free to use your own code.
        youbot_joint_offsets = [170.0 * np.pi / 180.0,
                                -65.0 * np.pi / 180.0,
                                146 * np.pi / 180,
                                -102.5 * np.pi / 180,
                                -167.5 * np.pi / 180]

        # Apply joint offsets to dh parameters
        self.dh_params['theta'] = [theta + offset for theta, offset in
                                   zip(self.dh_params['theta'], youbot_joint_offsets)]

        # Joint reading polarity signs
        self.youbot_joint_readings_polarity = [-1, 1, 1, 1, 1]

    def forward_kinematics(self, joints_readings, up_to_joint=5):
        """This function solve forward kinematics by multiplying frame transformation up until a specified
        frame number. The frame transformation used in the computation are derived from dh parameters and
        joint_readings.
        Args:
            joints_readings (list): the state of the robot joints. In a youbot those are revolute
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematicks.
                Defaults to 5.
        Returns:
            np.ndarray: A 4x4 homogeneous tranformation matrix describing the pose of frame_{up_to_joint}
                w.r.t the base of the robot.
        """
        assert isinstance(self.dh_params, dict)
        assert isinstance(joints_readings, list), "joint readings of type " + str(type(joints_readings))
        assert isinstance(up_to_joint, int)
        assert up_to_joint >= 0
        assert up_to_joint <= len(self.dh_params['a'])

        T = np.identity(4)

	# --> This was updated on 20/11/2021. Feel free to use your own code.
        # Apply offset and polarity to joint readings (found in URDF file)
        joints_readings = [sign * angle for sign, angle in zip(self.youbot_joint_readings_polarity, joints_readings)]

        for i in range(up_to_joint):
            A = self.standard_dh(self.dh_params['a'][i],
                                 self.dh_params['alpha'][i],
                                 self.dh_params['d'][i],
                                 self.dh_params['theta'][i] + joints_readings[i])
            T = T.dot(A)

        assert isinstance(T, np.ndarray), "Output wasn't of type ndarray"
        assert T.shape == (4, 4), "Output had wrong dimensions"
        return T

    def get_jacobian(self, joint):
        """Given the joint values of the robot, compute the Jacobian matrix. Coursework 2 Question 4a.
        Reference - Lecture 5 slide 24.

        Args:
            joint (list): the state of the robot joints. In a youbot those are revolute

        Returns:
            Jacobian (numpy.ndarray): NumPy matrix of size 6x5 which is the Jacobian matrix.
        """
        assert isinstance(joint, list)
        assert len(joint) == 5

        # Your code starts here ----------------------------

        # For your solution to match the KDL Jacobian, 
        # z0 needs to be set [0, 0, -1] instead of [0, 0, 1], 
        # since that is how its defined in the URDF.
        # Both are correct.
        #SEE LAB 6 EXAMPLE ##################################################################
        jacobian = np.empty((6,5))
        T = []
        z_prev = [0,0,-1]
        o_prev = [0, 0 , 0]
        #Forward Kinematic to calculate transformation matrices
        for i in range(5):
            T.append(self.forward_kinematics(joint, i+1))
            Tr = T[i]
            p = Tr[3, 0:3]
            jacobian[0:3, i ] = np.cross(z_prev, (p - o_prev)) #J_P_i
            jacobian[3:6, i] = z_prev
            z_prev = Tr[0:3,2] # Third column of the rotation matrix R\
            o_prev = Tr[0:3,3] # First three elements of the fourth colum of 0Te
        #jacobian = [-l1*sin(theta1)- l2sin(theta12)- l3sin(theta123), ]
        # Your code ends here ------------------------------
        print(T)
    
        print(p)
        assert jacobian.shape == (6, 5)
        return jacobian

    def check_singularity(self, joint):
        """Check for singularity condition given robot joints. Coursework 2 Question 4c.
        Reference Lecture 5 slide 30.

        Args:
            joint (list): the state of the robot joints. In a youbot those are revolute

        Returns:
            singularity (bool): True if in singularity and False if not in singularity.

        """
        assert isinstance(joint, list)
        assert len(joint) == 5
        
        # Your code starts here ----------------------------
        # At singular configurations det(J) = 0
        if (self.jacobian(joint).shape[0] ==6 and self.jacobian(joint).shape[1] ==6):
            if (np.linalg.det(self.jacobian(joint))==0):
                singularity = True
            else:
                singularity = False
        elif(self.jacobian(joint).shape[0] == 6 and self.jacobian(joint).shape[1] != 6):
            if (np.linalg.det((self.jacobian(joint)).T * self.jacobian(joint) ) == 0):
                singularity = True
            else:
                singularity = False

        # Your code ends here ------------------------------

        assert isinstance(singularity, bool)
        return singularity

if __name__ == '__main__':
    youbot_kine = YoubotKinematicStudent()
    print(youbot_kine.get_jacobian([1,2,3,4,5]))
    
    
    
    
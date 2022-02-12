#!/usr/bin/env python

import numpy as np
from iiwa14DynBase import Iiwa14DynamicBase
import rospy

class Iiwa14DynamicRef(Iiwa14DynamicBase):
    def __init__(self):
        super(Iiwa14DynamicRef, self).__init__(tf_suffix='ref')

    def forward_kinematics(self, joints_readings, up_to_joint=7):
        """This function solve forward kinematics by multiplying frame transformation up until a specified
        joint. Reference Lecture 9 slide 13.
        Args:
            joints_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematics.
                Defaults to 7.
        Returns:
            np.ndarray The output is a numpy 4*4 matrix describing the transformation from the 'iiwa_link_0' frame to
            the selected joint frame.
        """

        assert isinstance(joints_readings, list), "joint readings of type " + str(type(joints_readings))
        assert isinstance(up_to_joint, int)

        T = np.identity(4)
        # iiwa base offset
        T[2, 3] = 0.1575

        # 1. Recall the order from lectures. T_rot_z * T_trans * T_rot_x * T_rot_y. You are given the location of each
        # joint with translation_vec, X_alpha, Y_alpha, Z_alpha. Also available are function T_rotationX, T_rotation_Y,
        # T_rotation_Z, T_translation for rotation and translation matrices.
        # 2. Use a for loop to compute the final transformation.
        for i in range(0, up_to_joint):
            T = T.dot(self.T_rotationZ(joints_readings[i]))
            T = T.dot(self.T_translation(self.translation_vec[i, :]))
            T = T.dot(self.T_rotationX(self.X_alpha[i]))
            T = T.dot(self.T_rotationY(self.Y_alpha[i]))

        assert isinstance(T, np.ndarray), "Output wasn't of type ndarray"
        assert T.shape == (4, 4), "Output had wrong dimensions"

        return T

    def get_jacobian_centre_of_mass(self, joint_readings, up_to_joint=7):
        """Given the joint values of the robot, compute the Jacobian matrix at the centre of mass of the link.
        Reference - Lecture 9 slide 14.

        Args:
            joint_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame you want to compute the Jacobian.
            Defaults to 7.

        Returns:
            jacobian (numpy.ndarray): The output is a numpy 6*7 matrix describing the Jacobian matrix defining at the
            centre of mass of a link.
        """
        assert isinstance(joint_readings, list)
        assert len(joint_readings) == 7

        # Your code starts here ----------------------------
        # T07 * ROT * TRANS 
        jacobian = np.zeros((6,7)) #INitialise the jacobian
        T0Gi = self.forward_kinematics_centre_of_mass(joint_readings, up_to_joint)
        p   = T0Gi[0:3,3]
        T = []
        for i in range(up_to_joint):
            T.append(self.forward_kinematics(joint_readings, i))
            Tr = T[i]
            z_prev = Tr[0:3,2]
            p_prev = Tr[0:3,3]

            jacobian[0:3,i] = np.cross(z_prev, (p - p_prev))
            jacobian[3:6, i] = z_prev
        
        # Your code ends here ------------------------------

        assert jacobian.shape == (6, 7)
        return jacobian

    def forward_kinematics_centre_of_mass(self, joints_readings, up_to_joint=7):
        """This function computes the forward kinematics up to the centre of mass for the given joint frame.
        Reference - Lecture 9 slide 14.
        Args:
            joints_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematicks.
                Defaults to 5.
        Returns:
            np.ndarray: A 4x4 homogeneous transformation matrix describing the pose of frame_{up_to_joint} for the
            centre of mass w.r.t the base of the robot.
        """
        T= np.identity(4)
        T[2, 3] = 0.1575

        T = self.forward_kinematics(joints_readings, up_to_joint-1)
        T = T.dot(self.T_rotationZ(joints_readings[up_to_joint-1]))
        T = T.dot(self.T_translation(self.link_cm[up_to_joint-1, :]))

        return T

    def get_B(self, joint_readings):
        """Given the joint positions of the robot, compute inertia matrix B.
        Args:
            joint_readings (list): The positions of the robot joints.

        Returns:
            B (numpy.ndarray): The output is a numpy 7*7 matrix describing the inertia matrix B.
        """
        B = np.zeros((7, 7))
        
	    # Your code starts here ------------------------------
        #Lecture 9slide 16
       
        for i in range(1,8): 
            #i+1
            #Get transformation R from T
            R0G = self.forward_kinematics_centre_of_mass(joint_readings, i)[0:3, 0:3]
            
            Ioli = np.zeros((3,3))
            for j in range(3):
                Ioli[j,j] = self.Ixyz[i-1, j]
            Ioli = np.matmul(np.matmul(R0G,Ioli),R0G.T)

            mli = self.mass[i - 1]
            Jcm = self.get_jacobian_centre_of_mass(joint_readings, i)
            Jpli = Jcm[0:3, :]
            Joli = Jcm[3:6, :]
        
            
            B += mli * np.matmul(Jpli.T , Jpli) + np.matmul(np.matmul(Joli.T,Ioli),  Joli)
            
        # Your code ends here ------------------------------
        
        return B

    def get_C_times_qdot(self, joint_readings, joint_velocities):
        """Given the joint positions and velocities of the robot, compute Coriolis terms C.
        Args:
            joint_readings (list): The positions of the robot joints.
            joint_velocities (list): The velocities of the robot joints.

        Returns:
            C (numpy.ndarray): The output is a numpy 7*1 matrix describing the Coriolis terms C times joint velocities.
        """
        assert isinstance(joint_readings, list)
        assert len(joint_readings) == 7
        assert isinstance(joint_velocities, list)
        assert len(joint_velocities) == 7

        # Your code starts here ------------------------------
        #L9S19
        # C = h_ijk qdot_k
        B = self.get_B(joint_readings)
        C = np.zeros((7,7))
        
        delta = 0.001 # small delta to approximate pde well
        for i in range(7):
            for j in range(7):
                for k in range(7):
                    q_k_copy = np.copy(joint_readings)
                    q_k_copy[k] += delta
                    bij = self.get_B(q_k_copy.tolist() ) # do this sum elementwise!

                    q_i_copy = np.copy(joint_readings)
                    q_i_copy[i] += delta
                    #bjk = self.get_B(joint_readings[i] + delta )
                    bjk = self.get_B(q_i_copy.tolist() )
                    
                    dbij_dq = (bij[i,j] - B[i,j]) / delta
                    dbjk_dq = (bjk[j,k] - B[j,k]) / delta
                    
                    C[i,j] = (dbij_dq - (0.5 * dbjk_dq)) * joint_velocities[k]
                    #sum_j (sum_k(h_ijk *q_dot_k *q_dot_j))     
                    #print(str(type(C)))              
        C = np.matmul(C , np.array(joint_velocities))
        #print(C.shape)
        # Your code ends here ------------------------------
        
        assert isinstance(C, np.ndarray)
        assert C.shape == (7,)
        return C

    def get_G(self, joint_readings):
        """Given the joint positions of the robot, compute the gravity matrix g.
        Args:
            joint_readings (list): The positions of the robot joints.

        Returns:
            G (numpy.ndarray): The output is a numpy 7*1 numpy array describing the gravity matrix g.
        """
        assert isinstance(joint_readings, list)
        assert len(joint_readings) == 7
        
        # Your code starts here ------------------------------
        #l9s17
        G = np.zeros((7,))
        g = [0,0,self.g]
        g = np.array(g)
        delta = 0.001
        for i in range(7):
            G_i = 0
            for j in range(7):
                mli = self.mass[j]
                #change in pe = force 
                J_l = self.get_jacobian_centre_of_mass(joint_readings, j+1)[0:3,i]
                J_l = J_l.reshape(3,1)
                G_i += mli * np.matmul(np.array(g), J_l) 
                
            G[i] = G_i

        # Your code ends here ------------------------------

        assert isinstance(g, np.ndarray)
        assert G.shape == (7,)
        return G


mine = Iiwa14DynamicRef()
joint_readings = [1,1,1,1,1,1,1]
joint_velocities = [1,2,3,4,5,6,7]
#print(KDL.get_jacobian_centre_of_mass(joint_readings, up_to_joint=7))
print(mine.get_C_times_qdot(joint_readings, joint_velocities))
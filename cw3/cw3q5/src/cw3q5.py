#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
from scipy.linalg import expm
from scipy.linalg import logm
from scipy.linalg import inv
import rospy
import rosbag
import rospkg
import PyKDL
from visualization_msgs.msg import Marker
from itertools import permutations
import matplotlib.pyplot as plt
#from cw3q2 import iiwa14DynStudent, iiwa14DynBase, iiwa14DynKDL
from sensor_msgs.msg import JointState
#from cw3q2.iiwa14DynStudent import Iiwa14DynamicRef
from cw3q2.iiwa14DynKDL import Iiwa14DynamicKDL
#from cw3q2.iiwa14DynBase import Iiwa14DynamicBase
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# In this question, you are tasked with computing the joint accelerations throughout the tra-
# jectory defined in the bagfile ”cw3q5/bag/cw3q5.bag” using dynamic components. You
# should also plot the computed joint accelerations. You only need to edit the ”cw3q5.py”
# file, and the corresponding launch file. Note that a coding template is not provided for
# this question. For the dynamic components, you can use either your own implementation
# from Q2, or the corresponding KDL class.


class iiwaTrajectoryPlanning(object):


    def traj(self):               
        '''Import from bagfile and publish the trajectory to the appropriate topic to see 
        the robot moving in simulation.''' 
        #Open bagfile
        joint_traj  = JointTrajectory()
        #args  in trajectory_msgs joint_traj are ['header', 'joint_names', 'points']
        #Header
        joint_traj.header.stamp = rospy.Time.now()
        #Get bag path
        path = rospack.get_path('cw3q5') + '/bag/cw3q5.bag'
        #Open bag
        bag = rosbag.Bag(path)
        #Read messages in bagfile from specific topic
        for topic, msg, t in bag.read_messages(topics=['/iiwa/EffortJointInterface_trajectory_controller/command']):
            #trajectory_msgs/JointTrajectoryPoint has attributes positions, velocities, accelerations, effort, duration time_from_start
            
            # Get joint names
            for name in msg.joint_names:
                #Joint names
                joint_traj.joint_names.append(name)
                #Points
            for points in msg.points:
                points_obj = JointTrajectoryPoint()
                for i in range(7): 
                    #positions
                    points_obj.positions.append(points.positions[i])
                    #velocities
                    points_obj.velocities.append(points.velocities[i])
                    #accelerations
                    points_obj.accelerations.append(points.accelerations[i])
                    #duration
                    points_obj.time_from_start = points.time_from_start
                joint_traj.points.append(points_obj)
        #Close bag
        bag.close()
        return joint_traj

    def acceleration(self,joint_state): 
        '''Subscribe to the appropriate topic, 
        and calculate the joint accelerations throughout
        the trajectory using dynamic components'''
        #Import position, velocity, torque
        #print(joint_state)
        #SUbscribe to the joint_traj message and get position, velcoities, and torques
        
        q = np.array(joint_state.position)
        q_dot = np.array(joint_state.velocity)
        tau = np.array(joint_state.effort)

        #Get B using KDL incase of any mistakes in Q2
        B = Iiwa14DynamicKDL.get_B(Iiwa14DynamicKDL(), q)
        #Get C_times_q_dot
        C_qdot = Iiwa14DynamicKDL.get_C_times_qdot(Iiwa14DynamicKDL(), q , q_dot)
        #Get G
        G = Iiwa14DynamicKDL.get_G(Iiwa14DynamicKDL(), q)
        #Calculate accleration q_ddot
        q_ddot = np.matmul(np.linalg.inv(B), (tau - C_qdot - G)) 
        q_ddot = q_ddot.reshape((7,))
        
        
        stamp = joint_state.header.stamp
        time = stamp.secs + stamp.nsecs *1e-9
        plt.plot(time, q_ddot[:,0], 'k-')
        plt.plot(time, q_ddot[:,1], 'r+')
        plt.plot(time, q_ddot[:,2], 'b*')
        plt.plot(time, q_ddot[:,3], 'g-')
        plt.plot(time, q_ddot[:,4], 'k*')
        plt.plot(time, q_ddot[:,5], 'r*')
        plt.plot(time, q_ddot[:,6], 'g*')
        plt.legend()
        plt.title('Joint Accelerations')
        plt.xlabel('Time')
        plt.ylabel('Acceleration')
        plt.draw
        plt.pause(1e-5)
            

        return q_dot, q_ddot #Outputs necessary?


if __name__ == '__main__':
    try:        
        counter = 0
        counter +=10
        #Initialise node
        rospy.init_node('cw3', anonymous=True)
        

        iiwa_planner = iiwaTrajectoryPlanning()
        rospack = rospkg.RosPack()
        #rospy.spin()

        #Get joint_traj from bagfile
        joint_trajectory  = JointTrajectory()
        trajectory_publisher = rospy.Publisher('/iiwa/EffortJointInterface_trajectory_controller/command', JointTrajectory, queue_size= 5)
        #Calculate Joint trajectories
        joint_traj  = iiwa_planner.traj() #TODO 

        #Artificial delay
        rospy.sleep(1)
        #Publish joint trajectories
        trajectory_publisher.publish(joint_traj)
        #Artificial delay
        rospy.sleep(1)
        
        #Calculate Acceleration
        #Subscribe to the topic to get postion and velocities.
        state_subscriber = rospy.Subscriber('/iiwa/joint_states', JointState, iiwa_planner.acceleration)# subscriber

        plt.ion
        plt.show()
        rospy.spin()
        
        
    except rospy.ROSInterruptException:
        pass

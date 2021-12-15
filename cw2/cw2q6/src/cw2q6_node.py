#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from scipy.linalg import logm
from scipy.linalg import inv
import rospy
import rosbag
import rospkg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cw2q4.youbotKineKDL import YoubotKinematicKDL

import PyKDL
from visualization_msgs.msg import Marker
from itertools import permutations


class YoubotTrajectoryPlanning(object):
    def __init__(self):
        # Initialize node
        rospy.init_node('youbot_traj_cw2', anonymous=True)

        # Save question number for check in main run method
        self.kdl_youbot = YoubotKinematicKDL()

        # Create trajectory publisher and a checkpoint publisher to visualize checkpoints
        self.traj_pub = rospy.Publisher('/EffortJointInterface_trajectory_controller/command', JointTrajectory,
                                        queue_size=5)
        self.checkpoint_pub = rospy.Publisher("checkpoint_positions", Marker, queue_size=100)

    def run(self):
        """This function is the main run function of the class. When called, it runs question 6 by calling the q6()
        function to get the trajectory. Then, the message is filled out and published to the /command topic.
        """
        print("run q6a")
        rospy.loginfo("Waiting 5 seconds for everything to load up.")
        rospy.sleep(2.0)
        traj = self.q6()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
        self.traj_pub.publish(traj)

    def q6(self):
        """ This is the main q6 function. Here, other methods are called to create the shortest path required for this
        question. Below, a general step-by-step is given as to how to solve the problem.
        Returns:
            traj (JointTrajectory): A list of JointTrajectory points giving the robot joint positions to achieve in a
            given time period.
        """
        # Steps to solving Q6.
        # 1. Load in targets from the bagfile (checkpoint data and target joint positions).
        # 2. Compute the shortest path achievable visiting each checkpoint Cartesian position.
        # 3. Determine intermediate checkpoints to achieve a linear path between each checkpoint and have a full list of
        #    checkpoints the robot must achieve. You can publish them to see if they look correct. Look at slides 39 in lecture 7
        # 4. Convert all the checkpoints into joint values using an inverse kinematics solver.
        # 5. Create a JointTrajectory message.

        # Your code starts here ------------------------------
        # TODO
        #Create object (not necessary) youbot_traj_plan = YoubotTrajectoryPlanning()

        #Load targets from bagfile
        [target_cart_tf, target_joint_positions] = self.load_targets()

        #Sort targets to find shortest path
        [sorted_order, min_dist, index_shortest_dist] = self.get_shortest_path(target_cart_tf)

        #FInd intermediate points between checkpoints to ensure straight line path
        #num_points = 5, 5 intermediate points between checkpoints, for smooth straight movement 
        full_checkpoint_tfs = self.intermediate_tfs(index_shortest_dist, target_cart_tf, 5)
        
        #This function gets a np.ndarray of transforms and publishes them in a color coded fashion to show how the
        #Cartesian path of the robot end-effector.
        self.publish_traj_tfs(full_checkpoint_tfs)

        #This function converts checkpoint transformations (including intermediates) into joint positions
        init_joint_position = np.array(target_joint_positions[:,0])
        q_checkpoints = self.full_checkpoints_to_joints(full_checkpoint_tfs, init_joint_position) #What is init_joint_position in this?
        
        traj = JointTrajectory()
        dt = 2
        t = 10
        for i in range(q_checkpoints.shape[1]): 
            traj_point = JointTrajectoryPoint()
            traj_point.positions = q_checkpoints[:, i]
            t += dt
            traj_point.time_from_start.secs = t
            traj.points.append(traj_point)

        #This function converts joint positions to a kdl array
        #kdl_array = self.list_to_kdl_jnt_array(q_checkpoints) # is this traj??no
        
        # Your code ends here ------------------------------
        
        assert isinstance(traj, JointTrajectory)
        return traj

    def load_targets(self):
        """This function loads the checkpoint data from the 'data.bag' file. In the bag file, you will find messages
        relating to the target joint positions. You need to use forward kinematics to get the goal end-effector position.
        Returns:
            target_cart_tf (4x4x5 np.ndarray): The target 4x4 homogenous transformations of the checkpoints found in the
            bag file. There are a total of 5 transforms (4 checkpoints + 1 initial starting cartesian position).
            target_joint_positions (5x5 np.ndarray): The target joint values for the 4 checkpoints + 1 initial starting
            position.
        """
        # Defining ros package path
        rospack = rospkg.RosPack()
        path = rospack.get_path('cw2q6')

        # Initialize arrays for checkpoint transformations and joint positions
        target_joint_positions = np.zeros((5, 5))
        # Create a 4x4 transformation matrix, then stack 6 of these matrices together for each checkpoint
        target_cart_tf = np.repeat(np.identity(4), 5, axis=1).reshape((4, 4, 5))

        # Load path for selected question
        bag = rosbag.Bag(path + '/bags/data.bag')
        # Get the current starting position of the robot
        target_joint_positions[:, 0] = self.kdl_youbot.kdl_jnt_array_to_list(self.kdl_youbot.current_joint_position)
        # Initialize the first checkpoint as the current end effector position
        target_cart_tf[:, :, 0] = self.kdl_youbot.forward_kinematics(target_joint_positions[:, 0])

        # Your code starts here ------------------------------
        #if len(sys.argv) != 2:
        #    sys.stderr.write('[ERROR] This script only takes input bag file as argument.n')
        #else:
        #    inputFileName = sys.argv[1]
        #    print "[OK] Found bag: %s" % inputFileName

        topicList = []
        i = 1
        for topic, msgs, t in bag.read_messages(['joint_data']):
            target_joint_positions[:,i] = msgs.position
            target_cart_tf[:,:,i] = self.kdl_youbot.forward_kinematics(target_joint_positions[:,i], 5)
            i+=1 
            my_pt = JointTrajectoryPoint()
            if topicList.count(topic) == 0:
                topicList.append(topic)
        #print '{0} topics found:'.format(len(topicList))
       

        #print(target_cart_tf)

        # Your code ends here ------------------------------

        # Close the bag
        bag.close()

        assert isinstance(target_cart_tf, np.ndarray)
        assert target_cart_tf.shape == (4, 4, 5)
        assert isinstance(target_joint_positions, np.ndarray)
        assert target_joint_positions.shape == (5, 5)

        return target_cart_tf, target_joint_positions

    def get_shortest_path(self, checkpoints_tf):
        """This function takes the checkpoint transformations and computes the order of checkpoints that results
        in the shortest overall path.
        Args:
            checkpoints_tf (np.ndarray): The target checkpoint 4x4 transformations.
        Returns:
            sorted_order (np.array): An array of size 5 indicating the order of checkpoint
            min_dist:  (float): The associated distance to the sorted order giving the total estimate for travel
            distance.
        """

        # Your code starts here ------------------------------
        #Calculate the distance between all points, then choose the shortest distances in the cost matrix 
        #print(checkpoints_tf.shape)
        checkpoints = []
        perm = permutations(checkpoints_tf)
        for i in range(checkpoints_tf.shape[2]):
            
            #checkpoints[i] = checkpoints_tf[0:3, 3, i]
            #print(checkpoints_tf[0:3,3])
            checkpoints.append(checkpoints_tf[0:3, 3, i])
        # get checkpoint coordinates from checkpoint transformation matrix, rows 1-3 of last column
        
        # Calculate cost matrix, distance between all n points, giving n x n matrix 
        checkpoints= np.array(checkpoints)
        cost_matrix = np.zeros((checkpoints.shape[0], checkpoints.shape[0]))
        for i in range(checkpoints.shape[0]):
            for j in range(checkpoints.shape[0]):
                cost_matrix[i,j] = np.sqrt((checkpoints[i][0] - checkpoints[j][0])**2 + (checkpoints[i][1] - checkpoints[j][1])**2 + (checkpoints[i][2] - checkpoints[j][2])**2)
                #Make diagonals infinite so that distance between one point and itself isnt chosen
                cost_matrix[i,i] = np.inf
                # distance between each cartesian point
        # Find shortest path using Greedy algorithm
        index_shortest_dist = []
        shortest_dist = cost_matrix[:,i].min() # get minimum in each column ( shortest distance from first point) and next etc 
        index = np.argmin(cost_matrix[:,1])
        index_shortest_dist.append(index)
        i = 0 
        min_dist = 0
        while (i<6):
        #for i in range(1,5):
            shortest_dist = cost_matrix[:,index].min() # get minimum in each column ( shortest distance from first point) and next etc 
            index = np.argmin(cost_matrix[:,index])
            index_shortest_dist.append(index) # add the index of the shortest distance 
            cost_matrix[index,:] = np.inf #remove previous row from next loop by making distance infinite
            min_dist += shortest_dist # Add each shortest dist to get total min dist 
            i+=1

        #Sort checkpoints into order dictated by index_shortest_dist
        sorted_order = []
        for i in range(5):
            sorted_order.append(checkpoints[index_shortest_dist[i]])
        # this will Append  and sort checkpoints in order of shortest path


        # Your code ends here ------------------------------

        #assert isinstance(sorted_order, np.ndarray)
        #assert sorted_order.shape == (5,)
        assert isinstance(min_dist, float)
        #return sorted_order 
        return sorted_order, min_dist, index_shortest_dist

    def publish_traj_tfs(self, tfs):
        """This function gets a np.ndarray of transforms and publishes them in a color coded fashion to show how the
        Cartesian path of the robot end-effector.
        Args:
            tfs (np.ndarray): A array of 4x4xn homogenous transformations specifying the end-effector trajectory.
        """
        id = 0
        

        for i in range(0, tfs.shape[2]): # full_checkpoint_tfs wrong type? CHanged from shape[2] to len()
            marker = Marker()
            marker.id = id
            id += 1
            marker.header.frame_id = 'base_link'
            marker.header.stamp = rospy.Time.now()
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0 + id * 0.05
            marker.color.b = 1.0 - id * 0.05
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = tfs[0,-1, i]
            marker.pose.position.y = tfs[1,-1,i] 
            marker.pose.position.z = tfs[2,-1, i]
            self.checkpoint_pub.publish(marker)

    def intermediate_tfs(self, sorted_checkpoint_idx, target_checkpoint_tfs, num_points):
        """This function takes the target checkpoint transforms and the desired order based on the shortest path sorting, 
        and calls the decoupled_rot_and_trans() function.
        Args:
            sorted_checkpoint_idx (list): List describing order of checkpoints to follow.
            target_checkpoint_tfs (np.ndarray): the state of the robot joints. In a youbot those are revolute
            num_points (int): Number of intermediate points between checkpoints.
        Returns:
            full_checkpoint_tfs: 4x4x(4xnum_points+5) homogeneous transformations matrices describing the full desired
            poses of the end-effector position.
        """

        # Your code starts here ------------------------------
        #TODO
        full_checkpoint_tfs= np.repeat(np.identity(4), (4* num_points) +5, axis=1).reshape((4, 4, (4* num_points) +5))
        
        full_checkpoint_tfs[:,:,0] = target_checkpoint_tfs[:,:,0]

        #full_checkpoint_tfs= [target_checkpoint_tfs[0]]
        sorted_checkpoint_tfs = []
        #print(target_checkpoint_tfs)
        for i in range(5):
            sorted_checkpoint_tfs.append(target_checkpoint_tfs[:,:,sorted_checkpoint_idx[i]])
        #print(sorted_checkpoint_tfs[2].shape)
        for i in range(1,4): 
            full_checkpoint_tfs[:,:,i] = (self.decoupled_rot_and_trans(sorted_checkpoint_tfs[i], sorted_checkpoint_tfs[i+1], num_points))
            full_checkpoint_tfs[:,:,i+1] = (target_checkpoint_tfs[:,:,i+1])# append target after initial point, between intermediate points.
        
        #print(len(full_checkpoint_tfs))

        # Your code ends here ------------------------------
       
        return full_checkpoint_tfs

    def decoupled_rot_and_trans(self, checkpoint_a_tf, checkpoint_b_tf, num_points):
        """This function takes two checkpoint transforms and computes the intermediate transformations
        that follow a straight line path by decoupling rotation and translation.
        Args:
            checkpoint_a_tf (np.ndarray): 4x4 transformation describing pose of checkpoint a.
            checkpoint_b_tf (np.ndarray): 4x4 transformation describing pose of checkpoint b.
            num_points (int): Number of intermediate points between checkpoint a and checkpoint b.
        Returns:
            tfs: 4x4x(num_points) homogeneous transformations matrices describing the full desired
            poses of the end-effector position from checkpoint a to checkpoint b following a linear path.
        """

        # Your code starts here ------------------------------
        # tfs = combined rot and trans
        t = 1.0 / (num_points + 1)
        a_rot = np.empty([2,2])
        b_rot = np.empty([2,2])
        a_trans = []
        b_trans = []
        #print('hello')
        #print(checkpoint_a_tf)
        for i in range(2):
            for j in range(2):
                a_rot[i,j] = checkpoint_a_tf[i,j]#[0:2,0:2]
                b_rot[i,j] = checkpoint_b_tf[i,j]#[0:2,0:2]
                
        for i in range(3):
            b_trans.append(checkpoint_b_tf[i, -1])
            a_trans.append(checkpoint_a_tf[i, -1])
        c = [b_trans - a_trans for b_trans, a_trans in zip(b_trans, a_trans)]
        c = np.array(c)
        c = c.reshape((3,1))
        trans = a_trans + t * c#(b_trans - a_trans)
        #trans = a_trans + t * (b_trans - a_trans)
               
        rot = np.matmul(a_rot,expm((logm(np.matmul(np.linalg.inv(a_rot) , b_rot))) * t))
        tfs = np.empty([4,4])
        for i in range(2):
            for j in range(2):
                tfs[i,j] = rot[i,j]
        #tfs[0:3,0:3] = rot
        for i in range(3):
            tfs[i, 3] = trans[i] # for loop?? # should be 3x1
        tfs[3,:] = [0,0,0,1]
        tfs = np.array(tfs)
        #Combine back into one matrix 
        # Your code ends here ------------------------------

        return tfs

    def full_checkpoints_to_joints(self, full_checkpoint_tfs, init_joint_position):
        """This function takes the full set of checkpoint transformations, including intermediate checkpoints, 
        and computes the associated joint positions by calling the ik_position_only() function.
        Args:
            full_checkpoint_tfs (np.ndarray, 4x4xn): 4x4xn transformations describing all the desired poses of the end-effector
            to follow the desired path. (4x4x(4xnum_points+5))
            init_joint_position (np.ndarray):A 5x1 array for the initial joint position of the robot.
        Returns:
            q_checkpoints (np.ndarray, 5xn): For each pose, the solution of the position IK to get the joint position
            for that pose.
        """
        
        # Your code starts here ------------------------------
        q_checkpoints = []
        iter_count = 0
        q = init_joint_position
        for i in range(full_checkpoint_tfs.shape[2]):
            error = 10 # reset error to large for each point.
            while (error >= 0.1):
                [q, error] = self.ik_position_only(full_checkpoint_tfs[:,:,i], q, alpha = 0.1)
                iter_count += 1
                
                if (iter_count > 10000):
                    break
            q_checkpoints.append(q) # CHeck the indexing of full_checjpoint_tfs
        q_checkpoints = np.array(q_checkpoints)
            #Append position only inverse kinematic solution to the q_checkpoints, taking one sheet of 3d matrix full_checkpoint_tfs at once.
        # Your code ends here ------------------------------

        return q_checkpoints

    def ik_position_only(self, pose, q0, alpha = 0.1):
        """This function implements position only inverse kinematics.
        Args:
            pose (np.ndarray, 4x4): 4x4 transformations describing the pose of the end-effector position.
            q0 (np.ndarray, 5x1):A 5x1 array for the initial starting point of the algorithm.
        Returns:
            q (np.ndarray, 5x1): The IK solution for the given pose.
            error (float): The Cartesian error of the solution.
        """
        # Some useful notes:
        # We are only interested in position control - take only the position part of the pose as well as elements of the
        # Jacobian that will affect the position of the error.

        # Your code starts here ------------------------------
        Pd = pose[:3, 3].ravel()
        q = q0
        q = np.array(q)
        q0 = np.array(q0)
        J = self.kdl_youbot.get_jacobian(q0)[:3, :]
        J= np.array(J)
        # Take only first 3 rows as position only solution.
        P = np.array(self.kdl_youbot.forward_kinematics(q0))[:3, -1]
        e = Pd - P.ravel()
        e = np.array(e)
        
        q += alpha * np.matmul(J.T, e)
        error = np.linalg.norm(e)
        # Your code ends here ------------------------------

        return q, error

    @staticmethod
    def list_to_kdl_jnt_array(joints):
        """This converts a list to a KDL jnt array.
        Args:
            joints (joints): A list of the joint values.
        Returns:
            kdl_array (PyKDL.JntArray): JntArray object describing the joint position of the robot.
        """
        kdl_array = PyKDL.JntArray(5)
        for i in range(0, 5):
            kdl_array[i] = joints[i]
        return kdl_array


if __name__ == '__main__':
    try:
        youbot_planner = YoubotTrajectoryPlanning()
        youbot_planner.run()
        rospy.spin()
        #h = self.list_to_kdl_jnt_array(joints)
        #print(h)
    except rospy.ROSInterruptException:
        pass

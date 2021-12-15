#! /usr/bin/env python

import rospy
import numpy as np
import random
import time

#TODO: import the SRV file from its corresponding folder, as well as its Request
from cw1q4.srv import quat2rodrigues
from cw1q4.srv import quat2rodriguesRequest
from cw1q4.srv import quat2zyx
from cw1q4.srv import quat2zyxRequest
import math




def point_rotation_client():
    "rospy.wait_for_service('quat2zyx')"
    rospy.wait_for_service('quat2rodrigues')

    while not rospy.is_shutdown():
        #TODO: Initialise the ROS service client. It takes two arguments: The name of the service, and the service definition.
        "client_euler = rospy.ServiceProxy('quat2zyx', quat2zyx)"
        client_axisAngle = rospy.ServiceProxy('quat2rodrigues', quat2rodrigues)
        
        req = quat2rodriguesRequest()
        "req2 = quat2zyxRequest()"

        #TODO: create a random request point, and a random request quaternion
        quaternion = np.random.rand(4)
        quaternion = quaternion / np.linalg.norm(quaternion)

        """req2.q.x = quaternion[0]
        req2.q.y = quaternion[1]
        req2.q.z = quaternion[2]
        req2.q.w = quaternion[3]
        """

        req.q.x = quaternion[0]
        req.q.y = quaternion[1]
        req.q.z = quaternion[2]
        req.q.w = quaternion[3]
        
        

        "print(req2)"
        print(req)

        """
        res2 = client_euler(req2)
        print('euler angle is:')
        print(res2)
        """
        

        
        res = client_axisAngle(req)
        print('axis angle is:')
        print(res)
        

        time.sleep(3)


if __name__ == '__main__':
    try:
        point_rotation_client()
    except rospy.ROSInterruptException:
        pass

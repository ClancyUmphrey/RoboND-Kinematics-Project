#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

#-----------------------------------------------------------------------------------------
# Load My Extra Modules
#-----------------------------------------------------------------------------------------
from kinematics import eval_ee, eval_theta
import numpy as np
from kuka_arm.msg import Float64List
#-----------------------------------------------------------------------------------------

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here

        #---------------------------------------------------------------------------------
        # See kinematics.py for the FK implementation. 
        #---------------------------------------------------------------------------------

        ###

        # Initialize service response
        joint_trajectory_list = []
        ee_error = np.zeros(len(req.poses))
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	        # Extract end-effector position and orientation from request
	        # px,py,pz = end-effector position
	        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 

            #-----------------------------------------------------------------------------
            # See kinematics.py for the IK implementation.
            #-----------------------------------------------------------------------------

            #-----------------------------------------------------------------------------
            # Get Thetas
            #-----------------------------------------------------------------------------
            theta = [None for _ in xrange(7)]
            for i in xrange(1,4):
                theta[i] = eval_theta[i](pitch,yaw,px,py,pz)
            for i in xrange(4,7):
                theta[i] = eval_theta[i](roll,pitch,yaw,theta[1],theta[2],theta[3])

            #-----------------------------------------------------------------------------
            # Get End Effector Error 
            #-----------------------------------------------------------------------------
            ee_error[x] = np.linalg.norm(eval_ee(*theta[1:]) - np.array([[px],[py],[pz]]))

            ###
		
            # Populate response for the IK request
            # In the next line replace theta1,...,theta6 by your joint angle variables
            joint_trajectory_point.positions = theta[1:]
            joint_trajectory_list.append(joint_trajectory_point)
            

        #-----------------------------------------------------------------------------
        # Publish End Effector Error 
        #-----------------------------------------------------------------------------
        pub.publish( Float64List(data=ee_error) )

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))

        return CalculateIKResponse(joint_trajectory_list)


def IK_server():

    #-------------------------------------------------------------------------------------
    # Setup Publisher
    #-------------------------------------------------------------------------------------
    global pub
    pub = rospy.Publisher('ee_error', Float64List, queue_size=2)
    #-------------------------------------------------------------------------------------

    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()

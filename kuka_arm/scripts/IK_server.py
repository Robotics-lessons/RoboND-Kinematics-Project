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
import numpy as np
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

# Create a Transformation matrix function
def create_T_matrics(alpha, a, d, q):
    T_matrics = Matrix([[	    cos(q),		-sin(q),	     0,		a],
		[sin(q)*cos(alpha),  cos(q)*cos(alpha),   -sin(alpha), -sin(alpha)*d],
		[sin(q)*sin(alpha),  cos(q)*sin(alpha),    cos(alpha),  cos(alpha)*d],
		[		 0,  		 0,   		0, 		 1]])
    return T_matrics

# Calculate IK angles
def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
	# Conversion Factors
	rtd = 180./np.pi # radians to degrees
	dtr = np.pi/180. # degrees to radians

	# Define a correction between Gripper Link and WC
    	R_z = Matrix([[	cos(np.pi),    -sin(np.pi),		0,	0],
			  [	sin(np.pi),	cos(np.pi),		0,	0],
			  [		 0,		 0,		1,	0],
			  [		 0,		 0,		0,	1]])

    	R_y = Matrix([[  cos(-np.pi/2),    		 0, sin(-np.pi/2),	0],
			  [		 0,		 1,		0,	0],
			  [ -sin(-np.pi/2),	         0, cos(-np.pi/2),	0],
			  [		 0,		 0,		0,	1]])	

	# Create a correction matrix
    	R_correct = simplify(R_z * R_y)

        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()
	    rospy.loginfo("x = %d and total len = %d" % (x , len(req.poses)))
            # Define DH param symbols
	    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
	    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
	    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
	               
            # Joint angle symbols
	    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 =  symbols('alpha0:7')
     
            # Modified DH params
   	    dh = {'alpha0':	    0,  'a0':	  0,  'd1':	 0.75, 
		'alpha1':    -np.pi/2,  'a1':   0.35, 'd2':     0, 'q2':   -np.pi/2,
		'alpha2':		    0,  'a2':   1.25, 'd3':     0, 
		'alpha3':    -np.pi/2,  'a3': -0.054, 'd4':  1.50, 
		'alpha4':     np.pi/2,  'a4':      0, 'd5':     0, 
		'alpha5':    -np.pi/2,  'a5':      0, 'd6':     0, 
		'alpha6':           0,  'a6':      0, 'd7': 0.303, 'q7':       0}

#	    rospy.loginfo("dh = %s" % dh)            
           
            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z    
	    rospy.loginfo("============== px = %.6f, py = %.6f, pz = %.6f," % (px, py, pz))  
	    # calculate roll, pitch, yaw value
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     	    rospy.loginfo("Received %s eef-poses.orientation from the plan" % req.poses[x].orientation)

            # Calculate joint angles using Geometric IK method

 
	    # Build Rrpy
	    R_roll = Matrix([[  1,	    0,	          0],
			   [	0,  cos(roll),   -sin(roll)],
			   [	0,  sin(roll),    cos(roll)]])
	    R_pitch = Matrix([[  cos(pitch),   0, sin(pitch)],
			   [		  0,   1,          0],
			   [	-sin(pitch),   0, cos(pitch)]])
	    R_yaw = Matrix([[  cos(yaw), -sin(yaw), 	0],
			   [   sin(yaw),  cos(yaw), 	0],
			   [	       0,   	 0,     1]])
#	    Rrpy = simplify(R_roll * R_pitch * R_yaw)
	    Rrpy = simplify(R_yaw * R_pitch * R_roll)

#	    rospy.loginfo("Rrpy = %s" % Rrpy) 

            # Apply the correction
	    Rrpy = simplify(Rrpy * R_correct[0:3,0:3])

	    # calculate n for x, y, z position

	    nx = Rrpy[0, 2]
    	    ny = Rrpy[1, 2]
    	    nz = Rrpy[2, 2]

   	    d6 = dh['d6']
    	    l =  dh['d7']

	    # calculate  wrist center x, y, z position
    	    Wx = px - (d6 + l) * nx 
    	    Wy = py - (d6 + l) * ny			
    	    Wz = pz - (d6 + l) * nz
            rospy.loginfo("Wx = %.4f answer = 2.1245, Wy = %.4f answer = 0.23555, Wz = %.4f answer = 2.3144" % (Wx, Wy, Wz))
#            rospy.loginfo("Wx = %.4f, Wy = %.4f , Wz = %.4f " % (Wx, Wy, Wz))
            # calculate theta 1	
	    theta1 = atan2(Wy, Wx)
#	    rospy.loginfo("theta1 = %.4f" % theta1)	   
	    s = Wz - dh['d1']
	    r = sqrt(Wx**2+Wy**2) - dh['a1'] 
	    rospy.loginfo("s = %s, r = %s" % (s, r))

	    # beta angle is the angle between Joint 2 and Joint 5:WC
   	    beta = atan2(s, r)
    	    distance_c = sqrt(r**2+s**2)
#	    print("d4 = %s" % dh['d4'])
    	    distance_a = sqrt(dh['d4']**2 + dh['a3']**2)
    	    distance_b = dh['a2'] 


#    	    alpha = atan2(distance_b + distance_a * cos(np.pi - theta3), distance_a * sin(np.pi - theta3))
            # calculate theta 2	
            Cos_alpha = (distance_c ** 2 + distance_b ** 2 - distance_a ** 2 ) / (2 * distance_c * distance_b)
            C_alpha = atan2(sqrt(abs(1 - Cos_alpha ** 2)), Cos_alpha)
    	    theta2 = np.pi/2 - beta - C_alpha

            # calculate theta 3	
    	    D = (distance_a ** 2 + distance_b ** 2 - distance_c ** 2 ) / (2 * distance_a * distance_b)
    	    theta3 = atan2(D, sqrt(abs(1 - D ** 2)))
#    	    theta3 = atan2(sqrt(abs(1 - D ** 2)), D)

            # set HD table q1 to q3	
	    dh['q1'] = theta1
	    dh['q2'] = theta2-np.pi/2
	    dh['q3'] = theta3

	    # DH table with q1, q2 and q3 value
	    rospy.loginfo("dh = %s" % dh) 
 
	    T0_1 = create_T_matrics(dh['alpha0'], dh['a0'], dh['d1'], dh['q1'])
	    R0_1 = T0_1.extract([0,1,2],[0,1,2])
	 #   rospy.loginfo("R0_1 = %s" % R0_1) 
	    T1_2 = create_T_matrics(dh['alpha1'], dh['a1'], dh['d2'], dh['q2'])
	    R1_2 = T1_2.extract([0,1,2],[0,1,2])
	 #   rospy.loginfo("R1_2 = %s" % R1_2) 
	    T2_3 = create_T_matrics(dh['alpha2'], dh['a2'], dh['d3'], dh['q3'])
	    R2_3 = T2_3.extract([0,1,2],[0,1,2])
	#    rospy.loginfo("R2_3 = %s" % R2_3) 

	    # create R0_3
	    R0_3 = simplify(R0_1 * R1_2 * R2_3)
	#    rospy.loginfo("R0_3 = %s" % R0_3) 

	    # Calculate R3_6 using inv(R0_3) * Rrpy
	    R3_6 = simplify(R0_3.inv() * Rrpy)

#	    rospy.loginfo("R3_6 = %s" % R3_6) 

	    beta = atan2(-R3_6[1,2], sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]))
#	    rospy.loginfo("beta = %s" % beta) 
	    gamma = atan2(R3_6[1,1], R3_6[1,0])
#	    rospy.loginfo("gamma = %s" % gamma) 
	    alpha = atan2(R3_6[2,2], R3_6[0,2])
#	    rospy.loginfo("alpha = %s" % alpha) 
#	    rospy.loginfo("--------------------------------------------") 
	    theta4 = alpha
	    theta5 = beta
	    theta6 = gamma

#	    rospy.loginfo("theta1 = %.4f" % theta1)
#	    rospy.loginfo("theta2 = %.4f" % theta2)
#	    rospy.loginfo("theta3 = %.4f" % theta3)
#	    rospy.loginfo("theta4 = %.4f" % theta4)
#	    rospy.loginfo("theta5 = %.4f" % theta5)
#	    rospy.loginfo("theta6 = %.4f" % theta6)

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()

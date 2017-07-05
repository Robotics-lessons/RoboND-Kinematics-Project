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


def handle_calculate_IK():
    print("Start testing")
 
    joint_trajectory_list = []
	# Conversion Factors
    rtd = 180./np.pi # radians to degrees
    dtr = np.pi/180. # degrees to radians

        # IK code starts here
    joint_trajectory_point = JointTrajectoryPoint()

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
        
            # Define Modified DH Transformation matrix
    T0_1 = Matrix([[		    cos(q1),		 -sin(q1),		0,		a0],
			   [	sin(q1)*cos(alpha0),  cos(q1)*cos(alpha0),   -sin(alpha0), -sin(alpha0)*d1],
			   [	sin(q1)*sin(alpha0),  cos(q1)*sin(alpha0),    cos(alpha0),  cos(alpha0)*d1],
			   [			  0,  		 	0,   		0, 		 1]])

    T1_2 = Matrix([[		    cos(q2),		 -sin(q2),		0,		a1],
			   [	sin(q2)*cos(alpha1),  cos(q2)*cos(alpha1),   -sin(alpha1), -sin(alpha1)*d2],
			   [	sin(q2)*sin(alpha1),  cos(q2)*sin(alpha1),    cos(alpha1),  cos(alpha1)*d2],
			   [			  0,  		 	0,   		0, 		 1]])

    T2_3 = Matrix([[		    cos(q3),		 -sin(q3),		0,		a2],
			   [	sin(q3)*cos(alpha2),  cos(q3)*cos(alpha2),   -sin(alpha2), -sin(alpha2)*d3],
			   [	sin(q3)*sin(alpha2),  cos(q3)*sin(alpha2),    cos(alpha2),  cos(alpha2)*d3],
			   [			  0,  		 	0,   		0, 		 1]])

    T3_4 = Matrix([[		    cos(q4),		 -sin(q4),		0,		a3],
			   [	sin(q4)*cos(alpha3),  cos(q4)*cos(alpha3),   -sin(alpha3), -sin(alpha3)*d4],
			   [	sin(q4)*sin(alpha3),  cos(q4)*sin(alpha3),    cos(alpha3),  cos(alpha3)*d4],
			   [			  0,  		 	0,   		0, 		 1]])

    T4_5 = Matrix([[		    cos(q5),		 -sin(q5),		0,		a4],
			   [	sin(q5)*cos(alpha4),  cos(q5)*cos(alpha4),   -sin(alpha4), -sin(alpha4)*d5],
			   [	sin(q5)*sin(alpha4),  cos(q5)*sin(alpha4),    cos(alpha4),  cos(alpha4)*d5],
			   [			  0,  		 	0,   		0, 		 1]])

    T5_6 = Matrix([[		    cos(q6),		 -sin(q6),		0,		a5],
			   [	sin(q6)*cos(alpha5),  cos(q6)*cos(alpha5),   -sin(alpha5), -sin(alpha5)*d6],
			   [	sin(q6)*sin(alpha5),  cos(q6)*sin(alpha5),    cos(alpha5),  cos(alpha5)*d6],
			   [			  0,  		 	0,   		0, 		 1]])

    T6_G = Matrix([[		    cos(q7),		 -sin(q7),		0,		a6],
			   [	sin(q7)*cos(alpha6),  cos(q7)*cos(alpha6),   -sin(alpha6), -sin(alpha6)*d7],
			   [	sin(q7)*sin(alpha6),  cos(q7)*sin(alpha6),    cos(alpha6),  cos(alpha6)*d7],
			   [			  0,  		 	0,   		0, 		 1]])

#	    T0_1 = T0_1.subs(dh)
#	    T1_2 = T1_2.subs(dh)
#	    T2_3 = T2_3.subs(dh)
#	    T3_4 = T3_4.subs(dh)
#	    T4_5 = T4_5.subs(dh)
#	    T5_6 = T5_6.subs(dh)
#	    T6_G = T6_G.subs(dh)
#	    print("T6_G = %s" % T6_G) 

            # Create individual transformation matrices
#	    T0_2 = simplify(T0_1 * T1_2)
#	    print("T0_2 = %s" % T0_2)  
#	    T0_3 = simplify(T0_2 * T2_3)
#	    print("T0_3 = %s" % T0_3)  
#	    T0_4 = simplify(T0_3 * T3_4)
#	    T0_5 = simplify(T0_4 * T4_5)
#	    T0_6 = simplify(T0_5 * T5_6)
#	    print("T0_6 = %s" % T0_6)  
#	    T0_G = simplify(T0_6 * T6_G)
#	    print("T0_G = %s" % T0_G)  

	    # Define a correction between Gripper Link and DH convertion
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

	    # Get a final matrix to convert input postion paramters to output angle parameters
#	    R_final = simplify(T0_G * R_correct)
#	    print("R_final = %s" % R_final)  
            
            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
    px = 2.3146
    py = 0.11282
    pz = 2.1129    
    x = -0.24965
    y = 0.41624
    z = -0.11376
    w = 0.86688
    print("px = %s" % px)
    print("py = %s" % py)
    print("pz = %s" % pz)
    print("x = %s" % x)
    print("y = %s" % y)
    print("z = %s" % z)
    print("w = %s" % w)
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [x, y,z, w])
#     	    print("Received %s eef-poses.orientation from the plan" % req.poses[x].orientation)
            # Calculate joint angles using Geometric IK method
#    theta1 = atan2(py, px)
#    print("theta1 = %s" % theta1)
    print("roll = %s, pitch = %s, yaw = %s" % (roll, pitch, yaw))

    d6 = dh['d6']
    l =  dh['d7']
    print("d6 = %s, i = %s" % (d6, l))
    lx = cos(roll) * cos(pitch)
    ly = sin(roll) * cos(pitch)
    lz = -sin(roll)

    Wx = px - (d6 + l) * lx #2.1245 #
    Wy =  py - (d6 + l) * ly	# 0.23555 #		
    Wz =  pz - (d6 + l) * lz	   # 2.3144 #
    print("Wx = %s answer = 2.1245, Wy = %s answer = 0.23555, Wz = %s answer = 2.3144" % (Wx, Wy, Wz))
    v = (px - Wx)/(d6 + l)
    print("roll cal = %s" % v)
    theta1 = atan2(Wy, Wx)
    print("theta1 = %s" % theta1)
#    print("link 5 theta = %s" % atan2(0.23555, 2.1245))
    s = Wz - dh['d1'] 
    r = sqrt(Wx*Wx+Wy*Wy)
    print("s = %s" % s)
    print("r = %s" % r)
    beta = atan2(s, r)
    print("beta = %s" % beta)
    distance_c = sqrt(Wx**2+Wy**2+s**2)
#	    print("d4 = %s" % dh['d4'])
    distance_a = dh['d4']
    distance_b = dh['a2'] - dh['a3']
    print("distance_c = %s, distance_a = %s, distance_b = %s" % (distance_c, distance_a, distance_b))
    D = (distance_c ** 2 - distance_a ** 2 - distance_b ** 2) / (2 * distance_a * distance_b)
    print("D = %s, D^2 = %s" % (D, D ** 2))
    print("sqrt(1 - D ** 2) = %s" % sqrt(1 - D ** 2))
    theta3 = atan2(D, sqrt(1 - D ** 2))
    print("theta3 = %s" % theta3)
    alpha = atan2(distance_b + distance_a * cos(theta3), distance_a * sin(theta3))
    theta2 = beta = alpha
    print("theta2 = %s" % theta2)

    dh['q1'] = theta1
    dh['q2'] = theta2
    dh['q3'] = theta3
#	    dh['q4'] = 'q4'
#	    dh['q5'] = 'q5'
#	    dh['q6'] = 'q6'
    print("dh = %s" % dh)  
    return
    R0_1 = Matrix([[		    cos(q1),		 -sin(q1),		0],
			   [	sin(q1)*cos(alpha0),  cos(q1)*cos(alpha0),   -sin(alpha0)],
			   [	sin(q1)*sin(alpha0),  cos(q1)*sin(alpha0),    cos(alpha0)]])

    R1_2 = Matrix([[		    cos(q2),		 -sin(q2),		0],
			   [	sin(q2)*cos(alpha1),  cos(q2)*cos(alpha1),   -sin(alpha1)],
			   [	sin(q2)*sin(alpha1),  cos(q2)*sin(alpha1),    cos(alpha1)]])

    R2_3 = Matrix([[		    cos(q3),		 -sin(q3),		0],
			   [	sin(q3)*cos(alpha2),  cos(q3)*cos(alpha2),   -sin(alpha2)],
			   [	sin(q3)*sin(alpha2),  cos(q3)*sin(alpha2),    cos(alpha2)]])

    R3_4 = Matrix([[		    cos(q4),		 -sin(q4),		0],
			   [	sin(q4)*cos(alpha3),  cos(q1)*cos(alpha3),   -sin(alpha3)],
			   [	sin(q4)*sin(alpha3),  cos(q1)*sin(alpha3),    cos(alpha3)]])

    R4_5 = Matrix([[		    cos(q5),		 -sin(q5),		0],
			   [	sin(q5)*cos(alpha4),  cos(q2)*cos(alpha4),   -sin(alpha4)],
			   [	sin(q5)*sin(alpha4),  cos(q2)*sin(alpha4),    cos(alpha4)]])

    R5_6 = Matrix([[		    cos(q6),		 -sin(q6),		0],
			   [	sin(q6)*cos(alpha5),  cos(q3)*cos(alpha5),   -sin(alpha5)],
			   [	sin(q6)*sin(alpha5),  cos(q3)*sin(alpha5),    cos(alpha5)]])

    R0_1 = R0_1.subs(dh)
    R1_2 = R1_2.subs(dh)
    R2_3 = R2_3.subs(dh)
    R3_4 = R3_4.subs(dh)
    R4_5 = R4_5.subs(dh)
    R5_6 = R5_6.subs(dh)
    R0_3 = simplify(R0_1 * R1_2 * R2_3)
    R3_6_smibol = simplify(R3_4 * R4_5 * R5_6)
    R_roll = Matrix([[  1,	    0,	          0],
			   [	0,  cos(roll),   -sin(roll)],
			   [	0,  sin(roll),    cos(roll)]])
    R_pitch = Matrix([[  cos(pitch),   0, sin(pitch)],
			   [		  0,   1,          0],
			   [	-sin(pitch),   0, cos(pitch)]])
    R_yaw = Matrix([[  cos(yaw), -sin(yaw), 	0],
			   [   sin(yaw),  cos(yaw), 	0],
			   [	       0,   	 0,     1]])
    Rrpy = simplify(R_roll * R_pitch * R_yaw)

    R3_6 = simplify(R0_3.inv() * Rrpy)

    print("R3_6 = %s" % R3_6) 
    print("R3_6_smibol = %s" % R3_6_smibol) 
#	    print("R3_6[0,0] = %s" % R3_6[0,0]) 
#	    print("R3_6[0,1] = %s" % R3_6[0,1]) 
#	    print("R3_6[0,2] = %s" % R3_6[0,2]) 
#	    print("R3_6[1,2] = %s" % R3_6[1,2]) 

#	    beta = atan2(-R3_6[2,0], sqrt(R3_6[0,0]*R3_6[0,0] + R3_6[1,0]*R3_6[1,0]))*rtd
#	    print("beta = %s" % beta) 
#	    gamma = atan2(R3_6[2,1], R3_6[2,2])*rtd
#	    print("gamma = %s" % gamma) 
#	    alpha = atan2(R3_6[1,0], R3_6[1,1])*rtd
#	    print("alpha = %s" % alpha) 
    print("--------------------------------------------") 
    theta4 = 0.0

    theta5 = 0.0
    theta6 = 0.0

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
    joint_trajectory_list.append(joint_trajectory_point)

    print("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
    return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    handle_calculate_IK()
#    IK_server()

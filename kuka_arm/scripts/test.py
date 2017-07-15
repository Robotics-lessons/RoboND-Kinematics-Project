#!/usr/bin/env python

import rospy
#from geometry_msgs.msg import PoseStamped, PointStamped, QuaternionStamped, Pose, Point, Quaternion
import geometry_msgs.msg
from datetime import datetime
import IK_server

# inputdata format: [px, py pz, x, y, z, w]
inputdata = [[2.3146, 0.11282, 2.1129, -0.24965, 0.41624, -0.11376, 0.86688],
        [2.15, 0, 1.94, 0, 0, 0, 1]]
# outputdata format: [theta1, theta2, theta3, theta4, theta5, theta6]
outputdata = [[0.11, 0.27, -0.54, -0.53, 1.19, -0.07],
        [0, 0, 0, 0, 0, 0, 0]]

def handle_calculate_IK_1(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    for x in xrange(0, len(req.poses)):
        px = req.poses[x].position.x
        py = req.poses[x].position.y
        pz = req.poses[x].position.z    
        rospy.loginfo("============== px = %.6f, py = %.6f, pz = %.6f," % (px, py, pz)) 
        rospy.loginfo("Received %s eef-poses.orientation from the plan" % req.poses[x].orientation) 
    return

def printResults(results):
    rospy.loginfo("Number of pose = %d" % len(results.points))
    for i in xrange(0, len(results.points)):
        rospy.loginfo("Theta 1 = %.4f, expected answer = %.2f" % (results.points[i].positions[0], outputdata[i][0]))
        rospy.loginfo("Theta 2 = %.4f, expected answer = %.2f" % (results.points[i].positions[1], outputdata[i][1]))
        rospy.loginfo("Theta 3 = %.4f, expected answer = %.2f" % (results.points[i].positions[2], outputdata[i][2]))
        rospy.loginfo("Theta 4 = %.4f, expected answer = %.2f" % (results.points[i].positions[3], outputdata[i][3]))
        rospy.loginfo("Theta 5 = %.4f, expected answer = %.2f" % (results.points[i].positions[4], outputdata[i][4]))
        rospy.loginfo("Theta 6 = %.4f, expected answer = %.2f" % (results.points[i].positions[5], outputdata[i][5]))
        rospy.loginfo("-------------------------")
    return

if __name__ == "__main__":
    rospy.init_node('talker', anonymous=True)
    rospy.loginfo("data = %s, dim = %s" % (inputdata, len(inputdata)))
    pose_array = geometry_msgs.msg.PoseArray()
    

    for i in xrange(0, len(inputdata)):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = inputdata[i][0]
        pose.position.y = inputdata[i][1]
        pose.position.z = inputdata[i][2]
        pose.orientation.x = inputdata[i][3]
        pose.orientation.y = inputdata[i][4]
        pose.orientation.z = inputdata[i][5]
        pose.orientation.w = inputdata[i][6]
        pose_array.poses.append(pose)
    pose_array.header.frame_id = "/arm_link_gripper"
    pose_array.header.stamp = datetime.now()
#   results =  handle_calculate_IK_1(pose_array)
    results = IK_server.handle_calculate_IK(pose_array)
#    rospy.loginfo("return = %s" % results)
    printResults(results)


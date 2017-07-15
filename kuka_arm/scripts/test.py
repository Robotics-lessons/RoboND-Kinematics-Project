#!/usr/bin/env python

import rospy
#from geometry_msgs.msg import PoseStamped, PointStamped, QuaternionStamped, Pose, Point, Quaternion
import geometry_msgs.msg
from datetime import datetime
import IK_server

data = [[2.3146, 0.11282, 2.1129, -0.24965, 0.41624, -0.11376, 0.86688],
        [2.15, 0, 1.94, 0, 0, 0, 1]]


def handle_calculate_IK_1(req):
    print("Received %s eef-poses from the plan" % len(req.poses))
    for x in xrange(0, len(req.poses)):
        px = req.poses[x].position.x
        py = req.poses[x].position.y
        pz = req.poses[x].position.z    
        rospy.loginfo("============== px = %.6f, py = %.6f, pz = %.6f," % (px, py, pz)) 
        rospy.loginfo("Received %s eef-poses.orientation from the plan" % req.poses[x].orientation) 
    return

if __name__ == "__main__":
    rospy.init_node('talker', anonymous=True)
    rospy.loginfo("data = %s, dim = %s" % (data, len(data)))
    pose_array = geometry_msgs.msg.PoseArray()
    

    for i in xrange(0, len(data)):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = data[i][0]
        pose.position.y = data[i][1]
        pose.position.z = data[i][2]
        pose.orientation.x = data[i][3]
        pose.orientation.y = data[i][4]
        pose.orientation.z = data[i][5]
        pose.orientation.w = data[i][6]
        pose_array.poses.append(pose)
    pose_array.header.frame_id = "/arm_link_gripper"
    pose_array.header.stamp = datetime.now()
#    handle_calculate_IK_1(pose_array)
    IK_server.handle_calculate_IK(pose_array)


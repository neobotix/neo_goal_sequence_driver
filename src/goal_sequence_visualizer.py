#!/usr/bin/env python

import time
import rospy
import numpy as np
import goal_sequence_driver_server as server

from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, PoseArray, Quaternion

# visualizer of goals in form of axes
def init_goal_sequence_visualizer(mat):

	rospy.init_node('neo_goal_sequence_visualizer')
	rate = rospy.Rate(5)
	pub = rospy.Publisher('/neo_goal_sequence_visualizer', PoseArray, queue_size=1)
	pose_array = PoseArray()
	pose_array.header.seq = 1
	pose_array.header.frame_id = "/map"
	for i in range(np.shape(mat)[0]):

		pose = Pose()
		pose.position.x = mat[i][0]
		pose.position.y = mat[i][1]
		quaternion = quaternion_from_euler(0, 0, np.radians(mat[i][2]))
		pose.orientation.x = quaternion[0]
		pose.orientation.y = quaternion[1]
		pose.orientation.z = quaternion[2]
		pose.orientation.w = quaternion[3]
		pose_array.poses.append(pose)
	#	pub.publish(pose_array)
	#	rate.sleep()
	return pub, rate, pose_array

if __name__ == '__main__':

	pub, rate, pose_array =	init_goal_sequence_visualizer(server.mat)
	while(not rospy.is_shutdown()):

		pub.publish(pose_array)
		rate.sleep()
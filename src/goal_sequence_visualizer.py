#!/usr/bin/env python

import time
import rospy
import numpy as np
import goal_sequence_driver_server as server

from geometry_msgs.msg import Pose, PoseArray
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

# visualizer of goals in form of axes
def init_goal_sequence_visualizer(mat):

	server.load_from_yaml(server.goal_list, server.driver_config)
	rospy.init_node('neo_goal_sequence_visualizer')
	rate = rospy.Rate(0.5)
	pub = rospy.Publisher('/neo_goal_sequence_visualizer', PoseArray, queue_size=1)
	pub_txt = rospy.Publisher('/neo_goal_sequence_visualizer/Marker', MarkerArray, queue_size=1)
	pose_array = PoseArray()
	pose_array.header.seq = 1
	pose_array.header.frame_id = "/map"
	marker_array = MarkerArray()
	i = 1
	for vec in mat:
		# creating pose for goals to be visualized
		pose = Pose()
		marker = Marker()
		pose.position.x = vec[0]
		pose.position.y = vec[1]
		pose.position.z = 0
		quaternion = quaternion_from_euler(0, 0, np.radians(vec[2]))
		pose.orientation.x = quaternion[0]
		pose.orientation.y = quaternion[1]
		pose.orientation.z = quaternion[2]
		pose.orientation.w = quaternion[3]
		pose_array.poses.append(pose)
		marker.header.frame_id = "/map"
		marker.header.seq = i
		marker.id = i
		marker.type = marker.TEXT_VIEW_FACING
		marker.action = marker.ADD
		marker.scale.z = 0.3
		marker.color.a = 1.0
		marker.color.g = 1.0
		marker.pose.position.x = vec[0]
		marker.pose.position.y = vec[1]
		marker.pose.position.z = 0.5
		marker.text = str(i)
		marker_array.markers.append(marker)
		i += 1
	return pub, pub_txt, rate, pose_array, marker_array

if __name__ == '__main__':

	while(not rospy.is_shutdown()):

		pub, pub_txt, rate, pose_array, marker_array =	init_goal_sequence_visualizer(server.mat)
		pub.publish(pose_array)
		pub_txt.publish(marker_array)
		rate.sleep()
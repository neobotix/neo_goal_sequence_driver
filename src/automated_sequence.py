#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from tf.transformations import quaternion_from_euler
import roslaunch
import os
import yaml
import rospkg

# Accessing the goal list
rospack = rospkg.RosPack()
pkg_name = 'neo_goal_sequence_driver'
goal_list = rospack.get_path(pkg_name)+'/config/goal_list.yaml'


class AutomatedGoal:
# Class initialization
	def __init__(self, n_poses):
		self.goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.GoalCB)
		self.goal_list = []
		self.pose = Pose()
		self.pose_array = PoseArray()
		self.pose_array.header.seq = 1
		self.pose_array.header.frame_id = "/map"
		self.pub_marker = rospy.Publisher('pose_array', PoseArray, queue_size=1)
		self.launch = False
		self.n_poses = n_poses # Support variable for publishing the pose arrays
		self.n_goals = n_poses # Support variable for logging
		self.data = []

	def GoalCB(self,data):
		self.goal_list.append([data])
		if(len(self.goal_list)==self.n_poses):
			rospy.loginfo("Let's visualize the goals and initialize the goals for goal sequence driver ")
			for i in range(0, len(self.goal_list)):
				self.pose = Pose()
				self.pose.position.x = self.goal_list[i][0].pose.position.x
				self.pose.position.y = self.goal_list[i][0].pose.position.y
				self.pose.position.z = self.goal_list[i][0].pose.orientation.z
				self.pose.orientation.x = self.goal_list[i][0].pose.orientation.x
				self.pose.orientation.y = self.goal_list[i][0].pose.orientation.y
				self.pose.orientation.z = self.goal_list[i][0].pose.orientation.z
				self.pose.orientation.w = self.goal_list[i][0].pose.orientation.w
				self.pose_array.poses.append(self.pose)
			self.pub_marker.publish(self.pose_array)
			self.launch = True

		else:
			rospy.loginfo("%i goals are left", self.n_goals-1)
			self.n_goals = self.n_goals-1

		if(self.launch):
			for i in range(0, len(self.goal_list)):
				self.data.append([{'X': self.goal_list[i][0].pose.position.x, \
					'Y': self.goal_list[i][0].pose.position.y,\
					'theta': self.goal_list[i][0].pose.orientation.z}, 
			])
				
				
			with open(goal_list, 'w') as outfile:
				for i in range(0, len(self.goal_list)):
					yaml.dump(self.data[i], outfile, default_flow_style=False)

			os.system("roslaunch neo_simulation mpo_500_move_base.launch") # Launching move base after the markers are published


if __name__ == '__main__':
	rospy.init_node('Automated_goal')
	n_poses = 4 # Set number of goals you want to give for the automated process! 
	ag = AutomatedGoal(n_poses)
	rate = rospy.Rate(0.01)
	rospy.loginfo("Please select %i poses", n_poses)
	rospy.loginfo("%i goals are left", n_poses)
	while(not rospy.is_shutdown()):
		rate.sleep() #Publishing at 100 Hz

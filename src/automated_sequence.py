#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from tf.transformations import quaternion_from_euler
import roslaunch
import yaml
import rospkg

# Accessing the goal list
rospack = rospkg.RosPack()
pkg_name = 'neo_goal_sequence_driver'
goal_list = rospack.get_path(pkg_name)+'/config/goal_list.yaml'


class AutomatedGoal:
# Class initialization
	def __init__(self):
		self.goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.GoalCB)
		self.goal_list = []
		self.pose = Pose()
		self.pose_array = PoseArray()
		self.pose_array.header.seq = 1
		self.pose_array.header.frame_id = "/map"
		self.pub_marker = rospy.Publisher('pose_array', PoseArray, queue_size=1)
		self.data = [] 
		self.seq = 1 #Keeps a track of no of goals 

# CallBack function of the topic /move_base_simple/goal
	def GoalCB(self,data):
		if(self.seq == 1):
			rospy.loginfo("%i goal selected ", self.seq)
		else:
			rospy.loginfo("%i goals selected ", self.seq)
		self.pose = Pose()
		self.pose.position.x = data.pose.position.x
		self.pose.position.y = data.pose.position.y
		self.pose.position.z = data.pose.orientation.z
		self.pose.orientation.x = data.pose.orientation.x
		self.pose.orientation.y = data.pose.orientation.y
		self.pose.orientation.z = data.pose.orientation.z
		self.pose.orientation.w = data.pose.orientation.w
		self.pose_array.poses.append(self.pose)
		self.pub_marker.publish(self.pose_array)
		self.data.append([{'X': data.pose.position.x, \
					'Y': data.pose.position.y,\
					'theta': data.pose.orientation.z}, 
			])
		self.seq = self.seq+1
		with open(goal_list, 'w') as outfile:
			yaml.dump(self.data, outfile, default_flow_style=False)



if __name__ == '__main__':
	rospy.init_node('Automated_goal')
	ag = AutomatedGoal()
	rate = rospy.Rate(0.01)
	while(not rospy.is_shutdown()):
		rate.sleep()


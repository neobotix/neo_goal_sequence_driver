#!/usr/bin/env python
import rospy
import numpy as np
# from move_base_simple_nsgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler
import roslaunch
import os
import yaml
import rospkg


rospack = rospkg.RosPack()
pkg_name = 'neo_goal_sequence_driver'
goal_list = rospack.get_path(pkg_name)+'/config/goal_list.yaml'


class AutomatedGoal:

	def __init__(self):
		self.goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.GoalCB)
		self.goal_list = []
		self.pose = Pose()
		self.pose_array = PoseArray()
		self.pose_array.header.seq = 1
		self.pose_array.header.frame_id = "/map"
		# self.marker_array = MarkerArray()
		self.pub_marker = rospy.Publisher('pose_array', PoseArray, queue_size=1)
		self.launch = False

	def GoalCB(self,data):
		self.goal_list.append([data])
		if(len(self.goal_list)==3):
			rospy.loginfo("Let's publish the goals")
			for i in range(0, len(self.goal_list)):
				# print(self.goal_list[i][0].pose.position.x)
				self.pose = Pose()
				self.pose.position.x = self.goal_list[i][0].pose.position.x
				self.pose.position.y = self.goal_list[i][0].pose.position.y
				self.pose.position.z = 0
				quaternion = quaternion_from_euler(0, 0, np.radians(self.goal_list[i][0].pose.orientation.z))
				self.pose.orientation.x = quaternion[0]
				self.pose.orientation.y = quaternion[1]
				self.pose.orientation.z = quaternion[2]
				self.pose.orientation.w = quaternion[3]
				self.pose_array.poses.append(self.pose)
			self.pub_marker.publish(self.pose_array)
			self.launch = True

		else:
			rospy.loginfo("i goals are left")

		if(self.launch):
			data = [{'X': self.goal_list[0][0].pose.position.x, 'Y': self.goal_list[0][0].pose.position.y, 'theta': self.goal_list[0][0].pose.orientation.z}, \
			 {'X': self.goal_list[1][0].pose.position.x, 'Y': self.goal_list[1][0].pose.position.y, 'theta': self.goal_list[1][0].pose.orientation.z}, \
			 {'X': self.goal_list[2][0].pose.position.x, 'Y': self.goal_list[2][0].pose.position.y, 'theta': self.goal_list[2][0].pose.orientation.z}]
			with open(goal_list, 'w') as outfile:
				yaml.dump(data, outfile, default_flow_style=False)
			os.system("roslaunch neo_simulation mpo_500_move_base.launch")
			# os.system("roslaunch neo_goal_sequence_driver neo_goal_sequence_driver.launch")

		# if(self.launch):
		# 	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		# 	roslaunch.configure_logging(uuid)
		# 	launch = roslaunch.parent.ROSLaunchParent(uuid, [file_path])
		# 	launch.start()

if __name__ == '__main__':
	rospy.init_node('Automated_goal')
	ag = AutomatedGoal()
	rate = rospy.Rate(0.5)
	rospy.loginfo("Please select minimum of 3 poses")
	while(not rospy.is_shutdown()):
		rate.sleep()


	# goal = rospy.Subscriber("/move_base/goal", MoveBaseGoal, GoalCB)

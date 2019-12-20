#!/usr/bin/env python

import time
import yaml
import rospy
import rospkg
import actionlib
import numpy as np

from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus as goal_status
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from neo_goal_sequence_driver.srv import command, commandResponse

# service option
service_called = True

## global variables
# default set for config parameters in non-service mode
print_option = "all"
infi_param = False
patience = 5.0
mat = []
dist = 0.0
former_pose = None
SERVICE_REQ = False

# path to configuration files
rospack = rospkg.RosPack()
pkg_name = 'neo_goal_sequence_driver'
goal_list = rospack.get_path(pkg_name)+'/config/goal_list.yaml'
driver_config = rospack.get_path(pkg_name)+'/config/driver_config.yaml'

# creat goal structure for move_base node from a given vector(x, y, theta)
def create_goal(vec):
	# create goal from position & orientation(quaternion)
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = vec[0]
	goal.target_pose.pose.position.y = vec[1]
	quaternion = quaternion_from_euler(0, 0, np.radians(vec[2]))
	goal.target_pose.pose.orientation.z = quaternion[2]
	goal.target_pose.pose.orientation.w = quaternion[3]
	return goal

# fetch data from .yaml files
def load_from_yaml(goal_list, driver_config):

	global mat
	global patience
	global infi_param
	global print_option
	# read goal list and configuration from .yaml file
	file = open(goal_list, 'r')
	goals = yaml.load(file, Loader=yaml.SafeLoader)
	file = open(driver_config, 'r')
	config_params = yaml.load(file, Loader=yaml.SafeLoader)
	mat = []
	for goal in goals:
		row = [goals[goal]['X'], goals[goal]['Y'], goals[goal]['theta']]
		mat.append(row)
	rospy.loginfo("\nGoal list read:")
	for vec in mat:
		print(vec)
	print_option = config_params['config_parameters']['print_option']
	infi_param = config_params['config_parameters']['infi_param']
	patience = config_params['config_parameters']['patience']
	print("print_option is:"+print_option)
	print("infi_param is:"+str(infi_param))
	print("patience is:"+str(patience))

# printing current status
def print_status():
	# if it isn't the goal element of list
	rospy.loginfo("Goal reached:"+str(mat[current_goal]))
	if(not current_goal == final_goal):
		if(print_option == "all" or print_option == "local"):
			# print time duration of current task
			rospy.loginfo("Current task duration:"+str(time.time() - current_task_start)+"s")
	# if it is the last goal
	else:
		if(print_option == "all" or print_option == "total"):
			rospy.loginfo("Total distance traveled is:"+str(dist)+"m")
			rospy.loginfo("Total time traveled is:"+str(time.time() - time_start)+"s")

# callback function of odom subscriber
def odom_callback(odometry):

	global dist
	global former_pose

	pose = odometry.pose.pose.position
	if(former_pose == None):
		# initialize former_pose when first time running this callback
		former_pose = pose
	else:
		# calculating the distance traveled in every period of odom
		d_dist = np.sqrt((pose.x - former_pose.x)**2 + (pose.y - former_pose.y)**2)
		former_pose = pose
		# filter out the drifting and accumulate
		if(d_dist > 0.000002):
			dist += d_dist

# callback function of service run
def goal_sequence_driver_run(cmd):

	global SERVICE_REQ
	global current_goal
	global current_task_start

	if(SERVICE_REQ):
		return "goal_sequence_driver is already running."
	else:
		SERVICE_REQ = True
		current_goal = -1
		current_task_start = time.time()
		return "Service requested."

# callback function of service stop
def goal_sequence_driver_stop(cmd):

	global SERVICE_REQ

	if(SERVICE_REQ):
		SERVICE_REQ = False
		client.cancel_all_goals()
		return "Shut down."
	else:
		return "goal_sequence_driver is not running."

if __name__ == '__main__':

	global time_start
	global client
	global current_goal
	global final_goal
	first_goal = 0
	current_goal = -1
	printed_once = True

	# initialization of node, client and servers
	rospy.init_node('goal_sequence_driver')
	odom_sub = rospy.Subscriber("odom", Odometry, odom_callback)
	rospy.loginfo("Odometer is up.")
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	if(client.wait_for_server()):
		rospy.loginfo("Action client server up.")
	load_from_yaml(goal_list, driver_config)	# load configuration params & goals from .yaml
	final_goal = np.shape(mat)[0] - 1			# get the max index of goal list
	time_start = time.time()					# record the time of starting server
	rate = rospy.Rate(5)
	# in the service_called mode, servers would be initialized
	# run "rosservice call /goal_sequence_driver_run" 	to start the drive
	# run "rosservice call /goal_sequence_driver_stop" 	to stop
	if(service_called):
		server = rospy.Service('goal_sequence_driver_run', command, goal_sequence_driver_run)
		server_stop = rospy.Service('goal_sequence_driver_stop', command, goal_sequence_driver_stop)
		rospy.loginfo("goal_sequence_driver server ready.")
	# when not called by service, no server needed
	else:
		rospy.loginfo("goal_sequence_driver is not responding to any request.")
		cmd = None
		goal_sequence_driver_run(cmd)
	# initialization finished
	# loop of goal_sequence_driver below:
	while(not rospy.is_shutdown()):
		# send goal one by one, on condition:
			# 1) the robot is not busy
			# 2) or waited for too long
			# 3) and user called service "run"
		if not(client.get_state() == goal_status.PENDING or client.get_state() == goal_status.ACTIVE) and SERVICE_REQ:
			# if there's any unprinted but reached goal, print its pose information and time duration of task
			if(not printed_once):
				print_status()
				printed_once = True
			# if it hasn't reached the final goal, or waited too long, then go to next goal
			if(not current_goal == final_goal):
				current_goal += 1
			# if it's already the final goal
			else:
				# if infi_param set to True, start the loop again from 1st goal
				if(infi_param):
					current_goal = first_goal
				# if not, don't send goal anymore
				else:
					SERVICE_REQ = False
			# drive the robot to the current_goal
			client.send_goal(create_goal(mat[current_goal]))
			# the current_goal is not printed yet
			printed_once = False
		#################################
		# add your own functions below: #
		#################################

		rate.sleep()
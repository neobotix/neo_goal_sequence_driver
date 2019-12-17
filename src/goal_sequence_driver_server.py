#!/usr/bin/env python

import time
import yaml
import rospy
import rospkg
import actionlib
import numpy as np

from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from neo_goal_sequence_driver.srv import command, commandResponse

# global variables
print_option = "all"
infi_param = False
patience = 5.0
dist = 0.0
time_start = None
former_pose = None
smallest_step_length = 0.000002
pkg_name = 'neo_goal_sequence_driver'
switch = pkg_name+'/switch'
occupied = pkg_name+'/occupied'
mat = []

# service option
service_called = True

if(not service_called):
	class CMD:
		switch = "up"
	cmd_default = CMD()
	rospy.set_param(occupied, False)

# path to configuration files
rospack = rospkg.RosPack()
goal_list = rospack.get_path(pkg_name)+'/config/goal_list.yaml'
driver_config = rospack.get_path(pkg_name)+'/config/driver_config.yaml'

# get server ready for service, including embedded odometer
def goal_sequence_driver_server():

	global goal_list
	global driver_config

	rospy.init_node('goal_sequence_driver_server')
	server = rospy.Service('goal_sequence_driver_run', command, goal_sequence_driver_run)
	server_stop = rospy.Service('goal_sequence_driver_stop', command, goal_sequence_driver_stop)
	rospy.loginfo("goal_sequence_driver_server is up.")
	odom_sub = rospy.Subscriber("odom", Odometry, odom_callback)
	rospy.loginfo("Odometer is up.")
	rospy.spin()

# fetch data from .yaml files
def load_from_yaml():

	global print_option
	global infi_param
	global goal_list
	global driver_config
	goals = None

	try:

		with open(goal_list, 'r') as file:
			goals = yaml.load(file, Loader=yaml.SafeLoader)
		with open(driver_config, 'r') as file:
			config_params = yaml.load(file, Loader=yaml.SafeLoader)
	except:
		rospy.loginfo("Reading .yaml file failed.")
	mat_goals = []
	i = 1
	while(goals):

		try:

			mat_goals.append([goals['goals']['goal_'+str(i)]['X'], goals['goals']['goal_'+str(i)]['Y'], goals['goals']['goal_'+str(i)]['theta']])
		except:

			if(mat_goals):
				
				rospy.loginfo("All goal data recieved.")
				break
			else:

				rospy.loginfo("No goal data recieved.")
				break
		i += 1
	rospy.loginfo("\nGoal list read:")
	for j in range(np.shape(mat_goals)[0]):
		print(mat_goals[j])
	try:

		print_option = config_params['config_parameters']['print_option']
		infi_param = config_params['config_parameters']['infi_param']
		patience = config_params['config_parameters']['patience']
		print("print_option is:"+print_option)
		print("infi_param is:"+str(infi_param))
	except:

		rospy.loginfo("Some parameters are not set, applying default values.")
	return mat_goals


# transformation of data: euler -> quaternion
def create_quaternion(goal_num, mat):

	# read position & orientation(euler) from matrix and transfer them
	x = mat[goal_num][0]
	y = mat[goal_num][1]
	theta = np.radians(mat[goal_num][2])
	quaternion = quaternion_from_euler(0, 0, theta)
	return [x, y, quaternion]

# creat goal structure for move_base node
def create_goal(pose):

	# create goal from position & orientation(quaternion)
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = pose[0]
	goal.target_pose.pose.position.y = pose[1]
	goal.target_pose.pose.orientation.z = pose[2][2]
	goal.target_pose.pose.orientation.w = pose[2][3]
	return goal

# callback function of odom subscriber
def odom_callback(odometry):

	global dist
	global time_start
	global former_pose
	global smallest_step_length

	pose = odometry.pose.pose.position
	if(former_pose == None):

		former_pose = pose
		time_start = rospy.get_time()
	else:

		d_dist = np.sqrt((pose.x - former_pose.x)**2 + (pose.y - former_pose.y)**2)
		former_pose = pose
		if(d_dist < smallest_step_length):
			d_dist = 0
		dist += d_dist

# callback function of service run
def goal_sequence_driver_run(cmd):

	global mat

	rospy.loginfo("goal_sequence_driver up.")
	rospy.set_param(switch, True)	
	try:
	
		rospy.get_param(occupied)
	except:

		rospy.set_param(occupied, False)
	if(not rospy.get_param(occupied)):
		
		rospy.set_param(occupied, True)
		client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		current_task_start = rospy.get_time()
		if(client.wait_for_server()):

			rospy.loginfo("Client server up.")
		i = 0
		mat = load_from_yaml()
		while(not rospy.is_shutdown() and rospy.get_param(switch)):

			try:

				pose = create_quaternion(i, mat)
			except:
				if(i != 0):

					rospy.loginfo("All goals reached, sequence drive task completed.")

				else:

					rospy.loginfo("Process died of unknown reason.")
					rospy.set_param(occupied, False)
					return "Service failed."
				i = 0
				if(infi_param != True):

					break
			goal = create_goal(pose)
			client.send_goal(goal)
			while(not rospy.is_shutdown()):

				if(client.wait_for_result(rospy.Duration.from_sec(patience)) == True):

					rospy.loginfo("Goal reached.")
					if(print_option == "all" or print_option == "local"):

						rospy.loginfo("Current task duration:"+str(rospy.get_time() - current_task_start)+"s")						
					time.sleep(1)
					break
			i += 1
		rospy.set_param(occupied, False)
		if(print_option == "all" or print_option == "total"):

			rospy.loginfo("Total distance traveled is:"+str(dist)+"m")
			rospy.loginfo("Total time traveled is:"+str(rospy.get_time() - time_start)+"s")
	return "Service done."

# callback function of service stop
def goal_sequence_driver_stop(cmd):

	rospy.loginfo("goal_sequence_driver down.")
	rospy.set_param(switch, False)

	return "Shut down."


if __name__ == '__main__':

	if(service_called):

		goal_sequence_driver_server()

	else:

		rospy.init_node('goal_sequence_driver')
		rospy.loginfo("goal_sequence_driver is not responding to any request.")
		odom_sub = rospy.Subscriber("odom", Odometry, odom_callback)
		rospy.loginfo("Odometer is up.")
		goal_sequence_driver_run(cmd_default)
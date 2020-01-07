# neo_goal_sequence_driver

## It's a package for testing basic functionalities of navigation stack.


1. Currently tested on MP-500 under neo_track1.world, and it's working fine;

2. User can change (x, y, theta) values of each goal, also the number of goals and several configuration parameters;

3. For testing, please:
	
	1) run .../neo_simulation/simulation.launch;
	2) run .../neo_goal_sequence_driver/launch/neo_goal_sequence_driver_server.launch;
	   (The robot should be standing by.)
	3) call: 
		    rosservice call /goal_sequence_driver_run
		and the robot should be moving along goals;
	    there're 3 arguments available, if you want to modify them, read the driver_config.yaml file;
	4) call:
		    rosservice call /goal_sequence_driver_stop
		and the driver stops;
	5) manipulate goals in .../neo_goal_sequence_driver/config/goal_list.yaml;
	6) shut down the server, turn to ii. and repeat if you want to reload those parameters;

4. If you followed all the instructions above, in rviz it looks like:
	![image](https://github.com/Hezihao/neo_goal_sequence_driver/blob/master/IMG/example_sequence_driver.png)

	In the picture above a default goal list with 4 goals is visualized and the goal_sequence_driver is ready to drive.

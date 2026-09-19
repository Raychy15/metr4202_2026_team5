Autonomous Exploration and Vision Project

1. Launch Gazebo Simulation
	export TURTLEBOT3_MODEL=waffle_pi
	ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py (Or whatever world you choose)
	
2. Launch SLAM 
	export TURTLEBOT3_MODEL=waffle_pi
	ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true

3. Launch Nav2
	export TURTLEBOT3_MODEL=waffle_pi
	ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

4. Launch Exploration Package
	cd ~/turtlebot3_ws
	source install/setup.bash
	ros2 launch exploration_project vision.launch.py

5. Launch RViz2
	ros2 run rviz2 rviz2
	Required RViz2 Configuration:

	In the left panel under Global Options, change the Fixed Frame to map.

	Click Add (bottom left) -> By topic -> Select /map (under the Map display) to view the 					permanent floorplan. 

	Click Add -> By topic -> Select the detected_targets topic to view the persistent 3D green target markers anchored to the map.

Note the robot may not move or may get stuck, move the robot to another part of the map either through RViz waypoint or move it in gazebo and the navigation package will start to work

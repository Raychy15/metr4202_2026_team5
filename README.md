# Autonomous Exploration and Vision Project

This package integrates frontier-based autonomous exploration with real-time ArUco marker detection.

## Required Dependencies
* ROS 2 Humble
* `turtlebot3` and `turtlebot3_simulations`
* `turtlebot3_navigation2`
* `ros2_aruco`
* `slam_toolbox`

## Installation Instructions
Create a workspace, download the third-party `ros2_aruco` package from source, and clone this repository:

```bash
cd ~/turtlebot3_ws/src
git clone [https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco]
git clone [https://github.com/Raychy15/metr4202_2026_team5]
```

## Build Instructions
cd ~/turtlebot3_ws
colcon build
source install/setup.bash

## Launch Instructions
1. Launch Gazebo Simulation
	export TURTLEBOT3_MODEL=waffle_pi
	ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py (Or whatever world you choose)
	
2. Launch SLAM 
	export TURTLEBOT3_MODEL=waffle_pi
	ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

3. Launch Nav2
	export TURTLEBOT3_MODEL=waffle_pi
	ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true

4. Launch Exploration Package
	cd ~/turtlebot3_ws
	source install/setup.bash
	ros2 launch exploration_project vision.launch.py

5. Launch RViz2
	ros2 run rviz2 rviz2
	Required RViz2 Configuration:

	In the left panel under Global Options, change the Fixed Frame to map.

	Click Add (bottom left) -> By topic -> Select /map (under the Map display) to view the permanent floorplan. 

	Click Add -> By topic -> Select the detected_targets topic to view the green target markers anchored to the map.

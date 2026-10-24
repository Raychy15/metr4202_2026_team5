# Project Overview

The project makes use of ROS2 Humble in order for a TurtleBot3 Waffle Pi to explore and create a map of an unknown environment, combining SLAM Toolbox, frontier-based exploration, Nav2 navigation and ArUco marker detection.

# Main Features
- **Frontier Detection:** The method makes use of the occupancy grid produced by SLAM Toolbox in order to identify unexplored regions. The frontier cells are grouped by means of a breadth-first search, and larger clusters are divided in order to create more exploration points.
- **Frontier Selection:** The location to be chosen next is determined by considering both the distance and the size of the frontier. The map is divided up into 3 m sectors, and the robot aims to have explored 80% of each sector before proceeding to the next one.
– Navigation and recovery: The robot uses Nav2 to move between designated locations, and if it becomes stuck or comes near an obstacle, it tries out recovery using LiDAR measurements.
- **ArUco Detection:** The robot's camera is used to detect ArUco markers and to estimate the markers' positions on the map.

# How it Works
As it moves, the robot creates a map using LiDAR and the SLAM Toolbox; the frontier detector determines the boundaries between areas that have been explored and those that have not, and the selector decides where to go next. Nav2 is in charge of navigating to these places, after which the process is carried out again as more of the environment is discovered. The camera also picks up the ArUco markers and notes their positions



# Autonomous Exploration and Vision Project

This package integrates frontier-based autonomous exploration with real-time ArUco marker detection. It uses a custom frontier detection algorithm to navigate an enclosed maze while translating camera-relative ArUco marker detections into a permanent global map.

## Package Overview
* **`ros2_aruco`**: A third-party package acting as a ROS 2 wrapper for OpenCV. It detects ArUco markers in the raw camera feed and calculates their 3D poses relative to the camera lens.
* **`slam_toolbox`**: Generates a real-time 2D Occupancy Grid (`/map`) of the environment.
* **`turtlebot3_navigation2`**: Handles obstacle avoidance and path planning to drive the robot to designated coordinate goals.
* **`exploration_project` (Custom)**:
  * **Frontier Detector (`frontier_detector.py`)**: Subscribes to the map, uses BFS to find border cells (free space adjacent to unknown space), groups them into clusters, and publishes them.
  * **Frontier Selector (`frontier_selector.py`)**: Evaluates frontier clusters based on size and distance, sends navigation goals to Nav2, and includes deadlock recovery logic to prevent the robot from getting stuck.
  * **Target Localizer (`target_localizer.py`)**: Uses the TF tree to transform camera-relative ArUco detections into permanent global map coordinates, filters out duplicates, and publishes them to RViz.

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
git clone [https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco](https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco)
git clone [https://github.com/Raychy15/metr4202_2026_team5](https://github.com/Raychy15/metr4202_2026_team5)
```

## Build Instructions
```bash
cd ~/turtlebot3_ws
colcon build
source install/setup.bash
```

## Launch Instructions

### Option A: Simulation (Gazebo)
When running in the simulation, you **must** use simulation time (`use_sim_time:=true`) so the ROS 2 TF tree synchronizes with the Gazebo clock.

1. **Launch Gazebo Simulation**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo metr4202.launch.py
```

2. **Launch SLAM**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

3. **Launch Nav2**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true
```

4. **Launch Exploration Package**
```bash
cd ~/turtlebot3_ws
source install/setup.bash
ros2 launch exploration_project vision.launch.py use_sim_time:=true
```

---

### Option B: Physical Hardware
When running on the actual TurtleBot3, do **not** use simulation time. The system will default to the real-time hardware clock.

1. **Launch SLAM**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch slam_toolbox online_async_launch.py
```

2. **Launch Nav2**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_navigation2 navigation2.launch.py
```

3. **Launch Exploration Package**
```bash
cd ~/turtlebot3_ws
source install/setup.bash
ros2 launch exploration_project vision.launch.py
```

---

### 5. Launch RViz2 (For both Sim and Hardware)
```bash
ros2 run rviz2 rviz2
```

**Required RViz2 Configuration:**
* In the left panel under **Global Options**, change the **Fixed Frame** to `map`.
* Click **Add** (bottom left) -> **By topic** -> Select `/map` (under the Map display) to view the permanent floorplan. 
* Click **Add** -> **By topic** -> Select the `/detected_targets` topic to view the green target markers anchored to the map.

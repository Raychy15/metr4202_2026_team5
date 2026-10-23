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

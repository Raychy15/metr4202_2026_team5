"""
Gazebo + TurtleBot3 in narrow_test_v4.world (tight spaces, built from course-style 1 m boxes).

Keep this file in the same folder as narrow_test_v4.world and run:
    export TURTLEBOT3_MODEL=waffle_pi
    ros2 launch ~/narrow_test_v4/narrow_test_v4.launch.py

It replaces README step 1 (turtlebot3_world.launch.py); everything else
(SLAM, Nav2, exploration, RViz) is launched exactly as usual.
Based on turtlebot3_gazebo/launch/turtlebot3_world.launch.py (Humble).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    world = os.path.join(here, 'narrow_test_v4.world')

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='1.2')   # start room, bottom-left
    y_pose = LaunchConfiguration('y_pose', default='1.2')

    return LaunchDescription([
        DeclareLaunchArgument('x_pose', default_value='1.2'),
        DeclareLaunchArgument('y_pose', default_value='1.2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()),
    ])

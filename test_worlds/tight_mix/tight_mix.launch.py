"""
Gazebo + TurtleBot3 (waffle_pi) in tight_mix.world.

Keep this file next to tight_mix.world and run (replaces README step 1):
    export TURTLEBOT3_MODEL=waffle_pi
    ros2 launch ~/test_worlds/tight_mix/tight_mix.launch.py

Start somewhere else:  x_pose:=X y_pose:=Y yaw:=RADIANS
Start in the open middle strip:  x_pose:=8.0 y_pose:=6.0 yaw:=3.14
SLAM, Nav2, exploration and RViz are launched exactly as in the README.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    world = os.path.join(here, 'tight_mix.world')
    model = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    sdf = os.path.join(tb3, 'models', 'turtlebot3_' + model, 'model.sdf')

    return LaunchDescription([
        DeclareLaunchArgument('x_pose', default_value='1.0'),
        DeclareLaunchArgument('y_pose', default_value='1.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', model, '-file', sdf,
                        '-x', LaunchConfiguration('x_pose'),
                        '-y', LaunchConfiguration('y_pose'),
                        '-z', '0.01',
                        '-Y', LaunchConfiguration('yaw')]),
    ])

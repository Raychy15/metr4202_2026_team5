from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Create a launch configuration variable
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch argument so it can be passed via command line
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    return LaunchDescription([
        declare_use_sim_time,
        Node(
            package='ros2_aruco',
            executable='aruco_node',
            name='aruco_node',
            parameters=[
                {'camera_info_topic': '/camera/camera_info'},
                {'use_sim_time': use_sim_time}
            ]
        ),
        Node(
            package='exploration_project',
            executable='target_localizer',
            name='target_localizer',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='exploration_project',
            executable='frontier_detector',
            name='frontier_detector',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='exploration_project',
            executable='frontier_selector',
            name='frontier_selector',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])

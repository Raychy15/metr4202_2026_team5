from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_aruco',
            executable='aruco_node',
            name='aruco_node',
            parameters=[
                {'camera_info_topic': '/camera/camera_info'},
                {'use_sim_time': True}
            ]
        ),
        Node(
            package='exploration_project',
            executable='target_localizer',
            name='target_localizer',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='exploration_project',
            executable='frontier_detector',
            name='frontier_detector',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='exploration_project',
            executable='frontier_selector',
            name='frontier_selector',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
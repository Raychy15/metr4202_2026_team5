import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ArUco detection node configured for project parameters
    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node',
        output='screen',
        parameters=[{
            'marker_size': 0.1,
            'aruco_dictionary_id': 'DICT_6X6_250',
            'image_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info'
        }]
    )

    # Custom coordinate transformation and RViz publisher node
    localizer_node = Node(
        package='metr4202_vision',
        executable='target_localizer',
        name='target_localizer',
        output='screen'
    )

    return LaunchDescription([
        aruco_node,
        localizer_node
    ])
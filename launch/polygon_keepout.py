#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch argument for polygon mode (keepout or keepin)
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='keepout',
        description='Polygon mode: "keepout" (default) or "keepin".'
    )

    mode = LaunchConfiguration('mode')

    return LaunchDescription([
        mode_arg,

        # Polygon Drawer (clicked points → PolygonStamped)
        Node(
            package='pose_annotation',
            executable='polygon',
            name='polygon_drawer',
            output='screen'
        ),

        # Polygon → OccupancyGrid bridge
        Node(
            package='pose_annotation',
            executable='polygon_to_occupancy_grid',
            name='polygon_to_occupancy_grid',
            output='screen',
            parameters=[{'mode': mode}]
        ),

        # FilterInfo publisher (Nav2 keepout mask topic)
        Node(
            package='pose_annotation',
            executable='filter_info_publisher',
            name='filter_info_publisher',
            output='screen'
        ),
    ])

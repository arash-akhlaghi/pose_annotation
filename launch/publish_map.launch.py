import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('pose_annotation')
    map_file_path = os.path.join(pkg_path, 'maps', 'mappp.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='static_map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_file_path} # Pass the full, direct path
            ],
            remappings=[('/map', '/static_map')]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_static_map',
            output='screen',
            parameters=[
                {'autostart': True},
                {'node_names': ['static_map_server']}
            ]
        )
    ])
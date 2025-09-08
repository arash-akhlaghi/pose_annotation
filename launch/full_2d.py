import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler,
    GroupAction, ExecuteProcess
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the entire simulation and processing pipeline.
    - Base simulation runs in parallel.
    - Custom data processing pipeline runs sequentially.
    - Polygon drawing and Foxglove Studio are launched at the end.
    """

    # --- Paths to ROS packages ---
    pkg_pose_annotation = get_package_share_directory('pose_annotation')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_turtlebot3_navigation2 = get_package_share_directory('turtlebot3_navigation2')
    pkg_foxglove_bridge = get_package_share_directory('foxglove_bridge')
    pkg_ros_gz_bridge = get_package_share_directory('ros_gz_bridge')

    # --- Environment Variable ---
    set_turtlebot_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='burger'
    )
    
    # --- Actions for Base Simulation (run in parallel) ---
    start_gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_house.launch.py')
        )
    )
    start_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    nav2_params_file = os.path.join(pkg_turtlebot3_navigation2, 'param', 'burger.yaml')
    start_nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'params_file': nav2_params_file, 'use_sim_time': 'true'}.items()
    )
    start_foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(pkg_foxglove_bridge, 'launch', 'foxglove_bridge_launch.xml')
        )
    )

    # --- Define Actions for the Custom Pipeline ---
    models_to_bridge = ["burger", "robot_2", "robot_3", "robot_4"] + [f"gate{i}" for i in range(1, 11)]
    bridge_group = GroupAction(
        actions=[
            Node(
                package='ros_gz_bridge', executable='parameter_bridge',
                arguments=[f'/model/{model_name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
                output='screen', name=f'odometry_bridge_{model_name}'
            ) for model_name in models_to_bridge
        ]
    )
    
    position_aggregator_node = Node(package='pose_annotation', executable='position', name='position_aggregator_node', output='screen')
    transform_node = Node(package='pose_annotation', executable='transform', name='gates_robots_to_map', output='screen')
    image_publisher_node = Node(package='pose_annotation', executable='image_publisher', name='map_image_publisher', output='screen')
    pose_annotation_node = Node(package='pose_annotation', executable='pose_annotation_node', name='pose_annotation_node', output='screen')

    # --- NEW: Define the polygon node and Foxglove action ---
    polygon_node = Node(
        package='pose_annotation',
        executable='polygon',
        name='polygon_node',
        output='screen'
    )
    open_foxglove_action = ExecuteProcess(
        cmd=['xdg-open', 'https://app.foxglove.dev'],
        shell=True
    )

    # --- Create the Sequential Chain of Events ---
    on_aggregator_start = RegisterEventHandler(OnProcessStart(target_action=position_aggregator_node, on_start=[transform_node]))
    on_transform_start = RegisterEventHandler(OnProcessStart(target_action=transform_node, on_start=[image_publisher_node]))
    on_image_pub_start = RegisterEventHandler(OnProcessStart(target_action=image_publisher_node, on_start=[pose_annotation_node]))

    # --- NEW: Event 4. After pose_annotation_node starts, launch the polygon drawer and open Foxglove ---
    on_pose_annotation_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=pose_annotation_node,
            on_start=[
                polygon_node,
                open_foxglove_action
            ]
        )
    )

    # --- Assemble the Final Launch Description ---
    ld = LaunchDescription()
    ld.add_action(set_turtlebot_model)
    ld.add_action(start_gazebo_sim)
    ld.add_action(start_slam_toolbox)
    ld.add_action(start_nav2_stack)
    ld.add_action(start_foxglove_bridge)
    ld.add_action(bridge_group)
    ld.add_action(position_aggregator_node)
    ld.add_action(on_aggregator_start)
    ld.add_action(on_transform_start)
    ld.add_action(on_image_pub_start)
    # --- NEW: Add the final event handler to the launch description ---
    ld.add_action(on_pose_annotation_start)

    return ld
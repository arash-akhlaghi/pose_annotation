import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the entire simulation and processing pipeline.
    The base simulation starts in parallel.
    The custom nodes are launched in a specific, sequential order.
    """

    # --- Paths ---
    pkg_pose_annotation = get_package_share_directory('pose_annotation')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_turtlebot3_navigation2 = get_package_share_directory('turtlebot3_navigation2')
    pkg_foxglove_bridge = get_package_share_directory('foxglove_bridge')
    # --- CHANGED: No longer need gates_position_publisher package ---
    # pkg_gates_position_publisher = get_package_share_directory('gates_position_publisher')

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

    # --- Define Actions for the Sequential Custom Pipeline ---

    # --- REPLACED: Use IncludeLaunchDescription for full_position.py ---
    start_position_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pose_annotation, 'launch', 'full_position.py')
        )
    )
    
    # This is the node that will be started by the event handler
    pose_annotation_node = Node(
        package='pose_annotation',
        executable='label',
        name='pose_annotation_label',
        output='screen'
    )
    
    # This is the final launch file in the sequence
    lidar_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pose_annotation, 'launch', 'lidar_launch.py')
        )
    )

    # --- Create the Sequential Chain of Events ---

    # --- CHANGED: The event handler now waits for a node from WITHIN full_position.py ---
    # We assume 'full_position.py' starts a node named 'position_aggregator_node'.
    # This node must start before 'pose_annotation_node' (the labeler) can run.
    on_aggregator_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            # This needs to be the specific name of the node from the included launch file.
            target_action_name='position_aggregator_node',
            on_start=[pose_annotation_node]
        )
    )

    # 2. When pose_annotation_node starts, trigger the final lidar_fusion_launch.
    # This part remains the same.
    on_pose_annotation_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=pose_annotation_node,
            on_start=[lidar_fusion_launch]
        )
    )

    # --- Assemble the Launch Description ---
    ld = LaunchDescription()

    # Add environment variable
    ld.add_action(set_turtlebot_model)

    # Add the base simulation components (start in parallel)
    ld.add_action(start_gazebo_sim)
    ld.add_action(start_slam_toolbox)
    ld.add_action(start_nav2_stack)
    ld.add_action(start_foxglove_bridge)

    # --- CHANGED: Add the included launch file to start the custom pipeline ---
    ld.add_action(start_position_pipeline)

    # Add the event handlers that will continue the chain
    ld.add_action(on_aggregator_start)
    ld.add_action(on_pose_annotation_start)

    return ld
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, GroupAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the entire simulation and processing pipeline.
    - Base simulation runs in parallel.
    - Custom data processing pipeline runs sequentially.
    """

    # --- 1. Find All Necessary Package Paths ---
    pkg_pose_annotation = get_package_share_directory('pose_annotation')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_turtlebot3_navigation2 = get_package_share_directory('turtlebot3_navigation2')
    pkg_foxglove_bridge = get_package_share_directory('foxglove_bridge')
    # This package is needed for the parameter_bridge executable
    pkg_ros_gz_bridge = get_package_share_directory('ros_gz_bridge')

    # --- 2. Set Environment Variable for TurtleBot3 Model ---
    set_turtlebot_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='burger'
    )
    
    # --- 3. Define the PARALLEL Simulation Actions ---
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

    # --- 4. Define the Custom Pipeline Actions (MERGED from full_position.py) ---

    # Create bridges for all models from Gazebo to ROS
    models_to_bridge = ["burger", "robot_2", "robot_3", "robot_4"] + [f"gate{i}" for i in range(1, 11)]
    bridge_group = GroupAction(
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[f'/model/{model_name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
                output='screen',
                name=f'odometry_bridge_{model_name}' # Unique name for each bridge
            ) for model_name in models_to_bridge
        ]
    )
    
    # This is the first node in our sequence.
    position_aggregator_node = Node(
        package='pose_annotation',        # Correct package
        executable='position',            # Correct executable
        name='position_aggregator_node',  # Runtime name
        output='screen'
    )
    
    # These are the subsequent nodes in the sequence.
    label_node = Node(
        package='pose_annotation',
        executable='label',
        name='pose_annotation_label_node',
        output='screen'
    )
    
    lidar_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pose_annotation, 'launch', 'lidar_launch.py')
        )
    )

    # --- 5. Create the Sequential Chain of Events (CORRECTED) ---

    # Event 1: When position_aggregator_node starts, trigger the label_node.
    on_aggregator_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=position_aggregator_node, # CORRECT: Pass the node object
            on_start=[label_node]
        )
    )

    # Event 2: When the label_node starts, trigger the final lidar_fusion_launch.
    on_label_node_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=label_node, # CORRECT: Pass the node object
            on_start=[lidar_fusion_launch]
        )
    )

    # --- 6. Assemble the Final Launch Description ---
    ld = LaunchDescription()

    # Add environment variable
    ld.add_action(set_turtlebot_model)

    # Add the base simulation components (run in parallel)
    ld.add_action(start_gazebo_sim)
    ld.add_action(start_slam_toolbox)
    ld.add_action(start_nav2_stack)
    ld.add_action(start_foxglove_bridge)

    # Add the first step of the custom pipeline (bridges and aggregator)
    ld.add_action(bridge_group)
    ld.add_action(position_aggregator_node)

    # Add the event handlers that will trigger the rest of the chain
    ld.add_action(on_aggregator_start)
    ld.add_action(on_label_node_start)

    return ld
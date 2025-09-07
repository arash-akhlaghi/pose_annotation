from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node

def generate_launch_description():
    
    # List of all models you want to bridge (14 total)
    models_to_bridge = [
        "burger",  # Corresponds to robot 1
        "robot_2", 
        "robot_3", 
        "robot_4"
    ]
    # Add gates 1 through 10
    for i in range(1, 11):
        models_to_bridge.append(f"gate{i}")

    # --- GroupAction for Parallel Bridges ---
    # This group will contain all the bridge nodes.
    # All actions inside a GroupAction are executed in parallel.
    bridge_group = GroupAction(
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    f'/model/{model_name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
                ],
                output='screen'
            ) for model_name in models_to_bridge
        ]
    )

    # Node for your position aggregator script
    position_aggregator_node = Node(
        package='pose_annotation',
        executable='position',
        name='position',
        output='screen'
    )

    # Create the launch description
    ld = LaunchDescription()
    
    # Add the group of bridge nodes. All nodes in the group will start in parallel.
    ld.add_action(bridge_group)
    
    # Add the processing node, which will also start in parallel with the bridges.
    ld.add_action(position_aggregator_node)
    
    return ld
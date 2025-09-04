from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform (new-style args): base_link -> vertical_scan
        # 90° about Y (pitch) so the scan plane is vertical; adjust translation to your mount
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.171',
                '--qx', '0', '--qy', '0.70710678', '--qz', '0', '--qw', '0.70710678',
                '--frame-id', 'base_link', '--child-frame-id', 'vertical_scan'
            ],
            output='screen'
        ),

        # Bridge ONLY the vertical point cloud from Gazebo -> ROS 2
        # If your /scan already exists in ROS 2, do not bridge it again.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/vertical_scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
                # Some bridge versions support directional flags like [gz2ros]. If available, append:
                # '/vertical_scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked[gz2ros]'
            ],
            output='screen'
        ),

        # Fusion node
        Node(
            package='pose_annotation',   # change if your package name differs
            executable='lidar_3d',       # entry point name installed for lidar_3d.py
            name='lidar_3d_fusion',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'scan_topic': '/scan'},                       # horizontal LaserScan (ROS 2)
                {'vertical_points_topic': '/vertical_scan/points'},  # vertical PointCloud2 (ROS 2)
                {'vertical_frame_override': 'vertical_scan'},  # used if frame_id is empty
                {'target_frame': 'map'},                       # set to 'base_link' if map TF isn’t available
                {'publish_topic': '/merged_cloud'},
                {'queue_size': 10},
                {'slop': 0.1},
            ]
        ),
    ])
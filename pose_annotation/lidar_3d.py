#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2_py

class MergeLidarToMap(Node):
    def __init__(self):
        super().__init__('merge_lidar_to_map_node')

        # Publisher for merged 3D cloud in map frame
        self.pc_pub = self.create_publisher(PointCloud2, '/lidar_3d_cloud_map', 10)

        # LaserScan to PointCloud2 converter
        self.lp = LaserProjection()

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.h_sub = self.create_subscription(LaserScan, '/scan', self.horizontal_callback, 10)
        self.v_sub = self.create_subscription(LaserScan, '/vertical_scan', self.vertical_callback, 10)

        # Store last scans
        self.horizontal_scan = None
        self.vertical_scan = None

        self.get_logger().info("MergeLidarToMap node started...")

    def horizontal_callback(self, scan_msg: LaserScan):
        self.horizontal_scan = scan_msg
        self.try_merge()

    def vertical_callback(self, scan_msg: LaserScan):
        self.vertical_scan = scan_msg
        self.try_merge()

    def try_merge(self):
        if self.horizontal_scan is None or self.vertical_scan is None:
            return

        # Convert LaserScans to PointCloud2
        h_pc = self.convert_scan_to_pc(self.horizontal_scan)
        v_pc = self.convert_scan_to_pc(self.vertical_scan)

        try:
            # Transform horizontal LIDAR to map
            h_trans = self.tf_buffer.lookup_transform(
                'map',
                h_pc.header.frame_id,  # base_scan
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            h_pc_map = do_transform_cloud(h_pc, h_trans)

            # Transform vertical LIDAR to map
            v_trans = self.tf_buffer.lookup_transform(
                'map',
                v_pc.header.frame_id,  # base_link
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            v_pc_map = do_transform_cloud(v_pc, v_trans)

            # Merge both clouds
            merged_pc = self.merge_pointcloud2(h_pc_map, v_pc_map)

            # Publish merged cloud
            self.pc_pub.publish(merged_pc)

        except TransformException as e:
            self.get_logger().warn(f"TF not available yet: {e}")

    def convert_scan_to_pc(self, scan_msg: LaserScan) -> PointCloud2:
        ranges = np.array(scan_msg.ranges)
        ranges[np.isnan(ranges)] = 0.0
        ranges[np.isinf(ranges)] = scan_msg.range_max
        scan_msg.ranges = ranges.tolist()
        return self.lp.projectLaser(scan_msg)

    @staticmethod
    def merge_pointcloud2(pc1: PointCloud2, pc2: PointCloud2) -> PointCloud2:
        points1 = list(pc2_py.read_points(pc1))
        points2 = list(pc2_py.read_points(pc2))
        merged_points = points1 + points2
        return pc2_py.create_cloud(pc1.header, pc1.fields, merged_points)


def main(args=None):
    rclpy.init(args=args)
    node = MergeLidarToMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

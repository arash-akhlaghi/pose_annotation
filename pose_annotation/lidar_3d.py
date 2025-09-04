#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import message_filters


def _newer_stamp(a, b):
    if a.sec > b.sec:
        return a
    if a.sec < b.sec:
        return b
    return a if a.nanosec >= b.nanosec else b


class Lidar3DFusion(Node):
    def __init__(self):
        super().__init__('lidar_3d_fusion')

        # Parameters
        self.declare_parameter('scan_topic', '/scan')                        # horizontal LaserScan (ROS 2)
        self.declare_parameter('vertical_points_topic', '/vertical_scan/points')  # vertical PointCloud2 (ROS 2)
        self.declare_parameter('vertical_frame_override', 'vertical_scan')   # used if vertical cloud has empty/mismatched frame_id
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('slop', 0.1)  # seconds for ApproximateTimeSynchronizer
        self.declare_parameter('publish_topic', '/merged_cloud')

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.vpoints_topic = self.get_parameter('vertical_points_topic').get_parameter_value().string_value
        self.vertical_frame_override = self.get_parameter('vertical_frame_override').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        queue_size = int(self.get_parameter('queue_size').get_parameter_value().integer_value)
        slop = float(self.get_parameter('slop').get_parameter_value().double_value)
        publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value

        # QoS for sensor data
        self.sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publisher
        self.pc_pub = self.create_publisher(PointCloud2, publish_topic, self.sensor_qos)

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # LaserProjection for converting /scan
        self.lp = LaserProjection()

        # Subscribers (message_filters) for time sync
        self.scan_sub = message_filters.Subscriber(self, LaserScan, self.scan_topic, qos_profile=self.sensor_qos)
        self.vpoints_sub = message_filters.Subscriber(self, PointCloud2, self.vpoints_topic, qos_profile=self.sensor_qos)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.scan_sub, self.vpoints_sub],
            queue_size=queue_size,
            slop=slop,
            allow_headerless=False
        )
        self.sync.registerCallback(self.sync_cb)

        self.get_logger().info(
            f"Lidar3DFusion started.\n"
            f"- scan_topic: {self.scan_topic} (LaserScan)\n"
            f"- vertical_points_topic: {self.vpoints_topic} (PointCloud2)\n"
            f"- target_frame: {self.target_frame}\n"
            f"- publish_topic: {publish_topic}\n"
            f"- sync queue={queue_size} slop={slop}s"
        )

    def sync_cb(self, scan_msg: LaserScan, v_pc_msg: PointCloud2):
        # Convert horizontal LaserScan to PointCloud2 (XYZ only)
        h_pc = self._scan_to_pointcloud(scan_msg)
        h_pc_xyz = self._to_xyz32(h_pc)

        # Sanitize vertical cloud to XYZ-only and ensure frame_id
        v_pc_xyz = self._to_xyz32(self._ensure_frame(v_pc_msg))

        # Transform both to target_frame using their timestamps
        try:
            h_pc_t = self._transform_cloud(h_pc_xyz, self.target_frame, h_pc_xyz.header.stamp)
            v_pc_t = self._transform_cloud(v_pc_xyz, self.target_frame, v_pc_xyz.header.stamp)
        except TransformException as e:
            # TF not ready or frame missing; skip this pair
            self.get_logger().warn(f"TF failed: {e}")
            return

        # Merge and publish
        merged = self._merge_pointclouds(h_pc_t, v_pc_t)
        self.pc_pub.publish(merged)

    def _scan_to_pointcloud(self, scan_msg: LaserScan) -> PointCloud2:
        # Clean ranges
        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        if ranges.size:
            ranges[np.isnan(ranges)] = 0.0
            ranges[np.isinf(ranges)] = scan_msg.range_max
            scan_msg.ranges = ranges.tolist()

        pc = self.lp.projectLaser(scan_msg)
        pc.header.stamp = scan_msg.header.stamp
        pc.header.frame_id = scan_msg.header.frame_id
        return pc

    def _ensure_frame(self, cloud: PointCloud2) -> PointCloud2:
        # If the vertical cloud has no frame_id or a placeholder, override it
        if not cloud.header.frame_id or cloud.header.frame_id.strip() == '':
            self.get_logger().warn(
                f"Vertical cloud has empty frame_id. Overriding to '{self.vertical_frame_override}'."
            )
            cloud.header.frame_id = self.vertical_frame_override
        return cloud

    def _to_xyz32(self, cloud: PointCloud2) -> PointCloud2:
        # Extract x,y,z and rebuild a standard XYZ32 cloud to avoid field/dtype issues
        pts = list(pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))
        header = Header()
        header.frame_id = cloud.header.frame_id
        header.stamp = cloud.header.stamp
        return pc2.create_cloud_xyz32(header, pts)

    def _transform_cloud(self, cloud: PointCloud2, target_frame: str, stamp) -> PointCloud2:
        # Prefer transform at the message timestamp; wait briefly
        if not self.tf_buffer.can_transform(target_frame, cloud.header.frame_id, stamp, timeout=Duration(seconds=1.0)):
            # Try latest as a fallback
            if not self.tf_buffer.can_transform(target_frame, cloud.header.frame_id, rclpy.time.Time(), timeout=Duration(seconds=1.0)):
                raise TransformException(f"Transform {cloud.header.frame_id} -> {target_frame} not available")
            trans = self.tf_buffer.lookup_transform(target_frame, cloud.header.frame_id, rclpy.time.Time(), timeout=Duration(seconds=1.0))
        else:
            trans = self.tf_buffer.lookup_transform(target_frame, cloud.header.frame_id, stamp, timeout=Duration(seconds=1.0))

        # After sanitizing to XYZ32, do_transform_cloud is robust
        return do_transform_cloud(cloud, trans)

    def _merge_pointclouds(self, a: PointCloud2, b: PointCloud2) -> PointCloud2:
        pts_a = pc2.read_points(a, field_names=("x", "y", "z"), skip_nans=True)
        pts_b = pc2.read_points(b, field_names=("x", "y", "z"), skip_nans=True)

        merged_points = list(pts_a)
        merged_points.extend(list(pts_b))

        header = Header()
        header.frame_id = self.target_frame
        header.stamp = _newer_stamp(a.header.stamp, b.header.stamp)
        return pc2.create_cloud_xyz32(header, merged_points)


def main(args=None):
    rclpy.init(args=args)
    node = Lidar3DFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
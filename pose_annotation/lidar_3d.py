#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
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
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('vertical_points_topic', '/vertical_scan/points')
        self.declare_parameter('vertical_frame_override', 'vertical_scan')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('slop', 0.1)
        self.declare_parameter('publish_topic', '/merged_cloud')
        # --- New parameters for intensity and scaling ---
        self.declare_parameter('point_scale', 1.0)
        self.declare_parameter('h_scan_intensity', 50.0)
        self.declare_parameter('v_scan_intensity', 100.0)

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.vpoints_topic = self.get_parameter('vertical_points_topic').get_parameter_value().string_value
        self.vertical_frame_override = self.get_parameter('vertical_frame_override').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        queue_size = int(self.get_parameter('queue_size').get_parameter_value().integer_value)
        slop = float(self.get_parameter('slop').get_parameter_value().double_value)
        publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        # --- Get new parameters ---
        self.point_scale = self.get_parameter('point_scale').get_parameter_value().double_value
        self.h_intensity = self.get_parameter('h_scan_intensity').get_parameter_value().double_value
        self.v_intensity = self.get_parameter('v_scan_intensity').get_parameter_value().double_value

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

        # LaserProjection
        self.lp = LaserProjection()

        # Subscribers with time sync
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
            f"- sync queue={queue_size} slop={slop}s\n"
            f"- point_scale: {self.point_scale}\n"
            f"- h_scan_intensity: {self.h_intensity}, v_scan_intensity: {self.v_intensity}"
        )

    def sync_cb(self, scan_msg: LaserScan, v_pc_msg: PointCloud2):
        # Convert LaserScan to PointCloud2 and add intensity
        h_pc_raw = self._scan_to_pointcloud(scan_msg)
        h_pc_with_intensity = self._add_intensity(h_pc_raw, self.h_intensity)

        # Ensure vertical cloud has a frame_id and add intensity
        v_pc_ensured = self._ensure_frame(v_pc_msg)
        v_pc_with_intensity = self._add_intensity(v_pc_ensured, self.v_intensity)

        try:
            h_pc_t = self._transform_cloud(h_pc_with_intensity, self.target_frame, h_pc_with_intensity.header.stamp)
            v_pc_t = self._transform_cloud(v_pc_with_intensity, self.target_frame, v_pc_with_intensity.header.stamp)
        except TransformException as e:
            self.get_logger().warn(f"TF failed: {e}")
            return

        merged = self._merge_and_scale_pointclouds(h_pc_t, v_pc_t)
        self.pc_pub.publish(merged)

    def _scan_to_pointcloud(self, scan_msg: LaserScan) -> PointCloud2:
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
        if not cloud.header.frame_id or cloud.header.frame_id.strip() == '':
            self.get_logger().warn(
                f"Vertical cloud has empty frame_id. Overriding to '{self.vertical_frame_override}'."
            )
            cloud.header.frame_id = self.vertical_frame_override
        return cloud

    def _add_intensity(self, cloud: PointCloud2, intensity_val: float) -> PointCloud2:
        """Reads an XYZ cloud and returns an XYZI cloud with a constant intensity."""
        points_xyz = list(pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))
        
        # Add the intensity value to each point
        points_xyzi = [[p[0], p[1], p[2], intensity_val] for p in points_xyz]
        
        header = Header()
        header.frame_id = cloud.header.frame_id
        header.stamp = cloud.header.stamp
        
        # Define the fields for an XYZI point cloud
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        return pc2.create_cloud(header, fields, points_xyzi)

    def _transform_cloud(self, cloud: PointCloud2, target_frame: str, stamp) -> PointCloud2:
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame, cloud.header.frame_id, stamp, timeout=Duration(seconds=1.0)
            )
        except TransformException:
            trans = self.tf_buffer.lookup_transform(
                target_frame, cloud.header.frame_id, rclpy.time.Time(), timeout=Duration(seconds=1.0)
            )
        return do_transform_cloud(cloud, trans)

    def _merge_and_scale_pointclouds(self, a: PointCloud2, b: PointCloud2) -> PointCloud2:
        """Merges two XYZI point clouds and applies a scaling factor."""
        fields = ("x", "y", "z", "intensity")
        pts_a = list(pc2.read_points(a, field_names=fields, skip_nans=True))
        pts_b = list(pc2.read_points(b, field_names=fields, skip_nans=True))

        merged_points = pts_a
        merged_points.extend(pts_b)

        # Apply scaling if the factor is not 1.0
        if self.point_scale != 1.0:
            scaled_points = [
                [p[0] * self.point_scale, p[1] * self.point_scale, p[2] * self.point_scale, p[3]]
                for p in merged_points
            ]
        else:
            scaled_points = merged_points

        header = Header()
        header.frame_id = self.target_frame
        header.stamp = _newer_stamp(a.header.stamp, b.header.stamp)
        
        return pc2.create_cloud(header, a.fields, scaled_points)


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
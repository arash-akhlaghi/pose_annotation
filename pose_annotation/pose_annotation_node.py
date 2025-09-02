#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import MapMetaData
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

class PoseAnnotationNode(Node):
    def __init__(self):
        super().__init__('pose_annotation_node')
        self.get_logger().info("pose_annotation_node started")

        # Buffers
        self.map_metadata = None
        self.bridge = CvBridge()
        self.map_logged = False

        # Store transforms
        self.tf_positions = {}

        # Subscribers
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.create_subscription(MapMetaData, '/map_metadata', self.map_callback, 10)
        self.create_subscription(Image, '/map_image', self.map_image_callback, 10)

        # Publishers
        self.image_pub = self.create_publisher(Image, '/annotated_image', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/pose_markers', 10)

    def tf_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            child = transform.child_frame_id
            if child.startswith("robot_") or child.startswith("gate_"):
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                self.tf_positions[child] = (x, y)

    def map_callback(self, msg: MapMetaData):
        self.map_metadata = msg
        if not self.map_logged:
            self.get_logger().info(
                f"Map metadata received: origin=({msg.origin.position.x:.2f}, {msg.origin.position.y:.2f}), "
                f"res={msg.resolution}, size=({msg.width}x{msg.height})"
            )
            self.map_logged = True

    def map_image_callback(self, msg: Image):
        if self.map_metadata is None or 'robot_1' not in self.tf_positions:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge conversion failed: {e}")
            return

        annotated = cv_image.copy()
        width, height = annotated.shape[1], annotated.shape[0]

        # Step 1: Compute pixel positions of all points
        pixel_positions = {}
        for name, (x, y) in self.tf_positions.items():
            px, py = self.world_to_pixel(x, y)
            pixel_positions[name] = (px, py)

        # Step 2: Compute shift to place robot_1 at (67,37)
        px_robot1, py_robot1 = pixel_positions['robot_1']
        shift_x = 67 - px_robot1
        shift_y = 37 - py_robot1

        # Step 3: Shift all points
        for name in pixel_positions:
            px, py = pixel_positions[name]
            pixel_positions[name] = (px + shift_x, py + shift_y)

        # Step 4: Determine if expansion needed
        all_x = [px for px, py in pixel_positions.values()]
        all_y = [py for px, py in pixel_positions.values()]
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)

        add_left = -min_x if min_x < 0 else 0
        add_top = -min_y if min_y < 0 else 0
        new_w = max(width, max_x + 1 + add_left)
        new_h = max(height, max_y + 1 + add_top)

        if add_left > 0 or add_top > 0 or new_w > width or new_h > height:
            expanded = np.ones((new_h, new_w, 3), dtype=np.uint8) * 255
            expanded[add_top:add_top+height, add_left:add_left+width] = annotated
            annotated = expanded

            # Shift points by expansion
            for name in pixel_positions:
                px, py = pixel_positions[name]
                pixel_positions[name] = (px + add_left, py + add_top)

        # Step 5: Draw points
        for name, (px, py) in pixel_positions.items():
            color = (0, 0, 255)  # red
            cv2.circle(annotated, (px, py), 2, color, -1)

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
        self.image_pub.publish(annotated_msg)

        # Step 6: Publish markers for Foxglove
        marker_array = MarkerArray()
        marker_id = 0
        for name, (x, y) in self.tf_positions.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "pose_points"
            marker.id = marker_id
            marker_id += 1

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.lifetime = Duration(sec=0)
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def world_to_pixel(self, x, y):
        """Convert world coordinates to pixel coordinates (top-left origin)."""
        origin_x = self.map_metadata.origin.position.x
        origin_y = self.map_metadata.origin.position.y
        resolution = self.map_metadata.resolution
        width = self.map_metadata.width
        height = self.map_metadata.height

        px = int((x - origin_x) / resolution)
        py = int((y - origin_y) / resolution)
        py = height - py  # flip y-axis

        return px, py


def main(args=None):
    rclpy.init(args=args)
    node = PoseAnnotationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

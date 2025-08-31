#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import MapMetaData
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

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

        # Publisher
        self.image_pub = self.create_publisher(Image, '/annotated_image', 10)

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
        if self.map_metadata is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge conversion failed: {e}")
            return

        annotated = cv_image.copy()

        # Track required expansion
        min_x, min_y = 0, 0
        max_x, max_y = annotated.shape[1] - 1, annotated.shape[0] - 1

        pixel_positions = {}

        for name, (x, y) in self.tf_positions.items():
            px, py = self.world_to_pixel_raw(x, y)
            pixel_positions[name] = (px, py)

            min_x = min(min_x, px)
            min_y = min(min_y, py)
            max_x = max(max_x, px)
            max_y = max(max_y, py)

        # If expansion needed
        if min_x < 0 or min_y < 0 or max_x >= annotated.shape[1] or max_y >= annotated.shape[0]:
            shift_x = -min_x if min_x < 0 else 0
            shift_y = -min_y if min_y < 0 else 0

            new_w = max(annotated.shape[1], max_x + 1) + shift_x
            new_h = max(annotated.shape[0], max_y + 1) + shift_y

            expanded = np.ones((new_h, new_w, 3), dtype=np.uint8) * 255
            expanded[shift_y:shift_y + annotated.shape[0], shift_x:shift_x + annotated.shape[1]] = annotated
            annotated = expanded

            # Shift positions accordingly
            for name in pixel_positions:
                px, py = pixel_positions[name]
                pixel_positions[name] = (px + shift_x, py + shift_y)

        # Draw positions (smaller radius = 2)
        for name, (px, py) in pixel_positions.items():
            color = (0, 0, 255)
            cv2.circle(annotated, (px, py), 2, color, -1)
            cv2.putText(annotated, name, (px + 4, py - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
        self.image_pub.publish(annotated_msg)

    def world_to_pixel_raw(self, x, y):
        """Convert world coords (x, y) → raw pixel coords (no bounds check)."""
        origin_x = self.map_metadata.origin.position.x
        origin_y = self.map_metadata.origin.position.y
        resolution = self.map_metadata.resolution
        width = self.map_metadata.width
        height = self.map_metadata.height

        px = -int((x + origin_x) / resolution)
        py = int((y + origin_y) / resolution)
        py = -(height + py ) # flip y-axis

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

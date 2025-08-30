#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class PoseAnnotationNode(Node):
    def __init__(self):
        super().__init__('pose_annotation_node')
        self.get_logger().info("pose_annotation_node started")

        # Buffer for map metadata and image
        self.map_metadata = None
        self.map_image = None
        self.bridge = CvBridge()
        self.map_logged = False

        # Store transforms for robots and gates
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
            self.get_logger().info("Map metadata received")
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

        # Draw each robot/gate position
        for name, (x, y) in self.tf_positions.items():
            px, py = self.world_to_pixel(x, y)
            if px is not None:
                cv2.circle(annotated, (px, py), 5, (0, 0, 255), -1)  # Red circle
                cv2.putText(annotated, name, (px + 5, py - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
        self.image_pub.publish(annotated_msg)

    def world_to_pixel(self, x, y):
        """Convert world coords (x, y) → pixel coords in map image."""
        if self.map_metadata is None:
            return None, None

        origin_x = self.map_metadata.origin.position.x
        origin_y = self.map_metadata.origin.position.y
        resolution = self.map_metadata.resolution
        width = self.map_metadata.width
        height = self.map_metadata.height

        # Convert
        px = int((x - origin_x) / resolution)
        py = int((y - origin_y) / resolution)

        # Flip y-axis because image origin is top-left
        py = height - py

        if 0 <= px < width and 0 <= py < height:
            return px, py
        else:
            return None, None


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

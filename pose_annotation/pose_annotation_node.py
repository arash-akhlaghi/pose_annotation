#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
import cv2

class PixelAnnotator(Node):
    def __init__(self):
        super().__init__('pixel_annotator')
        self.bridge = CvBridge()

        # Parameters
        self.resolution = 0.05  # meters/pixel
        self.offset_x = -1.23
        self.offset_y = +2.09
        self.extra = 500  # border padding

        # Store TF positions
        self.tf_positions = {}

        # Subscribers
        self.create_subscription(Image, '/map_image', self.image_callback, 10)
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)

        # Publisher
        self.pub = self.create_publisher(Image, '/annotated_map', 10)

    def tf_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            child = transform.child_frame_id
            if child.startswith("robot_") or child.startswith("gate_"):
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                self.tf_positions[child] = (x, y)

    def world_to_pixel(self, x, y):
        # Apply offsets
        x += self.offset_x
        y += self.offset_y

        # Compute pixel X
        if x <= 0:
            px = self.extra + round(-x / self.resolution)
        else:
            px = self.extra - round(x / self.resolution)

        # Compute pixel Y
        py = self.extra - round(y / self.resolution)

        return px, py

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except Exception as e:
            self.get_logger().error(f"CV bridge conversion failed: {e}")
            return

        # Convert to BGR so we can draw colored circles
        annotated = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

        # Draw circles for all TF frames
        for name, (x, y) in self.tf_positions.items():
            px, py = self.world_to_pixel(x, y)
            cv2.circle(annotated, (px, py), 5, (0, 0, 255), -1)  # red circle

        # Publish annotated image
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
        annotated_msg.header = msg.header
        self.pub.publish(annotated_msg)
        self.get_logger().info("Published annotated map image")

def main(args=None):
    rclpy.init(args=args)
    node = PixelAnnotator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

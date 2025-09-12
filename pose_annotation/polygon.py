#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PointStamped, PolygonStamped, Point32


class PolygonDrawer(Node):
    """
    Subscribes to clicked points from a GUI (e.g. Foxglove / RViz)
    and publishes them as a PolygonStamped.
    """
    def __init__(self):
        super().__init__('polygon_drawer')

        # QoS: latched publisher
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Subscriber: clicked points
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10
        )

        # Publisher: polygon
        self.polygon_publisher = self.create_publisher(
            PolygonStamped,
            '/polygon_drawer',   # topic stays the same
            latching_qos
        )

        self.polygon_points = []
        self.header = None

        self.get_logger().info('Polygon Drawer node started.')
        self.get_logger().info('Publishing polygons on /polygon_drawer.')

    def point_callback(self, msg: PointStamped):
        self.header = msg.header

        point32 = Point32()
        point32.x = msg.point.x
        point32.y = msg.point.y
        point32.z = msg.point.z
        self.polygon_points.append(point32)

        # Publish polygon
        self.publish_polygon()

        # --- New: print coordinates to terminal ---
        print("\nCurrent Polygon Points:")
        for i, p in enumerate(self.polygon_points):
            print(f"  Point {i}: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")
        print("-------------------------")

    def publish_polygon(self):
        if not self.header or not self.polygon_points:
            return

        polygon_msg = PolygonStamped()
        polygon_msg.header = self.header
        polygon_msg.header.stamp = self.get_clock().now().to_msg()
        polygon_msg.polygon.points = self.polygon_points

        self.polygon_publisher.publish(polygon_msg)
        self.get_logger().info(f'Publishing polygon with {len(self.polygon_points)} points.')


def main(args=None):
    rclpy.init(args=args)
    node = PolygonDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

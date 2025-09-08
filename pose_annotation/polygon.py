import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PointStamped, PolygonStamped, Point32
from std_msgs.msg import Header

class PolygonDrawer(Node):
    """
    A node that subscribes to clicked points from a GUI like Foxglove
    and publishes them as a continuously updated PolygonStamped.
    """
    def __init__(self):
        super().__init__('polygon_drawer')
        
        # --- Change #1: Use a more robust QoS profile ---
        # Transient local means it "latches" the last message for new subscribers.
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Subscriber to the points clicked in Foxglove
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10)
        
        # --- Change #2: Publisher now uses PolygonStamped ---
        self.polygon_publisher = self.create_publisher(
            PolygonStamped, 
            'drawn_polygon', 
            latching_qos) # Use the new QoS profile
        
        # List to store the points of the polygon
        self.polygon_points = []
        # Store the header from the incoming points to use for the polygon
        self.header = None

        self.get_logger().info('Polygon Drawer node has started.')
        self.get_logger().info('Use the Foxglove 3D panel to publish points to /clicked_point.')
        self.get_logger().info('To clear the polygon, restart this node (Ctrl+C).')

    def point_callback(self, msg: PointStamped):
        """
        Callback for when a new point is clicked in the GUI.
        """
        self.get_logger().info(f'Received point in frame "{msg.header.frame_id}": (x: {msg.point.x:.2f}, y: {msg.point.y:.2f})')
        
        # --- Change #3: Store the header from the incoming message ---
        # This ensures our published polygon has the correct frame_id and timestamp
        self.header = msg.header

        # Convert the clicked point to a Point32 and add it to our list
        point32 = Point32()
        point32.x = msg.point.x
        point32.y = msg.point.y
        point32.z = msg.point.z
        
        self.polygon_points.append(point32)
        
        # Publish the updated polygon
        self.publish_polygon()

    def publish_polygon(self):
        """
        Publishes the current list of points as a PolygonStamped message.
        """
        if not self.header or not self.polygon_points:
            return

        # --- Change #4: Create and populate a PolygonStamped message ---
        polygon_stamped_msg = PolygonStamped()
        
        # Copy the header from the last clicked point
        polygon_stamped_msg.header = self.header
        # Update the timestamp to the current time
        polygon_stamped_msg.header.stamp = self.get_clock().now().to_msg()

        # Assign the points to the polygon field within the stamped message
        polygon_stamped_msg.polygon.points = self.polygon_points
        
        self.polygon_publisher.publish(polygon_stamped_msg)
        self.get_logger().info(f'Publishing polygon with {len(self.polygon_points)} points.')

def main(args=None):
    rclpy.init(args=args)
    polygon_drawer = PolygonDrawer()
    rclpy.spin(polygon_drawer)
    polygon_drawer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
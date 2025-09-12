import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav2_msgs.msg import CostmapFilterInfo

class FilterInfoPublisher(Node):
    """
    A node that continuously publishes the CostmapFilterInfo message.
    This ensures that the Nav2 costmap always knows how to handle the
    /drawn_polygon topic, even if the costmap restarts.
    """
    def __init__(self):
        super().__init__('filter_info_publisher')

        # Use a "transient local" (latched) QoS profile. The publisher will hold the
        # message and send it to any new subscribers.
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self.publisher_ = self.create_publisher(CostmapFilterInfo, '/costmap_filter_info', latching_qos)
        
        # Create the message to be published
        msg = CostmapFilterInfo()
        msg.header.frame_id = 'map'
        # The timestamp is set automatically by the publisher, but we can initialize it
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.type = 0  # This integer corresponds to the KEEP_OUT_ZONE type
        msg.filter_mask_topic = '/keepout_polygon' # This tells Nav2 which topic contains the keepout polygons

        # Publish the message once. Because the QoS is latched and the node
        # continues to spin, this configuration will be permanently available.
        self.publisher_.publish(msg)
        
        self.get_logger().info(
            f"Publishing and latching filter info: Polygons on '{msg.filter_mask_topic}' "
            "will be treated as Keepout Zones. Node will keep running."
        )

def main(args=None):
    rclpy.init(args=args)
    
    filter_info_publisher = FilterInfoPublisher()
    
    # rclpy.spin() keeps the node alive until it's shut down (e.g., by Ctrl+C)
    rclpy.spin(filter_info_publisher)
    
    filter_info_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
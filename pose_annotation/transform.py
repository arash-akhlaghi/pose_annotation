#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PointStamped
from std_msgs.msg import Float64MultiArray # <-- CHANGED: Import standard message
from tf_transformations import quaternion_from_euler

class GatesRobotsToMap(Node):
    def __init__(self):
        super().__init__('gates_robots_to_map')
        self.get_logger().info("GatesRobotsToMap node started")

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Subscriber (MODIFIED) ---
        # Subscribes to the output of your 'position.py' node
        self.subscription = self.create_subscription(
            Float64MultiArray,          # <-- CHANGED: Use the standard message type
            '/world_positions',         # <-- CHANGED: Use the correct topic name
            self.positions_callback,
            10
        )
        self.get_logger().info("Node initialized. Waiting for data on /world_positions.")


    def positions_callback(self, msg: Float64MultiArray):
        # We expect 14 models (10 gates, 4 robots), each with X and Y coordinates.
        if len(msg.data) < 28:
            self.get_logger().warn(f"Received message with {len(msg.data)} data points, expected 28. Skipping.")
            return

        try:
            # Get the transform from the robot's perspective ('base_link') to the 'map' frame
            base_to_map_tf = self.tf_buffer.lookup_transform(
                'map',          # Target frame
                'base_link',    # Source frame
                Time(),
                timeout=Duration(seconds=2.0)
            )
        except Exception as e:
            self.get_logger().warn(f"Could not get 'base_link'->'map' transform. Is navigation running? Error: {e}")
            return

        # Extract base_link's position (robot 1, at index 10) from the flat array.
        # Its X is at index 10*2=20, and its Y is at 10*2+1=21.
        base_link_in_world_x = msg.data[20]
        base_link_in_world_y = msg.data[21]
        
        # --- Process and transform all 14 models ---
        for i in range(14):
            # Define the name for the child frame
            if i < 10:
                name = f"gate_{i+1}"
            else: # i is 10, 11, 12, 13
                # Robot 1 (i=10) is our reference 'base_link', so we don't need to rebroadcast its position.
                if i == 10: 
                    continue
                name = f"robot_{i-9}" # robot_2, robot_3, robot_4

            # Extract the current model's world coordinates from the flat array
            world_x = msg.data[i * 2]
            world_y = msg.data[i * 2 + 1]

            # STEP 1: Calculate the model's position relative to base_link's position in the world.
            x_rel_to_base = world_x - base_link_in_world_x
            y_rel_to_base = world_y - base_link_in_world_y

            # STEP 2: Create a PointStamped message. This tells tf2 that these coordinates are in the 'base_link' frame.
            point_in_base_link = PointStamped()
            point_in_base_link.header.frame_id = "base_link"
            point_in_base_link.point.x = x_rel_to_base
            point_in_base_link.point.y = y_rel_to_base
            point_in_base_link.point.z = 0.0 # Assuming a 2D plane

            # STEP 3: Use tf2_geometry_msgs to perform the transformation to the 'map' frame.
            point_in_map = tf2_geometry_msgs.do_transform_point(point_in_base_link, base_to_map_tf)

            # STEP 4: Publish the resulting point as a new TF transform.
            t_msg = TransformStamped()
            t_msg.header.stamp = self.get_clock().now().to_msg()
            t_msg.header.frame_id = 'map'
            t_msg.child_frame_id = name
            
            # The transformed point gives us the translation
            t_msg.transform.translation.x = point_in_map.point.x
            t_msg.transform.translation.y = point_in_map.point.y
            t_msg.transform.translation.z = point_in_map.point.z

            # Since we only have position data, we publish a neutral orientation (no rotation).
            q = quaternion_from_euler(0.0, 0.0, 0.0)
            t_msg.transform.rotation.x = q[0]
            t_msg.transform.rotation.y = q[1]
            t_msg.transform.rotation.z = q[2]
            t_msg.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(t_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GatesRobotsToMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
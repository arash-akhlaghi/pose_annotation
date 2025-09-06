#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# Assuming your custom message is in a package named 'my_bridge'
from my_bridge.msg import Pose2Darray

class ManualWorldToMapConverter(Node):
    """
    Transforms coordinates from a conceptual 'world' frame to the 'map' frame
    by using 'base_link' as an intermediate. It then publishes markers for visualization.
    - Gates are shown as blue cubes.
    - Robots are shown as red spheres with labels correctly placed above them.
    """
    def __init__(self):
        super().__init__('manual_world_to_map_converter')
        self.get_logger().info("Starting Manual World-to-Map Converter.")

        # --- TF2 Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Subscriber ---
        self.positions_sub = self.create_subscription(
            Pose2Darray,
            '/gates_positions',
            self.positions_callback,
            10
        )

        # --- Publishers ---
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_markers', 10)
        self.pose_array_pub = self.create_publisher(PoseArray, '/map_frame_poses', 10)

        self.get_logger().info("Node initialized. Waiting for data.")

    def positions_callback(self, msg: Pose2Darray):
        if len(msg.gates_and_robots) < 14:
            self.get_logger().warn("Received message with fewer than 14 points. Skipping.")
            return

        # --- Get the base_link -> map transform from TF ---
        try:
            base_to_map_tf = self.tf_buffer.lookup_transform(
                'map',          # Target frame
                'base_link',    # Source frame
                Time(),
                timeout=Duration(seconds=2.0)
            )
        except Exception as e:
            self.get_logger().error(f"Could not get 'base_link'->'map' transform. Is navigation running? Error: {e}")
            return

        # --- Extract base_link's position in the 'world' frame ---
        base_link_in_world_x = msg.gates_and_robots[10].x
        base_link_in_world_y = msg.gates_and_robots[10].y
        self.get_logger().info(f"Base_link is at (x={base_link_in_world_x:.3f}, y={base_link_in_world_y:.3f}) in 'world' frame.")

        marker_array = MarkerArray()
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "map"
        marker_id = 0

        # --- Process every point ---
        for i, pos_in_world in enumerate(msg.gates_and_robots):
            # STEP 1: Manually transform from 'world' to 'base_link' frame
            x_rel_to_base = pos_in_world.x - base_link_in_world_x
            y_rel_to_base = pos_in_world.y - base_link_in_world_y

            point_in_base_link = PointStamped()
            point_in_base_link.header.frame_id = "base_link"
            point_in_base_link.point.x = x_rel_to_base
            point_in_base_link.point.y = y_rel_to_base
            point_in_base_link.point.z = 0.0

            # STEP 2: Use TF to transform from 'base_link' to 'map' frame
            point_in_map = tf2_geometry_msgs.do_transform_point(point_in_base_link, base_to_map_tf)

            # --- Add to PoseArray for debugging ---
            p = Pose()
            p.position.x = point_in_map.point.x
            p.position.y = point_in_map.point.y
            p.position.z = point_in_map.point.z
            p.orientation.w = 1.0
            pose_array.poses.append(p)

            # --- Create Visualization Markers ---
            is_gate = (i < 10)
            if is_gate:
                # --- Create a CUBE marker for a GATE (no label) ---
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "gates"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position = point_in_map.point
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.5
                marker.color = ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.8) # Blue
                marker.lifetime = Duration(seconds=5).to_msg()
                
                marker_array.markers.append(marker)
                marker_id += 1
            else:
                # --- Create a SPHERE and a TEXT marker for a ROBOT ---
                label = f"robot_{i-9}"
                
                # Sphere Marker (at ground level)
                sphere_marker = Marker()
                sphere_marker.header.frame_id = "map"
                sphere_marker.header.stamp = self.get_clock().now().to_msg()
                sphere_marker.ns = "robots"
                sphere_marker.id = marker_id
                sphere_marker.type = Marker.SPHERE
                sphere_marker.action = Marker.ADD
                sphere_marker.pose.position = point_in_map.point
                sphere_marker.pose.orientation.w = 1.0
                sphere_marker.scale.x = 0.3
                sphere_marker.scale.y = 0.3
                sphere_marker.scale.z = 0.3
                sphere_marker.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9) # Red
                sphere_marker.lifetime = Duration(seconds=5).to_msg()
                marker_array.markers.append(sphere_marker)
                marker_id += 1

                # Text Label Marker (placed above the sphere)
                text_marker = Marker()
                text_marker.header.frame_id = "map"
                text_marker.header.stamp = self.get_clock().now().to_msg()
                text_marker.ns = "robot_labels"
                text_marker.id = marker_id
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.text = label
                
                # ** THE FIX IS HERE **
                # We now explicitly set the text's position to be above the sphere's position
                # without modifying the sphere's position.
                text_marker.pose.position.x = point_in_map.point.x
                text_marker.pose.position.y = point_in_map.point.y
                text_marker.pose.position.z = point_in_map.point.z + 0.4 # Position text above the sphere
                
                text_marker.pose.orientation.w = 1.0
                text_marker.scale.z = 0.3 # Text height
                text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0) # White
                text_marker.lifetime = Duration(seconds=5).to_msg()
                marker_array.markers.append(text_marker)
                marker_id += 1

        # Publish the final results
        self.marker_pub.publish(marker_array)
        self.pose_array_pub.publish(pose_array)
        self.get_logger().info(f"Published {len(pose_array.poses)} transformed poses to the 'map' frame.")


def main(args=None):
    rclpy.init(args=args)
    node = ManualWorldToMapConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
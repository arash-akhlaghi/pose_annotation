#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Float64MultiArray

class ManualWorldToMapConverter(Node):
    """
    Transforms coordinates from a conceptual 'world' frame to the 'map' frame
    by using 'base_link' as an intermediate. It then publishes markers for visualization.
    - Gates are shown as larger blue cubes.
    - Robot_1 (the reference) is shown only as a text label.
    - Other robots are shown as red spheres with labels.
    - Gate pairs are labeled with 'in_X' and 'out_X' markers.
    """
    def __init__(self):
        super().__init__('manual_world_to_map_converter')
        self.get_logger().info("Starting Manual World-to-Map Converter.")

        # --- TF2 Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Subscriber ---
        self.positions_sub = self.create_subscription(
            Float64MultiArray,
            '/world_positions',
            self.positions_callback,
            10
        )

        # --- Publishers ---
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_markers', 10)
        self.pose_array_pub = self.create_publisher(PoseArray, '/map_frame_poses', 10)

        self.get_logger().info("Node initialized. Waiting for data on /world_positions.")

    def positions_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 28:
            self.get_logger().warn(f"Received message with {len(msg.data)} data points, expected 28. Skipping.")
            return

        try:
            base_to_map_tf = self.tf_buffer.lookup_transform('map', 'base_link', Time(), timeout=Duration(seconds=2.0))
        except Exception as e:
            self.get_logger().error(f"Could not get 'base_link'->'map' transform. Is navigation running? Error: {e}")
            return

        base_link_in_world_x = msg.data[20]
        base_link_in_world_y = msg.data[21]
        self.get_logger().info(f"Base_link is at (x={base_link_in_world_x:.3f}, y={base_link_in_world_y:.3f}) in 'world' frame.")

        marker_array = MarkerArray()
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "map"
        
        # Store transformed points to calculate midpoints later
        points_in_map = [Point() for _ in range(14)]
        marker_id = 0

        # --- STEP 1: Transform all points and create primary markers ---
        for i in range(14):
            world_x = msg.data[i * 2]
            world_y = msg.data[i * 2 + 1]

            x_rel_to_base = world_x - base_link_in_world_x
            y_rel_to_base = world_y - base_link_in_world_y

            point_in_base_link = PointStamped()
            point_in_base_link.header.frame_id = "base_link"
            point_in_base_link.point.x = x_rel_to_base
            point_in_base_link.point.y = y_rel_to_base

            point_in_map_stamped = tf2_geometry_msgs.do_transform_point(point_in_base_link, base_to_map_tf)
            points_in_map[i] = point_in_map_stamped.point

            p = Pose()
            p.position = points_in_map[i]
            p.orientation.w = 1.0
            pose_array.poses.append(p)

            is_gate = (i < 10)
            is_robot_1 = (i == 10)

            if is_gate:
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "gates"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position = points_in_map[i]
                marker.pose.orientation.w = 1.0
                marker.scale.x, marker.scale.y, marker.scale.z = 0.7, 0.7, 0.8
                marker.color = ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.8)
                marker.lifetime = Duration(seconds=5).to_msg()
                marker_array.markers.append(marker)
                marker_id += 1
            elif is_robot_1:
                label = f"robot_{i-9}"
                text_marker = self.create_label_marker(
                    label, points_in_map[i], marker_id, "robot_labels", offset_z=0.4
                )
                marker_array.markers.append(text_marker)
                marker_id += 1
            else:
                label = f"robot_{i-9}"
                sphere_marker = Marker()
                sphere_marker.header.frame_id = "map"
                sphere_marker.header.stamp = self.get_clock().now().to_msg()
                sphere_marker.ns = "robots"
                sphere_marker.id = marker_id
                sphere_marker.type = Marker.SPHERE
                sphere_marker.action = Marker.ADD
                sphere_marker.pose.position = points_in_map[i]
                sphere_marker.pose.orientation.w = 1.0
                sphere_marker.scale.x, sphere_marker.scale.y, sphere_marker.scale.z = 0.3, 0.3, 0.3
                sphere_marker.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9)
                sphere_marker.lifetime = Duration(seconds=5).to_msg()
                marker_array.markers.append(sphere_marker)
                marker_id += 1

                text_marker = self.create_label_marker(
                    label, points_in_map[i], marker_id + 100, "robot_labels", offset_z=0.4 # Use offset id to avoid collision
                )
                marker_array.markers.append(text_marker)
                marker_id += 1
        
        # --- STEP 2: Create labels for gate pairs ---
        gate_label_definitions = {
            "in_1": (0, 1), "in_2": (1, 2), "in_3": (2, 3), "in_4": (3, 4),
            "out_1": (5, 6), "out_2": (6, 7), "out_3": (7, 8), "out_4": (8, 9)
        }

        for label, (idx1, idx2) in gate_label_definitions.items():
            p1 = points_in_map[idx1]
            p2 = points_in_map[idx2]

            midpoint = Point()
            midpoint.x = (p1.x + p2.x) / 2.0
            midpoint.y = (p1.y + p2.y) / 2.0
            midpoint.z = (p1.z + p2.z) / 2.0

            # CHANGED: Increased offset_z from 0.6 to 0.9 to make the labels higher
            gate_label_marker = self.create_label_marker(
                label, midpoint, marker_id, "gate_pair_labels", offset_z=0.9, scale=0.35
            )
            gate_label_marker.color = ColorRGBA(r=0.9, g=0.9, b=0.2, a=1.0) # Yellow
            marker_array.markers.append(gate_label_marker)
            marker_id += 1

        self.marker_pub.publish(marker_array)
        self.pose_array_pub.publish(pose_array)
        self.get_logger().info(f"Published {len(pose_array.poses)} poses and {len(marker_array.markers)} markers.")

    def create_label_marker(self, text, position, marker_id, namespace, offset_z=0.0, scale=0.3):
        """Helper function to create a TEXT_VIEW_FACING marker."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.text = text
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z + offset_z
        marker.pose.orientation.w = 1.0
        marker.scale.z = scale
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0) # Default to white
        marker.lifetime = Duration(seconds=5).to_msg()
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = ManualWorldToMapConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
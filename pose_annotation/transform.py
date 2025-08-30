#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from my_bridge.msg import Pose2Darray
from tf_transformations import quaternion_from_euler
import numpy as np

class GatesRobotsToMap(Node):
    def __init__(self):
        super().__init__('gates_robots_to_map')
        self.get_logger().info("GatesRobotsToMap node started")

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber to your custom topic
        self.subscription = self.create_subscription(
            Pose2Darray,
            '/gates_positions',
            self.positions_callback,
            10
        )

    def positions_callback(self, msg: Pose2Darray):
        try:
            # Wait for the transform from base_link to map (up to 1 second)
            map_to_base_tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=15.0)
            )
        except Exception as e:
            self.get_logger().warn(f'base_link frame not yet available. Skipping this message. {e}')
            return

        # Transform gates (indices 0-9)
        for i in range(0, 10):
            self.publish_transformed_pose(
                msg.gates_and_robots[i].x,
                msg.gates_and_robots[i].y,
                f"gate_{i+1}",
                map_to_base_tf
            )

        # Transform robots (indices 10-13)
        for i in range(10, 14):
            self.publish_transformed_pose(
                msg.gates_and_robots[i].x,
                msg.gates_and_robots[i].y,
                f"robot_{i-9}",
                map_to_base_tf
            )

    def publish_transformed_pose(self, x, y, name, map_to_base_tf: TransformStamped):
        # Compose point in base_link frame
        point_base = np.array([x, y, 0.0])

        # Convert quaternion to rotation matrix
        q = map_to_base_tf.transform.rotation
        t = map_to_base_tf.transform.translation
        rot_matrix = self.quaternion_to_rot_matrix(q)
        trans_vec = np.array([t.x, t.y, t.z])

        # Transform point from base_link -> map
        point_map = rot_matrix.dot(point_base) + trans_vec

        # Publish as a static transform
        t_msg = TransformStamped()
        t_msg.header.stamp = self.get_clock().now().to_msg()
        t_msg.header.frame_id = 'map'
        t_msg.child_frame_id = name
        t_msg.transform.translation.x = float(point_map[0])
        t_msg.transform.translation.y = float(point_map[1])
        t_msg.transform.translation.z = 0.0

        # Set rotation to zero since you don't have orientation
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, 0.0)
        t_msg.transform.rotation.x = qx
        t_msg.transform.rotation.y = qy
        t_msg.transform.rotation.z = qz
        t_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t_msg)

    def quaternion_to_rot_matrix(self, q):
        from tf_transformations import quaternion_matrix
        quat = [q.x, q.y, q.z, q.w]
        return quaternion_matrix(quat)[:3, :3]

def main(args=None):
    rclpy.init(args=args)
    node = GatesRobotsToMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

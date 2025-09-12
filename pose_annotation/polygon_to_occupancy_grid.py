#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np
import cv2


class PolygonToOccupancyGrid(Node):
    def __init__(self):
        super().__init__('polygon_to_occupancy_grid')

        self.resolution = 0.05  # meters per cell
        self.margin = 0.5       # padding around polygon

        # Subscribe to polygon
        self.sub = self.create_subscription(
            PolygonStamped,
            '/polygon_drawer',   # <-- subscribe here
            self.polygon_callback,
            10
        )

        # Publisher: occupancy grid
        self.pub = self.create_publisher(
            OccupancyGrid,
            '/keepout_polygon',
            10
        )

        self.get_logger().info('Polygon → OccupancyGrid converter started.')

    def polygon_callback(self, msg: PolygonStamped):
        pts_world = [(p.x, p.y) for p in msg.polygon.points]
        if len(pts_world) < 3:
            self.get_logger().warn("Polygon has <3 points, skipping.")
            return

        xs, ys = zip(*pts_world)
        min_x, max_x = min(xs) - self.margin, max(xs) + self.margin
        min_y, max_y = min(ys) - self.margin, max(ys) + self.margin

        width = int(np.ceil((max_x - min_x) / self.resolution))
        height = int(np.ceil((max_y - min_y) / self.resolution))

        pts_grid = [
            [(int((x - min_x) / self.resolution), int((y - min_y) / self.resolution))]
            for (x, y) in pts_world
        ]
        pts_array = np.array([pts_grid], dtype=np.int32)

        grid = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(grid, pts_array, 255)

        occ_grid = np.where(grid > 0, 100, 0).astype(np.int8)

        map_info = MapMetaData()
        map_info.resolution = self.resolution
        map_info.width = width
        map_info.height = height
        map_info.origin.position.x = float(min_x)
        map_info.origin.position.y = float(min_y)
        map_info.origin.orientation.w = 1.0

        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = msg.header.frame_id
        grid_msg.info = map_info
        grid_msg.data = occ_grid.flatten().tolist()

        self.pub.publish(grid_msg)
        self.get_logger().info(
            f'Published OccupancyGrid {width}x{height} at origin ({min_x:.2f}, {min_y:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PolygonToOccupancyGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

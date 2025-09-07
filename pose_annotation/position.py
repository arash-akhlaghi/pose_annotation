import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

# Define the order and names of the 14 models
MODEL_NAMES = [
    "gate1", "gate2", "gate3", "gate4", "gate5",
    "gate6", "gate7", "gate8", "gate9", "gate10",
    "burger",  # This will be at index 10 (robot 1)
    "robot_2", # This will be at index 11 (robot 2)
    "robot_3", # This will be at index 12 (robot 3)
    "robot_4"  # This will be at index 13 (robot 4)
]

class PositionAggregator(Node):
    def __init__(self):
        super().__init__('position_aggregator_node')
        
        # Total number of models (10 gates + 4 robots)
        self.num_models = 14
        
        # Initialize a list to hold all positions.
        # Each element is a tuple (x, y), initialized to (0.0, 0.0)
        self.positions = [(0.0, 0.0)] * self.num_models
        
        # Create a publisher for the aggregated positions
        self.publisher_ = self.create_publisher(Float64MultiArray, 'world_positions', 10)
        
        # Create a subscriber for each model's odometry topic
        for i, model_name in enumerate(MODEL_NAMES):
            topic_name = f'/model/{model_name}/odometry'
            
            # The lambda function captures the index 'i' for the callback
            self.create_subscription(
                Odometry,
                topic_name,
                lambda msg, index=i: self.odometry_callback(msg, index),
                10
            )
            self.get_logger().info(f'Subscribing to {topic_name}')

        # Create a timer to publish the aggregated data at a regular interval (e.g., 10 Hz)
        self.timer = self.create_timer(0.1, self.publish_positions)

    def odometry_callback(self, msg, index):
        """Callback to update the position for a specific model."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Update the position in our list
        self.positions[index] = (x, y)

    def publish_positions(self):
        """Publish the aggregated list of positions."""
        msg = Float64MultiArray()
        
        # Flatten the list of tuples [(x1, y1), (x2, y2)] into [x1, y1, x2, y2]
        flat_positions = [coord for pos in self.positions for coord in pos]
        
        # Set the layout of the multi-array
        msg.layout.dim.append(MultiArrayDimension(label="models", size=self.num_models, stride=self.num_models * 2))
        msg.layout.dim.append(MultiArrayDimension(label="coordinates", size=2, stride=2))
        msg.layout.data_offset = 0
        
        # Assign the data
        msg.data = flat_positions
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing {self.num_models} aggregated positions.')

def main(args=None):
    rclpy.init(args=args)
    node = PositionAggregator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
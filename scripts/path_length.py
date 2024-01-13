import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
import numpy as np
from std_msgs.msg import Float64

class CalculatePathLengthNode(Node):
    def __init__(self):
        super().__init__('calculate_path_length_node')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # Replace with your odometry topic
            self.odometry_callback,
            10
        )

        self.path_length_publisher = self.create_publisher(Float64, '/path_length', 10)
        self.last_position = None
        self.path_length = 0.0
        self.start_time = None
        self.end_time = None

    def odometry_callback(self, msg):
        current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        if self.last_position is not None:
            # Calculate Euclidean distance between consecutive positions
            distance = np.linalg.norm(current_position - self.last_position)
            self.path_length += distance

        self.last_position = current_position

        if self.start_time is None:
            self.start_time = self.get_time_in_sec(msg.header.stamp)

        self.end_time = self.get_time_in_sec(msg.header.stamp)

        # Print path length continuously

        path_length_msg = Float64()
        path_length_msg.data = self.path_length
        self.path_length_publisher.publish(path_length_msg)
        # self.get_logger().info("Current Path Length: {:.4f} meters".format(self.path_length))

    def get_time_in_sec(self, stamp):
        return stamp.sec + stamp.nanosec / 1e9

    def main(self):
        self.get_logger().info("Calculating path length and traversal time using odometry. Press Ctrl+C to exit.")
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass  # Catch KeyboardInterrupt to print final path length

        if self.start_time is not None and self.end_time is not None:
            traversal_time = self.end_time - self.start_time
            self.get_logger().info("Final Path Length: {:.4f} meters".format(self.path_length))
            self.get_logger().info("Traversal Time: {:.4f} seconds".format(traversal_time))
        else:
            self.get_logger().info("Not enough data to calculate path length and traversal time.")

def main(args=None):
    rclpy.init(args=args)
    calculate_path_length_node = CalculatePathLengthNode()
    calculate_path_length_node.main()
    calculate_path_length_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

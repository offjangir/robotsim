import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class TurtlebotOdometry(Node):
    def __init__(self):
        super().__init__('turtlebot_odometry')
        self.get_logger().info('Starting Profiling Publisher')
        self.robot_mass =  1
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            10  # QoS profile, adjust as needed
        )
        self.total_energy_consumtion_publisher = self.create_publisher(Float64, '/energy_consumption', 10)

        self.linear_velocity_publisher = self.create_publisher(Float64, '/linear_velocity', 10)
        self.angular_velocity_publisher = self.create_publisher(Float64, '/angular_velocity', 10)
        self.linear_acceleration_publisher = self.create_publisher(Float64, '/linear_acceleration', 10)
        
        self.angular_acceleration_publisher = self.create_publisher(Float64, '/angular_acceleration', 10)
        self.linear_jerk_publisher = self.create_publisher(Float64, '/linear_jerk', 10)
        self.angular_jerk_publisher = self.create_publisher(Float64, '/angular_jerk', 10)

        # Initialize variables to store previous values
        self.prev_time = self.get_clock().now().to_msg()
        self.total_energy_consumtion = 0.0
        self.prev_linear_velocity = 0.0
        self.prev_angular_velocity = 0.0
        self.prev_linear_acceleration = 0.0
        self.prev_angular_acceleration = 0.0

    def odometry_callback(self, msg):
        current_time = self.get_clock().now().to_msg()

        # Calculate time difference
        dt = (current_time.sec - self.prev_time.sec) + (current_time.nanosec - self.prev_time.nanosec) * 1e-9

        # Extract linear velocity from the odometry message
        current_linear_velocity = msg.twist.twist.linear.x
        current_angular_velocity = msg.twist.twist.angular.z

        # Calculate acceleration
        linear_acceleration = (current_linear_velocity - self.prev_linear_velocity) / dt
        angular_acceleration = (current_angular_velocity - self.prev_angular_velocity) / dt

        # Calculate jerk (rate of change of acceleration)
        linear_jerk = (linear_acceleration - self.prev_linear_acceleration) / dt
        angular_jerk = (angular_acceleration - self.prev_angular_acceleration) / dt

        # Publish velocity
        linear_velocity_msg = Float64()
        linear_velocity_msg.data = current_linear_velocity
        self.linear_velocity_publisher.publish(linear_velocity_msg)

        angular_velocity_msg = Float64()
        angular_velocity_msg.data = current_angular_velocity
        self.angular_velocity_publisher.publish(angular_velocity_msg)


        # energy
        energy = self.total_energy_consumtion + 0.5*self.robot_mass*((current_linear_velocity ** 2)+(current_angular_velocity ** 2) )
        total_energy_consumtion_msg = Float64()
        total_energy_consumtion_msg.data = energy
        self.total_energy_consumtion_publisher.publish(total_energy_consumtion_msg)

        angular_velocity_msg = Float64()
        angular_velocity_msg.data = current_angular_velocity
        self.angular_velocity_publisher.publish(angular_velocity_msg)

        # Publish acceleration
        linear_acceleration_msg = Float64()
        linear_acceleration_msg.data = linear_acceleration
        self.linear_acceleration_publisher.publish(linear_acceleration_msg)

        angular_acceleration_msg = Float64()
        angular_acceleration_msg.data = angular_acceleration
        self.angular_acceleration_publisher.publish(angular_acceleration_msg)

        # Publish jerk
        linear_jerk_msg = Float64()
        linear_jerk_msg.data = linear_jerk
        self.linear_jerk_publisher.publish(linear_jerk_msg)

        angular_jerk_msg = Float64()
        angular_jerk_msg.data = angular_jerk
        self.angular_jerk_publisher.publish(angular_jerk_msg)

        # Update previous values for the next iteration
        self.total_energy_consumtion = energy
        self.prev_time = current_time
        self.prev_linear_velocity = current_linear_velocity
        self.prev_angular_velocity = current_angular_velocity
        self.prev_linear_acceleration = linear_acceleration
        self.prev_angular_acceleration = angular_acceleration

def main(args=None):
    rclpy.init(args=args)

    turtlebot_odometry = TurtlebotOdometry()
    rclpy.spin(turtlebot_odometry)
    turtlebot_odometry.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

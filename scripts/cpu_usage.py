import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import psutil
import time
import matplotlib.pyplot as plt

class CPUMonitorNode(Node):

    def __init__(self):
        super().__init__('cpu_monitor_node')

        self.publisher_controller = self.create_publisher(Float32, 'cpu_usage_controller', 10)
        self.publisher_planner = self.create_publisher(Float32, 'cpu_usage_planner', 10)

        node_name_controller = 'controller_server'
        node_name_planner = 'planner_server'

        pid_controller = self.get_ros2_pid(node_name_controller)
        pid_planner = self.get_ros2_pid(node_name_planner)

        self.process_controller_id = pid_controller
        self.process_planner_id = pid_planner

        self.timer_period = 1.0  # in seconds
        self.timer = self.create_timer(self.timer_period, self.publish_cpu_usage)

        # Data storage for final plotting
        self.times_controller = []
        self.cpu_usages_controller = []
        self.times_planner = []
        self.cpu_usages_planner = []

    def get_ros2_pid(self, node_name):
        # Iterate through all running processes
        for process in psutil.process_iter(['pid', 'name']):
            try:
                # Check if the process name matches the ROS 2 node name
                if node_name in process.info['name']:
                    return process.info['pid']
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass

    def get_cpu_usage(self, pid):
        process = psutil.Process(pid)
        return process.cpu_percent(interval=0.5)

    def publish_cpu_usage(self):
        cpu_usage_controller = self.get_cpu_usage(self.process_controller_id)
        cpu_usage_planner = self.get_cpu_usage(self.process_planner_id)
        self.get_logger().info(f'Current CPU Usage Controller: {cpu_usage_controller}%')
        self.get_logger().info(f'Current CPU Usage Planner: {cpu_usage_planner}%')

        msg_controller = Float32()
        msg_controller.data = cpu_usage_controller
        self.publisher_controller.publish(msg_controller)

        msg_planner = Float32()
        msg_planner.data = cpu_usage_planner
        self.publisher_planner.publish(msg_planner)

        # Append data for final plotting
        self.times_controller.append(time.time())
        self.cpu_usages_controller.append(cpu_usage_controller)
        self.times_planner.append(time.time())
        self.cpu_usages_planner.append(cpu_usage_planner)

def main(args=None):
    rclpy.init(args=args)

    cpu_monitor_node = CPUMonitorNode()

    try:
        rclpy.spin(cpu_monitor_node)
    except KeyboardInterrupt:
        pass

    cpu_monitor_node.destroy_node()
    rclpy.shutdown()

    # Plotting at the end
    plt.figure()

    plt.subplot(2, 1, 1)
    plt.plot(cpu_monitor_node.times_controller, cpu_monitor_node.cpu_usages_controller)
    plt.title('Controller CPU Usage')
    plt.xlabel('Time')
    plt.ylabel('CPU Usage (%)')

    plt.subplot(2, 1, 2)
    plt.plot(cpu_monitor_node.times_planner, cpu_monitor_node.cpu_usages_planner)
    plt.title('Planner CPU Usage')
    plt.xlabel('Time')
    plt.ylabel('CPU Usage (%)')

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()

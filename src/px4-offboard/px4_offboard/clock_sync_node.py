import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class ClockSyncNode(Node):
    def __init__(self):
        super().__init__('clock_sync_node')

        # --- QoS setup 
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(
            Int64, 
            '/sync_time', 
            self.qos_profile
        )
        # Publish the timestamp every 0.05 seconds (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)  

    def timer_callback(self):
        # Generate a unified timestamp for all drones
        timestamp_us = int(Clock().now().nanoseconds / 1000)

        msg = Int64()
        msg.data = timestamp_us
        self.publisher_.publish(msg)

        # Print the timestamp to the console
        # self.get_logger().info(f"Published timestamp: {timestamp_us}")

def main(args=None):
    rclpy.init(args=args)
    node = ClockSyncNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

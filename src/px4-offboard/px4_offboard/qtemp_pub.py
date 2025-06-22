import rclpy
from rclpy.node import Node
from mpc_msgs.msg import QuadReturnMPC
import numpy as np

class QPResultPublisher(Node):
    def __init__(self):
        super().__init__('qp_result_publisher')
        self.publisher = self.create_publisher(QuadReturnMPC, 'qmpc_result', 10)
        self.timer = self.create_timer(1.0, self.publish_result)
        self.h = 10
        self.nxi, self.nui = 13, 13
        self.idx = 2

    def publish_result(self):
        msg = QuadReturnMPC()
        msg.idx = self.idx
        msg.xi_temp = np.random.rand(self.h + 1, self.nxi).astype(np.float32).flatten().tolist()
        msg.ui_temp = np.random.rand(self.h, self.nui).astype(np.float32).flatten().tolist()
        msg.max_viol_i = np.random.rand()
        self.publisher.publish(msg)
        self.get_logger().info(f'Published result for UAV {self.idx}')
        self.get_logger().info(f'xi_temp: {msg.xi_temp}')
        self.get_logger().info(f'ui_temp: {msg.ui_temp}')

def main():
    rclpy.init()
    node = QPResultPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

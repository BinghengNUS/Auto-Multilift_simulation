import rclpy
from rclpy.node import Node
from mpc_msgs.msg import QuadReturnMPC
import numpy as np


class QPResultSubscriber(Node):
    def __init__(self):
        super().__init__('qp_result_subscriber')
        self.subscription = self.create_subscription(
            QuadReturnMPC, 'qmpc_result', self.callback, 10)

        self.h = 10
        self.nxi, self.nui = 13, 13
        self.nq = 6  # Number of UAVs
        self.xq_traj_temp = [None] * self.nq
        self.uq_traj_temp = [None] * self.nq
        self.max_vil = 0.0
        self.REC_TEMP = np.zeros(self.nq, dtype=bool)

    def callback(self, msg):
        quad_idx = msg.idx
        xi_temp = np.array(msg.xi_temp, dtype=np.float32).reshape(
            (self.h+1, self.nxi))
        ui_temp = np.array(msg.ui_temp, dtype=np.float32).reshape(
            (self.h, self.nui))
        max_viol_i = msg.max_viol_i

        self.xq_traj_temp[quad_idx] = xi_temp
        self.uq_traj_temp[quad_idx] = ui_temp
        self.max_vil = max(self.max_vil, max_viol_i)
        self.REC_TEMP[quad_idx] = True

        self.get_logger().info(f'Received result from UAV {quad_idx}')
        self.get_logger().info(f'xi_temp: {xi_temp}')
        self.get_logger().info(f'ui_temp: {ui_temp}')
        self.get_logger().info(f'Max violation: {max_viol_i}')


def main():
    rclpy.init()
    node = QPResultSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

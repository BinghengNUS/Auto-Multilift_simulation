import rclpy
from rclpy.node import Node
import numpy as np
from mpc_msgs.msg import BroadcastMPC


class DistributedMPCSubscriber(Node):
    def __init__(self):
        super().__init__('distributed_mpc_subscriber')
        self.subscription = self.create_subscription(
            BroadcastMPC,
            'distributed_mpc',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.flag = 0

    def recover_traj_list(self, flat_data: list, n_traj: int, traj_shape: tuple) -> list:
        """
        Restore flattened data into its original list structure.
        Args:
            flat_data (list): Flattened list received from a ROS2 message, e.g., msg.xq_traj.
            n_traj (int): Number of arrays in the list, e.g., nq.
            traj_shape (tuple): The shape to which each array should be restored, e.g., (horizon+1, nxi).
        Returns:
            list of np.ndarray: Original list structure, where each element is an ndarray with shape=traj_shape.
        """
        single_len = traj_shape[0] * traj_shape[1]
        flat_np = np.array(flat_data, dtype=np.float32)
        traj_list = [
            flat_np[i * single_len: (i + 1) * single_len].reshape(traj_shape)
            for i in range(n_traj)
        ]
        return traj_list

    def listener_callback(self, msg):
        self.get_logger().info('Received DistributedMPC message')

        # Reconstruct the data
        nq = 6
        horizon = 10
        nxi = 13
        nui = 13
        nxl = 13
        nul = 13

        self.flag = 0
        mpc_traj = msg.mpc_traj
        time_traj = msg.time_traj
        rec_temp = np.array(msg.rec_temp).reshape((nq, 1))
        # xq_traj = np.array(msg.xq_traj).reshape((nq, horizon + 1, nxi))
        # uq_traj = np.array(msg.uq_traj).reshape((nq, horizon, nui))
        xq_traj = self.recover_traj_list(msg.xq_traj, nq, (horizon + 1, nxi))
        uq_traj = self.recover_traj_list(msg.uq_traj, nq, (horizon, nui))
        xl_traj = np.array(msg.xl_traj).reshape((horizon + 1, nxl))
        ul_traj = np.array(msg.ul_traj).reshape((horizon, nul))

        self.get_logger().info(f'MPC_TRAJ: {mpc_traj}')
        self.get_logger().info(f'Time Trajectory: {time_traj}')
        self.get_logger().info(f'REC_TEMP: {rec_temp}')

        if self.flag == 0:
            self.get_logger().info(f'XQ Trajectory: {xq_traj[0]}')
            self.flag = 1
        # self.get_logger().info(f'UQ Trajectory Shape: {uq_traj.shape}')
        # self.get_logger().info(f'XL Trajectory Shape: {xl_traj.shape}')
        # self.get_logger().info(f'UL Trajectory Shape: {ul_traj.shape}')


def main(args=None):
    rclpy.init(args=args)
    distributed_mpc_subscriber = DistributedMPCSubscriber()
    rclpy.spin(distributed_mpc_subscriber)
    distributed_mpc_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

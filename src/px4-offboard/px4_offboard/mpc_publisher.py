import rclpy
from rclpy.node import Node
import numpy as np
from mpc_msgs.msg import BroadcastMPC

class DistributedMPCPublisher(Node):
    def __init__(self):
        super().__init__('distributed_mpc_publisher')
        self.publisher = self.create_publisher(BroadcastMPC, 'distributed_mpc', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.flag = 0
        # Initialize the data
        self.nq = 6
        self.horizon = 10
        self.nxi = 13
        self.nui = 13
        self.nxl = 13
        self.nul = 13

        self.MPC_TRAJ = False
        self.time_traj = 0.0
        self.REC_TEMP = np.ones((self.nq, 1), dtype=bool)
        self.xq_traj = []  # (nq * [horizon+1, nxi])
        self.uq_traj = []  # (nq * [horizon, nui])
        self.xl_traj = np.zeros((self.horizon + 1, self.nxl), dtype=np.float32)
        self.ul_traj = np.zeros((self.horizon, self.nul), dtype=np.float32)

        # Initialize the first time
        for ki in range(self.nq):
            self.ref_xq = np.random.rand(self.nxi, self.horizon + 1).astype(np.float32)
            self.ref_uq = np.random.rand(self.nui, self.horizon).astype(np.float32)
            self.xq_traj += [self.ref_xq.T]
            self.uq_traj += [self.ref_uq.T]

    def timer_callback(self):
        msg = BroadcastMPC()
        msg.mpc_traj = self.MPC_TRAJ
        msg.time_traj = self.time_traj
        msg.rec_temp = self.REC_TEMP.flatten().tolist()
        msg.xq_traj = np.concatenate([xq.flatten() for xq in self.xq_traj]).tolist()
        msg.uq_traj = np.concatenate([uq.flatten() for uq in self.uq_traj]).tolist()
        msg.xl_traj = self.xl_traj.flatten().tolist()
        msg.ul_traj = self.ul_traj.flatten().tolist()
        self.publisher.publish(msg)
        if self.flag == 0:
            self.get_logger().info(f'Published DistributedMPC message, xq_traj: {self.xq_traj[0]}')
            self.flag = 1

def main(args=None):
    rclpy.init(args=args)
    distributed_mpc_publisher = DistributedMPCPublisher()
    rclpy.spin(distributed_mpc_publisher)
    distributed_mpc_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
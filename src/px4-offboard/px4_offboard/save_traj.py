#!/usr/bin/env python3
"""
Preprocess offline trajectory and save per-drone trajectories to CSV files.
Each drone gets its own trajectory file for C++ consumption.
"""

import numpy as np
import csv
from pathlib import Path
from scipy.spatial.transform import Rotation


class TrajectoryPreprocessor:
    """Extract and save individual drone trajectories from multi-drone data."""
    
    def __init__(self, data_path: str, output_dir: str = "./drone_trajectories", num_drones: int = 6):
        self.data_path = Path(data_path)
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # Physical parameters (must match your system)
        self.num_drones = num_drones
        self.cable_length = 1.0
        self.payload_radius = 0.25
        self.dt = 0.01  # 100Hz
        
        # Coordinate transform: ENU -> NED
        self.T_enu2ned = np.array([[0, 1, 0],
                                    [1, 0, 0],
                                    [0, 0, -1]])
        
        # Compute attachment points on payload (body frame)
        self.rho = []
        self.offset_pos = []
        for i in range(self.num_drones):
            angle = 2 * np.pi * i / self.num_drones
            x = self.payload_radius * np.sin(angle)
            y = self.payload_radius * np.cos(angle)
            self.rho.append(np.array([x, y, 0.0]))
            
            offset_r = self.cable_length + self.payload_radius
            x_off = offset_r * np.sin(angle)
            y_off = offset_r * np.cos(angle)
            self.offset_pos.append(np.array([x_off, y_off, 0.0]))
        
        self.rho = np.array(self.rho)
        self.offset_pos = np.array(self.offset_pos)
        
        print(f"Initialized preprocessor for {self.num_drones} drones")
        print(f"Output directory: {self.output_dir}")
    
    def load_trajectory_data(self):
        """Load offline trajectory from numpy files."""
        # Payload trajectory
        xl_traj = np.load(self.data_path / 'xl_traj.npy', allow_pickle=True)
        self.payload_pos = xl_traj[:, 0:3]      # (T, 3) position
        self.payload_vel = xl_traj[:, 3:6]      # (T, 3) velocity
        self.payload_q = xl_traj[:, 6:10]       # (T, 4) quaternion
        self.payload_omega = xl_traj[:, 10:13]  # (T, 3) angular velocity
        
        # Cable directions and states
        xq_traj = np.load(self.data_path / 'xq_traj.npy', allow_pickle=True)
        self.cable_dir = xq_traj[:, :, 0:3]     # (N, T, 3)
        self.cable_omega = xq_traj[:, :, 3:6]   # (N, T, 3)
        
        self.num_timesteps = self.payload_pos.shape[0]
        print(f"Loaded trajectory: {self.num_timesteps} timesteps at {1/self.dt:.0f}Hz")
    
    def quat_to_rotation_matrix(self, q):
        """Convert quaternion [w,x,y,z] to rotation matrix."""
        return Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
    
    def compute_drone_trajectories(self):
        """Compute position and velocity for each drone at each timestep."""
        drone_trajs = []
        
        for drone_id in range(self.num_drones):
            traj_data = []
            
            for t in range(self.num_timesteps):
                # Payload state in ENU frame
                x_l_enu = self.payload_pos[t]
                v_l_enu = self.payload_vel[t]
                q_l_enu = self.payload_q[t]
                omega_l_enu = self.payload_omega[t]
                
                # Cable direction in ENU frame
                q_cable_enu = self.cable_dir[drone_id, t]
                
                # Convert to NED frame
                x_l = self.T_enu2ned @ x_l_enu
                v_l = self.T_enu2ned @ v_l_enu
                q_cable = self.T_enu2ned @ q_cable_enu
                
                # Payload rotation matrix: ENU body -> NED inertial
                R_l_enu = self.quat_to_rotation_matrix(q_l_enu)
                # R_l = self.T_enu2ned @ R_l_enu @ self.T_enu2ned  # Transform to NED
                R_l = self.T_enu2ned @ np.eye(3) @ self.T_enu2ned  # Transform to NED
                # Compute drone position
                x_drone = (x_l + R_l @ self.rho[drone_id] 
                          + self.cable_length * q_cable )
                
                # Approximate velocity (simple finite difference for cable)
                if t < self.num_timesteps - 1:
                    q_cable_next = self.T_enu2ned @ self.cable_dir[drone_id, t+1]
                    q_cable_dot = (q_cable_next - q_cable) / self.dt
                else:
                    q_cable_dot = np.zeros(3)
                
                # Compute R_l_dot approximately
                omega_l = self.T_enu2ned @ omega_l_enu
                omega_l_hat = self.hat(omega_l)
                R_l_dot = R_l @ omega_l_hat
                
                # Drone velocity
                v_drone = (v_l + R_l_dot @ self.rho[drone_id] 
                          - self.cable_length * q_cable_dot)
                
                # Store: [time, x, y, z, vx, vy, vz]
                traj_data.append([
                    t * self.dt,
                    x_drone[0], x_drone[1], x_drone[2],
                    v_drone[0], v_drone[1], v_drone[2]
                ])
            
            drone_trajs.append(np.array(traj_data))
            print(f"Computed trajectory for drone {drone_id}")
        
        return drone_trajs
    
    @staticmethod
    def hat(v):
        """Skew-symmetric matrix from 3D vector."""
        return np.array([[0, -v[2], v[1]],
                        [v[2], 0, -v[0]],
                        [-v[1], v[0], 0]])
    
    def save_trajectories(self, drone_trajs):
        """Save each drone's trajectory to CSV file."""
        for drone_id, traj in enumerate(drone_trajs):
            filename = self.output_dir / f"drone_{drone_id}_traj.csv"
            
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                # Header
                writer.writerow(['time', 'x', 'y', 'z', 'vx', 'vy', 'vz'])
                # Data
                writer.writerows(traj)
            
            print(f"Saved: {filename} ({len(traj)} points)")
    
    def run(self):
        """Execute full preprocessing pipeline."""
        print("\n=== Starting Trajectory Preprocessing ===")
        self.load_trajectory_data()
        drone_trajs = self.compute_drone_trajectories()
        self.save_trajectories(drone_trajs)
        print("\n=== Preprocessing Complete ===")


def main():
    # Path to your trajectory data
    num_drones = 3
    CoM = "001_-001"
    traj_path = "/home/carlson/ros2/multilift_ws/3quad_traj/Planning_plots_multiagent_meta_evaluation (rg_001_-001_3quad_l=1m_100Hz_smooth_useThis)"
    # num_drones = 6
    # CoM = "-003_002"
    # traj_path = "/home/carlson/ros2/multilift_ws/6quad_traj/Planning_plots_multiagent_meta_evaluation (rg_-003_002_100Hz_6s_l=1m_large_dist_new_smooth_useThis)"
    
    # Create preprocessor and run
    preprocessor = TrajectoryPreprocessor(
        data_path=traj_path,
        output_dir=f"./{num_drones}drone_trajectories_{CoM}",
        num_drones=num_drones
    )
    preprocessor.run()


if __name__ == "__main__":
    main()
# This launch file starts:
# 1) A visualizer node
# 2) A LMPC node, which synchronizes the drones and broadcast the control inputs
# 3) Multiple QMPC nodes, one for each drone

import os
import numpy as np
from rclpy.clock import Clock
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    '''  Process Illustration:
    Period1. Arm and take off:
        SyncNode: Waiting, only publish the timestamp to all quadrotors.
        QuadNode: Arm and take off together to the same altitude, by Geom Ctrl.

    Period2. Auto-multilift trajectory tracking (Distributed MPC):
        SyncNode: Broadcast info to control the state of the quadrotors.
            (MPC_TRAJ, time_traj, REC_TEMP[num_drones], opt_system.)
    Step(1).
        SyncNode: If (altitude of the payload satisfied),
            -> MPC Ctrl, MPC_TRAJ=TRUE.
        QuadNode: If (MPC_TRAJ=TRUE && REC_TEMP[idx] == False), 
            -> solve QMPC parallelly, 
            -> return x/u_temp_traj, x/u_maxviol_i to SyncNode.
    Step(2).
        Syncs
        SyncNode: If (REC_TEMP[all] == True), -> start LMPC, update xl_traj, ul_traj
            -> If (max_viol_i & max_viol_l < epsilon && ke > k_max) 
            -> update the opt_system and publish.

    Period3. Finish the task:
        SyncNode: If (time_traj > stm.Tc) -> MPC_TRAJ = False.
        QuadNode: If (MPC_TRAJ = False) -> ST_TRAJECTORY switch to ST_DONE.
    '''
    
    package_dir = get_package_share_directory('px4_offboard')
    nodes = [] 

    dt_ctrl = 5e-2 # 20Hz
    dt_broadcast = 1e-2 # 100Hz
    num_drones = 3.0
    takeoff_altitude = 4.0
    init_timestamp_us = int(Clock().now().nanoseconds / 1000)

    # Visualizer node
    nodes.append(
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='visualizer',
            name='visualizer'
        )
    )

    # Synchronization node and LMPC node
    nodes.append(
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='multilift_sync_node',
            name='multilift_sync_node',
            parameters=[{   
                'uav_para': [1.5, 0.02912, 0.02912, 0.05522, num_drones, 0.2], # Align with the Iris Quadrotor XXX
                'load_para': [3.0, 1.0], # payload mass, payload radius
                'cable_para': [1e9, 8e-6, 1e-2, 2.0], # Young's modulus, cross-sectional area, damping coefficient, cable length
                'Jl': [0.5 * x for x in [2.0, 2.0, 2.5]], # payload inertia, 0.5*Jl for 3 quadrotors, Jl for 6 quadrotors
                'rg': [0.1, 0.1, -0.1], # coordinate of the payload's CoM in {Bl}
                
                'dt_ctrl': dt_ctrl,
                'dt_broadcast': dt_broadcast,
                'angle_t': np.pi / 9,
                'altitude': takeoff_altitude,
                'trajectory_type': 'fig8'
            }]
        )
    )

    # Spawn multiple Drone QMPC nodes
    for idx in range(int(num_drones)):
        nodes.append(
            Node(
                package='px4_offboard',
                namespace='px4_offboard',
                executable='multilift_quad_node',
                name=f'multilift_quad_{idx}',
                parameters=[{
                    'uav_para': [1.5, 0.02912, 0.02912, 0.05522, num_drones, 0.2],
                    'load_para': [3.0, 1.0],
                    'cable_para': [1e9, 8e-6, 1e-2, 2.0],
                    'Jl': [0.5 * x for x in [2.0, 2.0, 2.5]],
                    'rg': [0.1, 0.1, -0.1],

                    'drone_idx': idx,
                    'dt_ctrl': dt_ctrl,
                    'dt_broadcast': dt_broadcast,
                    'angle_t': np.pi / 9,
                    'altitude': takeoff_altitude,
                    'init_timestamp': init_timestamp_us,
                }]
            )
        )

    return LaunchDescription(nodes)

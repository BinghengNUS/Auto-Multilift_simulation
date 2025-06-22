# This launch file starts:
# 1) A visualizer node
# 2) A Central node, which synchronizes the rest drones, responsible for the LoadMPC (LMPC)
# 3) Multiple QuadMPC nodes, one for each drone (except the central one)

'''  Process Illustration:
    Period1. Arm and take off:
        CentralNode: Arm and take off the corresponding drone, by Geom Ctrl.
        QuadNode: Arm and take off the corresponding drone, by Geom Ctrl.

    Period2. Auto-multilift trajectory tracking (Distributed MPC):
        CentralNode: Broadcast service requests to control the state of the quadrotors.
            (MPC_TRAJ, time_traj, REC_TEMP[num_drones], opt_system.)
    Step(1).
        CentralNode: If (altitude of the payload satisfied),
            -> MPC Ctrl, MPC_TRAJ=TRUE,
            -> Send service requests to ALL clients. (including itself)
        QuadNode: If request (MPC_TRAJ=TRUE && REC_TEMP[idx] == False), 
            -> solve QMPC parallelly, 
            -> return response (x/u_temp_traj, x/u_maxviol_i) to CentralNode.
    Step(2).
        CentralNode: If (receives service responses from the specific client), 
            -> update xq_traj, uq_traj, max_viol_i -> REC_TEMP[idx] = True.
    Step(3).
        CentralNode: If (REC_TEMP[all] == True), -> start LoadMPC, update xl_traj, ul_traj
            -> If (max_viol_i & max_viol_l < epsilon OR ke > k_max) 
            -> update the opt_system and publish, start next while loop.

    Period3. Finish the task:
        CentralNode: If (time_traj > stm.Tc) -> MPC_TRAJ = False. -> ST_MPC switch to ST_DONE.
        QuadNode: If (MPC_TRAJ = False) -> ST_MPC switch to ST_DONE.
'''

import os
import numpy as np
from rclpy.clock import Clock
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
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

    # Central Node
    nodes.append(
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='srv_mpc_central',
            name='srv_mpc_central',
            parameters=[{   
                'uav_para': [1.5, 0.02912, 0.02912, 0.05522, num_drones, 0.2], # Align with the Iris Quadrotor XXX
                'load_para': [3.0, 1.0], # payload mass, payload radius
                'cable_para': [1e9, 8e-6, 1e-2, 2.0], # Young's modulus, cross-sectional area, damping coefficient, cable length
                'Jl': [0.5 * x for x in [2.0, 2.0, 2.5]], # payload inertia, 0.5*Jl for 3 quadrotors, Jl for 6 quadrotors
                'rg': [0.1, 0.1, -0.1], # coordinate of the payload's CoM in {Bl}
                
                'drone_idx': 0,
                'dt_ctrl': dt_ctrl,
                'dt_broadcast': dt_broadcast,
                'angle_t': np.pi / 9,
                'altitude': takeoff_altitude,
                'init_timestamp': init_timestamp_us,
                'trajectory_type': 'fig8'
            }]
        )
    )

    # Spawn multiple Drone QMPC nodes
    for idx in range(int(num_drones) - 1):  # Central node is drone_idx=0
        nodes.append(
            Node(
                package='px4_offboard',
                namespace='px4_offboard',
                executable='srv_mpc_quad',
                name=f'srv_mpc_quad_{idx +1}',
                parameters=[{
                    'uav_para': [1.5, 0.02912, 0.02912, 0.05522, num_drones, 0.2],
                    'load_para': [3.0, 1.0],
                    'cable_para': [1e9, 8e-6, 1e-2, 2.0],
                    'Jl': [0.5 * x for x in [2.0, 2.0, 2.5]],
                    'rg': [0.1, 0.1, -0.1],

                    'drone_idx': idx + 1,  # Start from 1 for the first quadrotor
                    'dt_ctrl': dt_ctrl,
                    'dt_broadcast': dt_broadcast,
                    'angle_t': np.pi / 9,
                    'altitude': takeoff_altitude,
                    'init_timestamp': init_timestamp_us
                }]
            )
        )

    return LaunchDescription(nodes)

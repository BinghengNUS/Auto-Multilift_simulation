import math
import numpy as np
import launch
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

try:
    from px4_offboard.get_data import DataLoader       
except ImportError as e:
    raise RuntimeError(
        "Cannot import DataLoader from get_data.py. "
        "Add the directory that contains get_data.py to PYTHONPATH "
        "or move the launch file into the same Python package."
    ) from e


def launch_setup(context, *args, **kwargs):
    """Called by OpaqueFunction at launch-generation time."""
    # Instantiate the loader and compute the drone positions (shape: N×3).
    loader = DataLoader()                       # heavy imports (casadi, numpy) are okay here
    R_enu_2_ned = np.array([
        [0, 1,  0],
        [1, 0,  0],
        [0, 0, -1]
    ])
    cable_length = loader.cable_length
    payload_radius = loader.rl
    num_drones = int(loader.num_drones)
    angle = loader.alpha
    offset_radius = cable_length + payload_radius
    drone_pos_init = loader.get_drone_pos()          # ndarray, metres, ENU frame
    drone_pos = drone_pos_init.copy()  # N×3, metres, NED frame
    for i in range(num_drones):
        ri = np.array([
            offset_radius * math.cos(i * angle),
            offset_radius * math.sin(i * angle),
            0.0
        ])  
        drone_pos[i,:] = drone_pos_init[i,:] - ri
    drone_pos_NED = (R_enu_2_ned @ drone_pos.T).T

    fsm_nodes = []
    for i in range(num_drones):
        fsm_nodes.append(
            Node(
                package="offboard_state_machine",
                executable="offboard_fsm_node",
                name=f"offboard_fsm_node_{i}",
                output="screen",
                parameters=[{
                    "drone_id": i,
                    "goto_x":   float(drone_pos_NED[i, 0]),
                    "goto_y":   float(drone_pos_NED[i, 1]),
                    "goto_z":   float(drone_pos_NED[i, 2]),
                    "num_drones": num_drones,
                }],
                # keep default remappings – the FSM already builds its own namespace
            )
        )

    return fsm_nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])

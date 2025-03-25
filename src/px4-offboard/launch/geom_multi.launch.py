# This launch file starts:
# 1) A visualizer node
# 2) A clock synchronization node
# 3) Multiple geom_controller nodes for each drone agent
# 4) RViz for visualization

from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.clock import Clock
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')

    # Collect all nodes in a list, then return a LaunchDescription at the end.
    nodes = []
    init_timestamp_us = int(Clock().now().nanoseconds / 1000)
    num_drones = 6

    # Visualizer node
    nodes.append(
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='visualizer',
            name='visualizer'
        )
    )

    # Clock synchronization node
    nodes.append(
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='clock_sync_node',
            name='clock_sync_node'
        )
    )

    # Spawn multiple Drone Geometry Controllers
    for idx in range(num_drones):
        nodes.append(
            Node(
                package='px4_offboard',
                namespace='px4_offboard',
                executable='geom_multi',
                name=f'geom_multidrone_{idx}',
                parameters=[{
                    'init_timestamp': init_timestamp_us,
                    'drone_idx': idx,
                    'altitude': 4.0,
                    # 'trajectory_type': 'hover'

                    # FIXME: Just for single drone, cause collision for multiple drones
                    'trajectory_type': 'circle'
                    # 'trajectory_type': 'helix'
                    # 'trajectory_type': 'lemniscate'
                    # 'trajectory_type': 'sinusoid'
                }]
            )
        )

    # RViz node for visualization
    # nodes.append(
    #     Node(
    #         package='rviz2',
    #         namespace='',
    #         executable='rviz2',
    #         name='rviz2',
    #         arguments=['-d', os.path.join(package_dir, 'visualize.rviz')]
    #     )
    # )

    return LaunchDescription(nodes)

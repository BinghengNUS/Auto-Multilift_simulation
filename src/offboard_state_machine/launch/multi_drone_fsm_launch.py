import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

NUM_DRONES = 6
TAKEOFF_HEIGHT = 2.0  # meters

def generate_launch_description():
    # Declare the launch arguments for drones 
    drone_args = [
        DeclareLaunchArgument(f'drone_id_{i}',
                              default_value=str(i),
                              description=f'Drone {i} ID')
        for i in range(NUM_DRONES)
    ]

    # Launch one offboard_fsm_node per drone
    drone_nodes = [
        Node(
            package='offboard_state_machine',
            executable='offboard_fsm_node',
            name=f'offboard_fsm_node_{i}',
            output='screen',
            parameters=[{'drone_id': LaunchConfiguration(f'drone_id_{i}')}],
            remappings=[
                (f'/state/command_drone_{i}', f'/state/command_drone_{i}'),
                (f'/state/state_drone_{i}',   f'/state/state_drone_{i}')
            ]
        )
        for i in range(NUM_DRONES)
    ]

    return LaunchDescription(drone_args + drone_nodes)

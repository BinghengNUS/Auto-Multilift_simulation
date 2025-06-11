# Auto-Multilift_simulation

This is the ROS2 workspace of Auto-Multilift simulation. Please run:

```bash
colcon build
```

to build the ros2 workspace.

Run 

```bash
ros2 launch px4_offboard offboard_position_control.launch.py
```

to run the single iris drone offboard control demo.

## References

[PX4 control diagram](https://docs.px4.io/main/en/flight_stack/controller_diagrams.html)

[px4_offboard](https://github.com/Jaeyoung-Lim/px4-offboard) (Demo source)

[ROS 2 Offboard Control Example](https://docs.px4.io/main/en/ros2/offboard_control.html)

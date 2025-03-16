from px4_offboard.matrix_utils import hat, vee, deriv_unit_vector, saturate
from px4_offboard.rotation_to_quaternion import rotation_matrix_to_quaternion

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from std_msgs.msg import Int64
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleAttitudeSetpoint, OffboardControlMode, VehicleStatus, VehicleCommand


class MultiGeomNode(Node):
    ST_INIT = 0
    ST_ARMING = 1
    ST_TAKEOFF = 2
    ST_TRAJECTORY = 3
    ST_DONE = 4

    def __init__(self):
        super().__init__(f'MultiDrones_Offboard_GeomCtrl')

        ## State machine setup ##
        self.current_state = self.ST_INIT

        self.offboard_count = 0
        self.timer_period_offboard = 0.05  # Timer period (s)
        self.timer_offboard = self.create_timer(
            self.timer_period_offboard,
            self.offboard_state_machine
        )

        ## QoS setup ##
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        ## Declare parameters ##
        self.declare_parameter('init_timestamp', None).value
        self.init_timestamp = self.get_parameter('init_timestamp').value
        if self.init_timestamp is None:
            self.get_logger().error("Initial timestamp is not available, check the parameter.")
            raise ValueError(
                "Initial timestamp is not available, check the parameter.")

        self.declare_parameter('drone_idx', 0)
        self.drone_idx = self.get_parameter('drone_idx').value
        self.prefix = '' if self.drone_idx == 0 else f'/px4_{self.drone_idx}'

        self.declare_parameter('altitude', 5.0)
        self.altitude = self.get_parameter('altitude').value

        self.declare_parameter('trajectory_type', 'hover')
        self.trajectory_type = self.get_parameter('trajectory_type').value

        ## Initialize variables ##
        # flag to record initial offset of each drone for later trajectory generation
        self.xy_offset_flag = False
        self.x_offset = 0.0
        self.y_offset = 0.0
        # Takeoff time (s)
        self.takeoff_time = 10.0
        self.timestamp_us = self.init_timestamp

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        # Circle trajectory parameters
        self.circle_W = 0.5  # Angular velocity
        self.circle_radius = 3.0  # Circle radius

        ## Geometry Controller Initialization ##
        # Control initialization
        self.x = np.zeros(3)
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.R = np.identity(3)
        self.W = np.zeros(3)

        # Desired states
        self.xd = np.zeros(3)
        self.xd_dot = np.zeros(3)
        self.xd_2dot = np.zeros(3)
        self.xd_3dot = np.zeros(3)
        self.xd_4dot = np.zeros(3)

        self.Wd = np.zeros(3)
        self.Wd_dot = np.zeros(3)

        self.Rd = np.identity(3)

        self.b1d = np.array([0.0, 1.0, 0.0])
        self.b1d_dot = np.zeros(3)
        self.b1d_2dot = np.zeros(3)

        self.b3d = np.zeros(3)
        self.b3d_dot = np.zeros(3)
        self.b3d_2dot = np.zeros(3)

        self.b1c = np.zeros(3)
        self.wc3 = 0.0
        self.wc3_dot = 0.0

        self.e1 = np.array([1.0, 0.0, 0.0])
        self.e2 = np.array([0.0, 1.0, 0.0])
        self.e3 = np.array([0.0, 0.0, 1.0])

        self.m = 1.5  # Mass (kg)
        self.g = 9.81  # Gravity (m/s^2)
        self.J = np.diag([0.02912, 0.02912, 0.05522])  # Inertia matrix

        # XXX Controller gains XXX
        self.kX = np.diag([8.0, 8.0, 10.0])
        self.kV = np.diag([5.0, 5.0, 10.0])

        self.f_total = 0.0  # Total thrust

        # XXX Maximum thrust (modify based on actual vehicle) XXX
        self.thrust_ratio = 2.00
        self.max_thrust_newtons = self.m * self.g * self.thrust_ratio

        # Errors
        self.ex = np.zeros(3)
        self.ev = np.zeros(3)
        self.eR = np.zeros(3)
        self.eW = np.zeros(3)

        ## Initialize publishers ##
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            f'{self.prefix}/fmu/in/offboard_control_mode',
            self.qos_profile
        )
        self.publisher_vehicle_command = self.create_publisher(
            VehicleCommand,
            f'{self.prefix}/fmu/in/vehicle_command',
            self.qos_profile
        )
        self.publisher_vehicle_attitude_setpoint = self.create_publisher(
            VehicleAttitudeSetpoint,
            f'{self.prefix}/fmu/in/vehicle_attitude_setpoint',
            self.qos_profile
        )

        ## Initialize subscribers ##
        # Global Subscriber to **sync time between multiple drones**
        self.create_subscription(
            Int64,
            '/sync_time',
            self.sync_time_callback,
            self.qos_profile
        )

        # Local Subscriber to update arming, navigation state
        self.create_subscription(
            VehicleStatus,
            f'{self.prefix}/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            self.qos_profile
        )

        # Local Subscribers to update variables used in geometry controller
        self.create_subscription(
            VehicleOdometry,
            f'{self.prefix}/fmu/out/vehicle_odometry',
            self.vehicle_odometry_callback,
            self.qos_profile
        )

        self.create_subscription(
            VehicleLocalPosition,
            f'{self.prefix}/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            self.qos_profile
        )

    # --- State Machine ---

    def offboard_state_machine(self):
        """
        Main state machine to handle drone offboard states:
          ST_INIT       -> Wait a few cycles, then send OFFBOARD + ARM commands
          ST_ARMING     -> Wait until OFFBOARD + ARMED, then start takeoff
          ST_TAKEOFF    -> Generate takeoff trajectory; switch to self-defined trajectory if needed
          ST_TRAJECTORY -> Follow custom trajectory; optionally move to DONE
          ST_DONE       -> Maintains or ends operation
        """
        # Always publish OffboardControlMode
        offboard_msg = OffboardControlMode()
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = True
        offboard_msg.body_rate = False
        offboard_msg.timestamp = self.timestamp_us
        self.publisher_offboard_mode.publish(offboard_msg)

        # 1) ST_INIT: send offboard + arm after a certain number of cycles
        if self.current_state == self.ST_INIT:
            if (self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
                    or self.arming_state != VehicleStatus.ARMING_STATE_ARMED) \
                    and self.offboard_count >= 10:
                self.publish_vehicle_command(
                    drone_idx=self.drone_idx,
                    command=VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0,  # custom mode
                    param2=6.0,  # offboard
                    timestamp_us=self.timestamp_us
                )
                self.arm(drone_idx=self.drone_idx,
                         timestamp_us=self.timestamp_us)
                self.get_logger().info("--- Sent OFFBOARD + ARM command. ---")
                self.current_state = self.ST_ARMING

        # 2) ST_ARMING: wait until OFFBOARD + ARMED
        elif self.current_state == self.ST_ARMING:
            # Check if drone is in offboard + armed, then start takeoff
            if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                    and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
                self.get_logger().info("----- Starting takeoff -----")
                self.takeoff_start_count = self.offboard_count
                self.takeoff_traj_timer = self.create_timer(
                    0.05, self.generate_takeoff_trajectory)
                self.geom_ctrl_timer = self.create_timer(
                    0.05, self.geom_publish_command)
                self.current_state = self.ST_TAKEOFF
            else:
                # NOTE The command might fail! Retry mode + arm command if not ready.
                self.publish_vehicle_command(
                    drone_idx=self.drone_idx,
                    command=VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0,  # custom mode
                    param2=6.0,  # offboard
                    timestamp_us=self.timestamp_us
                )
                self.arm(drone_idx=self.drone_idx,
                         timestamp_us=self.timestamp_us)
                self.get_logger().info("--- Resent OFFBOARD + ARM command. ---")

        # 3) ST_TAKEOFF: switch to self-defined trajectory if altitude reached
        elif self.current_state == self.ST_TAKEOFF:
            altitude_threshold = 0.01 * self.altitude

            # Check both altitude and time as conditions
            if (abs(self.x[2] - self.last_z) < altitude_threshold
                and self.trajectory_type != 'hover'
                and (self.offboard_count - self.takeoff_start_count)
                    >= int(self.takeoff_time * 2 / self.timer_period_offboard)):
                self.get_logger().info("----- Starting self-defined trajectory -----")
                self.takeoff_traj_timer.cancel()
                self.selfdefine_traj_timer = self.create_timer(
                    0.05,
                    lambda: self.generate_trajectory(type=self.trajectory_type)
                )
                self.traj_start_count = self.offboard_count
                self.current_state = self.ST_TRAJECTORY

        # 4) ST_TRAJECTORY: after some time, stop the offboard_state_machine if desired
        elif self.current_state == self.ST_TRAJECTORY:
            if (self.offboard_count - self.traj_start_count) >= 3e3:  # 150s
                self.get_logger().info("----- Stopping trajectory -----")
                self.timer_offboard.cancel()
                self.current_state = self.ST_DONE

        # TODO
        # 5) ST_DONE: do nothing or add more logic if needed
        elif self.current_state == self.ST_DONE:
            pass

        # Count up each cycle
        self.offboard_count += 1

    # --- Trajectory Generation ---

    def generate_takeoff_trajectory(self):
        """ 
        Generate the takeoff trajectory to reach the hover attitude.
        """
        # Initialize starting point only when first entering trajectory mode
        if not hasattr(self, 'takeoff_init_flag'):
            self.takeoff_start = self.timestamp_us
            self.takeoff_init_flag = True

        # Convert timestamp (microseconds) to seconds
        dt_s = (self.timestamp_us - self.takeoff_start) * 1e-6

        # record initial z to avoid jumps
        current_z = self.x[2]
        z_target = -self.altitude
        if dt_s < self.takeoff_time:
            # Linear interpolation to reach target altitude
            alpha = dt_s / self.takeoff_time
            self.xd[2] = current_z + alpha * (z_target - current_z)
            self.xd_dot[2] = (z_target - current_z)/self.takeoff_time
            self.xd_2dot[2] = 0.0
            self.xd_3dot[2] = 0.0
            self.xd_4dot[2] = 0.0
        else:
            self.xd[2] = z_target
            self.xd_dot[2] = 0.0
            self.xd_2dot[2] = 0.0
            self.xd_3dot[2] = 0.0
            self.xd_4dot[2] = 0.0

        # Keep horizontal position unchanged during takeoff, only modify z direction
        x_init = self.x_offset
        y_init = self.y_offset
        x_final = self.x_offset / 3.0
        y_final = self.y_offset / 3.0
        if dt_s < self.takeoff_time:
            alpha = dt_s / self.takeoff_time
            self.xd[0] = x_init + alpha * (x_final - x_init)
            self.xd[1] = y_init + alpha * (y_final - y_init)
            # First-order derivative during linear interpolation
            self.xd_dot[0] = (x_final - x_init)/self.takeoff_time
            self.xd_dot[1] = (y_final - y_init)/self.takeoff_time
            self.xd_2dot[0] = self.xd_2dot[1] = 0.0
            self.xd_3dot[0] = self.xd_3dot[1] = 0.0
            self.xd_4dot[0] = self.xd_4dot[1] = 0.0
        else:
            self.xd[0] = x_final
            self.xd[1] = y_final
            self.xd_dot[0] = self.xd_dot[1] = 0.0
            self.xd_2dot[0] = self.xd_2dot[1] = 0.0
            self.xd_3dot[0] = self.xd_3dot[1] = 0.0
            self.xd_4dot[0] = self.xd_4dot[1] = 0.0

        # Set the forward direction b1d, default to global y-axis direction
        self.b1d = np.array([0.0, 1.0, 0.0])
        self.b1d_dot = np.zeros(3)
        self.b1d_2dot = np.zeros(3)

    def generate_trajectory(self, type='helix'):
        """
        FIXME: just for single drone, cause collision for multiple drones.
        Generate the desired trajectory and update the desired states, after the takeoff.
        """
        # Initialize starting point only when first entering trajectory mode
        if not hasattr(self, 'traj_init_flag'):
            self.x_start = self.x.copy()
            self.t_start = self.timestamp_us
            self.traj_init_flag = True

        # Calculate the elapsed time since the trajectory started (in seconds)
        dt_traj = (self.timestamp_us - self.t_start) * 1e-6

        if type == 'circle':
            circle_W = self.circle_W
            circle_radius = self.circle_radius

            theta = circle_W * dt_traj

            circle_W2 = circle_W * circle_W
            circle_W3 = circle_W2 * circle_W
            circle_W4 = circle_W3 * circle_W

            offset_x = self.x_start[0] - circle_radius * np.cos(0)
            offset_y = self.x_start[1] - circle_radius * np.sin(0)

            # x component
            self.xd[0] = circle_radius * np.cos(theta) + offset_x
            self.xd_dot[0] = - circle_radius * circle_W * np.sin(theta)
            self.xd_2dot[0] = - circle_radius * circle_W2 * np.cos(theta)
            self.xd_3dot[0] = circle_radius * circle_W3 * np.sin(theta)
            self.xd_4dot[0] = circle_radius * circle_W4 * np.cos(theta)

            # y component
            self.xd[1] = circle_radius * np.sin(theta) + offset_y
            self.xd_dot[1] = circle_radius * circle_W * np.cos(theta)
            self.xd_2dot[1] = - circle_radius * circle_W2 * np.sin(theta)
            self.xd_3dot[1] = - circle_radius * circle_W3 * np.cos(theta)
            self.xd_4dot[1] = circle_radius * circle_W4 * np.sin(theta)

            # z component (altitude)
            self.xd[2] = - self.altitude

            # XXX Body forward axis (b1d) orientation XXX
            w_b1d = 2.0 * np.pi / 10.0

            theta_b1d = w_b1d * dt_traj
            self.b1d = np.array([np.cos(theta_b1d), np.sin(theta_b1d), 0.0])
            self.b1d_dot = np.array(
                [- w_b1d * np.sin(theta_b1d), w_b1d * np.cos(theta_b1d), 0.0])
            self.b1d_2dot = np.array([- w_b1d*w_b1d * np.cos(theta_b1d),
                                      - w_b1d*w_b1d * np.sin(theta_b1d),
                                      0.0])

        elif type == 'helix':
            # Helix parameters
            helix_radius = 2.0
            helix_omega = 0.5   # Horizontal rotation speed
            helix_vert_speed = 0.1  # Vertical climb rate

            theta = helix_omega * dt_traj

            offset_x = self.x_start[0] - helix_radius * np.cos(0)
            offset_y = self.x_start[1] - helix_radius * np.sin(0)

            helix_omega2 = helix_omega**2
            helix_omega3 = helix_omega2 * helix_omega
            helix_omega4 = helix_omega3 * helix_omega

            # x(t)
            self.xd[0] = helix_radius * np.cos(theta) + offset_x
            self.xd_dot[0] = -helix_radius * helix_omega * np.sin(theta)
            self.xd_2dot[0] = -helix_radius * helix_omega2 * np.cos(theta)
            self.xd_3dot[0] = helix_radius * helix_omega3 * np.sin(theta)
            self.xd_4dot[0] = helix_radius * helix_omega4 * np.cos(theta)

            # y(t)
            self.xd[1] = helix_radius * np.sin(theta) + offset_y
            self.xd_dot[1] = helix_radius * helix_omega * np.cos(theta)
            self.xd_2dot[1] = -helix_radius * helix_omega2 * np.sin(theta)
            self.xd_3dot[1] = -helix_radius * helix_omega3 * np.cos(theta)
            self.xd_4dot[1] = helix_radius * helix_omega4 * np.sin(theta)

            # z(t)
            if (self.xd[2] < -0.5):
                self.xd[2] = -self.altitude + helix_vert_speed * dt_traj
                self.xd_dot[2] = helix_vert_speed
            else:
                self.xd[2] = -0.5
                self.xd_dot[2] = 0.

            self.xd_2dot[2] = 0.0
            self.xd_3dot[2] = 0.0
            self.xd_4dot[2] = 0.0

            #  b1d = [-sinθ, cosθ, 0]
            # make w_b1d = helix_omega, compute first and second derivatives
            w_b1d = helix_omega
            b1d_theta = theta
            self.b1d = np.array([-np.sin(b1d_theta), np.cos(b1d_theta), 0.0])

            self.b1d_dot = np.array([
                -np.cos(b1d_theta)*w_b1d,
                -np.sin(b1d_theta)*w_b1d,
                0.0
            ])
            self.b1d_2dot = np.array([
                np.sin(b1d_theta)*(w_b1d**2),
                -np.cos(b1d_theta)*(w_b1d**2),
                0.0
            ])

        elif type == 'lemniscate':
            # Bernoulli lemniscate
            lem_scale = 3.0
            lem_omega = 0.5

            theta = lem_omega * dt_traj
            denom = 1.0 + (np.sin(theta)**2)

            offset_x = self.x_start[0] - lem_scale * np.cos(0)/denom
            offset_y = self.x_start[1] - lem_scale * np.sin(0)*np.cos(0)/denom

            # x(t) ~ [lem_scale * cos(θ)/ denom], y(t) ~ [lem_scale * sin(θ)*cos(θ)/ denom]
            self.xd[0] = lem_scale * np.cos(theta)/denom + offset_x
            self.xd_dot[0] = 0.0
            self.xd_2dot[0] = 0.0
            self.xd_3dot[0] = 0.0
            self.xd_4dot[0] = 0.0

            # y(t)
            self.xd[1] = lem_scale * \
                np.sin(theta)*np.cos(theta)/denom + offset_y
            self.xd_dot[1] = 0.0
            self.xd_2dot[1] = 0.0
            self.xd_3dot[1] = 0.0
            self.xd_4dot[1] = 0.0

            # z(t)
            self.xd[2] = -self.altitude
            self.xd_dot[2] = 0.0
            self.xd_2dot[2] = 0.0
            self.xd_3dot[2] = 0.0
            self.xd_4dot[2] = 0.0

            # b1d always points to the tangential direction of the trajectory (can be changed to a fixed direction if needed)
            vx = -lem_scale*np.sin(theta)/denom - lem_scale * \
                np.cos(theta)*2*np.sin(theta)*np.cos(theta)/denom**2
            vy = (lem_scale*(np.cos(2*theta) - np.sin(2*theta)))/(2*denom)
            vel_norm = np.hypot(vx, vy)
            if vel_norm < 1e-6:
                vel_norm = 1e-6
            self.b1d = np.array([vx/vel_norm, vy/vel_norm, 0.0])
            self.b1d_dot = np.zeros(3)
            self.b1d_2dot = np.zeros(3)

        elif type == 'sinusoid':
            # Simple sinusoidal curve
            sin_amp = 2.0
            sin_wave_num = 5  # Wave number
            sin_omega = 0.05     # Horizontal forward speed

            a = sin_wave_num * sin_omega * dt_traj
            a_dot = sin_wave_num * sin_omega
            a_dot2 = a_dot**2
            a_dot3 = a_dot**3
            a_dot4 = a_dot**4

            offset_x = self.x_start[0] - sin_omega * 0
            offset_y = self.x_start[1] - sin_amp * np.sin(0)

            # x(t)
            self.xd[0] = sin_omega * dt_traj + offset_x
            self.xd_dot[0] = sin_omega
            self.xd_2dot[0] = 0.0
            self.xd_3dot[0] = 0.0
            self.xd_4dot[0] = 0.0

            # y(t) = sin_amp * sin(a) + center_y
            self.xd[1] = sin_amp * np.sin(a) + offset_y
            self.xd_dot[1] = sin_amp * np.cos(a) * a_dot
            self.xd_2dot[1] = -sin_amp * np.sin(a) * (a_dot2)
            self.xd_3dot[1] = -sin_amp * np.cos(a) * (a_dot3)
            self.xd_4dot[1] = sin_amp * np.sin(a) * (a_dot4)

            # z(t)
            self.xd[2] = -self.altitude
            self.xd_dot[2] = 0.0
            self.xd_2dot[2] = 0.0
            self.xd_3dot[2] = 0.0
            self.xd_4dot[2] = 0.0

            # b1d direct to the direction of the sinusoidal curve
            vx = sin_omega
            vy = sin_amp * sin_wave_num * sin_omega * \
                np.cos(sin_wave_num * sin_omega * dt_traj)
            vel_norm = np.hypot(vx, vy)
            if vel_norm < 1e-6:
                vel_norm = 1e-6
            self.b1d = np.array([vx/vel_norm, vy/vel_norm, 0.0])
            self.b1d_dot = np.zeros(3)
            self.b1d_2dot = np.zeros(3)

        else:  # hover
            self.xd = self.x_start
            self.xd_dot = np.zeros(3)
            self.xd_2dot = np.zeros(3)
            self.b1d = np.array([0.0, 1.0, 0.0])

    # --- Publisher Functions ---
    def geom_publish_command(self):
        """ 
        Implement the geometry control law and,
        publish the desired attitude setpoint and thrust.
        """
        m = self.m
        g = self.g
        e3 = self.e3

        kX = self.kX
        kV = self.kV

        R = self.R
        R_T = R.T

        x = self.x
        v = self.v
        W = self.W

        b1d = self.b1d
        b1d_dot = self.b1d_dot
        b1d_2dot = self.b1d_2dot

        xd = self.xd
        xd_dot = self.xd_dot
        xd_2dot = self.xd_2dot
        xd_3dot = self.xd_3dot
        xd_4dot = self.xd_4dot

        # Position and velocity errors
        eX = x - xd
        eV = v - xd_dot

        # Control law: A = -kX*eX - kV*eV - m*g*e3 + m*xd_2dot
        A = - kX @ eX - kV @ eV - m*g*e3 + m*xd_2dot
        hatW = hat(W)

        b3 = R @ e3
        b3_dot = R @ hatW @ e3

        f_total = -A @ b3

        # Compute acceleration error and derivatives for trajectory tracking
        ea = g*e3 - f_total/m * b3 - xd_2dot
        A_dot = - kX @ eV - kV @ ea + m*xd_3dot
        fdot = - A_dot @ b3 - A @ b3_dot
        eb = - fdot/m * b3 - f_total/m * b3_dot - xd_3dot
        A_2dot = - kX @ ea - kV @ eb + m*xd_4dot

        # Compute desired body frame orientation
        b3c, b3c_dot, b3c_2dot = deriv_unit_vector(-A, -A_dot, -A_2dot)

        hat_b1d = hat(b1d)
        hat_b1d_dot = hat(b1d_dot)
        A2 = -hat_b1d @ b3c
        A2_dot = - hat_b1d_dot @ b3c - hat_b1d @ b3c_dot
        A2_2dot = - hat(b1d_2dot) @ b3c \
                  - 2.0*hat_b1d_dot @ b3c_dot \
                  - hat_b1d @ b3c_2dot

        b2c, b2c_dot, b2c_2dot = deriv_unit_vector(A2, A2_dot, A2_2dot)
        hat_b2c = hat(b2c)
        hat_b2c_dot = hat(b2c_dot)

        b1c = hat_b2c @ b3c
        b1c_dot = hat_b2c_dot @ b3c + hat_b2c @ b3c_dot
        b1c_2dot = hat(b2c_2dot) @ b3c \
            + 2.0*hat_b2c_dot @ b3c_dot \
            + hat_b2c @ b3c_2dot

        # Desired rotation matrix and derivatives
        Rd = np.vstack((b1c, b2c, b3c)).T
        Rd_dot = np.vstack((b1c_dot, b2c_dot, b3c_dot)).T
        Rd_2dot = np.vstack((b1c_2dot, b2c_2dot, b3c_2dot)).T

        Rd_T = Rd.T
        Wd = vee(Rd_T @ Rd_dot)
        hat_Wd = hat(Wd)
        Wd_dot = vee(Rd_T @ Rd_2dot - hat_Wd @ hat_Wd)

        # Store computed values
        self.f_total = f_total
        self.Rd = Rd
        self.Wd = Wd
        self.Wd_dot = Wd_dot

        self.b3d = b3c
        self.b3d_dot = b3c_dot
        self.b3d_2dot = b3c_2dot

        self.b1c = b1c
        self.wc3 = e3 @ (R_T @ Rd @ Wd)
        self.wc3_dot = e3 @ (R_T @ Rd @ Wd_dot) - e3 @ (hatW @ R_T @ Rd @ Wd)

        self.ex = eX
        self.ev = eV

        # Publish attitude setpoint and normalized thrust
        msg = VehicleAttitudeSetpoint()
        msg.q_d = rotation_matrix_to_quaternion(Rd)

        # Normalize thrust and apply saturation
        # Use simple min/max for scalar saturation instead of the vector saturate function
        norm_thrust = f_total / self.max_thrust_newtons + 0.1
        # Simple scalar saturation
        norm_thrust = max(0.0, min(norm_thrust, 1.0))
        msg.thrust_body = [0.0, 0.0, -norm_thrust]

        # Timestamp
        msg.timestamp = self.timestamp_us

        self.publisher_vehicle_attitude_setpoint.publish(msg)

    def publish_vehicle_command(self, drone_idx: int, command: int,
                                param1: float = 0.0, param2: float = 0.0,
                                timestamp_us: int = None):
        """
        Publish a VehicleCommand (mode change + following arm) to a specific drone w. idx, 
        we map drone_idx → target_system = drone_idx + 1 .
        """
        if timestamp_us is None:
            self.get_logger().error(
                f'Timestamp is not available at drone_{self.drone_idx}, check the clock node.')

        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        # NOTE: Drone system which should execute the command
        msg.target_system = drone_idx + 1
        msg.target_component = 1

        # NOTE: Drone system which sends the command
        msg.source_system = drone_idx + 1
        msg.source_component = 1

        msg.from_external = True
        msg.timestamp = timestamp_us

        self.publisher_vehicle_command.publish(msg)

    def arm(self, drone_idx: int, timestamp_us: int = None):
        """
        Send arm command to the specified drone, 
        reusing the same timestamp for better synchronization.
        """
        self.publish_vehicle_command(
            drone_idx=drone_idx,
            command=VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
            timestamp_us=timestamp_us
        )

    # --- Callback Functions ---

    def sync_time_callback(self, msg: Int64):
        """
        Callback to sync time between multiple drones.
        """
        self.timestamp_us = msg.data

        # self.get_logger().info(f"Timestamp received at drone_{self.drone_idx}: {self.timestamp_us}")

    # NOTE: Callbacks to update variables used in geometry controller
    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        """
        Callback to read vehicle odometry and update the state variables.
        """
        for i in range(3):
            # record the initial offset of each drone
            if (self.xy_offset_flag == False):
                self.x_offset = msg.position[i]
                self.y_offset = msg.position[i]
                self.xy_offset_flag = True

            self.last_z = self.x[2]  # For state transition threshold
            self.x[i] = msg.position[i]
            self.v[i] = msg.velocity[i]
            self.W[i] = msg.angular_velocity[i]

        # Convert quaternion to rotation matrix
        q0, q1, q2, q3 = msg.q
        self.R = np.array([
            [1 - 2*(q2**2 + q3**2),  2*(q1*q2 - q0*q3),     2*(q1*q3 + q0*q2)],
            [2*(q1*q2 + q0*q3),     1 - 2*(q1**2 + q3**2), 2*(q2*q3 - q0*q1)],
            [2*(q1*q3 - q0*q2),     2*(q2*q3 + q0*q1),     1 - 2*(q1**2 + q2**2)]
        ])

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        """
        Callback to read vehicle local position and update the acceleration.
        """
        self.a[0] = msg.ax
        self.a[1] = msg.ay
        self.a[2] = msg.az

    def vehicle_status_callback(self, msg: VehicleStatus):
        """
        Callback to read vehicle status and judge if to send trajectory.
        """
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state


def main(args=None):
    rclpy.init(args=args)
    MultiGeom_control = MultiGeomNode()
    rclpy.spin(MultiGeom_control)

    MultiGeom_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

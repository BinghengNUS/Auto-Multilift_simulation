#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROSGeomState pose collector for geometric controllers
Author : Yichao Gao ( 17th-June 2025 modified)
"""

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
# from sensor_msgs.msg   import Imu
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy,
    QoSReliabilityPolicy, QoSDurabilityPolicy,
)
import atexit
import pathlib
from datetime import datetime
import numpy as np
from scipy.spatial.transform import Rotation
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleAttitudeSetpoint, TrajectorySetpoint
from px4_offboard.matrix_utils import hat, vee, ensure_SO3
from rclpy.executors import MultiThreadedExecutor
from px4_offboard.rotation_to_quaternion import rotation_matrix_to_quaternion
from px4_offboard.get_data_new import DataLoader
from std_msgs.msg import Int32

import math, pathlib, atexit
from   datetime import datetime
from typing import Tuple



NUM_DRONES = 6  # default number of drones
CTRL_HZ = 25.0  # default control frequency [Hz]

# states
INIT   = 0
ARMING = 1
TAKEOFF= 2
GOTO   = 3
HOVER  = 4
TRAJ   = 5
LAND   = 6
DONE   = 7

log_dir = pathlib.Path.home() / "lift_log"
log_dir.mkdir(exist_ok=True)
log_time = datetime.now().strftime("%Y%m%d_%H%M%S")
LOG_PATH = log_dir / f"pd_errors_{log_time}.npz"

EPS = 1e-9 

class StopTrajPlanner:
    """
    Quintic stop trajectory.
    Matches p(0)=x0, v(0)=v0, a(0)=a0; p(T)=xf, v(T)=0, a(T)=0.
    v0,a0 default to 0 for backward compatibility.
    """
    def __init__(self,
                 x0: np.ndarray,
                 xf: np.ndarray,
                 T_b: float,
                 v0: np.ndarray | None = None,
                 a0: np.ndarray | None = None):
        self.dim = x0.size
        self.T   = float(T_b)
        t        = self.T
        self.xf  = np.array(xf, dtype=float).copy()

        # default initial vel/acc = 0 (old behavior)
        if v0 is None: v0 = np.zeros_like(x0)
        if a0 is None: a0 = np.zeros_like(x0)
        v0 = np.asarray(v0, dtype=float).reshape(self.dim)
        a0 = np.asarray(a0, dtype=float).reshape(self.dim)

        # 6 boundary conditions -> 6x6
        A = np.array([
            [1,    0,     0,      0,       0,        0],   # p(0)
            [0,    1,     0,      0,       0,        0],   # v(0)
            [0,    0,     2,      0,       0,        0],   # a(0)
            [1,    t,   t**2,   t**3,    t**4,     t**5],  # p(T)
            [0,    1,   2*t,   3*t**2,  4*t**3,   5*t**4], # v(T)=0
            [0,    0,     2,    6*t,   12*t**2,  20*t**3], # a(T)=0
        ], dtype=float)

        self.coeff = np.empty((self.dim, 6), dtype=float)
        for k in range(self.dim):
            rhs = np.array([x0[k], v0[k], a0[k], xf[k], 0.0, 0.0], dtype=float)
            self.coeff[k] = np.linalg.solve(A, rhs)

    def polyval(self, tau: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return (x, x_dot, x_ddot) for tau ∈ [0,T]."""
        t = np.clip(tau, 0.0, self.T)  # clamp
        P   = np.array([1, t, t**2, t**3, t**4, t**5], dtype=float)
        dP  = np.array([0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4], dtype=float)
        d2P = np.array([0, 0, 2,   6*t,  12*t**2, 20*t**3], dtype=float)
        x     = self.coeff @ P
        x_dot = self.coeff @ dP
        x_dd  = self.coeff @ d2P
        return x, x_dot, x_dd
    

# wrapper for external calls
polyval_stop = lambda coeff, tau: (
    coeff.polyval(tau) if isinstance(coeff, StopTrajPlanner) else (None, None, None)
)


def cylinder_inertia(m, r_outer, h, r_inner=0.0):
    # Check validity
    if r_inner < 0 or r_inner >= r_outer:
        raise ValueError("0 ≤ r_inner < r_outer must hold.")
    
    # Solid cylinder special-case
    if r_inner == 0.0:  
        Izz = 0.5  * m * r_outer**2
        Ixx = Iyy = (1/12) * m * (3*r_outer**2 + h**2)
    else:
        # Hollow cylinder inertia: subtract inner solid from outer solid
        V_outer = np.pi * r_outer**2 * h
        V_inner = np.pi * r_inner**2 * h
        rho       = m / (V_outer - V_inner)       # uniform density
        
        m_outer = rho * V_outer
        m_inner = rho * V_inner
        
        Izz = 0.5 * (m_outer * r_outer**2 - m_inner * r_inner**2)
        Ixx = Iyy = (1/12) * (
            m_outer * (3*r_outer**2 + h**2) - 
            m_inner * (3*r_inner**2 + h**2)
        )
    return np.diag([Ixx, Iyy, Izz])

def body_angular_velocity(R_prev: np.ndarray,
                          R_curr: np.ndarray,
                          dt: float) -> np.ndarray:
    if dt <= 0.0:                   
        return np.zeros(3)

    # delta R = R_prev.T * R_curr  (SO(3))
    delta_R = R_prev.T @ R_curr
    rot_vec = Rotation.from_matrix(delta_R).as_rotvec()   # rad
    return rot_vec / dt      

class FirstOrderLowPass:
    def __init__(self, cutoff_hz: float):
        self.tau = 1.0 / (2.0 * math.pi * cutoff_hz)   
        self.y   = None                                

    def reset(self, x0: np.ndarray):
        self.y = np.array(x0, copy=True)

    def __call__(self, x: np.ndarray, dt: float) -> np.ndarray:
        if self.y is None:
            self.reset(x)              
            return self.y
        alpha = dt / (self.tau + dt)   
        self.y = alpha * x + (1.0 - alpha) * self.y
        return self.y


class AccKalmanFilter:
    """
    Constant-acceleration Kalman filter:
      State x = [p (3), v (3), a (3)].
      Measure position only.
    """
    def __init__(self,
                 dt_init: float = 0.04,
                 q_var: float = 1e-5,   # process noise (accel)
                 r_var: float = 2e-5    # measurement noise (pos)
                 ):
        # initial settings
        self.dt   = dt_init
        self.q    = q_var
        I3        = np.eye(3)

        # build measurement model
        self.H = np.hstack((I3, np.zeros((3, 6))))  # measure p only
        self.R = r_var * I3

        # allocate matrices
        self.x = np.zeros(9)             # [p, v, a]
        self.P = np.eye(9) * 1e3         # large initial uncertainty
        self.F = np.eye(9)
        self.Q = np.zeros((9, 9))
        self._init = False

    def _build_matrices(self, dt: float):
        I3 = np.eye(3)
        dt2 = dt * dt / 2.0

        # state-transition
        F = np.eye(9)
        F[0:3, 3:6]   = dt * I3
        F[0:3, 6:9]   = dt2 * I3
        F[3:6, 6:9]   = dt * I3
        self.F = F

        # process noise Q
        q = self.q
        Q = np.zeros((9,9))
        Q[0:3, 0:3]   = (dt**5/20) * q * I3
        Q[0:3, 3:6]   = (dt**4/8)  * q * I3
        Q[0:3, 6:9]   = (dt**3/6)  * q * I3
        Q[3:6, 0:3]   = Q[0:3,3:6].T
        Q[3:6, 3:6]   = (dt**3/3)  * q * I3
        Q[3:6, 6:9]   = (dt**2/2)  * q * I3
        Q[6:9, 0:3]   = Q[0:3,6:9].T
        Q[6:9, 3:6]   = Q[3:6,6:9].T
        Q[6:9, 6:9]   = dt * q * I3
        self.Q = Q

    def predict(self, dt: float):
        """Kalman predict step."""
        self.dt = dt
        self._build_matrices(dt)
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z: np.ndarray):
        """Kalman update with position measurement z."""
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x += K @ y
        I9 = np.eye(9)
        self.P = (I9 - K @ self.H) @ self.P

    def step(self, z: np.ndarray, dt: float):
        """
        One full predict+update cycle.
        First call seeds position, velocity and accel to zero.
        """
        if not self._init:
            # seed p, leave v,a at zero
            self.x[0:3] = z
            self._init   = True
            return
        self.predict(dt)
        self.update(z)

    @property
    def pos(self) -> np.ndarray:
        return self.x[0:3]

    @property
    def vel(self) -> np.ndarray:
        return self.x[3:6]

    @property
    def acc(self) -> np.ndarray:
        return self.x[6:9]



class ROSGeomState(Node):
    """
    Subscribes to:
      /payload_odom
      /simulation/position_drone_i  (PoseStamped, i=1..N)
      {ns}/fmu/out/vehicle_local_position  (VehicleLocalPosition)
      {ns}/fmu/out/vehicle_odometry        (VehicleOdometry)
    Publishes nothing. Provides getters for controller use, with
    payload acceleration estimated via finite difference + filtering.
    """

    def __init__(self, num_drones: int = NUM_DRONES) -> None:
        super().__init__('ros_geom_state')

        # simulation time
        self.sim_time = None

        #  payload state 
        self.payload_pos     = None        # position (3,)
        self.prev_payload_pos = None
        self.payload_q      = None         # ENU quaternion
        self.payload_R       = None        # rotation (3,3); from {B} to {I}
        self.prev_payload_R   = None
        self.payload_vel     = None        # linear velocity (3,)
        self.payload_ang_v   = None        # body-frame _Omega_ (3,)
        self.payload_lin_acc = None        # filtered linear acc (3,)

        # accel estimation helpers
        self.prev_payload_vel      = None
        self.prev_vel_time         = None
        self.kf                    = AccKalmanFilter(dt_init=1.0 / CTRL_HZ)
        self.fof                   = FirstOrderLowPass(cutoff_hz=10.0)

        #  drone state (lists) 
        self.n              = num_drones
        self.drone_pos      = [None] * num_drones
        self.drone_R        = [None] * num_drones
        self.drone_vel      = [None] * num_drones
        self.drone_omega    = [None] * num_drones
        self.drone_lin_acc  = [None] * num_drones
        self.drone_state    = [None] * num_drones

        self.ready = False
        self.T_enu2ned = np.array([[0, 1, 0],
                     [1, 0, 0],
                     [0, 0, -1]])
        self.T_enu2flu = np.array([[0, -1, 0],
                     [1, 0, 0],
                     [0, 0, 1]])
        self.T_flu2frd = np.diag([1, -1, -1])
        self.T_trans = self.T_enu2flu @ self.T_flu2frd

        # QoS: best-effort, keep last sample
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        #  SUBSCRIPTIONS 
        # Payload odometry (pose + twist in one message)
        self.create_subscription(Odometry, '/payload_odom',
                                 self._cb_payload_odom, qos)

        # Drone pose in sim-frame
        for i in range(1, num_drones + 1):
            topic = f'/simulation/position_drone_{i}'
            self.create_subscription(
                PoseStamped, topic,
                lambda msg, idx=i-1: self._cb_drone_pose(msg, idx), qos
            )

        # PX4 local-position & odometry per drone
        for idx in range(num_drones):
            lp_topic   = '/fmu/out/vehicle_local_position' if idx == 0   \
                         else f'/px4_{idx}/fmu/out/vehicle_local_position'
            odom_topic = '/fmu/out/vehicle_odometry'       if idx == 0   \
                         else f'/px4_{idx}/fmu/out/vehicle_odometry'
            state_topic = f'/state/state_drone_{idx}'

            self.create_subscription(
                VehicleLocalPosition, lp_topic,
                lambda msg, i=idx: self._cb_drone_local_position(msg, i), qos)
            self.create_subscription(
                VehicleOdometry, odom_topic,
                lambda msg, i=idx: self._cb_drone_odometry(msg, i), qos)
            self.create_subscription(
                Int32, state_topic,
                lambda msg, i=idx: self._cb_drone_state(msg, i), qos)

        # Heartbeat
        # self.create_timer(1, self._heartbeat)
        self.get_logger().info(f'ROSGeomState running with {num_drones} drones.')

    #  Callbacks 
    def _cb_payload_odom(self, msg: Odometry) -> None:
        """Handle /payload_odom, extract pose, velocity, angular velocity."""
        # now = self.get_clock().now().nanoseconds * 1e-9

        # Pose -> position & rotation matrix
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.payload_q = q

        # tf: ENU -> NED 
        self.payload_pos = np.array([p.y, p.x, -p.z], float)

        # FRD -> BODY -> ENU -> NED
        self.payload_R   = self.T_enu2ned @ Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix() @ self.T_trans
        self.sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.sim_time

        # use the sim time for kalman update 
        # dt = 1/CTRL_HZ if self.prev_vel_time is None else max(1e-4, now - self.prev_vel_time)
        if self.prev_vel_time is None:
            dt = 1.0 / CTRL_HZ
        elif now != self.prev_vel_time and self.prev_vel_time is not None:
            dt = max(1e-5, now - self.prev_vel_time)
            self.prev_vel_time = now
        

        # finite-difference acceleration + Kalman smoother 
        self.kf.step(self.payload_pos, dt)
        self.payload_vel     = self.kf.vel.copy()
        self.payload_lin_acc = self.kf.acc.copy()

        # get the angular velocity of the payload
        if self.prev_payload_R is not None and self.prev_vel_time is not None:
            payload_ang_v = body_angular_velocity(self.prev_payload_R, self.payload_R, dt)
            self.payload_ang_v = self.fof(payload_ang_v, dt)
        else:
            self.payload_ang_v = np.zeros(3)

        self.prev_vel_time = now
        self.prev_payload_pos = self.payload_pos
        self.prev_payload_R = self.payload_R
        # print(f"-----------payload-----R={self.payload_R}-----omega={self.payload_ang_v}")
        self._check_ready()

    def _cb_drone_pose(self, msg: PoseStamped, idx: int) -> None:
        p = msg.pose.position
        # ENU -> NED
        self.drone_pos[idx] = np.array([p.y, p.x, -p.z], float)
        self._check_ready()

    def _cb_drone_local_position(self, msg: VehicleLocalPosition, idx: int) -> None:
        # NED frame
        self.drone_vel[idx]     = np.array([ msg.vx,  msg.vy, msg.vz], float)
        self.drone_lin_acc[idx] = np.array([ msg.ax,  msg.ay, msg.az], float)
        self._check_ready()

    def _cb_drone_odometry(self, msg: VehicleOdometry, idx: int) -> None:
        angular_v = msg.angular_velocity
        self.drone_omega[idx] = np.array([ angular_v[0], angular_v[1], angular_v[2] ], float)
        q = msg.q
        self.drone_R[idx] = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()    # Rotation matrix under NED frame
        self._check_ready()

    def _cb_drone_state(self, msg: Int32, idx: int) -> None:
        state = msg.data
        self.drone_state[idx] = state
        self._check_ready()

    def _check_ready(self) -> None:
        """Mark node 'ready' once every critical field has been set."""
        payload_ready = all(val is not None for val in (
            self.payload_pos, self.payload_R, self.payload_vel,
            self.payload_ang_v, self.payload_lin_acc
        ))
        drones_ready = all(
            p is not None and v is not None and o is not None and a is not None
            for p, v, o, a in zip(
                self.drone_pos, self.drone_vel, self.drone_omega, self.drone_lin_acc
            )
        )
        self.ready = payload_ready and drones_ready

    def _heartbeat(self) -> None:
        """Periodic log of payload state once ready."""
        if not self.ready:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f'{t:2.4f}s payload position {self.payload_pos} vel {self.payload_vel}'
        )

    # Public accessor
    def get_state(self) -> dict | None:
        """Return deep-copied dict with payload & drone states, or None if not ready."""
        if not self.ready:
            return None

        payload = {
            'pos':     self.payload_pos.copy(),
            'R':       self.payload_R.copy(),
            'vel':     self.payload_vel.copy(),
            'lin_acc': self.payload_lin_acc.copy(),
            'omega':   self.payload_ang_v.copy(),
        }

        drones = []
        for i, (p, R, v, o, a) in enumerate(zip(
            self.drone_pos, self.drone_R,
            self.drone_vel, self.drone_omega,
            self.drone_lin_acc,
        )):
            drones.append({
                'pos':     p.copy(),
                'R':       R.copy(),
                'vel':     v.copy(),
                'omega':   o.copy(),
                'lin_acc': a.copy(),
                'state':   self.drone_state[i],   
            })
        return {'payload': payload, 'drones': drones}

    
class GeomLiftCtrl(Node):

    def __init__(self,
                 n: int = NUM_DRONES,
                 hz: float = CTRL_HZ,
                 state_node: ROSGeomState | None = None):
        super().__init__("geom_lift_ctrl")

        # QoS
        reliable_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                          depth=1)
        reliable_qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.n = n
        self.state = state_node or ROSGeomState(n)
        self.qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        #--- publishers ----------------------------------------------
        self.pub_att = [
            self.create_publisher(
                VehicleAttitudeSetpoint,
                ("" if i == 0 else f"/px4_{i}") 
                + "/fmu/in/vehicle_attitude_setpoint_v1",
                self.qos_profile,
            )
            for i in range(self.n)
        ]

        self.pub_pos = [
            self.create_publisher(
                TrajectorySetpoint,
                ("" if i == 0 else f"/px4_{i}") 
                + "/fmu/in/trajectory_setpoint",
                self.qos_profile,
            )
            for i in range(self.n)
        ]

        self.pub_cmd = [
            self.create_publisher(
                Int32,
                f"/state/command_drone_{i}",
                reliable_qos,
            )
            for i in range(self.n)
        ]

        

        #--- trajectory ----------------------------------------------
        self.trajectory_ENU = DataLoader()  # get the offline trajectory
        

        #--- physical parameters (edit for real hardware) ------------
        self.payload_m = 3.0     # payload mass  [kg]
        self.m_drones = 1.0      # drone mass    [kg]
        self.max_thrust = 2.85 * self.m_drones * 9.81    # TODO: test the thrust of the modified Iris drone
        self.l    = 1.25         # cable length   [m]
        self.g    = np.array([0.0, 0.0, 9.81])
        
        # link-attachment points on the payload body frame
        self.payload_r = 0.25
        self.offset_r = self.l + self.payload_r
        self.offset_pos = []

        self.J_p  = cylinder_inertia(m=self.payload_m, r_outer=self.payload_r, h=self.payload_r/4)
        # self.J_p = np.array([0.25, 0.25, 0.25])
        self.rho = []  # list of (3,) vectors

        for i in range(NUM_DRONES):
            angle = 2 * np.pi * i / NUM_DRONES
            y = self.payload_r * np.cos(angle)
            x = self.payload_r * np.sin(angle)
            self.rho.append(np.array([x, y, 0.0]))
            y_offset = self.offset_r * np.cos(angle)
            x_offset = self.offset_r * np.sin(angle)
            self.offset_pos.append(np.array([x_offset, y_offset, 0.0]))
        self.rho = np.array(self.rho)  # shape (num_drones, 3)
        self.offset_pos = np.array(self.offset_pos)  # shape (num_drones, 3)

        I3 = np.eye(3)
        self.P = np.zeros((6, 3*self.n))           
        for i in range(self.n):
            self.P[0:3, 3*i:3*(i+1)] = I3
            self.P[3:6, 3*i:3*(i+1)] = hat(self.rho[i])


        # timer
        self.dt_nom = 1.0 / hz
        self.dt      = self.dt_nom  
        self.t_prev = None        
        self.t0     = None  
        self.sim_t_prev = None
        self.sim_t0 = None     
        self.x_start = None 
        self.t_wait_traj = 0.0
        self.create_timer(self.dt_nom, self._step)

        self.get_logger().info("GeomLiftCtrl started.")

        # desired values initialization
        self.x_id = np.zeros((self.n, 3))
        self.mu_id = np.zeros((self.n, 3))  # desired force on each drone
        self.q_id = np.zeros((self.n, 3))    # desired cable direction
        self.omega_id = np.zeros((self.n, 3))  # desired angular velocity of each drone
        self.omega_id_dot = np.zeros((self.n, 3))  # desired angular acceleration of each drone
        self.b1d = np.array([1, 0, 0])  

        self.mu_id_dot_f = FirstOrderLowPass(cutoff_hz=10.0)
        self.mu_id_ddot_f = FirstOrderLowPass(cutoff_hz=10.0)
        self.Omega_0_dot_f = FirstOrderLowPass(cutoff_hz=15.0)
        self.mu_f = [FirstOrderLowPass(cutoff_hz=15.0)   for _ in range(self.n)]

        #--- control gains (PD only ) --------------------
        # self.kx = np.array([20.0,20.0,5.8])
        # self.kv = np.array([25.8,25.8,4.8])

        # self.kR = np.array([5.5,5.5,5.8])
        # self.kO = np.array([1.6,1.6,3.8])

        # self.kx = np.array([6.8,6.8,5.8])
        # self.kv = np.array([8.8,8.8,3.8])

        # self.kR = np.array([5.2,5.2,5.6])
        # self.kO = np.array([4.6,4.6,4.8])

        # Translational gains
        self.kx = np.array([0.9, 0.9, 1.0])   
        self.kv = np.array([0.4, 0.4, 0.5])   

        # Rotational gains
        # self.kR = np.array([1.9, 1.9, 0.70])   
        self.kR = np.array([1.9, 1.9, 0.30]) 
        self.kO = np.array([1.0, 1.0, 0.10])   



        self.kq = 12.8
        self.kw = 3.5

        self.k_ddp = np.zeros((6,13))
        # self.alpha = 0.0       # DDP feedback gain 
        self.alpha = 1.0

        self.slowdown = 1.0     # for test only, no slowdown

        self.thrust_bias = 0.05

        self.time_switch = 4.0

        self.T_enu2ned = np.array([[0, 1, 0],
            [1, 0, 0],
            [0, 0, -1]])
        self.T_body = np.array([[0, 1, 0],
            [1, 0, 0],
            [0, 0, -1]])
        self.T_xy = np.array([[0, 1, 0],
            [1, 0, 0],
            [0, 0, 1]])


        self.x_id_final = np.zeros((self.n, 3))  # final desired position of each drone
        for i in range(self.n):
            self.x_id_final[i] = self.T_enu2ned @ self.trajectory_ENU.payload_x[99, :] + self.rho[i] + self.l * self.T_enu2ned @ self.trajectory_ENU.cable_direction[i, 99, :] - self.offset_pos[i]
        print(f"Final desired position of each drone: {self.x_id_final}")
        # flag
        self.traj_ready = False

        
        
        self.t_log = None

        self.log = {
            't' : [],       
            'ex': [],       
            'ev': [],      
            'eR': [],       
            'eO': [],       
            'eq': [],       
            'ew': [],       
            "q_ref": [], "q_act": [],  
            "w_ref": [], "w_act": [],  
        }

        atexit.register(self._save_log)

    def _save_log(self):
        if not self.log['t']:           
            return
        np.savez(
            LOG_PATH,
            t  = np.asarray(self.log['t']),
            ex = np.asarray(self.log['ex']),
            ev = np.asarray(self.log['ev']),
            eR = np.asarray(self.log['eR']),
            eO = np.asarray(self.log['eO']),
            eq = np.asarray(self.log['eq']),
            ew = np.asarray(self.log['ew']),
            q_ref = np.asarray(self.log['q_ref']),
            q_act = np.asarray(self.log['q_act']),
            w_ref = np.asarray(self.log['w_ref']),
            w_act = np.asarray(self.log['w_act']),
        )
        self.get_logger().info(f"Log saved in: {LOG_PATH}")


    
    def _desired(self, t: float) -> None:
        if t <= self.time_switch * self.slowdown:
            idx = int(round(t / (0.04 * self.slowdown)))
            idx = np.clip(idx, 0, self.trajectory_ENU.cable_direction.shape[1] - 2)
            # print(idx)
            dirs_enu =  self.trajectory_ENU.cable_direction[:, idx, :]   # shape: (6, 3), xl - xq
            mu_enu = self.trajectory_ENU.cable_mu[:, idx]                  # shape: (6,) scalar
            omega_enu = self.trajectory_ENU.cable_omega[:, idx, :]
            omega_enu_dot = self.trajectory_ENU.cable_omega_dot[:, idx, :]

            # DDP feedback gain
            self.k_ddp = self.trajectory_ENU.Kb[idx]
            # desired attitude
            payload_q_d_ENU = self.trajectory_ENU.payload_q[idx, :]   # w, x, y, z
            payload_q_ENU = [self.state.payload_q.w, self.state.payload_q.x, self.state.payload_q.y, self.state.payload_q.z]    # x, y, z, w
            # print(payload_q_d_ENU, payload_q_ENU)
            R_ENU_d = self.trajectory_ENU.quat_2_rot(payload_q_d_ENU)   # {B} 2 {I}
            self.R_d = self.T_body @ R_ENU_d @ self.T_enu2ned
            self.Omega_0_d = self.T_body @ self.trajectory_ENU.payload_w[idx, :]  # (3,) under FRD frame
            Omega_0_d_ENU = self.trajectory_ENU.payload_w[idx, :]   # BODY frame


            self.x_d = self.T_enu2ned @ self.trajectory_ENU.payload_x[idx, :]  # NED frame
            self.v_d = self.T_enu2ned @ self.trajectory_ENU.payload_v[idx, :]  # NED frame
            # if t >= 4.0 * self.slowdown:
            #     self.x_d = self.T_enu2ned @ self.trajectory_ENU.payload_x[99, :]
            #     self.v_d = np.array([0.0, 0.0, 0.0])  # desired velocity
            # self.a_d = self.T_enu2ned @ self.trajectory_ENU.payload_a[idx, :]  
            self.j_d = np.array([0.0, 0.0, 0.0])  # desired jerk

            # error (for original geometric control)
            self.e_x = self.x_0 - self.x_d  # NED frame
            self.e_v = self.v_0 - self.v_d  # velocity error
            e_x_ENU = self.T_enu2ned @ self.e_x # ENU frame
            e_v_ENU = self.T_enu2ned @ self.e_v
            e_q_ENU = payload_q_ENU - payload_q_d_ENU   # NOTE: use the subtraction of quaternions directly
            self.e_R_0 = 0.5 * vee(self.R_d.T @ self.R_0 - self.R_0.T @ self.R_d )  # rotation error
            self.e_Omega_0 = self.Omega_0 - self.R_0.T @ self.R_d @ self.Omega_0_d  # angular velocity error
            e_Omega_0_ENU = self.T_body @ self.Omega_0 - Omega_0_d_ENU
            e_ddp_ENU = np.concatenate((e_x_ENU, e_v_ENU, e_q_ENU, e_Omega_0_ENU))
            if self.t_log is None or t - self.t_log > 0.1:
                print(f'Trajectory Time {t}')
                print(f'error x: {e_x_ENU}')
                print(f'error v: {e_v_ENU}')
                print(f'error q: {e_q_ENU}')
                print(f'error w: {e_Omega_0_ENU}')
                self.t_log = t
            
            weight_decay = 1.0 - np.clip(((self.time_switch - 1.0) * self.slowdown - t )/1.00, 0.0, 1.0)
            # if t > (self.time_switch - 0.1) * self.slowdown:
            #     weight_decay = 1.0
            
            FM_BODY = self.alpha * weight_decay * self.k_ddp @ e_ddp_ENU   # need to make sure that this should under the ENU frame
            F_BODY = FM_BODY[0:3]
            M_BODY = FM_BODY[3:6]
            F_FRD = self.T_body @ F_BODY
            M_FRD = self.T_body @ M_BODY
            FM_FRD = np.concatenate((F_FRD, M_FRD))
            P_pseudo = self.P.T @ np.linalg.inv(self.P @ self.P.T)  # This is under NED frame
            delta_mu_FRD = P_pseudo @ FM_FRD
            
            # desired 
            for i in range(self.n):
                self.q_id[i] = self.T_enu2ned @ dirs_enu[i]    # (6, 3) q is point down
                # print(f'q_id[{i}]',self.q_id[i])
                self.mu_id[i] = mu_enu[i] * self.q_id[i] + self.R_0 @ delta_mu_FRD[i*3:i*3+3]  
                # self.mu_id[i] = self.mu_f[i](self.mu_id[i], self.sim_dt) 
                self.q_id[i] = - self.mu_id[i] / np.linalg.norm(self.mu_id[i])
                # print(f'Drone {i} cable direction: {self.q_id[i]}')
                self.omega_id[i] = self.T_enu2ned @ omega_enu[i, :]  # (6, 3)
                self.omega_id_dot[i] = self.T_enu2ned @ omega_enu_dot[i, :]
                self.x_id[i] = self.x_d + self.R_d @ self.rho[i] - self.l * self.q_id[i] - self.offset_pos[i]

                # if t >= 3.5 * self.slowdown:
                #     for i in range(self.n):
                #         self.x_id[i] = self.x_id_final[i]  # set the final position of each drone
                # if idx == 30:
                #     breakpoint()
                # breakpoint()
                # print(self.x_id)

        # The original geometric control after 4s 
        if t >= self.time_switch * self.slowdown:
            if not hasattr(self, "_stop_planner"):     # first time entering brake
                # capture current payload state (nonzero v,a allowed)
                p0 = self.x_0.copy()
                v0 = self.v_0.copy()                  
                a0 = self.a_0.copy()                   
                xf = np.array([3.00, 3.00, -0.50])     # target (NED)

                self._stop_planner = StopTrajPlanner(
                    x0 = p0,
                    xf = xf,
                    T_b = (4.5 - self.time_switch) * self.slowdown,
                    v0 = v0,                            
                    a0 = a0,                            
                )
                self._t_stop0 = t

            tau = t - self._t_stop0

            if tau >= self._stop_planner.T:
                # hold exactly at planner.xf (avoid typos & drift)
                self.x_d = self._stop_planner.xf.copy()  
                self.v_d = self.a_d = np.zeros(3)
            else:
                self.x_d, self.v_d, self.a_d = self._stop_planner.polyval(tau)


            self.j_d  = np.array([0.0, 0.0, 0.0])  # desired jerk
            self.R_d = np.eye(3)  # identity rotation
            self.Omega_0_d = np.zeros(3)
            self.Omega_0_d_dot = np.zeros(3)
            # compute errors
            self.e_x = self.x_0 - self.x_d
            self.e_v = self.v_0 - self.v_d  # velocity error
            self.e_R_0 = 0.5 * vee(self.R_d.T @ self.R_0 - self.R_0.T @ self.R_d )  # rotation error
            self.e_Omega_0 = self.Omega_0 - self.R_0.T @ self.R_d @ self.Omega_0_d  # angular velocity error

            # compute desired forces and torques 
            self.F_d = self.payload_m * (-self.g + self.a_d - self.kx * self.e_x - self.kv * self.e_v)  # desired force on payload
            # print(f"F_d : {self.F_d}")
            self.M_d = (
                -self.kR * self.e_R_0 - self.kO * self.e_Omega_0
                + hat(self.R_0.T @ self.R_d @ self.Omega_0_d) @ self.J_p @ self.R_0.T @ self.R_d @ self.Omega_0_d
                + self.J_p @ self.R_0.T @ self.R_d @ self.Omega_0_d_dot
            )

            z = self.P.T @ np.linalg.inv(self.P @ self.P.T) @ np.hstack(( self.R_0.T @ self.F_d,  self.M_d )) 
            self.mu_id = np.array([ self.R_0 @ z[3*i:3*i+3] for i in range(self.n) ])
            self.q_id = np.array([ -self.mu_id[i] / np.linalg.norm(self.mu_id[i]) for i in range(self.n) ])
            if not hasattr(self, "mu_id_prev"):          # first iteration
                self.mu_id_prev = self.mu_id.copy()
                self.mu_id_dot  = np.zeros_like(self.mu_id)
            else:
                mu_id_dot  = (self.mu_id - self.mu_id_prev) / self.sim_dt
                self.mu_id_dot = self.mu_id_dot_f(mu_id_dot, self.sim_dt)
                self.mu_id_prev = self.mu_id.copy()
            
            if not hasattr(self, "mu_id_dot_prev"):          # first iteration
                self.mu_id_dot_prev = self.mu_id_dot.copy()
                self.mu_id_ddot  = np.zeros_like(self.mu_id)
            else:
                mu_id_ddot  = (self.mu_id_dot - self.mu_id_dot_prev) / self.sim_dt
                self.mu_id_ddot = self.mu_id_ddot_f(mu_id_ddot, self.sim_dt)
                self.mu_id_dot_prev = self.mu_id_dot.copy()
            proj = np.eye(3)[None,:,:] - np.einsum('ij,ik->ijk', self.q_id, self.q_id)
            self.q_id_dot = -np.einsum('ijk,ik->ij', proj, self.mu_id_dot) / np.linalg.norm(self.mu_id, axis=1, keepdims=True)
            self.omega_id = np.cross(self.q_id, self.q_id_dot)
            P_dot = - ( np.einsum('ij,ik->ijk', self.q_id_dot, self.q_id) + np.einsum('ij,ik->ijk', self.q_id,     self.q_id_dot) )
            L     = np.linalg.norm(self.mu_id,      axis=1, keepdims=True)                   # (n,1)
            L_dot = np.einsum('ij,ij->i', self.mu_id, self.mu_id_dot)[:,None] / L            # (n,1)
            term1 = np.einsum('ijk,ik->ij', P_dot,    self.mu_id_dot)                      # P' * mu_dot
            term2 = np.einsum('ijk,ik->ij', proj,        self.mu_id_ddot)                     # P  * mu_ddot
            term3 = (proj @ self.mu_id_dot[:,:,None])[:,:,0] * (L_dot / L)                    # (P*mu_dot)*(L_dot/L)
            self.q_id_ddot = - (term1 + term2) / L + term3    
            self.omega_id_dot = np.cross(self.q_id_dot, self.q_id_ddot)
            for i in range(self.n):
                self.x_id[i] = self.x_d + self.R_d @ self.rho[i] - self.l * self.q_id[i] - self.offset_pos[i]
                # self.x_id[i] = self.x_id_final[i]
        

    def _step(self) -> None:
        snap = self.state.get_state()
        if snap is None:
            return  # not ready yet
        
        self.fsm_states = [drone["state"] for drone in snap["drones"]]
        # print(f"Drone states: {self.states}")
        
        t_now = self.get_clock().now().nanoseconds * 1e-9   
        self.sim_time = self.state.sim_time
        self.check_state(self.sim_time)
        # print(f"Drone states: {self.fsm_states}")
        if not self.traj_ready:
            return
        if self.t_prev is None:            
            self.t_prev = t_now
            self.t0     = t_now             
            self.dt     = self.dt_nom
            self.x_start = snap["payload"]["pos"].copy()
        else:
            self.dt     = max(2e-2, t_now - self.t_prev)   
            self.t_prev = t_now

        t_rel = t_now - self.t0 
        
        if self.sim_t_prev is None:
            # init
            self.sim_t_prev = self.sim_time
            self.sim_dt     = self.dt_nom          # or 0
            self.x_start    = snap["payload"]["pos"].copy()
        elif self.sim_time - self.sim_t_prev > EPS:
            # update dt
            raw_dt   = self.sim_time - self.sim_t_prev
            self.sim_dt = max(4e-5, raw_dt)        
            self.sim_t_prev = self.sim_time

        sim_t_rel = self.sim_time - self.sim_t0 
        

        #---- payload state ----
        self.x_0  = snap["payload"]["pos"]      # (3,)
        self.R_0  = snap["payload"]["R"]        # (3,3)
        self.v_0  = snap["payload"]["vel"]      # (3,)
        # print(f"payload v {self.v_0}")
        self.a_0 = snap["payload"]["lin_acc"]  # (3,)
        # self.a_0 = 0
        self.Omega_0  = snap["payload"]["omega"]      # (3,)
        # print(f"----------------payload_omega----------{self.Omega_0}--------------")
        self.Omega_0_hat = hat(self.Omega_0)  # (3,3)
        self.R_0_dot = self.R_0 @ self.Omega_0_hat
        if not hasattr(self, "omega_0_prev"):
            # first iteration → no history yet
            self.omega_0_prev = self.Omega_0.copy()
            self.omega_0_dot  = np.zeros_like(self.Omega_0)
        else:
            omega_0_dot  = (self.Omega_0 - self.omega_0_prev) / self.sim_dt
            self.omega_0_dot = self.Omega_0_dot_f(omega_0_dot, self.sim_dt)
            self.omega_0_prev = self.Omega_0.copy()
        

        #---- drone states ----
        drones = snap["drones"]
        # q   = np.zeros((self.n, 3))
        mu = np.zeros((self.n, 3))
        a = np.zeros((self.n, 3))
        u_parallel = np.zeros((self.n, 3))
        u_vertical = np.zeros((self.n, 3))
        u = np.zeros((self.n, 3))
        b3 = np.zeros((self.n, 3))  # unit thrust vector
        self._desired(sim_t_rel)  # update desired values
        mu_id = self.mu_id
        q_d = self.q_id
        omega_id = self.omega_id
        omega_id_dot = self.omega_id_dot
        eq_step   = np.zeros((self.n, 3))
        ew_step   = np.zeros((self.n, 3))
        q_ref     = np.zeros((self.n, 3))
        q_act     = np.zeros((self.n, 3))
        w_ref     = np.zeros((self.n, 3))
        w_act     = np.zeros((self.n, 3))

        for i, drone in enumerate(drones):
            # get drone position
            x_i = drone['pos']                   # (3,)
            Omega_i = drone['omega']             # (3,)
            v_i = drone['vel']                   # (3,)
            # print(f"drone {i} v : {v_i}")
            R_i = drone['R']                     # (3,3)
            # compute raw cable vector and normalize to unit q
            vec =   - x_i + self.x_0 + self.R_0 @ self.rho[i]   # vector from attach point to drone
            q_i = vec / np.linalg.norm(vec)          # unit direction along cable
            q_i_dot = (-v_i + self.v_0 + self.R_0_dot @ self.rho[i]) / self.l
            q_i_hat = hat(q_i)  # (3,3) skew-symmetric matrix of q_i
            mu[i] = np.dot(mu_id[i], q_i) * q_i  # desired force on drone i along cable
            a[i] = (
                self.a_0 
                - self.g 
                + self.R_0 @ (self.Omega_0_hat @ (self.Omega_0_hat @ self.rho[i]))
                - self.R_0 @ hat(self.rho[i]) @ self.omega_0_dot
            )
            # print(f"Drone {i}: a: {a[i]}")
            omega_i = np.cross(q_i, q_i_dot)
            omega_sq = np.dot(omega_i, omega_i) 
            # compute u parallel 
            u_parallel[i] = (
                mu[i]
                + self.m_drones * self.l * omega_sq * q_i
                + self.m_drones * (np.dot(q_i, a[i]) * q_i)
            )
            # print(f"Drone {i}: u_parallel: {u_parallel[i]}")

            # compute u vertical 
            e_qi = np.cross(q_d[i], q_i)
            # print(f"Drone {i}: q_d: {q_d[i]}, q_i: {q_i}, e_qi: {e_qi}")
            q_i_hat_sqr = np.dot(q_i_hat , q_i_hat)  # (3,3) outer product of q_i
            e_omega_i = omega_i + q_i_hat_sqr @ omega_id[i]
            # print(f"e_omega_i:{e_omega_i}")
            
            u_vertical[i] = self.m_drones * self.l * q_i_hat @ (
                - self.kq * e_qi 
                - self.kw * e_omega_i 
                - np.dot(q_i, omega_id[i]) * q_i_dot
                - q_i_hat_sqr @ omega_id_dot[i]
            ) - self.m_drones * q_i_hat_sqr @ a[i]

            u[i] = u_parallel[i] + u_vertical[i]
            if np.linalg.norm(u[i]) < 1e-3:
                continue 
            # print(f"u[{i}] = {u[i]}, u_parallel = {u_parallel[i]}, u_vertival = {u_vertical[i]}")

            # compute thrust
            b3[i] = - u[i] / np.linalg.norm(u[i])  # unit vector along thrust
            # print("b3i",b3[i])
            # print(f"Drone {i}: b3: {b3[i]}")

            # thrust
            f_i =  - u[i] @ (R_i @ np.array([0, 0, 1]))
            # print(R_i)

            # attitude
            A2 = np.cross(b3[i], self.b1d)
            b2c = A2 / np.linalg.norm(A2)
            A1 = np.cross(b2c, b3[i])
            b1c = A1 / np.linalg.norm(A1)
            R_ic = np.column_stack((b1c, b2c, b3[i]))
            ensure_SO3(R_ic)

            # print control reference for debugging
            ts_us = int(self.get_clock().now().nanoseconds // 1000)
            att = VehicleAttitudeSetpoint()
            att.timestamp = ts_us
            # if sim_t_rel < 4.0:
            # SAFE cast: convert list->np.array->list(float32)
            att.q_d = np.array(rotation_matrix_to_quaternion(R_ic), dtype=np.float32).tolist()
            # print(f_i / self.max_thrust)
            # norm = np.clip((f_i / self.max_thrust) + self.thrust_bias , 0.3, 1.0)
            norm = - f_i / self.max_thrust - self.thrust_bias
            norm = np.clip(norm , -1.0, -0.2)
            # print(f"Drone {i}: Thrust norm: {norm:.2f} )")
            att.thrust_body = [0.0, 0.0, norm]
            self.pub_att[i].publish(att)
            # else:
            #     # SAFE cast: convert list->np.array->list(float32)
            #     att.q_d = np.array(rotation_matrix_to_quaternion(np.eye(3)), dtype=np.float32).tolist()
            #     # print(f_i / self.max_thrust)
            #     # norm = np.clip((f_i / self.max_thrust) + self.thrust_bias , 0.3, 1.0)
            #     norm = - f_i / self.max_thrust - self.thrust_bias
            #     norm = np.clip(norm , -1.0, -0.2)
            #     # print(f"Drone {i}: Thrust norm: {norm:.2f} )")
            #     att.thrust_body = [0.0, 0.0, norm]
            #     self.pub_att[i].publish(att)
            # breakpoint()
            pos = TrajectorySetpoint()
            pos.timestamp = ts_us
            pos.position = self.x_id[i].astype(np.float32)
            self.pub_pos[i].publish(pos)
            e_qi      = np.cross(q_d[i], q_i)          # (3,)
            eq_step[i]   = e_qi                        
            ew_step[i]   = e_omega_i                  
            q_ref[i]     = q_d[i]                      
            q_act[i]     = q_i                         
            w_ref[i]     = omega_id[i]                 
            w_act[i]     = omega_i       
            
            
        self.log["t"].append(sim_t_rel)
        self.log["ex"].append(self.e_x.copy())
        self.log["ev"].append(self.e_v.copy())
        self.log["eR"].append(self.e_R_0.copy())
        self.log["eO"].append(self.e_Omega_0.copy())

        self.log["eq"].append(eq_step.copy())        
        self.log["ew"].append(ew_step.copy())
        self.log["q_ref"].append(q_ref.copy())
        self.log["q_act"].append(q_act.copy())
        self.log["w_ref"].append(w_ref.copy())
        self.log["w_act"].append(w_act.copy()) 
        # breakpoint()



    def check_state(self, t):
        ready = np.isin(self.fsm_states, [3, 4]).all()
        part_ready = np.isin(self.fsm_states, [3, 4, 5]).all()
        traj_ready = np.isin(self.fsm_states, [5]).all()
        msg = Int32()
        msg.data = 5
        if np.any((self.fsm_states == 1) | (self.fsm_states == 2)):
            return
        
        if ready:
            if self.sim_t0 is None:
                self.sim_t0 = t
            self.t_wait_traj = t - self.sim_t0
            for i in range(self.n):
                self.pub_cmd[i].publish(msg)
        
        if part_ready:
            idxs = [i for i, v in enumerate(self.fsm_states) if v != 5]
            for i in idxs:
                self.pub_cmd[i].publish(msg)

        if not self.traj_ready and (self.t_wait_traj > 5.0 or traj_ready):
            self.get_logger().info(f"All drones are ready, starting simulation at t={t:.2f}s")
            self.traj_ready = True


def main() -> None:
    rclpy.init()
    state_node = ROSGeomState(num_drones=NUM_DRONES)
    ctrl_node  = GeomLiftCtrl(n=NUM_DRONES,
                              hz=CTRL_HZ,
                              state_node=state_node)

    executor = MultiThreadedExecutor()
    executor.add_node(state_node)
    executor.add_node(ctrl_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        state_node.destroy_node()
        ctrl_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

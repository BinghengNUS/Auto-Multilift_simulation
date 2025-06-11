#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROSGeomState pose collector for geometric controllers
Author : Yichao Gao (modified)
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
from px4_msgs.msg import VehicleThrustSetpoint, VehicleOdometry, VehicleLocalPosition, VehicleAttitudeSetpoint
from geometry_msgs.msg import Quaternion 
from px4_offboard.matrix_utils import hat, vee, deriv_unit_vector, saturate, ensure_SO3
from rclpy.executors import MultiThreadedExecutor
from px4_offboard.rotation_to_quaternion import rotation_matrix_to_quaternion



NUM_DRONES = 3  # default number of drones
CTRL_HZ = 50.0  # default control frequency [Hz]

log_dir = pathlib.Path.home() / "lift_log"
log_dir.mkdir(exist_ok=True)
log_time = datetime.now().strftime("%Y%m%d_%H%M%S")
LOG_PATH = log_dir / f"pd_errors_{log_time}.npz"


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
                 dt_init: float = 0.01,
                 q_var: float = 1e-2,   # process noise (accel)
                 r_var: float = 2e-3    # measurement noise (pos)
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

        #  payload state 
        self.payload_pos     = None        # ENU position (3,)
        self.prev_payload_pos = None
        self.payload_R       = None        # ENU rotation (3,3)
        self.prev_payload_R   = None
        self.payload_vel     = None        # ENU linear velocity (3,)
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

            self.create_subscription(
                VehicleLocalPosition, lp_topic,
                lambda msg, i=idx: self._cb_drone_local_position(msg, i), qos)
            self.create_subscription(
                VehicleOdometry, odom_topic,
                lambda msg, i=idx: self._cb_drone_odometry(msg, i), qos)

        # Heartbeat
        self.create_timer(1, self._heartbeat)
        self.get_logger().info(f'ROSGeomState running with {num_drones} drones.')

    #  Callbacks 
    def _cb_payload_odom(self, msg: Odometry) -> None:
        """Handle /payload_odom, extract pose, velocity, angular velocity."""
        now = self.get_clock().now().nanoseconds * 1e-9

        # Pose -> position & rotation matrix
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.payload_pos = np.array([p.y, p.x, -p.z], float)
        self.payload_R   = self.T_enu2ned.T @ Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix() @ self.T_trans

        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        # vel_world  = np.array([v.y, v.x, -v.z], float)

        #  finite-difference acceleration + Kalman smoother 
        dt = 0.02 if self.prev_vel_time is None else max(1e-4, now - self.prev_vel_time)

        self.kf.step(self.payload_pos, dt)
        self.payload_vel     = self.kf.vel.copy()
        self.payload_lin_acc = self.kf.acc.copy()
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
        q = msg.pose.orientation
        self.drone_pos[idx] = np.array([p.y, p.x, -p.z], float)
        self.drone_R[idx]   = self.T_enu2ned @ (Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()) @ self.T_trans
        self._check_ready()

    def _cb_drone_local_position(self, msg: VehicleLocalPosition, idx: int) -> None:
        self.drone_vel[idx]     = np.array([ msg.vx,  msg.vy, msg.vz], float)
        self.drone_lin_acc[idx] = np.array([ msg.ax,  msg.ay, msg.az], float)
        self._check_ready()

    def _cb_drone_odometry(self, msg: VehicleOdometry, idx: int) -> None:
        av = msg.angular_velocity
        self.drone_omega[idx] = np.array([ av[0], av[1], av[2] ], float)
        # q = msg.q
        # self.drone_R[idx] = Rotation.from_quat([q[0], q[1], q[2], q[3]]).as_matrix()
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
            f'{t:2.4f}s payload pos {self.payload_pos} vel {self.payload_vel}'
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

        drones = [{
            'pos':     p.copy(),
            'R':       R.copy(),
            'vel':     v.copy(),
            'omega':   o.copy(),
            'lin_acc': a.copy(),
        } for p, R, v, o, a in zip(
            self.drone_pos, self.drone_R,
            self.drone_vel, self.drone_omega,
            self.drone_lin_acc
        )]

        return {'payload': payload, 'drones': drones}

    
class GeomLiftCtrl(Node):

    def __init__(self,
                 n: int = NUM_DRONES,
                 hz: float = CTRL_HZ,
                 state_node: ROSGeomState | None = None):
        super().__init__("geom_lift_ctrl")

        self.n = n
        self.state = state_node or ROSGeomState(n)
        self.qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        #--- publishers ----------------------------------------------
        self.pub_att = [
            self.create_publisher(
                VehicleAttitudeSetpoint,
                ("" if i == 0 else f"/px4_{i}") 
                + "/fmu/in/vehicle_attitude_setpoint",
                self.qos_profile,
            )
            for i in range(self.n)
        ]
        

        #--- physical parameters (edit for real hardware) ------------
        self.payload_m = 1.5      # payload mass  [kg]
        self.m_drones = 1.5      # drone mass    [kg]
        self.max_thrust = 2.1 * self.m_drones * 9.81 
        self.l    = 1.0      # link length   [m]
        self.g    = np.array([0.0, 0.0, 9.81])
        
        # link-attachment points on the payload body frame
        self.payload_r = 0.6

        self.J_p  = cylinder_inertia(m=self.payload_m, r_outer=self.payload_r, h=self.payload_r/4)
        self.rho = []  # list of (3,) vectors
        for i in range(NUM_DRONES):
            angle = 2 * np.pi * i / NUM_DRONES
            y = self.payload_r * np.cos(angle)
            x = self.payload_r * np.sin(angle)
            self.rho.append(np.array([x, y, 0.0]))
        self.rho = np.array(self.rho)  # shape (num_drones, 3)

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
        self.x_start = None 
        self.create_timer(self.dt_nom, self._step)

        self.get_logger().info("GeomLiftCtrl started.")

        # desired values initialization
        self.mu_id = np.zeros((self.n, 3))  # desired force on each drone
        self.q_id = np.zeros((self.n, 3))    # desired cable direction
        self.omega_id = np.zeros((self.n, 3))  # desired angular velocity of each drone
        self.omega_id_dot = np.zeros((self.n, 3))  # desired angular acceleration of each drone
        self.b1d = np.array([1, 0, 0])  

        self.mu_id_dot_f = FirstOrderLowPass(cutoff_hz=10.0)
        self.mu_id_ddot_f = FirstOrderLowPass(cutoff_hz=7.0)
        self.Omega_0_dot_f = FirstOrderLowPass(cutoff_hz=8.0)

        #--- control gains (PD only ) --------------------
        self.kx = np.array([7.0,7.0,5.8])
        self.kv = np.array([3.8,3.8,3.8])

        self.kR = np.array([3.5,3.5,5.8])
        self.kO = np.array([1.6,1.6,1.0])

        self.kq = 12.8
        self.kw = 6.8

        self.thrust_bias = 0.05

        self.log = {
            't' : [],       
            'ex': [],       
            'ev': [],      
            'eR': [],       
            'eO': [],       
            'eq': [],       
            'ew': [],       
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
        )
        self.get_logger().info(f"Log saved in: {LOG_PATH}")


    
    def _desired(self, t: float) -> None:
        # print(t)

        if t < 20:
            self.x_d  = np.array([1.0, 1.0, -8.0])  # desired position
            self.v_d  = np.array([0.0, 0.0, 0.0])  # desired velocity
            self.a_d  = np.array([0.0, 0.0, 0.0])  # desired acceleration
            self.j_d  = np.array([0.0, 0.0, 0.0])  # desired jerk
        # elif t < 80:
        #     # desired of payload for testing
        #     self.x_d  = np.array([
        #                      2.4*np.cos(0.1*np.pi*t) - 2.4,
        #                      3.8*np.sin(0.1*np.pi*t),
        #                     -8.0])
        #     self.v_d  = np.array([
        #                     -2.4*0.1*np.pi*np.sin(0.1*np.pi*t),
        #                     3.8*0.1*np.pi*np.cos(0.1*np.pi*t) - 3.8*0.1*np.pi,
        #                      0.0])
        #     self.a_d  = np.array([
        #                     -2.4*(0.1*np.pi)**2*np.cos(0.1*np.pi*t) -2.4*(0.1*np.pi)**2,
        #                     -3.8*(0.1*np.pi)**2*np.sin(0.1*np.pi*t),
        #                      0.0])
        #     self.j_d = np.array([
        #                     2.4*(0.1*np.pi)**3*np.sin(0.1*np.pi*t),
        #                     -3.8*(0.1*np.pi)**3*np.cos(0.1*np.pi*t) -3.8*(0.1*np.pi)**3,
        #                      0.0])
        else:
            self.x_d  = np.array([0.0, 0.0, -8.0])  # desired position
            self.v_d  = np.array([0.0, 0.0, 0.0])  # desired velocity
            self.a_d  = np.array([0.0, 0.0, 0.0])  # desired acceleration
            self.j_d  = np.array([0.0, 0.0, 0.0])  # desired jerk

        # v_des   = 0.5                        
        # dir_vec = np.array([1.0, 0.0, 0.0])    
        # self.x_d = self.x_start + v_des * t * dir_vec
        # self.x_d[2] = 3.0                      
        # self.v_d = v_des * dir_vec
        # self.a_d = np.zeros(3)
        # self.j_d = np.zeros(3)

        # payload desired attitude
        b3      = np.array([0.0, 0.0, 1.0])               
        s       = np.linalg.norm(self.v_d)      
        # if s < 1e-6:
        if True:
            b1d = np.array([1.0, 0.0, 0.0])
            self.R_d = np.eye(3)  # identity rotation
            self.Omega_0_d = np.zeros(3)
            self.Omega_0_d_dot = np.zeros(3)
        else:      
            b1d     = self.v_d / s
            # b1d = np.array([0.0, 1.0, 0.0])
            self.b1d = b1d                            # body-x
            b1_dot  = (self.a_d - np.dot(b1d, self.a_d)*b1d) / s

            
            k  = np.dot(b1d, self.a_d)                
            N  = self.a_d - k*b1d
            u_dot = N / s
            k_dot = np.dot(u_dot, self.a_d) + np.dot(b1d, self.j_d)
            N_dot = self.j_d - k_dot*b1d - k*u_dot
            b1_ddot = N_dot / s - N*k / s**2   

            w        = np.cross(b3, b1d)          # b2
            n2       = np.linalg.norm(w)
            b2d      = w / n2
            w_dot    = np.cross(b3, b1_dot)
            n2_dot   = np.dot(w, w_dot) / n2
            b2_dot   = ( w_dot*n2 - w*n2_dot ) / n2**2

            w_ddot   = np.cross(b3, b1_ddot)
            n2_ddot  = ( np.dot(w_dot, w_dot) + np.dot(w, w_ddot) - n2_dot**2 ) / n2
            b2_ddot  = (
                w_ddot*n2 - 2*w_dot*n2_dot - w*n2_ddot
            ) / n2**2
            self.R_d = np.column_stack((b1d,  b2d,  b3))
            ensure_SO3(self.R_d)  # ensure R_d is a valid rotation matrix
            R_d_dot = np.column_stack((b1_dot,  b2_dot,  np.zeros(3)))
            R_d_ddot = np.column_stack((b1_ddot, b2_ddot, np.zeros(3)))
            hat_Omega_d    = self.R_d.T @ R_d_dot
            self.Omega_0_d = vee(hat_Omega_d) 
            self.hat_Omega_d = hat_Omega_d
            A = self.R_d.T @ R_d_ddot - hat_Omega_d @ hat_Omega_d
            A = 0.5 * (A - A.T) 
            self.Omega_0_d_dot = vee(A)

        # compute errors
        self.e_x = self.x_0 - self.x_d
        self.e_v = self.v_0 - self.v_d  # velocity error
        self.e_R_0 = 0.5 * vee(self.R_d.T @ self.R_0 - self.R_0.T @ self.R_d )  # rotation error
        # print(f"e_R_0:{self.e_R_0}, R_d = {self.R_d}, R_0 = {self.R_0}")
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
            mu_id_dot  = (self.mu_id - self.mu_id_prev) / self.dt
            self.mu_id_dot = self.mu_id_dot_f(mu_id_dot, self.dt)
            self.mu_id_prev = self.mu_id.copy()
        
        if not hasattr(self, "mu_id_dot_prev"):          # first iteration
            self.mu_id_dot_prev = self.mu_id_dot.copy()
            self.mu_id_ddot  = np.zeros_like(self.mu_id)
        else:
            mu_id_ddot  = (self.mu_id_dot - self.mu_id_dot_prev) / self.dt
            self.mu_id_ddot = self.mu_id_ddot_f(mu_id_ddot, self.dt)
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


        
        

    def _step(self) -> None:
        snap = self.state.get_state()
        if snap is None:
            return  # not ready yet
        
        t_now = self.get_clock().now().nanoseconds * 1e-9   

        if self.t_prev is None:            
            self.t_prev = t_now
            self.t0     = t_now             
            self.dt     = self.dt_nom
            self.x_start = snap["payload"]["pos"].copy()
        else:
            self.dt     = max(2e-2, t_now - self.t_prev)   
            self.t_prev = t_now

        t_rel = t_now - self.t0 

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
            omega_0_dot  = (self.Omega_0 - self.omega_0_prev) / self.dt
            self.omega_0_dot = self.Omega_0_dot_f(omega_0_dot, self.dt)
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
        self._desired(t_rel)  # update desired values
        mu_id = self.mu_id
        q_d = self.q_id
        omega_id = self.omega_id
        omega_id_dot = self.omega_id_dot

        for i, drone in enumerate(drones):
            # get drone position
            x_i = drone['pos']                   # (3,)
            Omega_i = drone['omega']             # (3,)
            v_i = drone['vel']                   # (3,)
            # print(f"drone {i} v : {v_i}")
            R_i = drone['R']                     # (3,3)
            # compute raw cable vector and normalize to unit q
            v =   - x_i + self.x_0 + self.R_0 @ self.rho[i]   # vector from attach point to drone
            q_i = v / np.linalg.norm(v)          # unit direction along cable
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
                + self.m_drones * (q_i @ a[i]) * q_i   
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
            # print(f"u[{i}] = {u[i]}, u_parallel = {u_parallel}, u_vertival = {u_vertical}")

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
            # print(f"q_cmd {q_cmd}")

            # print control reference for debugging
            ts_us = int(self.get_clock().now().nanoseconds // 1000)
            att = VehicleAttitudeSetpoint()
            att.timestamp = ts_us
            # SAFE cast: convert list->np.array->list(float32)
            att.q_d = np.array(rotation_matrix_to_quaternion(R_ic), dtype=np.float32).tolist()
            # print(f_i / self.max_thrust)
            norm = np.clip((f_i / self.max_thrust) + self.thrust_bias , 0.3, 1.0)
            # print(f"Drone {i}: Thrust norm: {norm:.2f} )")
            att.thrust_body = [0.0, 0.0, -norm]
            self.pub_att[i].publish(att)


            self.log['t'].append(t_rel)
            self.log['ex'].append(self.e_x.copy())
            self.log['ev'].append(self.e_v.copy())
            self.log['eR'].append(self.e_R_0.copy())
            self.log['eO'].append(self.e_Omega_0.copy())
            eq_norm   = [np.linalg.norm(np.cross(self.q_id[i], q_i))     for i in range(self.n)]
            ew_norm   = [np.linalg.norm(e_omega_i)                       for i in range(self.n)]
            self.log['eq'].append(eq_norm)
            self.log['ew'].append(ew_norm)
            # breakpoint()
        


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

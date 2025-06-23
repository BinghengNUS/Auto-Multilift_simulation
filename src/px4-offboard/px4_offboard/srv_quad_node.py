import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from px4_offboard.matrix_utils import hat, vee, deriv_unit_vector, saturate, R_to_q, q_multiply
from std_msgs.msg import Int64
from mpc_msgs.msg import QuadReturnMPC, BroadcastMPC
from mpc_msgs.srv import SyncMPC
from geometry_msgs.msg import PoseStamped, TwistStamped
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleAttitudeSetpoint, OffboardControlMode, VehicleStatus, VehicleCommand, TrajectorySetpoint

# Automultilift imports
import os
import math
import torch
import threading
import time as TM
import numpy as np
from casadi import *
import importlib.util
import ament_index_python
from numpy import linalg as LA
import matplotlib.pyplot as plt
from px4_offboard import Dynamics
from px4_offboard import NeuralNet
from px4_offboard import Robust_Flight_MPC_acados
from scipy.spatial.transform import Rotation as Rot
from multiprocessing import Process, Array, Manager


class QuadNode(Node):
    ST_INIT = 0
    ST_ARMING = 1
    ST_TAKEOFF = 2
    ST_MPC = 3
    ST_DONE = 4

    def __init__(self):
        super().__init__(f'AutoMultilift_Quadrotor_MPC_service')
        np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)}) # show np.array with 3 decimal places
        # Get the package share directory
        self.package_share_directory = ament_index_python.get_package_share_directory('px4_offboard')
        
        ## QoS setup ##
        self._initialize_qos()

        ## Declare parameters ##
        self._declare_ros_parameters()

        ## Initialize the AutoMultilift system ##
        # HACK Parameters need to adjust according to the settings in Isaac Sim 
        self._initialize_automultilift()

        ## Initialize the neural network ##
        self._initialize_neural_network()

        ## Initial state variables ##
        self._initialize_MPC_states()
        self._initialize_geom_states()
        self._initialize_geom_trajectory()
        
        ## Initialize the service and clients ##
        self._initialize_services()
        self._initialize_clients()
        
        ## Initialize the publishers and subscribers ##
        self._initialize_publishers()
        self._initialize_subscribers()

        ## State machine setup ##
        self.Qnode_state = self.ST_INIT
        # self.Qnode_state = self.ST_MPC

        self.offboard_count = 0
        self.timer_QNode = self.create_timer(
            self.dt_ctrl,
            self.Qnode_state_machine,
            callback_group=self.stm_pre_callback_group
        )

    # --- State Machine ---
    def Qnode_state_machine(self):
        """
        Main state machine to handle drone offboard states:
          ST_INIT       -> Wait a few cycles, then send OFFBOARD + ARM commands
          ST_ARMING     -> Wait until OFFBOARD + ARMED, then start takeoff
          ST_TAKEOFF    -> Takeoff by Geom; switch to MPC trajectory when received MPC_TRAJ=True
          ST_MPC        -> Follow the signals from the CentralNode, check REC_TEMP[i] to start & stop QMPC
          ST_DONE       -> Received MPC_TRAJ=False, maintains or ends operation
        """
        # if self.Qnode_state == self.ST_MPC:
        #     offboard_msg              = OffboardControlMode()
        #     offboard_msg.position     = True
        #     offboard_msg.velocity     = False
        #     offboard_msg.acceleration = False
        #     offboard_msg.attitude     = False
        #     offboard_msg.body_rate    = False
        #     offboard_msg.timestamp    = self.timestamp_us
        #     self.publisher_offboard_mode.publish(offboard_msg)
        # else:

        # Always publish OffboardControlMode
        offboard_msg              = OffboardControlMode()
        offboard_msg.position     = False
        offboard_msg.velocity     = False
        offboard_msg.acceleration = False
        offboard_msg.attitude     = True
        offboard_msg.body_rate    = False
        offboard_msg.timestamp    = self.timestamp_us
        self.publisher_offboard_mode.publish(offboard_msg)


        # 1) ST_INIT: send offboard + arm after a certain number of cycles
        if self.Qnode_state == self.ST_INIT:
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
                self.arm(
                    drone_idx=self.drone_idx,
                    timestamp_us=self.timestamp_us
                )
                self.get_logger().info("--- Sent OFFBOARD + ARM command. ---")
                self.Qnode_state = self.ST_ARMING

        # 2) ST_ARMING: wait until OFFBOARD + ARMED
        elif self.Qnode_state == self.ST_ARMING:
            if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                    and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
                
                self.get_logger().info("----- Starting takeoff -----")
                self.takeoff_traj_timer = self.create_timer(
                    self.dt_ctrl, 
                    self.generate_takeoff_trajectory,
                    callback_group=self.stm_pre_callback_group
                )
                self.geom_ctrl_timer = self.create_timer(
                    self.dt_ctrl, 
                    self.geom_publish_command,
                    callback_group=self.control_logic_callback_group
                )
                self.Qnode_state = self.ST_TAKEOFF
            else:
                self.publish_vehicle_command(
                    drone_idx=self.drone_idx,
                    command=VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0,
                    param2=6.0,
                    timestamp_us=self.timestamp_us
                )
                self.arm(
                    drone_idx=self.drone_idx,
                    timestamp_us=self.timestamp_us
                )
                self.get_logger().info("--- Resent OFFBOARD + ARM command. ---")

        # 3) ST_TAKEOFF: takeoff by Geom Ctrl; switch to MPC trajectory when received MPC_TRAJ=True
        elif self.Qnode_state == self.ST_TAKEOFF:
            if (self.MPC_TRAJ): 
                # CentralNode start the distributed MPC, quadNode handle it in service callback
                
                self.get_logger().info("----- Starting MPC trajectory -----")
                self.takeoff_traj_timer.cancel()
                
                # # Transfer to MPC, get control inputs from broadcast 'opt_system'
                # self.qmpc_ctrl_timer = self.create_timer(
                #     self.dt_ctrl, 
                #     self.qmpc_publish_command,
                #     callback_group=self.stm_pre_callback_group)
                # self.geom_ctrl_timer.cancel()
                self.Qnode_state = self.ST_MPC

        # 4) ST_MPC: follow the signals from the CentralNode, check REC_TEMP[i] to start & stop QMPC
        elif self.Qnode_state == self.ST_MPC:
            if not self.MPC_TRAJ:
                self.get_logger().info("----- Stopping the distributed MPC -----")
                self.Qnode_state = self.ST_DONE

        # 5) ST_DONE: received MPC_TRAJ=False, maintains or ends operation
        elif self.Qnode_state == self.ST_DONE:
            self.get_logger().info("----- Task finished -----")
            self.timer_QNode.cancel()
            pass

        self.offboard_count += 1

    # --- QMPC Forward --- 
    def QuadrotorMPC(self,
                    xq_traj, uq_traj, xl_traj, ul_traj,
                    Ref_xi, Ref_ui, Para_i, i):
        xi_opt      = np.zeros((self.horizon+1,self.nxi))
        ui_opt      = np.zeros((self.horizon,self.nui))

        xi_traj     = xq_traj[i] # this trajectory list should be updated after each iteration
        ui_traj     = uq_traj[i] # this trajectory list should also be updated after each iteration
        xi_fb       = np.reshape(self.xi,self.nxi) # REAL-TIME quadrotor's state

        ref_xi      = np.zeros(self.nxi*(self.horizon+1))
        ref_ui      = np.zeros(self.nui*self.horizon)
        # Calculate the quadrotor's 
        xl_trajh    = np.zeros(self.nxl*(self.horizon+1))

        for k in range(self.horizon):
            ref_xik = np.reshape(Ref_xi[:,k],self.nxi)
            ref_xi[k*self.nxi:(k+1)*self.nxi]=ref_xik
            ref_uik = np.reshape(Ref_ui[:,k],self.nui)
            ref_ui[k*self.nui:(k+1)*self.nui]=ref_uik
            xl_k    = np.reshape(xl_traj[k,:],self.nxl)
            xl_trajh[k*self.nxl:(k+1)*self.nxl]=xl_k
        ref_xi[self.horizon*self.nxi:(self.horizon+1)*self.nxi]=np.reshape(Ref_xi[:,self.horizon],self.nxi)
        xl_trajh[self.horizon*self.nxl:(self.horizon+1)*self.nxl]=np.reshape(xl_traj[self.horizon,:],self.nxl)
        
        Para_i      = np.reshape(Para_i,self.npi)
        xq_i        = np.zeros((self.nq-1)*2*(self.horizon+1))
        kj = 0

        for j in range(self.nq):
            if j!=i:
                xqj     = xq_traj[j]
                xqj_xy  = np.reshape(xqj[:,0:2],2*(self.horizon+1))
                xq_i[kj*2*(self.horizon+1):(kj+1)*2*(self.horizon+1)] = xqj_xy
                kj += 1

        uli_traj    = np.reshape(ul_traj[:,i],self.horizon)
        opt_sol_i   = self.DistMPC.MPCsolverQuadrotor_ros2_acados(xi_fb, xq_i, xl_trajh, uli_traj, 
                                                        ref_xi, ref_ui, Para_i, i)
        xi_opt      = np.array(opt_sol_i['xi_opt'])
        ui_opt      = np.array(opt_sol_i['ui_opt'])
        # NOTE Finish current QMPC, update the xi_temp and ui_temp, publish to CentralNode
        self.xi_temp     = xi_opt
        self.ui_temp     = ui_opt
        
        sum_viol_xi = 0
        sum_viol_ui = 0
        for ki in range(len(ui_traj)):
            sum_viol_xi  += LA.norm(xi_opt[ki,:]-xi_traj[ki,:])
            sum_viol_ui  += LA.norm(ui_opt[ki,:]-ui_traj[ki,:])

        sum_viol_xi  += LA.norm(xi_opt[-1,:]-xi_traj[-1,:])
        viol_xi  = sum_viol_xi/len(xi_opt)
        viol_ui  = sum_viol_ui/len(ui_opt)
        self.get_logger().info(f"drone_ID: {self.drone_idx}, viol_xi: {viol_xi:.5f}, viol_ui: {viol_ui:.5f}") 
        max_viol_i = max(viol_xi, viol_ui)
        # NOTE update the max_viol_i
        # self.max_viol_i = max(self.max_viol_i, max_viol_i)
        self.max_viol_i = max_viol_i

    # --- Initialization Functions ---
    def _initialize_automultilift(self):
        """--------------------------------------Load environment---------------------------------------"""
        self.uav_para = np.array(self.get_parameter('uav_para').value) # L quadrotors
        self.load_para = np.array(self.get_parameter('load_para').value) # 2.5 kg for 3 quadrotors, 7.5 kg for 6 quadrotors
        self.cable_para = np.array(self.get_parameter('cable_para').value) # E=1 Gpa, A=7mm^2 (pi*1.5^2), c=10, L0=2, Nylon-HD, [5], np.array([5e3, 1e-2, 2])
        self.Jl = np.array(self.get_parameter('Jl').value).reshape(-1, 1)  # payload's moment of inertia
        self.rg = np.array(self.get_parameter('rg').value).reshape(-1, 1)  # coordinate of the payload's CoM in {Bl}
        self.dt_ctrl = self.get_parameter('dt_ctrl').value

        self.stm          = Dynamics.multilifting(self.uav_para, self.load_para, self.cable_para, self.dt_ctrl)
        self.stm.model()
        self.horizon      = 10 # MPC's horizon
        self.horizon_loss = 20 # horizon of the high-level loss for training, which can be longer than the MPC's horizon
        self.nxl          = self.stm.nxl # dimension of the payload's state
        self.nxi          = self.stm.nxi # dimension of the quadrotor's state
        self.nui          = self.stm.nui # dimension of the quadrotor's control 
        self.nul          = self.stm.nul # dimension of the payload's control which is equal to the number of quadrotors
        self.nwsi         = 12 # dimension of the quadrotor state weightings
        self.nwsl         = 12 # dimension of the payload state weightings
        self.nwui         = 4 # dimension of the quadrotor control weightings

        """--------------------------------------Define neural network models-----------------------------------------"""
        # quadrotor and load parameters
        self.nq         = int(self.uav_para[4])
        self.alpha      = 2*np.pi/self.nq
        self.rl         = self.load_para[1]
        self.L0         = self.cable_para[3]
        self.loadp      = np.vstack((self.Jl,self.rg)) # payload's inertial parameter
        self.Di_in, self.Di_h, self.Di_out = 6, 30, 2*self.nwsi + self.nui # for quadrotors
        self.Dl_in, self.Dl_h, self.Dl_out = 12, 30, 2*self.nwsl + self.nul # for the payload
        self.npi        = 2*self.nwsi + self.nui
        self.npl        = 2*self.nwsl + self.nul
        self.nlp        = len(self.loadp)

        """--------------------------------------Define controller--------------------------------------------------"""
        self.gamma      = 1e-4 # barrier parameter, cannot be too small
        self.gamma2     = 1e-15
        self.GeoCtrl    = Robust_Flight_MPC_acados.Controller(self.uav_para, self.dt_ctrl)

        # XXX DistMPC solver couple with stm
        self.DistMPC    = Robust_Flight_MPC_acados.MPC(self.uav_para, self.load_para, self.cable_para, self.dt_ctrl, self.horizon, self.gamma, self.gamma2)
        self.DistMPC.SetStateVariable(self.stm.xi,self.stm.xq,self.stm.xl,self.stm.index_q)
        self.DistMPC.SetCtrlVariable(self.stm.ui,self.stm.ul,self.stm.ti)
        self.DistMPC.SetLoadParameter(self.stm.Jldiag,self.stm.rg)
        self.DistMPC.SetDyn(self.stm.model_i,self.stm.model_l,self.stm.dyni,self.stm.dynl)
        self.DistMPC.SetLearnablePara()
        self.DistMPC.SetQuadrotorCostDyn()
        self.DistMPC.SetPayloadCostDyn()
        self.DistMPC.SetConstraints_Qaudrotor()
        self.DistMPC.SetConstraints_Load()
        # self.DistMPC.MPCsolverQuadrotorInit_acados()
        self.DistMPC.MPCsolverQuadrotorInit_ros2_acados(self.drone_idx)
        # self.DistMPC.MPCsolverPayloadInit_acados()

        """--------------------------------------Define reference trajectories----------------------------------------"""
        # tilt angle to enlarge the inter-robot separation space, manually tuned, which will be learned in the future work
        self.angle_t = self.get_parameter('angle_t').value 
        
        # Figure 8 trajectory
        self.Coeffx        = np.zeros((8,8))
        self.Coeffy        = np.zeros((8,8))
        self.Coeffz        = np.zeros((8,8))
        for k in range(8):
            self.Coeffx[k,:] = np.load(os.path.join(self.package_share_directory, 'Reference_traj_fig8/coeffxl_'+str(k+1)+'.npy'))
            self.Coeffy[k,:] = np.load(os.path.join(self.package_share_directory, 'Reference_traj_fig8/coeffyl_'+str(k+1)+'.npy'))
            self.Coeffz[k,:] = np.load(os.path.join(self.package_share_directory, 'Reference_traj_fig8/coeffzl_'+str(k+1)+'.npy'))

    def _initialize_qos(self):
        """Initialize QoS settings for the node."""
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.control_logic_callback_group = MutuallyExclusiveCallbackGroup() 
        self.state_update_callback_group = ReentrantCallbackGroup() 
        self.stm_pre_callback_group = ReentrantCallbackGroup()  

    def _declare_ros_parameters(self):
        """Declare parameters for the node."""
        self.declare_parameter('uav_para', [1.0, 0.02, 0.02, 0.04, 6.0, 0.2])
        self.declare_parameter('load_para', [7.0, 1.0])
        self.declare_parameter('cable_para', [1e9, 8e-6, 1e-2, 2.0]) # E=1 Gpa, A=7mm^2 (pi*1.5^2), c=10, L0=2, Nylon-HD
        self.declare_parameter('Jl', [0.7 * x for x in [2.0, 2.0, 2.5]])  
        self.declare_parameter('rg', [0.1, 0.1, -0.1]) 
        self.declare_parameter('angle_t', np.pi / 9)

        self.declare_parameter('drone_idx', 0)
        self.declare_parameter('altitude', 5.0) 
        self.declare_parameter('dt_ctrl', 5e-2)  
        self.declare_parameter('dt_broadcast', 2e-2) 
        self.declare_parameter('init_timestamp', None).value
        self.dt_broadcast = self.get_parameter('dt_broadcast').value
        self.altitude = self.get_parameter('altitude').value

        self.drone_idx = self.get_parameter('drone_idx').value
        self.prefix = '' if self.drone_idx == 0 else f'/px4_{self.drone_idx}'

        self.init_timestamp = self.get_parameter('init_timestamp').value
        if self.init_timestamp is None:
            self.get_logger().error("Initial timestamp is not available, check the parameter.")
            raise ValueError("Initial timestamp is not available, check the parameter.")
    
    def _initialize_neural_network(self):
        """Initialize the neural network and load the model."""
        self.pmin, self.pmax = 0.01, 100 # lower and upper bounds of the weightings
        # XXX Registe the NeuralNet Class as model to load the nn_load.pt
        neuralnet_path = os.path.join(self.package_share_directory, "NeuralNet.py")
        spec = importlib.util.spec_from_file_location("NeuralNet", neuralnet_path)
        neuralnet_module = importlib.util.module_from_spec(spec)
        sys.modules["NeuralNet"] = neuralnet_module
        spec.loader.exec_module(neuralnet_module)

        # Load the trained neural network model of the drone
        self.nn_quad = torch.load(os.path.join(self.package_share_directory, "trained data/trained_nn_quad_"+str(self.drone_idx)+".pt"))
        
    def _initialize_MPC_states(self):
        """Initialize the state of the drone and payload."""
        # # Initialize the payload's state
        # self.pl = np.zeros((3, 1))  # payload position
        # self.vl = np.zeros((3, 1))  # payload velocity
        # self.ql = np.array([[1.0], [0.0], [0.0], [0.0]])  # payload quaternion
        # self.wl = np.zeros((3, 1))  # payload angular velocity
        # self.xl = np.vstack((self.pl, self.vl, self.ql, self.wl))

        # Initialize the drone's state
        self.pi    = np.zeros((3, 1))  # drone position
        self.vi    = np.zeros((3, 1))  # drone velocity
        self.qi    = np.array([[1.0], [0.0], [0.0], [0.0]])  # drone quaternion
        self.wi    = np.zeros((3, 1))  # drone angular velocity
        self.xi    = np.vstack((self.pi, self.vi, self.qi, self.wi))
        self.T_end = self.stm.Tc # Trajectory duration

        # L1-AC, initial values used in the low-pass filter
        self.sig_f_prev   = 0
        self.u_prev       = np.array([[self.uav_para[0]*9.81,0,0,0]]).T
        self.z_hat        = np.zeros((3,1))

        # HACK Service msg variables
        self.MPC_TRAJ              = False # QNode state transition flag
        # self.MPC_TRAJ              = True # QNode state transition flag
        # NOTE: Initial from the CentralNode and compare with msg.time_traj
        self.time_traj             = 0.0
        self.REC_TEMP_i            = True 
        # self.opt_system -> solve QMPC, CAN NOT BE UPDATED, only READ and USE
        self.xq_traj, self.uq_traj = [],[]
        self.xl_traj, self.ul_traj = np.zeros((self.horizon+1, self.nxl)), np.zeros((self.horizon, self.nul))
        
        # QMPC's return temp_traj, UPDATED VARIABLES
        self.xi_temp, self.ui_temp = np.zeros((self.horizon+1, self.nxi)), np.zeros((self.horizon, self.nui))
        # NOTE QMPC's real control inputs
        self.xi_ctrl, self.ui_ctrl = np.zeros((self.nxi, 1)), np.zeros((self.nui, 1))

        # Generate the reference trajectory here, update when msg.time_traj > self.time_traj
        self.Ref_xq, self.Ref_uq, self.Ref_xl, self.Ref_ul, self.Ref0_xq, self.Ref0_l = self.Reference_for_MPC(self.time_traj, self.angle_t)
        
        self.transfer_ctrl_flag = False  # Flag to transfer Geometric control to MPC control
        self.qmpc_start_time = 0.0 

        # Add reference frame offset in world ENU
        angle_increment = 2 * math.pi / int(self.uav_para[4])
        angle = angle_increment * self.drone_idx
        self.ref_translation = np.array([
            (self.load_para[1] + self.cable_para[3] + 0.005) * math.cos(angle),
            (self.load_para[1] + self.cable_para[3] + 0.005) * math.sin(angle),
            0.1
        ])  # This is already ENU world frame offset

    def _initialize_geom_states(self):
        """Initialize the geometric controller parameters."""
        # Control initialization
        self.x = np.zeros(3)
        self.v = np.zeros(3)
        self.W = np.zeros(3)
        self.a = np.zeros(3)
        self.R = np.identity(3)

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

        # Controller gains
        self.kX = np.diag([8.0, 8.0, 10.0])
        self.kV = np.diag([5.0, 5.0, 10.0])

        self.f_total = 0.0  # Total thrust

        # Maximum thrust (modify based on actual vehicle)
        self.thrust_ratio = 2.00  # Ratio of thrust to weight
        self.max_thrust_newtons = self.m * self.g * self.thrust_ratio

        # Errors
        self.ex = np.zeros(3)
        self.ev = np.zeros(3)
        self.eR = np.zeros(3)
        self.eW = np.zeros(3)

    def _initialize_geom_trajectory(self):
        """Initialize the trajectory parameters."""
        self.timestamp_us = self.init_timestamp
        # flag to record initial offset of each drone
        self.xy_offset_flag = False
        self.x_init = 0.0
        self.y_init = 0.0
        self.x_final = 0.0
        self.y_final = 0.0
        self.payload_xy_offset_flag = False
        self.x_init_load = 0.0
        self.y_init_load = 0.0

        self.takeoff_time = 10.0 # Takeoff time (s)
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        # Circle trajectory parameters
        self.circle_W = 0.2  
        self.circle_radius = 5.0  

    def _initialize_services(self):
        """Initialize the services for the node."""
        self.QMPC_service = self.create_service(
            SyncMPC,
            f'/quadMPC_{self.drone_idx}_srv',
            self.qmpc_srv_callback
        )
    
    def _initialize_clients(self):
        """Initialize clients to all UAVs' services."""
        self.qmpc_clients = {}  

        num_uavs = int(self.uav_para[4])
        for i in range(num_uavs):
            client = self.create_client(SyncMPC, f'/quadMPC_{i}_srv')
            self.qmpc_clients[i] = client

            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for /quadMPC_{i}_srv...')

    def _initialize_publishers(self):
        """Initialize the publishers for the node."""
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
        self.publisher_vehicle_position_setpoint = self.create_publisher(
            TrajectorySetpoint,
            f'{self.prefix}/fmu/in/trajectory_setpoint',
            self.qos_profile
        )

    def _initialize_subscribers(self):
        """Initialize the subscribers for the node."""
        # Global Subscriber to sync time between multiple drones
        self.create_subscription(
            Int64,
            '/sync_time',
            self.sync_time_callback,
            self.qos_profile,
            callback_group=self.state_update_callback_group
        )
        # Local Subscriber to update arming, navigation state
        self.create_subscription(
            VehicleStatus,
            f'{self.prefix}/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            self.qos_profile,
            callback_group=self.state_update_callback_group
        )
        # Local Subscribers to update x, v, R(q), W of the drone
        self.create_subscription(
            VehicleOdometry,
            f'{self.prefix}/fmu/out/vehicle_odometry',
            self.vehicle_odometry_callback,
            self.qos_profile,
            callback_group=self.state_update_callback_group
        )
        # Local Subscribers to update acceleration
        self.create_subscription(
            VehicleLocalPosition,
            f'{self.prefix}/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            self.qos_profile,
            callback_group=self.state_update_callback_group
        )
        # # Subscribers to the payload's state (xl, vl, ql, wl) from IsaacSim ros2 node
        # self.one_time_subscriber = self.create_subscription(
        #     PoseStamped,
        #     '/payload_pose',
        #     self.payload_pos_callback,
        #     self.qos_profile,
        #     callback_group=self.state_update_callback_group
        # )
        # self.create_subscription(
        #     TwistStamped,
        #     '/payload_twist',
        #     self.payload_twist_callback,
        #     self.qos_profile,
        #     callback_group=self.state_update_callback_group
        # )

    ## --- Parameterization of the neural network ---
    def SetPara_quadrotor(self, nn_i_output):
        Qik_diag   = np.zeros((1,self.nwsi)) # diagonal weighting for the quadrotor's state in the running cost
        QiN_diag   = np.zeros((1,self.nwsi)) # diagonal weighting for the quadrotor's state in the terminal cost
        Rik_diag   = np.zeros((1,self.nui)) # diagonal weighting for the quadrotor's control in the running cost
        for k in range(self.nwsi):
            Qik_diag[0,k] = self.pmin + (self.pmax-self.pmin)*nn_i_output[0,k]
            QiN_diag[0,k] = self.pmin + (self.pmax-self.pmin)*nn_i_output[0,self.nwsi+k]
        for k in range(self.nui):
            Rik_diag[0,k] = self.pmin + (self.pmax-self.pmin)*nn_i_output[0,2*self.nwsi+k]
        weight_i   = np.hstack((Qik_diag,QiN_diag,Rik_diag))

        return weight_i

    def SetPara_load(self, nn_l_output):
        Qlk_diag   = np.zeros((1,self.nwsl)) # diagonal weighting for the payload's state in the running cost
        QlN_diag   = np.zeros((1,self.nwsl)) # diagonal weighting for the payload's state in the terminal cost
        Rlk_diag   = np.zeros((1,self.nul)) # diagonal weighting for the payload's control in the running cost
        for k in range(self.nwsl):
            Qlk_diag[0,k] = self.pmin + (self.pmax-self.pmin)*nn_l_output[0,k]
            QlN_diag[0,k] = self.pmin + (self.pmax-self.pmin)*nn_l_output[0,self.nwsl+k]
        for k in range(self.nul):
            Rlk_diag[0,k] = self.pmin + (self.pmax-self.pmin)*nn_l_output[0,2*self.nwsl+k]
        weight_l   = np.hstack((Qlk_diag,QlN_diag,Rlk_diag))

        return weight_l

    def chainRule_gradient_quad(self, nn_i_output):
        tunable = SX.sym('tp',1,self.Di_out)
        Qik_dg  = SX.sym('Qik_dg',1,self.nwsi)
        QiN_dg  = SX.sym('QiN_dg',1,self.nwsi)
        Rik_dg  = SX.sym('Rik_dg',1,self.nui)
        for k in range(self.nwsi):
            Qik_dg[0,k] = self.pmin + (self.pmax-self.pmin)*tunable[0,k]
            QiN_dg[0,k] = self.pmin + (self.pmax-self.pmin)*tunable[0,self.nwsi+k]
        for k in range(self.nui):
            Rik_dg[0,k] = self.pmin + (self.pmax-self.pmin)*tunable[0,2*self.nwsi+k]
        weighti = horzcat(Qik_dg,QiN_dg,Rik_dg)
        w_i_jaco= jacobian(weighti,tunable)
        w_i_jaco_fn = Function('w_i_jaco',[tunable],[w_i_jaco],['tp0'],['w_i_jacof'])
        weight_i_grad = w_i_jaco_fn(tp0=nn_i_output)['w_i_jacof'].full()
        return weight_i_grad

    def chainRule_gradient_load(self, nn_l_output):
        tunable = SX.sym('tp',1,self.Dl_out)
        Qlk_dg  = SX.sym('Qlk_dg',1,self.nwsl)
        QlN_dg  = SX.sym('QlN_dg',1,self.nwsl)
        Rlk_dg  = SX.sym('Rlk_dg',1,self.nul)
        for k in range(self.nwsl):
            Qlk_dg[0,k] = self.pmin + (self.pmax-self.pmin)*tunable[0,k]
            QlN_dg[0,k] = self.pmin + (self.pmax-self.pmin)*tunable[0,self.nwsl+k]
        for k in range(self.nul):
            Rlk_dg[0,k] = self.pmin + (self.pmax-self.pmin)*tunable[0,2*self.nwsl+k]
        weightl = horzcat(Qlk_dg,QlN_dg,Rlk_dg)
        w_l_jaco= jacobian(weightl,tunable)
        w_l_jaco_fn = Function('w_l_jaco',[tunable],[w_l_jaco],['tp0'],['w_l_jacof'])
        weight_l_grad = w_l_jaco_fn(tp0=nn_l_output)['w_l_jacof'].full()
        return weight_l_grad

    def convert_quadrotor_nn(self, nn_i_outcolumn):
        # convert a column tensor to a row np.array
        nn_i_row = np.zeros((1,self.Di_out))
        for i in range(self.Di_out):
            nn_i_row[0,i] = nn_i_outcolumn[i,0]
        return nn_i_row

    def convert_load_nn(self, nn_l_outcolumn):
        # convert a column tensor to a row np.array
        nn_l_row = np.zeros((1,self.Dl_out))
        for i in range(self.Dl_out):
            nn_l_row[0,i] = nn_l_outcolumn[i,0]
        return nn_l_row

    ## --- Ref Trajectory Generation ---
    def Reference_for_MPC(self, time_traj, angle_t):
        # The input time_traj, angle_t belongs to the class, with self.
        Ref_xq  = [] # quadrotors' state reference trajectories for MPC, ranging from the current k to future k + horizon
        Ref_uq  = [] # quadrotors' control reference trajectories for MPC, ranging from the current k to future k + horizon
        Ref_xl  = np.zeros((self.nxl,self.horizon+1))
        Ref_ul  = np.zeros((self.nul,self.horizon))
        Ref0_xq = [] # current quadrotors' reference position and velocity

        # quadrotor's reference
        for i in range(self.nq):
            Ref_xi  = np.zeros((self.nxi,self.horizon+1))
            Ref_ui  = np.zeros((self.nui,self.horizon))
            # quadrotor's reference in Horizon
            for j in range(self.horizon):
                ref_p, ref_v, ref_a   = self.stm.minisnap_quadrotor_fig8(self.Coeffx, self.Coeffy, self.Coeffz,
                                                                    time_traj + j*self.dt_ctrl, angle_t, i)
                # ref_p, ref_v, ref_a   = self.stm.new_circle_quadrotor(self.coeffa,time_traj + j*self.dt_ctrl, self.angle_t, i)
                # ref_p, ref_v, ref_a   = self.stm.hovering_quadrotor(self.angle_t, i)

                if i==0: # we only need to compute the payload's reference for an arbitrary quadrotor
                    ref_pl, ref_vl, ref_al   = self.stm.minisnap_load_fig8(self.Coeffx, self.Coeffy, self.Coeffz,
                                                                        time_traj + j*self.dt_ctrl)
                    # ref_pl, ref_vl, ref_al   = self.stm.new_circle_load(coeffa,time_traj + j*dt_ctrl)
                    # ref_pl, ref_vl, ref_al   = self.stm.hovering_load()

                qd, wd, f_ref, fl_ref, M_ref = self.GeoCtrl.system_ref(ref_a, self.load_para[0], ref_al)
                ref_xi    = np.vstack((ref_p,ref_v,qd,wd))
                ref_ui    = np.vstack((f_ref,M_ref)) 
                Ref_xi[:,j:j+1] = ref_xi
                Ref_ui[:,j:j+1] = ref_ui

                if i==0:
                    qld       = np.array([[1,0,0,0]]).T # desired quaternion of the payload, representing the identity matrix
                    wld       = np.zeros((3,1)) # deisred angular velocity of the payload
                    ref_xl    = np.vstack((ref_pl, ref_vl, qld, wld))
                    ref_ul    = fl_ref/self.nul*np.ones((self.nul,1))
                    Ref_xl[:,j:j+1] = ref_xl
                    Ref_ul[:,j:j+1] = ref_ul
                    if j==0:
                        Ref0_l   = ref_xl
                if j == 0:
                    Ref0_xq += [np.vstack((ref_p,ref_v))]

            # horizon:horizon+1
            ref_p, ref_v, ref_a  = self.stm.minisnap_quadrotor_fig8(self.Coeffx, self.Coeffy, self.Coeffz,
                                                                time_traj + self.horizon*self.dt_ctrl, self.angle_t, i)    
            # ref_p, ref_v, ref_a  = self.stm.new_circle_quadrotor(self.coeffa,time_traj + self.horizon*self.dt_ctrl, self.angle_t, i)
            # ref_p, ref_v, ref_a   = self.stm.hovering_quadrotor(self.angle_t, i)
            if i==0:
                ref_pl, ref_vl, ref_al   = self.stm.minisnap_load_fig8(self.Coeffx, self.Coeffy, self.Coeffz,
                                                                    time_traj + self.horizon*self.dt_ctrl)
                # ref_pl, ref_vl, ref_al   = self.stm.new_circle_load(self.coeffa,time_traj + self.horizon*self.dt_ctrl)
                # ref_pl, ref_vl, ref_al   = self.stm.hovering_load()

            qd, wd, f_ref, fl_ref, M_ref = self.GeoCtrl.system_ref(ref_a, self.load_para[0], ref_al)
            ref_xi    = np.vstack((ref_p,ref_v,qd,wd))
            Ref_xi[:,self.horizon:self.horizon+1] = ref_xi
            Ref_xq   += [Ref_xi]
            Ref_uq   += [Ref_ui]
            if i==0:
                ref_xl    = np.vstack((ref_pl, ref_vl, qld, wld))
                Ref_xl[:,self.horizon:self.horizon+1] = ref_xl
            
        return Ref_xq, Ref_uq, Ref_xl, Ref_ul, Ref0_xq, Ref0_l

    # --- Geom Trajectory Generation ---
    def generate_takeoff_trajectory(self):
        # NOTE The desired position xd should be in the NED local frame.
        """Generate the takeoff trajectory to reach the hover attitude."""
        if not hasattr(self, 'takeoff_init_flag'):
            self.takeoff_start = self.timestamp_us
            d_ref_position_enu_world = self.Ref0_xq[self.drone_idx][0:3,0] # Ref in time_traj = 0

            self.x_final = d_ref_position_enu_world[0]
            self.y_final = d_ref_position_enu_world[1]
            self.takeoff_init_flag = True

        dt_s = (self.timestamp_us - self.takeoff_start) * 1e-6
        current_z = self.x[2]  # Current altitude in NED local frame
        z_target = - self.altitude # Target altitude in NED local frame
        if dt_s < self.takeoff_time:
            alpha = dt_s / self.takeoff_time
            self.xd[2] = current_z + alpha * (z_target - current_z)
            self.xd_dot[2] = (z_target - current_z) / self.takeoff_time
            self.xd_2dot[2] = 0.0
            self.xd_3dot[2] = 0.0
            self.xd_4dot[2] = 0.0
        else:
            self.xd[2] = z_target
            self.xd_dot[2] = 0.0
            self.xd_2dot[2] = 0.0
            self.xd_3dot[2] = 0.0
            self.xd_4dot[2] = 0.0

        # NOTE x_init, y_init; x_final, y_final are in the ENU world frame. xd is in the NED local frame
        if dt_s < self.takeoff_time:
            alpha = dt_s / self.takeoff_time
            self.xd[1] = self.x_init - self.ref_translation[0] + alpha * (self.x_final - self.x_init)
            self.xd[0] = self.y_init - self.ref_translation[1] + alpha * (self.y_final - self.y_init)
            self.xd_dot[1] = (self.x_final - self.x_init) / self.takeoff_time
            self.xd_dot[0] = (self.y_final - self.y_init) / self.takeoff_time
            self.xd_2dot[0] = self.xd_2dot[1] = 0.0
            self.xd_3dot[0] = self.xd_3dot[1] = 0.0
            self.xd_4dot[0] = self.xd_4dot[1] = 0.0
        else:
            self.xd[1] = self.x_final - self.ref_translation[0]
            self.xd[0] = self.y_final - self.ref_translation[1]
            self.xd_dot[0] = self.xd_dot[1] = 0.0
            self.xd_2dot[0] = self.xd_2dot[1] = 0.0
            self.xd_3dot[0] = self.xd_3dot[1] = 0.0
            self.xd_4dot[0] = self.xd_4dot[1] = 0.0

        self.b1d = np.array([0.0, 1.0, 0.0])
        self.b1d_dot = np.zeros(3)
        self.b1d_2dot = np.zeros(3)

    def generate_trajectory(self, type='circle'):
        """
        Generate the desired trajectory and update the desired states, after the takeoff.
        (Note: current implementation is for a single drone.)
        """
        if not hasattr(self, 'traj_init_flag'):
            self.x_start = self.x.copy()
            self.t_start = self.timestamp_us
            self.traj_init_flag = True

        dt_traj = (self.timestamp_us - self.t_start) * 1e-6

        if type == 'circle':
            circle_W = self.circle_W
            circle_radius = self.circle_radius
            theta = circle_W * dt_traj

            # self.x is in the NED local frame
            offset_y = self.x_start[0] - circle_radius * np.cos(0)
            offset_x = self.x_start[1] - circle_radius * np.sin(0)

            # Position and its derivatives in the NED local frame
            self.xd[0] = circle_radius * np.cos(theta) + offset_y
            self.xd_dot[0] = -circle_radius * circle_W * np.sin(theta)
            self.xd_2dot[0] = -circle_radius * (circle_W**2) * np.cos(theta)
            self.xd_3dot[0] = circle_radius * (circle_W**3) * np.sin(theta)
            self.xd_4dot[0] = circle_radius * (circle_W**4) * np.cos(theta)

            self.xd[1] = circle_radius * np.sin(theta) + offset_x
            self.xd_dot[1] = circle_radius * circle_W * np.cos(theta)
            self.xd_2dot[1] = -circle_radius * (circle_W**2) * np.sin(theta)
            self.xd_3dot[1] = -circle_radius * (circle_W**3) * np.cos(theta)
            self.xd_4dot[1] = circle_radius * (circle_W**4) * np.sin(theta)

            self.xd[2] = - self.altitude
            self.xd_dot[2] = 0.0
            self.xd_2dot[2] = 0.0
            self.xd_3dot[2] = 0.0
            self.xd_4dot[2] = 0.0

            # Compute desired yaw angle based on the velocity
            # For a circle the unit tangent is given by:
            #    (-sin(theta), cos(theta)) = [cos(theta+pi/2), sin(theta+pi/2)]
            psi_des = theta + np.pi/2
            self.b1d = np.array([np.cos(psi_des), np.sin(psi_des), 0.0])
            self.b1d_dot = np.array([-circle_W * np.sin(psi_des),
                                      circle_W * np.cos(psi_des),
                                      0.0])
            self.b1d_2dot = np.array([-circle_W**2 * np.cos(psi_des),
                                       -circle_W**2 * np.sin(psi_des),
                                       0.0])
        else:  # hover
            self.xd = self.x_start
            self.xd_dot = np.zeros(3)
            self.xd_2dot = np.zeros(3)
            self.b1d = np.array([0.0, 1.0, 0.0])

    # --- Publisher Functions ---
    # FIXME Thrust + Attitude or Thrust + Torque, CHECK HERE
    def qmpc_publish_command(self):
        """
        Actual control logic for the QNode. Robostify by L1-AC.
        Params: opt_system xq_traj (ith quaternion) + uq_traj (ith thrust + torque),
        -> Normalized Thrust + desired attitude setpoint,
        -> publish VehicleAttitudeSetpoint msgs
        """
        # # FIXME 1. Check the logic here! L1-AC robustify ui_ctrl[0,0], thrust
        # xi       = self.xi # REAL_TIME quadrotor's state
        # z_hat    = self.z_hat
        # dm_hat, dum_hat, A_s = self.GeoCtrl.L1_adaptive_law(xi, z_hat)
        # # Low-pass filter
        # wf_coff  = 20 
        # time_constf = 1/wf_coff
        # f_prev   = self.sig_f_prev
        # f_lpf    = self.GeoCtrl.lowpass_filter(time_constf, dm_hat, f_prev)
        # # only the first control command is applied to the system
        # # NOTE Robustify the nominal control using the L1-AC compensation
        # self.ui_ctrl[0,0]     += -f_lpf
        # # update the state prediction z_hat in L1-AC
        # ui       = self.ui_ctrl
        # ti       = self.ul_traj[0,self.drone_idx]
        # z_hatnew = self.stm.predictor_L1(z_hat, xi, ui, self.xl, ti, dm_hat, dum_hat, A_s, self.drone_idx, self.dt_ctrl)
        # self.z_hat = z_hatnew

        # FIXME 2. Use the current xi_ctrl, ui_ctrl (update from CentralNode) to apply Attitude setpoint and thrust control
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = self.timestamp_us

        q_frd_to_enu_d = self.xi_ctrl[6:10, 0]
        q_norm = np.linalg.norm(q_frd_to_enu_d)
        if q_norm < 1e-6:
            q_d = np.array([1.0, 0.0, 0.0, 0.0])  # fallback
        else:
            q_d = q_frd_to_enu_d / q_norm

        SQRT2_INV = 0.70710678118
        q_enu_to_ned = [SQRT2_INV, 0, 0, SQRT2_INV]
        q_frd_to_ned_d = q_multiply(q_enu_to_ned, q_d)
        msg.q_d = q_frd_to_ned_d.tolist()  # desired quaternion

        norm_thrust = self.ui_ctrl[0,0] / self.max_thrust_newtons
        norm_thrust = max(0.0, min(norm_thrust, 1.0))
        msg.thrust_body = [0.0, 0.0, -norm_thrust]

        self.get_logger().info(f"drone_ID: {self.drone_idx}, thrust:{-self.ui_ctrl[0,0]:.2f}, norm_thrust:{-norm_thrust:.2f}, q_d:{q_d.tolist()}")
        
        # 3. Publish the VehicleAttitudeSetpoint msg
        self.publisher_vehicle_attitude_setpoint.publish(msg)

    def qmpc_publish_position(self):
        """Publish the desired position setpoint for the vehicle."""
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_us

        # d_ref_position_enu_world = self.Ref0_xq[self.drone_idx][0:3,0]
        # d_ref_position_enu_local = d_ref_position_enu_world - self.ref_translation
        # d_ref_position_ned_local = np.array([d_ref_position_enu_local[1], d_ref_position_enu_local[0], -d_ref_position_enu_local[2]])
        # msg.position[0] = d_ref_position_ned_local[0]
        # msg.position[1] = d_ref_position_ned_local[1]
        # msg.position[2] = d_ref_position_ned_local[2]

        d_position_enu_world = self.xi_ctrl[0:3, 0]
        d_position_enu_local = d_position_enu_world - self.ref_translation
        d_position_ned_local = np.array([d_position_enu_local[1], d_position_enu_local[0], -d_position_enu_local[2]])
        msg.position[0] = d_position_ned_local[0]
        msg.position[1] = d_position_ned_local[1]
        msg.position[2] = d_position_ned_local[2]

        # Publish the TrajectorySetpoint msg
        self.publisher_vehicle_position_setpoint.publish(msg)

    def geom_publish_command(self):
        """Implement the geometric control law and publish the desired attitude setpoint and thrust."""
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

        A = - kX @ eX - kV @ eV - m*g*e3 + m*xd_2dot
        hatW = hat(W)

        b3 = R @ e3
        b3_dot = R @ hatW @ e3

        f_total = -A @ b3

        ea = g*e3 - (f_total/m)*b3 - xd_2dot
        A_dot = - kX @ eV - kV @ ea + m*xd_3dot
        fdot = - A_dot @ b3 - A @ b3_dot
        eb = - (fdot/m)*b3 - (f_total/m)*b3_dot - xd_3dot
        A_2dot = - kX @ ea - kV @ eb + m*xd_4dot

        b3c, b3c_dot, b3c_2dot = deriv_unit_vector(-A, -A_dot, -A_2dot)

        hat_b1d = hat(b1d)
        hat_b1d_dot = hat(b1d_dot)
        A2 = -hat_b1d @ b3c
        A2_dot = - hat_b1d_dot @ b3c - hat_b1d @ b3c_dot
        A2_2dot = - hat(b1d_2dot) @ b3c - 2.0*hat_b1d_dot @ b3c_dot - hat_b1d @ b3c_2dot

        b2c, b2c_dot, b2c_2dot = deriv_unit_vector(A2, A2_dot, A2_2dot)
        hat_b2c = hat(b2c)
        hat_b2c_dot = hat(b2c_dot)

        b1c = hat_b2c @ b3c
        b1c_dot = hat_b2c_dot @ b3c + hat_b2c @ b3c_dot
        b1c_2dot = hat(b2c_2dot) @ b3c + 2.0*hat_b2c_dot @ b3c_dot + hat_b2c @ b3c_2dot

        # --- New: ensure b1c aligns with the desired b1d ---
        if np.dot(b1c, b1d) < 0:
            b1c = -b1c

        Rd = np.vstack((b1c, b2c, b3c)).T
        Rd_dot = np.vstack((b1c_dot, b2c_dot, b3c_dot)).T
        Rd_2dot = np.vstack((b1c_2dot, b2c_2dot, b3c_2dot)).T

        Rd_T = Rd.T
        Wd = vee(Rd_T @ Rd_dot)
        hat_Wd = hat(Wd)
        Wd_dot = vee(Rd_T @ Rd_2dot - hat_Wd @ hat_Wd)

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

        # --- Publish the VehicleAttitudeSetpoint msg ---
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = self.timestamp_us
        msg.q_d = R_to_q(Rd)

        norm_thrust = f_total / self.max_thrust_newtons + 0.25
        norm_thrust = max(0.0, min(norm_thrust, 1.0))
        msg.thrust_body = [0.0, 0.0, -norm_thrust]
        self.get_logger().info(f"drone_ID: {self.drone_idx}, thrust:{-f_total:.2f}, norm_thrust:{-norm_thrust:.2f}, q_d:{R_to_q(Rd)}")
        # self.get_logger().info(f"thrust:{-norm_thrust:.2f}, q_d:{R_to_q(Rd)}")
        self.publisher_vehicle_attitude_setpoint.publish(msg)

    def publish_vehicle_command(self, drone_idx: int, command: int,
                                param1: float = 0.0, param2: float = 0.0,
                                timestamp_us: int = None):
        if timestamp_us is None:
            self.get_logger().error(
                f'Timestamp is not available at drone_{self.drone_idx}, check the clock node.')
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = drone_idx + 1
        msg.target_component = 1
        msg.source_system = drone_idx + 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = timestamp_us
        self.publisher_vehicle_command.publish(msg)

    def arm(self, drone_idx: int, timestamp_us: int = None):
        self.publish_vehicle_command(
            drone_idx=drone_idx,
            command=VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
            timestamp_us=timestamp_us
        )

    # --- Subscriber Callback Functions ---
    def qmpc_srv_callback(self, request, response):
        """Callback function for the QMPC service request, solve the QMPC and response."""

        # 1. Handle the request and update the local QMPC variables
        self.MPC_TRAJ = request.mpc_traj
        rec_temp      = np.array(request.rec_temp).reshape((self.nq, 1))

        self.REC_TEMP_i = rec_temp[self.drone_idx, 0]
        if (self.REC_TEMP_i == False or self.time_traj == 0):
            # Trajectory from the CentralNode -->
            # if update_ctrl == True, the while loop ends, update control and local trajectory to start the next loop and solve the QMPC;
            # elif update_ctrl == False, the while loop continues, QMPC using the trajectory from the CentralNode to solve the QMPC.
            
            self.xq_traj = self.recover_traj_list(request.xq_traj, self.nq, (self.horizon + 1, self.nxi))
            self.uq_traj = self.recover_traj_list(request.uq_traj, self.nq, (self.horizon, self.nui))
            self.xl_traj = np.array(request.xl_traj, dtype=np.float32).reshape((self.horizon + 1, self.nxl))
            self.ul_traj = np.array(request.ul_traj, dtype=np.float32).reshape((self.horizon, self.nul))

        if (request.time_traj > self.time_traj):
            self.time_traj = request.time_traj
            # Update reference trajectory
            self.Ref_xq, self.Ref_uq, self.Ref_xl, self.Ref_ul, self.Ref0_xq, self.Ref0_l = self.Reference_for_MPC(self.time_traj, self.angle_t)

        # CentralNode finishes the current while loop, thus each drone can update its local variables
        if (request.update_ctrl):
            # -- BUG Update the local MPC control variables FROM the CentralNode --
            self.ui_ctrl = self.uq_traj[self.drone_idx][0,:].reshape((self.nui,1)) # opt control -> thrust
            self.xi_ctrl = self.xq_traj[self.drone_idx][0,:].reshape((self.nxi,1)) # opt state -> quaternion
            
            # Cancel the Geometric control timer, transfer to the MPC control
            if not self.transfer_ctrl_flag:
                self.geom_ctrl_timer.cancel()
                self.transfer_ctrl_flag = True

            # self.qmpc_publish_position() # TrajectorySetpoint msg (position setpoint)
            self.qmpc_publish_command() # VehicleAttitudeSetpoint msg (thrust + attitude setpoint)

            # -- Update the local xi, ui for the next while loop, prepare for the next MPC solve --
            xq_traj = []
            uq_traj = []
            # Update the trajectory using the reference trajectory
            for ki in range(len(self.Ref_uq)):
                xq_traj  += [self.Ref_xq[ki].T]
                uq_traj  += [self.Ref_uq[ki].T]
            xq_traj += [self.Ref_xq[-1].T]
            self.xq_traj = xq_traj
            self.uq_traj = uq_traj
            self.xl_traj  = self.Ref_xl.T
            self.ul_traj  = self.Ref_ul.T

            # Horizon moved forward by one time-step, Only moved forward 1 time for each while loop!
            xq_traj_prev = self.xq_traj
            uq_traj_prev = self.uq_traj
            xl_traj_prev = self.xl_traj
            ul_traj_prev = self.ul_traj
            xq_traj = []
            uq_traj = []
            for iq in range(self.nq):
                xiq_traj = np.zeros((self.horizon+1,self.nxi))
                uiq_traj = np.zeros((self.horizon,self.nui))
                xi_prev  = xq_traj_prev[iq]
                ui_prev  = uq_traj_prev[iq]
                for iqk in range(self.horizon):
                    xiq_traj[iqk,:] = xi_prev[iqk+1,:]
                    if iqk <self.horizon-1:
                        uiq_traj[iqk,:] = ui_prev[iqk+1,:]
                    else:
                        uiq_traj[-1,:] = ui_prev[-1,:]
                xiq_traj[-1,:] = xi_prev[-1,:]
                xq_traj += [xiq_traj]
                uq_traj += [uiq_traj]
            self.xq_traj = xq_traj
            self.uq_traj = uq_traj
                
            xl_traj = np.zeros((self.horizon+1,self.nxl))
            ul_traj = np.zeros((self.horizon,self.nul))
            for il in range(self.horizon):
                xl_traj[il,:] = xl_traj_prev[il+1,:]
                if il <self.horizon-1:
                    ul_traj[il,:] = ul_traj_prev[il+1,:]
                else:
                    ul_traj[-1,:] = ul_traj_prev[-1,:]
            xl_traj[-1,:] = xl_traj_prev[-1,:]
            self.xl_traj = xl_traj
            self.ul_traj = ul_traj


        # 2. Solve the Quadrotor MPC 
        if (self.MPC_TRAJ and not self.REC_TEMP_i):
            self.qmpc_start_time = TM.time()
            self.max_viol_i = 0.0 # reset the max_viol_i for each iteration

            # Get the nn output based on the REAL-TIME quadrotor's state
            track_e_i   = self.xi[0:6,0]-self.Ref0_xq[self.drone_idx][0:6,0] 
            self.get_logger().info(f"drone_ID: {self.drone_idx}, track_e_i: {track_e_i}")
            input_i     = np.reshape(track_e_i,(self.Di_in,1))
            nn_i_output = self.convert_quadrotor_nn(self.nn_quad(input_i))
            weight_i    = self.SetPara_quadrotor(nn_i_output)

            # Solve the QMPC, get the temp_traj and max_viol_i, update into self. and return to CentralNode
            self.QuadrotorMPC(self.xq_traj, self.uq_traj, self.xl_traj, self.ul_traj,
                            self.Ref_xq[self.drone_idx], self.Ref_uq[self.drone_idx], weight_i, self.drone_idx)
            qmpctime = (TM.time() - self.qmpc_start_time)*1000
            self.get_logger().info(f"drone_ID: {self.drone_idx}, --- qmpc time: {qmpctime:.2f} ms ---")
        

        # 3. Return the response
        response.idx = self.drone_idx
        response.xi_temp = self.xi_temp.astype(np.float32).flatten().tolist()
        response.ui_temp = self.ui_temp.astype(np.float32).flatten().tolist()
        response.max_viol_i = self.max_viol_i
        return response

    def sync_time_callback(self, msg: Int64):
        self.timestamp_us = msg.data

    def recover_traj_list(self, flat_data: list, n_traj: int, traj_shape: tuple) -> list:
        """
        Restore flattened data into its original list structure.
        Args:
            flat_data (list): Flattened list received from a ROS2 message, e.g., msg.xq_traj.
            n_traj (int): Number of arrays in the list, e.g., nq.
            traj_shape (tuple): The shape to which each array should be restored, e.g., (horizon+1, nxi).
        Returns:
            list of np.ndarray: Original list structure, where each element is an ndarray with shape=traj_shape.
        """
        single_len = traj_shape[0] * traj_shape[1]
        flat_np = np.array(flat_data, dtype=np.float32)
        traj_list = [
            flat_np[i * single_len : (i + 1) * single_len].reshape(traj_shape)
            for i in range(n_traj)
        ]
        return traj_list

    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        if not self.xy_offset_flag:
            # Initial position in NED local frame
            self.x_init = msg.position[1] + self.ref_translation[0]
            self.y_init = msg.position[0] + self.ref_translation[1]
            self.xy_offset_flag = True
        
        # NED → ENU position and velocity conversion
        ned_position = np.array(msg.position)  # NED: [North, East, Down]
        ned_velocity = np.array(msg.velocity)

        enu_position_local = np.array([ned_position[1], ned_position[0], -ned_position[2]])  # ENU: [East, North, Up]
        enu_velocity_local = np.array([ned_velocity[1], ned_velocity[0], -ned_velocity[2]])

        enu_position_world = enu_position_local + self.ref_translation
        enu_velocity_world = enu_velocity_local 

        self.x = ned_position
        self.v = ned_velocity
        self.W = np.array(msg.angular_velocity)  # FRD frame

        # Store ENU world frame position and velocity
        self.pi = enu_position_world.reshape((3, 1))
        self.vi = enu_velocity_world.reshape((3, 1))
        # Angular velocity remains in body-FRD, no change here
        self.wi = np.array(msg.angular_velocity).reshape((3, 1))

        # XXX Convert quaternion from NED → ENU
        SQRT2_INV = 0.70710678118
        q_ned_to_enu = np.array([SQRT2_INV, 0, 0, -SQRT2_INV])  # [w, x, y, z]
        q_frd_to_ned = np.array(msg.q)
        # self.get_logger().info(f"Quaternion NED: {q_frd_to_ned}")
        q_frd_to_enu = q_multiply(q_ned_to_enu, q_frd_to_ned)
        # self.get_logger().info(f"Quaternion ENU: {q_frd_to_enu}")

        q0, q1, q2, q3 = msg.q
        self.qi = q_frd_to_enu.reshape((4, 1))
        self.R = np.array([
            [1 - 2*(q2**2 + q3**2),  2*(q1*q2 - q0*q3),     2*(q1*q3 + q0*q2)],
            [2*(q1*q2 + q0*q3),     1 - 2*(q1**2 + q3**2), 2*(q2*q3 - q0*q1)],
            [2*(q1*q3 - q0*q2),     2*(q2*q3 + q0*q1),     1 - 2*(q1**2 + q2**2)]
        ])

        # Final full state vector for the quadrotor
        self.xi = np.vstack((self.pi, self.vi, self.qi, self.wi))
        # self.get_logger().info(f"xi: {self.xi.flatten()}")

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        self.a[0] = msg.ax
        self.a[1] = msg.ay
        self.a[2] = msg.az

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def payload_pos_callback(self, msg: PoseStamped):
        # Update the payload's initial position and then destroy the subscriber
        if not self.payload_xy_offset_flag:
            self.x_init_load = msg.pose.position.x
            self.y_init_load = msg.pose.position.y
            self.payload_xy_offset_flag = True
        position = msg.pose.position
        orientation = msg.pose.orientation
        # update the payload's state
        self.pl = np.array([[position.x], [position.y], [position.z]])
        self.ql = np.array([[orientation.w], [orientation.x], [orientation.y], [orientation.z]])
        # update the payload's state in the system
        self.xl[0:3,0:1] = self.pl
        self.xl[6:10,0:1] = self.ql
        # self.get_logger().info(f"payload position: {self.pl.flatten()}, orientation: {self.ql.flatten()}")
    def payload_twist_callback(self, msg: TwistStamped):
        linear = msg.twist.linear
        angular = msg.twist.angular
        # update the payload's state
        self.vl = np.array([[linear.x], [linear.y], [linear.z]])
        self.wl = np.array([[angular.x], [angular.y], [angular.z]])
        # update the payload's state in the system
        self.xl[3:6,0:1] = self.vl
        self.xl[10:13,0:1] = self.wl
        # self.get_logger().info(f"payload velocity: {self.vl.flatten()}, angular velocity: {self.wl.flatten()}")
    
   
def main(args=None):
    rclpy.init(args=args)
    node = QuadNode()
    executor = MultiThreadedExecutor()

    executor.add_node(node)
    try:
        node.get_logger().info('Beginning Quadrotor Node, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    # rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

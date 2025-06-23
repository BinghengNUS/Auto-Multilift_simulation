import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import Int64
from mpc_msgs.msg import BroadcastMPC, QuadReturnMPC
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

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


class SyncNode(Node):
    ST_WAIT = 0
    ST_MPC = 1
    ST_END = 2

    def __init__(self):
        super().__init__('AutoMultilift_Sync_LMPC')
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

        ## Initialize the publishers and subscribers ##
        self._initialize_publishers()
        self._initialize_subscribers()

        ## State machine setup ##
        self.Lnode_state = self.ST_WAIT
        
        self.timer_LNode = self.create_timer(
            self.dt_ctrl ,
            self.Lnode_state_machine,
            callback_group=self.stm_pre_callback_group
        )

        
    # --- State Machine ---
    def Lnode_state_machine(self):
        """
        Main state machine to manage and synchronize the LMPC, QMPCs solve process.
          ST_WAIT       -> Wait for drones takeoff to the desired altitude (By Geom Ctrl)
          ST_MPC        -> Start the distributed MPC process
          ST_END        -> Finish the task, according to the self.T_end (self.stm.TC)
        """
        # 1) ST_WAIT
        if self.Lnode_state == self.ST_WAIT:
            # self.get_logger().info("----- Waiting for the quadrotors to take off -----")
            # Check if the payload is at the desired altitude
            if (self.pl[2] >= self.payload_altitude * 0.95 and self.vl[2] < 0.05):
                self.get_logger().info("----- Payload reached the desired altitude -----")
                # 1. Initialize xq_traj.. to broadcast, use REC_TEMP to start the QMPCs
                self.mpc_forward_timer = self.create_timer(
                    # self.dt_ctrl,
                    self.dt_broadcast,
                    self.mpc_forward_main,
                    callback_group=self.control_logic_callback_group)
                # 2. Start the timer to broadcast the distributed MPC
                self.mpc_broadcast_timer = self.create_timer(
                    self.dt_broadcast,
                    self.broadcast_distributed_mpc,
                    callback_group=self.stm_pre_callback_group)
                self.get_logger().info("----- Starting the distributed MPC -----")
                self.Lnode_state = self.ST_MPC

        # 2) ST_MPC: solve the distributed MPC, step the simulation by MPC controller
        # NOTE Actual control is applied in the Qnodes, from brocasted uq_traj & xq_traj
        elif self.Lnode_state == self.ST_MPC:
            self.MPC_TRAJ = True
            # End the tracking task after designated time (20s), traj_time accumulated by dt_ctrl
            if (self.time_traj >= self.T_end):
                self.get_logger().info("----- Stopping the distributed MPC -----")
                self.MPC_TRAJ = False
                if hasattr(self, 'mpc_forward_timer') and self.mpc_forward_timer and not self.mpc_forward_timer.is_canceled():
                    self.mpc_forward_timer.cancel()
                self.Lnode_state = self.ST_END

        # 3) ST_END: finish the task
        elif self.Lnode_state == self.ST_END:
            self.get_logger().info("----- Task finished -----")
            # NOTE leave timer_timestamp to publish the timestamp
            if hasattr(self, 'mpc_broadcast_timer') and self.mpc_broadcast_timer and not self.mpc_broadcast_timer.is_canceled():
                self.mpc_broadcast_timer.cancel()
            if hasattr(self, 'timer_LNode') and self.timer_LNode and not self.timer_LNode.is_canceled():
                self.timer_LNode.cancel()
            pass

    # --- MPC Forward dt_ctrl---
    def mpc_forward_main(self):
        if self.flag == False and self.while_flag == False:
            self.while_start = TM.time()

            # NOTE Use the reference trajectory to initialize the xq_traj, uq_traj, xl_traj, ul_traj for each while loop.
            xq_traj = []
            uq_traj = [] 
            for ki in range(len(self.Ref_uq)):
                xq_traj  += [self.Ref_xq[ki].T]
                uq_traj  += [self.Ref_uq[ki].T]
            xq_traj += [self.Ref_xq[-1].T]
            self.xq_traj = xq_traj
            self.uq_traj = uq_traj
            self.xl_traj  = self.Ref_xl.T
            self.ul_traj  = self.Ref_ul.T

            if self.temp_flag == False:
                self.xq_traj_temp = self.xq_traj
                self.uq_traj_temp = self.uq_traj
                self.temp_flag = True

            # self.flag = True

        # NOTE THE BROADCASTED DATA self.opt_system is updated here, broadcasted to solve QMPC.
        # THE CALCULATION DATA xq\uq_traj_temp is updated by QMPC in REAL-TIME and check inside this function.
        self.Distributed_forwardMPC(self.xq_traj,self.uq_traj,self.xl_traj,self.ul_traj, # transfer the cur as prev
                                    self.Ref_xl,self.Ref_ul,self.Ref0_l,self.Jl,self.rg)

    def Distributed_forwardMPC(self,
                            xq_traj_prev, uq_traj_prev, xl_traj_prev, ul_traj_prev, 
                            Ref_xl, Ref_ul, Ref0_l, Jl, rg):
        epsilon = 1e-2 # threshold for stopping the iteration
        k_max   = 5 # maximum number of iterations

        if self.while_flag == False:
            self.ke = 1
            self.max_viol = 5 # initial value of max_viol, defined as the maximum value of the differences between two trajectories in successive iterations for all quadrotors

            # NOTE 1. Horizon moved forward by one time-step, Only moved forward 1 time for each while loop!!!
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
            # NOTE update the broadcasted data 
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
            # NOTE update the broadcasted data 
            self.xl_traj = xl_traj
            self.ul_traj = ul_traj

            # NOTE 2. Mark the initial start of distributed QMPCs, QNode checks [i]== False to start.
            self.REC_TEMP = np.zeros((int(self.nq),1),dtype=bool)

            self.while_flag = True
        # self.get_logger().info(f"Before the while loop, ctrl step={self.k_ctrl}, ke={self.ke}, max_viol={self.max_viol}, REC_TEMP={self.REC_TEMP.flatten()}")

        # 3. Solve the QMPCs, until REC_TEMP[all] == True FIXME
        while self.max_viol>=epsilon and self.ke<=k_max and np.all(self.REC_TEMP):
            # self.get_logger().info("Enter the while loop")
            if self.ke > 1:
                self.iter_end = (TM.time() - self.iter_start) * 1000
                self.iter_start = TM.time()
                self.get_logger().info(f"ctrl step={self.k_ctrl}, ke={self.ke}, ----iter waiting time={self.iter_end:.3f}ms-----")

            lmpc_start = TM.time()

            # 4. Recive all QMPC's return traj_temp, all QMPCs stop, start the LMPC
            xl_fbh      = np.reshape(self.xl, self.nxl) # REAL-TIME payload state
            ref_xl      = np.zeros(self.nxl*(self.horizon+1))
            ref_ul      = np.zeros(self.nul*self.horizon)
            for k in range(self.horizon):
                ref_xlk = np.reshape(Ref_xl[:,k],self.nxl)
                ref_xl[k*self.nxl:(k+1)*self.nxl] = ref_xlk
                ref_ulk = np.reshape(Ref_ul[:,k],self.nul)
                ref_ul[k*self.nul:(k+1)*self.nul] = ref_ulk
            ref_xl[self.horizon*self.nxl:(self.horizon+1)*self.nxl]=np.reshape(Ref_xl[:,self.horizon],self.nxl)

            # NOTE k_ctrl's REAL-TIME tracking error for LMPC, as the input into nn_load
            pl_error     = np.reshape(self.xl[0:3,0] - Ref0_l[0:3,0],(3,1))
            vl_error     = np.reshape(self.xl[3:6,0] - Ref0_l[3:6,0],(3,1))
            ql           = self.xl[6:10,0]
            qlref        = Ref0_l[6:10,0]
            Rl           = self.stm.q_2_rotation(ql,1)
            Rlref        = self.stm.q_2_rotation(qlref,1)
            error_Rl     = Rlref.T@Rl - Rl.T@Rlref
            att_error_l  = 1/2*self.stm.vee_map(error_Rl)
            w_error_l    = np.reshape(self.xl[10:13,0] - Ref0_l[10:13,0],(3,1))
            track_e_l    = np.vstack((pl_error,vl_error,att_error_l,w_error_l))
            input_l      = np.reshape(track_e_l,(self.Dl_in,1))
            nn_l_output  = self.convert_load_nn(self.nn_load(input_l))
            Para_l       = self.SetPara_load(nn_l_output)
            self.get_logger().info(f"ctrl step={self.k_ctrl}, error_l={track_e_l.flatten()}")
            # self.get_logger().info(f"ctrl step={self.k_ctrl}, payload= Ql_k[0:3]={Para_l[0,0:3]}, Ql_k[6:9]={Para_l[0,6:9]}")
            # self.get_logger().info(f"ctrl step={self.k_ctrl}, payload= Ql_N[0:3]={Para_l[0,12:15]}, Rl_k={Para_l[0,2*self.nwsl:]}")
            Para_lh     = np.reshape(Para_l,self.npl) # Weight_l based on real-time state error
            
            xq_h        = np.zeros(self.nq*self.nxi*(self.horizon+1))
            for j in range(self.nq):
                # NOTE Using the xq_traj_temp to solve the LMPC
                xq_j    = self.xq_traj_temp[j]
                xq_jp   = np.reshape(xq_j,self.nxi*(self.horizon+1)) # row-by-row
                xq_h[j*self.nxi*(self.horizon+1):(j+1)*self.nxi*(self.horizon+1)]=xq_jp
            Jlh         = np.reshape(Jl,3)
            rgh         = np.reshape(rg,3)
            # self.get_logger().info(f"ctrl step={self.k_ctrl}, xl_fbh={xl_fbh.flatten()}, xq_h={xq_h.flatten()}")
            opt_sol_l   = self.DistMPC.MPCsolverPayload_acados(xl_fbh, xq_h,
                                                        ref_xl, ref_ul, Para_lh, Jlh, rgh)
            xl_opt      = np.array(opt_sol_l['xl_opt'])
            ul_opt      = np.array(opt_sol_l['ul_opt'])  
            sum_viol_xl = 0
            sum_viol_ul = 0
            for kl in range(len(self.ul_traj)):
                sum_viol_xl  += LA.norm(xl_opt[kl,:]-self.xl_traj[kl,:])
                sum_viol_ul  += LA.norm(ul_opt[kl,:]-self.ul_traj[kl,:])
            sum_viol_xl  += LA.norm(xl_opt[-1,:]-self.xl_traj[-1,:])
            viol_xl  = sum_viol_xl/len(xl_opt)
            viol_ul  = sum_viol_ul/len(ul_opt)
            max_viol_l = max(viol_xl,viol_ul)
            # initial_error = LA.norm(np.reshape(xl_opt[0,:],(self.nxl,1))-self.xl)
            # self.get_logger().info('iteration=',ke,'payload:','viol_xl=',format(viol_xl,'.5f'),'viol_ul=',format(viol_ul,'.5f'),'viol_x0l=',initial_error)
            # self.get_logger().info(f"ctrl step={self.k_ctrl}, iteration={self.ke}, viol_xl={viol_xl:.5f}, viol_ul={viol_ul:.5f}, max_viol_l={max_viol_l:.5f}")
            # NOTE Finish all MPCs & LMPC, Update the BROCASTED DATA
            self.xl_traj  = xl_opt
            self.ul_traj  = ul_opt
            self.xq_traj = self.xq_traj_temp
            self.uq_traj = self.uq_traj_temp
            lmpc_end = (TM.time() - lmpc_start) * 1000
            self.get_logger().info(f"ctrl step={self.k_ctrl}, ----lmpc time={lmpc_end:.3f}ms-----")

            if self.ke>1:
                # Received max_viol_i from all quadrotors and make comparison before this step
                self.max_viol = max(self.max_viol, max_viol_l)
            self.get_logger().info(f"ctrl step={self.k_ctrl}, iteration={self.ke}, max_violation={self.max_viol:.5f}")
            # update the iteration number
            self.ke += 1

            # 5. Mark the QMPCs to start again
            self.REC_TEMP = np.zeros((int(self.nq),1),dtype=bool)
            
            # Check the waiting time for each while iteration
            self.iter_start = TM.time()

            # NOTE 6. Check the termination condition, if satisfied, break and k_ctrl += 1, traj_time += dt_ctrl. Else, continue to solve the next while iteration.
            if self.max_viol < epsilon or self.ke > k_max:
                while_end = (TM.time() - self.while_start) * 1000
                self.get_logger().info(f"ctrl step={self.k_ctrl}, ----while loop time={while_end:.3f}ms-----")

                self.k_ctrl += 1
                # NOTE time_traj update, QNoded check and update NEW REAL-TIME CONTROL variable from the broadcasted uq_traj
                self.time_traj += self.dt_ctrl 
                # Update reference trajectory
                self.Ref_xq, self.Ref_uq, self.Ref_xl, self.Ref_ul, self.Ref0_xq, self.Ref0_l = self.Reference_for_MPC(self.time_traj, self.angle_t)

                self.while_flag = False

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
        # self.DistMPC.SetQuadrotorCostDyn()
        self.DistMPC.SetPayloadCostDyn()
        # self.DistMPC.SetConstraints_Qaudrotor()
        self.DistMPC.SetConstraints_Load()
        # self.DistMPC.MPCsolverQuadrotorInit_acados()
        self.DistMPC.MPCsolverPayloadInit_acados()

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
        # self.control_logic_callback_group = ReentrantCallbackGroup()
        self.control_logic_callback_group = MutuallyExclusiveCallbackGroup() 
        self.state_update_callback_group = ReentrantCallbackGroup()  
        self.return_callback_group = ReentrantCallbackGroup()
        self.stm_pre_callback_group = MutuallyExclusiveCallbackGroup() 

    def _declare_ros_parameters(self):
        """Declare parameters for the node."""
        self.declare_parameter('uav_para', [1.0, 0.02, 0.02, 0.04, 6.0, 0.2])
        self.declare_parameter('load_para', [7.0, 1.0])
        self.declare_parameter('cable_para', [1e9, 8e-6, 1e-2, 2.0]) # E=1 Gpa, A=7mm^2 (pi*1.5^2), c=10, L0=2, Nylon-HD
        self.declare_parameter('Jl', [0.7 * x for x in [2.0, 2.0, 2.5]])  
        self.declare_parameter('rg', [0.1, 0.1, -0.1])  
        self.declare_parameter('angle_t', np.pi / 9)

        self.declare_parameter('dt_ctrl', 5e-2)  
        self.declare_parameter('dt_broadcast', 2e-2)  
        self.dt_broadcast = self.get_parameter('dt_broadcast').value

        self.declare_parameter('altitude', 5.0)
        self.altitude = self.get_parameter('altitude').value
        self.declare_parameter('trajectory_type', 'fig8')
        self.trajectory_type = self.get_parameter('trajectory_type').value

    def _initialize_neural_network(self):
        """Initialize the neural network and load the model."""
        self.pmin, self.pmax = 0.01, 100 # lower and upper bounds of the weightings
        # XXX Registe the NeuralNet Class as model to load the nn_load.pt
        neuralnet_path = os.path.join(self.package_share_directory, "NeuralNet.py")
        spec = importlib.util.spec_from_file_location("NeuralNet", neuralnet_path)
        neuralnet_module = importlib.util.module_from_spec(spec)
        sys.modules["NeuralNet"] = neuralnet_module
        spec.loader.exec_module(neuralnet_module)
        # Load the trained neural network model of the payload
        self.nn_load = torch.load(os.path.join(self.package_share_directory, "trained data/trained_nn_load.pt"))

    def _initialize_MPC_states(self):
        """Initialize the state of the drone and payload."""
        # Initialize the payload's state
        self.pl = np.zeros((3, 1))  # payload position
        self.vl = np.zeros((3, 1))  # payload velocity
        self.ql = np.array([[1.0], [0.0], [0.0], [0.0]])  # payload quaternion
        self.wl = np.zeros((3, 1))  # payload angular velocity
        self.xl           = np.vstack((self.pl, self.vl, self.ql, self.wl))

        # HACK Broadcast data: MPC_TRAJ, time_traj, bool[num_drones], opt_system
        self.MPC_TRAJ = False
        self.time_traj = 0.0 # Generate the SAME Reference trajectory, related to the k_ctrl
        self.REC_TEMP = np.ones((int(self.uav_para[4]), 1), dtype=bool) # Mark the return of QMPC, init to True
        # self.opt_system, transfer to solve QMPC
        self.xq_traj, self.uq_traj = [],[] # (nq * [horizon+1, nxi]), (nq * [horizon, nui])
        self.xl_traj, self.ul_traj = np.zeros((self.horizon+1, self.nxl)), np.zeros((self.horizon, self.nul))
        # store the QMPC's return temp_traj
        self.xq_traj_temp, self.uq_traj_temp = [],[]

        # Generate the reference trajectory here, update when msg.time_traj > self.time_traj
        self.Ref_xq, self.Ref_uq, self.Ref_xl, self.Ref_ul, self.Ref0_xq, self.Ref0_l = self.Reference_for_MPC(self.time_traj, self.angle_t)
        self.T_end = self.stm.Tc # Trajectory duration
        self.payload_altitude = self.Ref0_l[2,0] # payload's altitude, initialized at time_traj=0
        self.while_start = 0.0 # Initialize start time of the while loop

        self.k_ctrl = 0 # Control step of AutoMultilift
        self.flag = False # Flag to initialize the trajectory using the reference trajectories
        self.while_flag = False # Flag to check the while loop of SyncNode NOTE
        self.temp_flag = False # Flag to initialize the temp_traj using the reference trajectories
        
    def _initialize_publishers(self):
        """Initialize the publishers for the node."""
        self.publisher_timestamp = self.create_publisher(
            Int64, 
            '/sync_time', 
            self.qos_profile
        )
        # Always publish the timestamp to sync the offboard control commands
        self.timer_timestamp = self.create_timer(
            self.dt_broadcast, 
            self.sync_timestamp,
            )  

        # MPC Broadcaster
        self.publisher_distributed_mpc = self.create_publisher(
            BroadcastMPC,
            '/broadcast_mpc', 
            self.qos_profile
        )

    def _initialize_subscribers(self):
        """Initialize the subscribers for the node."""
        # Subscribers to the payload's state (xl, vl, ql, wl) from IsaacSim ros2 node
        self.create_subscription(
            PoseStamped,
            '/payload_pose',
            self.pose_callback,
            self.qos_profile,
            callback_group=self.state_update_callback_group
        )
        self.create_subscription(
            TwistStamped,
            '/payload_twist',
            self.twist_callback,
            self.qos_profile,
            callback_group=self.state_update_callback_group
        )
        # Subscribers to the quadrotor's return temp_traj and max_viol_i
        for i in range(int(self.uav_para[4])):
            self.create_subscription(
                QuadReturnMPC,
                f'/quad_{i}/QMPC_temp',
                self.QMPC_temp_callback,
                self.qos_profile,
                callback_group=self.return_callback_group
            )

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

    # --- Timer Broadcast Publisher ---
    def sync_timestamp(self):
        # Generate a unified timestamp for all drones
        timestamp_us = int(Clock().now().nanoseconds / 1000)

        msg = Int64()
        msg.data = timestamp_us
        self.publisher_timestamp.publish(msg)
        # self.get_logger().info(f"Published timestamp: {timestamp_us}")
    
    def broadcast_distributed_mpc(self):
        # FIXME
        # Check if the data is ready to be broadcasted
        if not self.xq_traj or not self.uq_traj or self.xl_traj.size == 0 or self.ul_traj.size == 0:
            self.get_logger().warning("Broadcast data is incomplete. Skipping broadcast.")
            return
        
        # Broadcast the data to solve QMPCs
        # self.get_logger().info("Broadcasting data to QMPCs...")
        msg = BroadcastMPC()
        msg.mpc_traj = self.MPC_TRAJ
        msg.time_traj = self.time_traj
        msg.rec_temp = self.REC_TEMP.flatten().tolist()

        # if not all(self.REC_TEMP):
        msg.xq_traj = np.concatenate([xq.flatten() for xq in self.xq_traj]).tolist()
        msg.uq_traj = np.concatenate([uq.flatten() for uq in self.uq_traj]).tolist()
        msg.xl_traj = self.xl_traj.flatten().tolist()
        msg.ul_traj = self.ul_traj.flatten().tolist()
        self.publisher_distributed_mpc.publish(msg)

    # --- Subscriber Callback Functions ---
    def QMPC_temp_callback(self, msg: QuadReturnMPC):
        # Receive the temp result from ith QMPC
        quad_idx = msg.idx
        # if self.REC_TEMP[quad_idx] == True:
        #     self.get_logger().info(f"Quad {quad_idx} temp result already received, waiting for LMPC to update...")
        #     return
        
        xi_temp = np.array(msg.xi_temp, dtype=np.float32).reshape((self.horizon+1, self.nxi))
        ui_temp = np.array(msg.ui_temp, dtype=np.float32).reshape((self.horizon, self.nui))
        max_viol_i = msg.max_viol_i

        # Update the QMPC's traj_temp prepared for LMPC
        # NOTE xi_temp: [horizon+1, nxi], ui_temp: [horizon, nui]
        self.xq_traj_temp[quad_idx] = xi_temp
        self.uq_traj_temp[quad_idx] = ui_temp
        self.max_viol = max(self.max_viol, max_viol_i) # Update the maximum violation value, compare with the ith QMPC
        self.REC_TEMP[quad_idx] = True # Marked as received, ith QMPC wait for next iteration
        # self.get_logger().info(f"Quad {quad_idx} temp result received, REC_TEMP={self.REC_TEMP.flatten()}")

    def pose_callback(self, msg: PoseStamped):
        position = msg.pose.position
        orientation = msg.pose.orientation
        # update the payload's state
        self.pl = np.array([[position.x], [position.y], [position.z]])
        self.ql = np.array([[orientation.w], [orientation.x], [orientation.y], [orientation.z]])
        # update the payload's state in the system
        self.xl[0:3,0:1] = self.pl
        self.xl[6:10,0:1] = self.ql
    def twist_callback(self, msg: TwistStamped):
        linear = msg.twist.linear
        angular = msg.twist.angular
        # update the payload's state
        self.vl = np.array([[linear.x], [linear.y], [linear.z]])
        self.wl = np.array([[angular.x], [angular.y], [angular.z]])
        # update the payload's state in the system
        self.xl[3:6,0:1] = self.vl
        self.xl[10:13,0:1] = self.wl


def main(args=None):
    rclpy.init(args=args)
    node = SyncNode()
    executor = MultiThreadedExecutor()

    # executor.add_node(node)
    # try:
    #     node.get_logger().info('Beginning Central Node, shut down with CTRL-C')
    #     executor.spin()
    # except KeyboardInterrupt:
    #     node.get_logger().info('Keyboard interrupt, shutting down.\n')
    rclpy.spin(node, executor=executor)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

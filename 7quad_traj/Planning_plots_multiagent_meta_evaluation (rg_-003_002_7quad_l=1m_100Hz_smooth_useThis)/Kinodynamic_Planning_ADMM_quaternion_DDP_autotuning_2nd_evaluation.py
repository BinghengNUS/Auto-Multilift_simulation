from casadi import *
import numpy as np
from numpy import linalg as LA
import math
from scipy.spatial.transform import Rotation as Rot
from scipy import linalg as sLA
from scipy.linalg import null_space
import time as TM
from scipy.linalg import eigh
from scipy.sparse.linalg import eigsh
from scipy.sparse.linalg import ArpackNoConvergence


class MPC_Planner:
    def __init__(self, sysm_para, dt_ctrl, horizon):
        # Payload's parameters
        self.ml     = sysm_para[0] # the payload's mass
        self.rl     = sysm_para[1] # radius of the payload
        self.Jlcom  = np.diag(sysm_para[2:5])
        self.rg     = np.reshape(sysm_para[5:8],(3,1)) # position of the load CoM in {Bl}
        self.S_rg   = self.skew_sym_numpy(self.rg)
        self.nq     = sysm_para[8] # number of the quadrotors
        self.cl0    = sysm_para[9] # cable length 
        self.rq     = sysm_para[10] # quadrotor radius
        self.ro     = sysm_para[11] # obstacle radius
        self.mi     = sysm_para[12] # the quadrotor's mass
        self.alpha  = 2*np.pi/self.nq
        r0          = np.array([[self.rl,0,0]]).T  # 1st cable attachment point in {Bl}
        self.ra     = r0
        S_r0        = self.skew_sym_numpy(r0)
        I3          = np.identity(3) # 3-by-3 identity matrix
        self.Pt      = np.vstack((I3,S_r0))
        for i in range(int(self.nq)-1):
            ri      = np.array([[self.rl*(math.cos((i+1)*self.alpha)),self.rl*(math.sin((i+1)*self.alpha)),0]]).T 
            S_ri    = self.skew_sym_numpy(ri)
            Pi      = np.vstack((I3,S_ri))
            self.Pt = np.append(self.Pt,Pi,axis=1) # the tension mapping matrix: 6-by-3nq with a rank of 6
            self.ra = np.append(self.ra,ri,axis=1) # a matrix that stores the attachment points
        # Unit direction vector free of coordinate
        self.ex     = np.array([[1, 0, 0]]).T
        self.ey     = np.array([[0, 1, 0]]).T
        self.ez     = np.array([[0, 0, 1]]).T
        # Gravitational acceleration
        self.g      = 9.81      
        self.dt     = dt_ctrl
        # MPC's horizon
        self.N      = horizon
        # barrier parameter
        self.p_bar  = 1e-4 #1e-2 works, cannot be too small
      

    def skew_sym_numpy(self, v):
        v_cross = np.array([
            [0, -v[2, 0], v[1, 0]],
            [v[2, 0], 0, -v[0, 0]],
            [-v[1, 0], v[0, 0], 0]]
        )
        return v_cross
    
    def skew_sym(self, v): # skew-symmetric operator
        v_cross = vertcat(
            horzcat(0, -v[2,0], v[1,0]),
            horzcat(v[2,0], 0, -v[0,0]),
            horzcat(-v[1,0], v[0,0], 0)
        )
        return v_cross

    def SetStateVariables(self, xl, xi):
        self.xl    = xl
        self.xi    = xi
        self.nxl   = xl.numel()
        self.nxi   = xi.numel()
        self.scxc  = SX.sym('scxc',self.nxi*int(self.nq))
        self.xc    = SX.sym('xc',self.nxi*int(self.nq))
        self.scxC  = SX.sym('scxC',self.nxi*int(self.nq))
        self.scxl  = SX.sym('scxl',self.nxl)
        self.scxi  = SX.sym('scxi',self.nxi)
        self.scxL  = SX.sym('scxL',self.nxl) # Lagrangian multiplier of xl
        self.scxI  = SX.sym('scxI',self.nxi) # Lagrangian multiplier of xi
        self.xl_lb = self.nxl*[-1e19]
        self.xl_ub = self.nxl*[1e19]
        self.xi_lb = self.nxi*[-1e19]
        self.xi_ub = self.nxi*[1e19]
        self.scxi_lb = [-1e19,-1e19,-1e19, -1e19,-1e19,-1e19, 0.01,-1e19]
        self.scxi_ub = [1e19,1e19,1e19, 1e19,1e19,1e19, 20, 1e19]

    def SetCtrlVariables(self, ul, ui):
        self.ul    = ul
        self.ui    = ui
        self.nul   = ul.numel()
        self.nui   = ui.numel()
        self.scuc  = SX.sym('scuc',self.nui*int(self.nq))
        self.uc    = SX.sym('uc',self.nui*int(self.nq))
        self.scuC  = SX.sym('scuC',self.nui*int(self.nq))
        self.scul  = SX.sym('scul',self.nul)
        self.scui  = SX.sym('scui',self.nui)
        self.scuL  = SX.sym('scuL',self.nul) # Lagrangian multiplier of ul
        self.scuI  = SX.sym('scuI',self.nui) # Lagrangian multiplier of ui
        self.ul_lb = self.nul*[-1e19]
        self.ul_ub = self.nul*[1e19]
        self.ui_lb = self.nui*[-1e19]
        self.ui_ub = self.nui*[1e19]

    def SetDyns(self, model_l, model_i):
        self.model_l = self.xl + self.dt*model_l # 4th-order Runge-Kutta discrete-time load dynamics model
        self.model_i = self.xi + self.dt*model_i # 4th-order Runge-Kutta discrete-time cable dynamics model
        self.model_l_fn = Function('mdynl',[self.xl, self.ul],[self.model_l],['xl0','ul0'],['mdynlf'])
        self.model_i_fn = Function('mdyni',[self.xi, self.ui],[self.model_i],['xi0','ui0'],['mdynif'])

    def SetWeightPara(self):
        # self.nwsl    = self.nxl
        self.para_l  = SX.sym('paral',1,2*self.nxl+self.nul+1) # including the ADMM penalty parameter
        self.npl     = self.para_l.numel()
        self.para_i  = SX.sym('parai',1,2*self.nxi+self.nui+1) # including the ADMM penalty parameter
        self.npi     = self.para_i.numel()
        self.P_auto  = horzcat(self.para_l,self.para_i)
        self.n_Pauto = self.P_auto.numel()


    def q_2_rotation(self, q): # from body frame to inertial frame
        # no normalization to avoid singularity in optimization
        q0, q1, q2, q3 = q[0], q[1], q[2], q[3] # q0 denotes a scalar while q1, q2, and q3 represent rotational axes x, y, and z, respectively
        R = vertcat(
        horzcat( 2 * (q0 ** 2 + q1 ** 2) - 1, 2 * q1 * q2 - 2 * q0 * q3, 2 * q0 * q2 + 2 * q1 * q3),
        horzcat(2 * q0 * q3 + 2 * q1 * q2, 2 * (q0 ** 2 + q2 ** 2) - 1, 2 * q2 * q3 - 2 * q0 * q1),
        horzcat(2 * q1 * q3 - 2 * q0 * q2, 2 * q0 * q1 + 2 * q2 * q3, 2 * (q0 ** 2 + q3 ** 2) - 1)
        )
        return R
    
    
    def vee_map(self, v):
        vect = vertcat(v[2, 1], v[0, 2], v[1, 0])
        return vect

    def SetPayloadCostDyn(self):
        self.ref_xl  = SX.sym('refxl',self.nxl,1)
        self.ref_ul  = SX.sym('reful',self.nul,1) 
        track_error_l = self.xl - self.ref_xl
        ctrl_error_l  = self.ul - self.ref_ul
        self.Ql_k     = diag(self.para_l[0,0:self.nxl])
        self.Ql_N     = diag(self.para_l[0,self.nxl:2*self.nxl])
        self.Rl_k     = diag(self.para_l[0,2*self.nxl:2*self.nxl+self.nul])
        self.p        = self.para_l[-1]
        # path cost
        self.scql          = self.scxl[6:10]
        self.h_norm_scq    = 1/(2*self.p_bar)*(norm_2(self.scql)-1)**2
        self.resid_xl = self.xl - self.scxl + self.scxL/self.p
        self.resid_ul = self.ul - self.scul + self.scuL/self.p
        self.Jl_k     = 1/2 * (track_error_l.T@self.Ql_k@track_error_l + ctrl_error_l.T@self.Rl_k@ctrl_error_l) + self.p/2*self.resid_xl.T@self.resid_xl + self.p/2*self.resid_ul.T@self.resid_ul
        self.Jl_kfn   = Function('Jl_k',[self.xl, self.ul, self.scxl, self.scxL, self.scul, self.scuL, self.ref_xl, self.ref_ul, self.para_l],[self.Jl_k],['xl0', 'ul0', 'scxl0', 'scxL0', 'scul0', 'scuL0', 'refxl0', 'reful0', 'paral0'],['Jl_kf'])
        # terminal cost
        self.Jl_N     = 1/2 * track_error_l.T@self.Ql_N@track_error_l + self.p/2*self.resid_xl.T@self.resid_xl
        self.Jl_Nfn   = Function('Jl_N',[self.xl, self.ref_xl, self.scxl, self.scxL, self.para_l],[self.Jl_N],['xl0', 'refxl0', 'scxl0', 'scxL0', 'paral0'],['Jl_Nf'])
        # path cost of ADMM subproblem2
        self.Jl_P2_k  = self.p/2*self.resid_xl.T@self.resid_xl + self.p/2*self.resid_ul.T@self.resid_ul + self.h_norm_scq
        self.Jl_P2_k_fn = Function('Jl_P2_k',[self.xl, self.ul, self.scxl, self.scxL, self.scul, self.scuL, self.p],[self.Jl_P2_k],['xl0', 'ul0', 'scxl0', 'scxL0', 'scul0', 'scuL0', 'p0'],['Jl_P2_kf'])
        # terminal cost of ADMM subproblem2
        self.Jl_P2_N  = self.p/2*self.resid_xl.T@self.resid_xl + self.h_norm_scq
        self.Jl_P2_N_fn = Function('Jl_P2_N',[self.xl,self.scxl, self.scxL,self.p],[self.Jl_P2_N],['xl0','scxl0','scxL0','p0'],['Jl_P2_Nf'])


    def SetCableCostDyn(self):
        self.ref_xi   = SX.sym('refxi',self.nxi,1)
        self.ref_ui   = SX.sym('refui',self.nui,1)
        track_error_i = self.xi - self.ref_xi
        ctrl_error_i  = self.ui - self.ref_ui
        self.Qi_k     = diag(self.para_i[0,0:self.nxi])
        self.Qi_N     = diag(self.para_i[0,self.nxi:2*self.nxi])
        self.Ri_k     = diag(self.para_i[0,2*self.nxi:2*self.nxi+self.nui])
        self.pi       = self.para_i[-1]
        # path cost
        scdi          = self.scxi[0:3]
        h_norm_sc     = 1/(2*self.p_bar)*(norm_2(scdi)-1)**2
        self.resid_xi = self.xi - self.scxi + self.scxI/self.pi
        self.resid_ui = self.ui - self.scui + self.scuI/self.pi   
        self.Ji_k     = 1/2 * (track_error_i.T@self.Qi_k@track_error_i + ctrl_error_i.T@self.Ri_k@ctrl_error_i) + self.pi/2*self.resid_xi.T@self.resid_xi + self.pi/2*self.resid_ui.T@self.resid_ui 
        self.Ji_k_fn  = Function('Ji_k',[self.xi, self.ui, self.scxi, self.scxI, self.scui, self.scuI, self.ref_xi, self.ref_ui, self.para_i],[self.Ji_k],['xi0', 'ui0', 'scxi0', 'scxI0', 'scui0', 'scuI0', 'refxi0', 'refui0', 'parai0'],['Ji_kf'])
        # terminal cost
        self.Ji_N     = 1/2 * track_error_i.T@self.Qi_N@track_error_i + self.pi/2*self.resid_xi.T@self.resid_xi 
        self.Ji_N_fn  = Function('Ji_N',[self.xi, self.ref_xi, self.scxi, self.scxI, self.para_i],[self.Ji_N],['xi0', 'refxi0', 'scxi0', 'scxI0', 'parai0'],['Ji_Nf'])
        # path cost of ADMM subproblem2
        self.Ji_P2_k  = self.pi/2*self.resid_xi.T@self.resid_xi + self.pi/2*self.resid_ui.T@self.resid_ui + h_norm_sc
        self.Ji_P2_k_fn = Function('Ji_P2_k',[self.xi, self.scxi, self.scxI, self.ui, self.scui, self.scuI, self.pi],[self.Ji_P2_k],['xi0', 'scxi0', 'scxI0', 'ui0', 'scui0', 'scuI0', 'pi0'],['Ji_P2_kf'])
        # terminal cost of ADMM subproblem2
        self.Ji_P2_N  = self.pi/2*self.resid_xi.T@self.resid_xi + h_norm_sc
        self.Ji_P2_N_fn = Function('Ji_P2_N',[self.xi, self.scxi, self.scxI, self.pi],[self.Ji_P2_N],['xi0','scxi0','scxI0','pi0'],['Jl_P2_Nf'])

    
    def SetConstriants(self, pob1, pob2):
        # dynamic coupling constraint at each step k
        pl_k     = self.scxl[0:3]
        ql_k     = self.scxl[6:10]
        wl_k     = self.scxl[10:self.nxl]
        Fl_k     = self.scul[0:3] #{Bl}
        Ml_k     = self.scul[3:6]
        tc_k     = SX.sym('tc_k',3*int(self.nq),1)
        Rl_k     = self.q_2_rotation(ql_k)
        k        = 0
        self.fi    = [] # list that stores all the quadrotor thruster limit constraints
        self.Gi1   = [] # list that stores the obstacle-avoidance constraints of all the quadrotors for the 1st obstacle
        self.Gi2   = [] # list that stores the obstacle-avoidance constraints of all the quadrotors for the 2nd obstacle
        self.Gij   = [] # list that stores all the safe inter-robot inequality constraints
        self.sumfi = 0 # barrier function of the quadrotor thrust limit
        self.gco   = 0 # barrier function of the safe collision-avoidance constraints on quadrotors' planar positions
        self.gij   = 0 # barrier functions of the safe inter-robot constraints on quadrotors' planar positions
        self.Tcon  = 0 # barrier functions of the tension magnitude constraints
        for i in range(int(self.nq)):
            xi_k   = self.scxc[i*self.nxi:(i+1)*self.nxi]
            ui_k   = self.scuc[i*self.nui:(i+1)*self.nui]
            di_k   = xi_k[0:3]
            wi_k   = xi_k[3:6]
            ti_k   = xi_k[6]
            gtimin = ti_k - self.scxi_lb[-2]
            gtimax = self.scxi_ub[-2] - ti_k
            self.Tcon += -self.p_bar * log(gtimin)
            self.Tcon += -self.p_bar * log(gtimax)
            dwi_k  = ui_k[0:3] # cable angular acceleration
            ri   = np.reshape(self.ra[:,i],(3,1))
            pi_k = pl_k + Rl_k@ri + self.cl0*di_k # ith quadrotor's position in {I}
            go1  = norm_2(pi_k[0:2]-pob1) - 1*(self.rq + self.ro) # safe constriant between the obstacle 1 and the ith quadrotor, which should be positive
            go1_fn = Function('go1'+str(i),[self.scxl,self.scxc],[go1],['scxl0','scxc0'],['go1f'+str(i)])
            self.gco += -self.p_bar * log(go1)
            self.Gi1 += [go1_fn]
            go2  = norm_2(pi_k[0:2]-pob2) - 1*(self.rq + self.ro) # safe constriant between the obstacle 2 and the ith quadrotor, which should be positive
            go2_fn = Function('go2'+str(i),[self.scxl,self.scxc],[go2],['scxl0','scxc0'],['go2f'+str(i)])
            self.gco += -self.p_bar * log(go2)
            self.Gi2 += [go2_fn]
            # We use simplified load dynamics to derive the dynamic coupling constraints, provided that the load's angular velocity and acceleration are stabilized at zero.
            S_wl_k  = self.skew_sym(wl_k)
            S_wi_k  = self.skew_sym(wi_k)
            S_dwi_k = self.skew_sym(dwi_k)
            al_k    = -self.g*self.ez + Rl_k@Fl_k/self.ml
            awl_k   = LA.inv(self.Jlcom)@(Ml_k-S_wl_k@(self.Jlcom@wl_k))
            S_awl_k = self.skew_sym(awl_k)
            fi_k    = self.mi*(al_k+Rl_k@(S_wl_k@S_wl_k+S_awl_k)@ri+self.cl0*(S_dwi_k@di_k+S_wi_k@(S_wi_k@di_k))+self.g*self.ez) + di_k*ti_k
            norm_fi = norm_2(fi_k)
            self.sumfi += -self.p_bar * log(50-norm_fi)
            norm_fi_fn = Function('norm_f'+str(i),[self.scxl,self.scul,self.scxc,self.scuc],[norm_fi],['scxl0','scul0','scxc0','scuc0'],['norm_ff'+str(i)])
            self.fi += [norm_fi_fn]
            for j in range(i+1,int(self.nq)): # safe inter-robot separation constraints
                pi_lb  = Rl_k@ri + self.cl0*di_k
                xj_k   = self.scxc[j*self.nxi:(j+1)*self.nxi]
                dj_k   = xj_k[0:3]
                rj     = np.reshape(self.ra[:,j],(3,1))
                pj_lb  = Rl_k@rj + self.cl0*dj_k
                gij    = norm_2(pi_lb[0:2]-pj_lb[0:2]) - 4.25*self.rq
                self.gij += -self.p_bar * log(gij)
                gij_fn = Function('g'+str(k),[self.scxl,self.scxc],[gij],['scxl0','scxc0'],['gf'+str(k)])
                self.Gij += [gij_fn]
                k     += 1
            ti_kb = Rl_k.T@di_k*ti_k # cable tension vector in {B}
            tc_k[i*3:(i+1)*3] = ti_kb
        # control consensus constraint that maps tension forces to the load control wrench
        wrench   = vertcat(Fl_k,Ml_k) #
        W_cons   = self.Pt@tc_k - wrench
        self.h_wcons  = 1/(2*(self.p_bar))*W_cons.T@W_cons
        self.W_cons_fn = Function('W_cons',[self.scxl,self.scul,self.scxc],[W_cons],['scxl0','scul0','scxc0'],['W_consf'])


    def SetADMMSubP2_SoftCost_k(self):
        # at each step k
        self.J_2_soft_k    = self.Jl_P2_k + self.sumfi + self.gco + self.gij + self.Tcon
        for i in range(int(self.nq)):
            xi      = self.xc[i*self.nxi:(i+1)*self.nxi]   # cable primal state
            scxi    = self.scxc[i*self.nxi:(i+1)*self.nxi] # safe copy state of each cable
            scdi    = scxi[0:3]
            scxI    = self.scxC[i*self.nxi:(i+1)*self.nxi] # Lagrangian multiplier
            ui      = self.uc[i*self.nui:(i+1)*self.nui]   # cable primal control
            scui    = self.scuc[i*self.nui:(i+1)*self.nui] # safe copy control of each cable
            scuI    = self.scuC[i*self.nui:(i+1)*self.nui] # Lagrangian multiplier
            resid_x = xi - scxi + scxI/self.pi
            resid_u = ui - scui + scuI/self.pi
            self.J_2_soft_k    += self.pi/2*resid_x.T@resid_x + self.pi/2*resid_u.T@resid_u + 1/(2*self.p_bar)*(norm_2(scdi)-1)**2
    

    def SetADMMSubP2_SoftCost_N(self):
        # at the terminal step N
        self.J_2_soft_N    = self.Jl_P2_N + self.gco + self.gij + self.Tcon
        for i in range(int(self.nq)):
            xi      = self.xc[i*self.nxi:(i+1)*self.nxi]   # cable primal state
            scxi    = self.scxc[i*self.nxi:(i+1)*self.nxi] # safe copy state of each cable
            scdi    = scxi[0:3]
            scxI    = self.scxC[i*self.nxi:(i+1)*self.nxi] # Lagrangian multiplier
            resid_x = xi - scxi + scxI/self.pi
            self.J_2_soft_N    += self.pi/2*resid_x.T@resid_x + 1/(2*self.p_bar)*(norm_2(scdi)-1)**2


    def Load_derivatives_DDP_ADMM(self):
        # alpha = 1
        self.Vxl      = SX.sym('Vxl',self.nxl)
        self.Vxlxl    = SX.sym('Vxlxl',self.nxl,self.nxl)
        # gradients of the system dynamics, the cost function, and the Q value function
        self.Fxl      = jacobian(self.model_l,self.xl)
        self.Fxl_fn   = Function('Fxl',[self.xl,self.ul],[self.Fxl],['xl0','ul0'],['Fxl_f'])
        self.Ful      = jacobian(self.model_l,self.ul)
        self.Ful_fn   = Function('Ful',[self.xl,self.ul],[self.Ful],['xl0','ul0'],['Ful_f'])
        self.lxl      = jacobian(self.Jl_k,self.xl)
        self.lxlN     = jacobian(self.Jl_N,self.xl)
        self.lxlN_fn  = Function('lxlN',[self.xl,self.ref_xl,self.scxl,self.scxL,self.para_l],[self.lxlN],['xl0', 'refxl0', 'scxl0', 'scxL0', 'paral0'],['lxlN_f'])
        self.lul      = jacobian(self.Jl_k,self.ul)
        self.Qxl      = self.lxl.T + self.Fxl.T@self.Vxl
        self.Qxl_fn   = Function('Qxl',[self.xl,self.ul,self.Vxl,self.ref_xl,self.ref_ul,self.scxl,self.scxL,self.scul,self.scuL,self.para_l],[self.Qxl],['xl0','ul0','Vxl0','refxl0','reful0','scxl0','scxL0','scul0','scuL0','paral0'],['Qxl_f'])
        self.Qul      = self.lul.T + self.Ful.T@self.Vxl
        self.Qul_fn   = Function('Qul',[self.xl,self.ul,self.Vxl,self.ref_xl,self.ref_ul,self.scxl,self.scxL,self.scul,self.scuL,self.para_l],[self.Qul],['xl0','ul0','Vxl0','refxl0','reful0','scxl0','scxL0','scul0','scuL0','paral0'],['Qul_f'])
        # hessians of the system dynamics, the cost function, and the Q value function
        self.FxlVxl   = self.Fxl.T@self.Vxl
        self.dFxlVxldxl= jacobian(self.FxlVxl,self.xl) # the hessian of the system dynamics may cause heavy computational burden
        self.dFxlVxldul= jacobian(self.FxlVxl,self.ul)
        self.FulVxl   = self.Ful.T@self.Vxl
        self.dFulVxldul= jacobian(self.FulVxl,self.ul)
        self.lxlxl    = jacobian(self.lxl,self.xl)
        self.lxlxlN   = jacobian(self.lxlN,self.xl)
        self.lxlxlN_fn= Function('lxlxlN',[self.para_l],[self.lxlxlN],['paral0'],['lxlxlN_f'])
        self.lxlul    = jacobian(self.lxl,self.ul)
        self.lulul    = jacobian(self.lul,self.ul)
        self.Qxlxl_bar    = self.lxlxl #+ alpha*self.dFxlVxldxl  # removing this model hessian can enhance the DDP stability for a larger time step!!!! The removal can also accelerate the DDP computation significantly!
        self.Qxlxl_bar_fn = Function('Qxlxl_bar',[self.xl,self.ul,self.Vxl,self.ref_xl,self.ref_ul,self.scxl,self.scxL,self.scul,self.scuL,self.para_l],[self.Qxlxl_bar],['xl0','ul0','Vxl0','refxl0','reful0','scxl0','scxL0','scul0','scuL0','paral0'],['Qxlxl_bar_f'])
        self.Qxlxl_hat    = self.Fxl.T@self.Vxlxl@self.Fxl
        self.Qxlxl_hat_fn = Function('Qxlxl_hat',[self.xl,self.ul,self.Vxlxl],[self.Qxlxl_hat],['xl0','ul0','Vxlxl0'],['Qxlxl_hat_f'])
        self.Qxlul_bar    = self.lxlul #+ alpha*self.dFxlVxldul  # including the model hessian entails a very small time step size (e.g., 0.01s)
        self.Qxlul_bar_fn = Function('Qxlul_bar',[self.xl,self.ul,self.Vxl,self.ref_xl,self.ref_ul,self.scxl,self.scxL,self.scul,self.scuL,self.para_l],[self.Qxlul_bar],['xl0','ul0','Vxl0','refxl0','reful0','scxl0','scxL0','scul0','scuL0','paral0'],['Qxlul_bar_f'])
        self.Qxlul_hat    = self.Fxl.T@self.Vxlxl@self.Ful
        self.Qxlul_hat_fn = Function('Qxlul_hat',[self.xl,self.ul,self.Vxlxl],[self.Qxlul_hat],['xl0','ul0','Vxlxl0'],['Qxlul_hat_f'])
        self.Qulul_bar    = self.lulul #+ alpha*self.dFulVxldul
        self.Qulul_bar_fn = Function('Qulul_bar',[self.xl,self.ul,self.Vxl,self.ref_xl,self.ref_ul,self.scxl,self.scxL,self.scul,self.scuL,self.para_l],[self.Qulul_bar],['xl0','ul0','Vxl0','refxl0','reful0','scxl0','scxL0','scul0','scuL0','paral0'],['Qulul_bar_f'])
        self.Qulul_hat    = self.Ful.T@self.Vxlxl@self.Ful
        self.Qulul_hat_fn = Function('Qulul_hat',[self.xl,self.ul,self.Vxlxl],[self.Qulul_hat],['xl0','ul0','Vxlxl0'],['Qulul_hat_f'])
        # hessians w.r.t. the hyperparameters
        self.lxlp     = jacobian(self.lxl,self.P_auto)
        self.lxlp_fn  = Function('lxlp',[self.xl,self.ul,self.ref_xl,self.ref_ul,self.scxl,self.scxL,self.scul,self.scuL,self.para_l],[self.lxlp],['xl0','ul0','refxl0','reful0','scxl0','scxL0','scul0','scuL0','paral0'],['lxlp_f'])
        self.lulp     = jacobian(self.lul,self.P_auto)
        self.lulp_fn  = Function('lulp',[self.xl,self.ul,self.ref_xl,self.ref_ul,self.scxl,self.scxL,self.scul,self.scuL,self.para_l],[self.lulp],['xl0','ul0','refxl0','reful0','scxl0','scxL0','scul0','scuL0','paral0'],['lulp_f'])
        self.lxlNp    = jacobian(self.lxlN,self.P_auto)
        self.lxlNp_fn = Function('lxlNp',[self.xl,self.ref_xl,self.scxl,self.scxL,self.para_l],[self.lxlNp],['xl0', 'refxl0', 'scxl0', 'scxL0', 'paral0'],['lxlNp_f'])


    
    def Cable_derivatives_DDP_ADMM(self):
        alpha = 1
        self.Vxi      = SX.sym('Vxi',self.nxi)
        self.Vxixi    = SX.sym('Vxixi',self.nxi,self.nxi)
        # gradients of the system dynamics, the cost function, and the Q value function
        self.Fxi      = jacobian(self.model_i,self.xi)
        self.Fxi_fn   = Function('Fxi',[self.xi,self.ui],[self.Fxi],['xi0','ui0'],['Fxi_f'])
        self.Fui      = jacobian(self.model_i,self.ui)
        self.Fui_fn   = Function('Fui',[self.xi,self.ui],[self.Fui],['xi0','ui0'],['Fui_f'])
        self.lxi      = jacobian(self.Ji_k,self.xi)
        self.lxiN     = jacobian(self.Ji_N,self.xi)
        self.lxiN_fn  = Function('lxiN',[self.xi,self.ref_xi,self.scxi,self.scxI,self.para_i],[self.lxiN],['xi0', 'refxi0', 'scxi0', 'scxI0', 'parai0'],['lxiN_f'])
        self.lui      = jacobian(self.Ji_k,self.ui)
        self.Qxi      = self.lxi.T + self.Fxi.T@self.Vxi
        self.Qxi_fn   = Function('Qxi',[self.xi,self.ui,self.Vxi,self.ref_xi,self.ref_ui,self.scxi,self.scxI,self.scui,self.scuI,self.para_i],[self.Qxi],['xi0','ui0','Vxi0','refxi0','refui0','scxi0','scxI0','scui0','scuI0','parai0'],['Qxi_f'])
        self.Qui      = self.lui.T + self.Fui.T@self.Vxi
        self.Qui_fn   = Function('Qui',[self.xi,self.ui,self.Vxi,self.ref_xi,self.ref_ui,self.scxi,self.scxI,self.scui,self.scuI,self.para_i],[self.Qui],['xi0','ui0','Vxi0','refxi0','refui0','scxi0','scxI0','scui0','scuI0','parai0'],['Qui_f'])
        # hessians of the system dynamics, the cost function, and the Q value function
        self.FxiVxi   = self.Fxi.T@self.Vxi
        self.dFxiVxidxi= jacobian(self.FxiVxi,self.xi) # the hessian of the system dynamics may cause heavy computational burden
        self.dFxiVxidui= jacobian(self.FxiVxi,self.ui)
        self.FuiVxi   = self.Fui.T@self.Vxi
        self.dFuiVxidui= jacobian(self.FuiVxi,self.ui)
        self.lxixi    = jacobian(self.lxi,self.xi) # already includes pi
        self.lxixiN   = jacobian(self.lxiN,self.xi)
        self.lxixiN_fn= Function('lxixiN',[self.para_i],[self.lxixiN],['parai0'],['lxixiN_f'])
        self.lxiui    = jacobian(self.lxi,self.ui)
        self.luiui    = jacobian(self.lui,self.ui)
        self.Qxixi_bar    = self.lxixi #+ alpha*self.dFxiVxidxi  # removing this model hessian can enhance the DDP stability for a larger time step!!!! The removal can also accelerate the DDP computation significantly!
        self.Qxixi_bar_fn = Function('Qxixi_bar',[self.xi,self.ui,self.Vxi,self.ref_xi,self.ref_ui,self.scxi,self.scxI,self.scui,self.scuI,self.para_i],[self.Qxixi_bar],['xi0','ui0','Vxi0','refxi0','refui0','scxi0','scxI0','scui0','scuI0','parai0'],['Qxixi_bar_f'])
        self.Qxixi_hat    = self.Fxi.T@self.Vxixi@self.Fxi
        self.Qxixi_hat_fn = Function('Qxixi_hat',[self.xi,self.ui,self.Vxixi],[self.Qxixi_hat],['xi0','ui0','Vxixi0'],['Qxixi_hat_f'])
        self.Qxiui_bar    = self.lxiui #+ alpha*self.dFxiVxidui  # including the model hessian entails a very small time step size (e.g., 0.01s)
        self.Qxiui_bar_fn = Function('Qxiui_bar',[self.xi,self.ui,self.Vxi,self.ref_xi,self.ref_ui,self.scxi,self.scxI,self.scui,self.scuI,self.para_i],[self.Qxiui_bar],['xi0','ui0','Vxi0','refxi0','refui0','scxi0','scxI0','scui0','scuI0','parai0'],['Qxiui_bar_f'])
        self.Qxiui_hat    = self.Fxi.T@self.Vxixi@self.Fui
        self.Qxiui_hat_fn = Function('Qxiui_hat',[self.xi,self.ui,self.Vxixi],[self.Qxiui_hat],['xi0','ui0','Vxixi0'],['Qxiui_hat_f'])
        self.Quiui_bar    = self.luiui #+ alpha*self.dFuiVxidui
        self.Quiui_bar_fn = Function('Quiui_bar',[self.xi,self.ui,self.Vxi,self.ref_xi,self.ref_ui,self.scxi,self.scxI,self.scui,self.scuI,self.para_i],[self.Quiui_bar],['xi0','ui0','Vxi0','refxi0','refui0','scxi0','scxI0','scui0','scuI0','parai0'],['Quiui_bar_f'])
        self.Quiui_hat    = self.Fui.T@self.Vxixi@self.Fui
        self.Quiui_hat_fn = Function('Quiui_hat',[self.xi,self.ui,self.Vxixi],[self.Quiui_hat],['xi0','ui0','Vxixi0'],['Quiui_hat_f'])
        # hessians w.r.t. the hyperparameters
        self.lxip     = jacobian(self.lxi,self.P_auto)
        self.lxip_fn  = Function('lxip',[self.xi,self.ui,self.ref_xi,self.ref_ui,self.scxi,self.scxI,self.scui,self.scuI,self.para_i],[self.lxip],['xi0','ui0','refxi0','refui0','scxi0','scxI0','scui0','scuI0','parai0'],['lxip_f'])
        self.luip     = jacobian(self.lui,self.P_auto)
        self.luip_fn  = Function('luip',[self.xi,self.ui,self.ref_xi,self.ref_ui,self.scxi,self.scxI,self.scui,self.scuI,self.para_i],[self.luip],['xi0','ui0','refxi0','refui0','scxi0','scxI0','scui0','scuI0','parai0'],['luip_f'])
        self.lxiNp    = jacobian(self.lxiN,self.P_auto)
        self.lxiNp_fn = Function('lxiNp',[self.xi,self.ref_xi,self.scxi,self.scxI,self.para_i],[self.lxiNp],['xi0', 'refxi0', 'scxi0', 'scxI0', 'parai0'],['lxiNp_f'])


    
    
    def Get_AuxSys_DDP_Load(self,opt_sol,Ref_xl,Ref_ul,scxl,scul,scxL,scuL,weight1):
        xl_opt   = opt_sol['xl_traj']
        ul_opt   = opt_sol['ul_traj']
        LxlNp    = self.lxlNp_fn(xl0=xl_opt[-1,:],refxl0=Ref_xl[self.N*self.nxl:(self.N+1)*self.nxl],scxl0=scxl[self.N*self.nxl:(self.N+1)*self.nxl],scxL0=scxL[self.N*self.nxl:(self.N+1)*self.nxl],paral0=weight1)['lxlNp_f'].full()
        LxlxlN   = self.lxlxlN_fn(paral0=weight1)['lxlxlN_f'].full()
        Lxlp     = self.N*[np.zeros((self.nxl,self.n_Pauto))]
        Lulp     = self.N*[np.zeros((self.nul,self.n_Pauto))]
        for k in range(self.N):
            Lxlp[k] = self.lxlp_fn(xl0=xl_opt[k,:],ul0=ul_opt[k,:],refxl0=Ref_xl[k*self.nxl:(k+1)*self.nxl],reful0=Ref_ul[k*self.nul:(k+1)*self.nul],
                                scxl0=scxl[k,:],scxL0=scxL[k,:],scul0=scul[k,:],scuL0=scuL[k,:],paral0=weight1)['lxlp_f'].full()
            Lulp[k] = self.lulp_fn(xl0=xl_opt[k,:],ul0=ul_opt[k,:],refxl0=Ref_xl[k*self.nxl:(k+1)*self.nxl],reful0=Ref_ul[k*self.nul:(k+1)*self.nul],
                                scxl0=scxl[k,:],scxL0=scxL[k,:],scul0=scul[k,:],scuL0=scuL[k,:],paral0=weight1)['lulp_f'].full()
        
        auxSysl = { "HxxN":LxlxlN,
                    "HxNp":LxlNp,
                    "Hxp":Lxlp,
                    "Hup":Lulp
                    }
        
        return auxSysl
    

    def Get_AuxSys_DDP_Cable(self,opt_sol,Ref_xi,Ref_ui,scxi,scui,scxI,scuI,weight2):
        xi_opt   = opt_sol['xi_traj']
        ui_opt   = opt_sol['ui_traj']
        LxiNp    = self.lxiNp_fn(xi0=xi_opt[-1,:],refxi0=Ref_xi[self.N*self.nxi:(self.N+1)*self.nxi],scxi0=scxi[self.N*self.nxi:(self.N+1)*self.nxi],scxI0=scxI[self.N*self.nxi:(self.N+1)*self.nxi],parai0=weight2)['lxiNp_f'].full()
        LxixiN   = self.lxixiN_fn(parai0=weight2)['lxixiN_f'].full()
        Lxip     = self.N*[np.zeros((self.nxi,self.n_Pauto))]
        Luip     = self.N*[np.zeros((self.nui,self.n_Pauto))]
        for k in range(self.N):
            Lxip[k] = self.lxip_fn(xi0=xi_opt[k,:],ui0=ui_opt[k,:],refxi0=Ref_xi[k*self.nxi:(k+1)*self.nxi],refui0=Ref_ui[k*self.nui:(k+1)*self.nui],
                                scxi0=scxi[k,:],scxI0=scxI[k,:],scui0=scui[k,:],scuI0=scuI[k,:],parai0=weight2)['lxip_f'].full()
            Luip[k] = self.luip_fn(xi0=xi_opt[k,:],ui0=ui_opt[k,:],refxi0=Ref_xi[k*self.nxi:(k+1)*self.nxi],refui0=Ref_ui[k*self.nui:(k+1)*self.nui],
                                scxi0=scxi[k,:],scxI0=scxI[k,:],scui0=scui[k,:],scuI0=scuI[k,:],parai0=weight2)['luip_f'].full()
        
        auxSysi = { "HxxN":LxixiN,
                    "HxNp":LxiNp,
                    "Hxp":Lxip,
                    "Hup":Luip
                    }
        
        return auxSysi
    

   
    def DDP_Load_ADMM_Subp1(self,xl_0,Ref_xl,Ref_ul,weight1,scxl,scul,scxL,scuL,max_iter,e_tol):
        reg        = 1e-6
        alpha_init = 1 # Initial alpha for line search
        alpha_min = 1e-2  # Minimum allowable alpha
        alpha_factor = 0.5 # 
        max_line_search_steps = 10 # 5 for training
        iteration = 1
        ratio = 10
        X_nominal = np.zeros((self.nxl,self.N+1))
        U_nominal = np.zeros((self.nul,self.N))
        X_nominal[:,0:1] = np.reshape(xl_0,(self.nxl,1))
        
        # Initial trajectory and initial cost 
        cost_prev = 0
        for k in range(self.N):
            u_k    = np.reshape(Ref_ul[k*self.nul:(k+1)*self.nul],(self.nul,1))
            X_nominal[:,k+1:k+2] = self.model_l_fn(xl0=X_nominal[:,k],ul0=u_k)['mdynlf'].full()
            U_nominal[:,k:k+1]   = u_k
            cost_prev     += self.Jl_kfn(xl0=X_nominal[:,k],ul0=u_k,scxl0=scxl[k*self.nxl:(k+1)*self.nxl],scxL0=scxL[k*self.nxl:(k+1)*self.nxl],
                                        scul0=scul[k*self.nul:(k+1)*self.nul],scuL0=scuL[k*self.nul:(k+1)*self.nul],
                                        refxl0=Ref_xl[k*self.nxl:(k+1)*self.nxl],reful0=Ref_ul[k*self.nul:(k+1)*self.nul],paral0=weight1)['Jl_kf'].full()
        cost_prev += self.Jl_Nfn(xl0=X_nominal[:,-1],refxl0=Ref_xl[self.N*self.nxl:(self.N+1)*self.nxl],scxl0=scxl[self.N*self.nxl:(self.N+1)*self.nxl],
                                 scxL0=scxL[self.N*self.nxl:(self.N+1)*self.nxl],paral0=weight1)['Jl_Nf'].full()
        
        while ratio>e_tol and iteration<=max_iter:
            Qxx_bar     = self.N*[np.zeros((self.nxl,self.nxl))]
            Qxu_bar     = self.N*[np.zeros((self.nxl,self.nul))]
            Quu_bar     = self.N*[np.zeros((self.nul,self.nul))]
            Qxu         = self.N*[np.zeros((self.nxl,self.nul))]
            Quuinv      = self.N*[np.zeros((self.nul,self.nul))]
            Fx      = self.N*[np.zeros((self.nxl,self.nxl))]
            Fu      = self.N*[np.zeros((self.nxl,self.nul))]
            Vx      = (self.N+1)*[np.zeros((self.nxl,1))]
            Vxx     = (self.N+1)*[np.zeros((self.nxl,self.nxl))]
            Vx[self.N] = self.lxlN_fn(xl0=X_nominal[:,self.N],refxl0=Ref_xl[self.N*self.nxl:(self.N+1)*self.nxl],
                                      scxl0=scxl[self.N*self.nxl:(self.N+1)*self.nxl],paral0=weight1)['lxlN_f'].full()
            Vxx[self.N]= self.lxlxlN_fn(paral0=weight1)['lxlxlN_f'].full()
            # list of the control gains 
            K_fb    = self.N*[np.zeros((self.nul,self.nxl))] # feedback
            k_ff    = self.N*[np.zeros((self.nul,1))] # feedforward
            # backward pass
            for k in reversed(range(self.N)): # N-1, N-2,...,0
                Qx_k  = self.Qxl_fn(xl0=X_nominal[:,k],ul0=U_nominal[:,k],Vxl0=Vx[k+1],refxl0=Ref_xl[k*self.nxl:(k+1)*self.nxl],reful0=Ref_ul[k*self.nul:(k+1)*self.nul],
                                    scxl0=scxl[k*self.nxl:(k+1)*self.nxl],scxL0=scxL[k*self.nxl:(k+1)*self.nxl],scul0=scul[k*self.nul:(k+1)*self.nul],scuL0=scuL[k*self.nul:(k+1)*self.nul],paral0=weight1)['Qxl_f'].full()
                Qu_k  = self.Qul_fn(xl0=X_nominal[:,k],ul0=U_nominal[:,k],Vxl0=Vx[k+1],refxl0=Ref_xl[k*self.nxl:(k+1)*self.nxl],reful0=Ref_ul[k*self.nul:(k+1)*self.nul],
                                    scxl0=scxl[k*self.nxl:(k+1)*self.nxl],scxL0=scxL[k*self.nxl:(k+1)*self.nxl],scul0=scul[k*self.nul:(k+1)*self.nul],scuL0=scuL[k*self.nul:(k+1)*self.nul],paral0=weight1)['Qul_f'].full()
                Qxx_bar_k = self.Qxlxl_bar_fn(xl0=X_nominal[:,k],ul0=U_nominal[:,k],Vxl0=Vx[k+1],refxl0=Ref_xl[k*self.nxl:(k+1)*self.nxl],reful0=Ref_ul[k*self.nul:(k+1)*self.nul],
                                    scxl0=scxl[k*self.nxl:(k+1)*self.nxl],scxL0=scxL[k*self.nxl:(k+1)*self.nxl],scul0=scul[k*self.nul:(k+1)*self.nul],scuL0=scuL[k*self.nul:(k+1)*self.nul],paral0=weight1)['Qxlxl_bar_f'].full()
                Qxx_hat_k = self.Qxlxl_hat_fn(xl0=X_nominal[:,k],ul0=U_nominal[:,k],Vxlxl0=Vxx[k+1])['Qxlxl_hat_f'].full()
                Qxx_k     = Qxx_bar_k + Qxx_hat_k
                Qxu_bar_k = self.Qxlul_bar_fn(xl0=X_nominal[:,k],ul0=U_nominal[:,k],Vxl0=Vx[k+1],refxl0=Ref_xl[k*self.nxl:(k+1)*self.nxl],reful0=Ref_ul[k*self.nul:(k+1)*self.nul],
                                    scxl0=scxl[k*self.nxl:(k+1)*self.nxl],scxL0=scxL[k*self.nxl:(k+1)*self.nxl],scul0=scul[k*self.nul:(k+1)*self.nul],scuL0=scuL[k*self.nul:(k+1)*self.nul],paral0=weight1)['Qxlul_bar_f'].full()
                Qxu_hat_k = self.Qxlul_hat_fn(xl0=X_nominal[:,k],ul0=U_nominal[:,k],Vxlxl0=Vxx[k+1])['Qxlul_hat_f'].full()
                Qxu_k     = Qxu_bar_k + Qxu_hat_k
                Quu_bar_k = self.Qulul_bar_fn(xl0=X_nominal[:,k],ul0=U_nominal[:,k],Vxl0=Vx[k+1],refxl0=Ref_xl[k*self.nxl:(k+1)*self.nxl],reful0=Ref_ul[k*self.nul:(k+1)*self.nul],
                                    scxl0=scxl[k*self.nxl:(k+1)*self.nxl],scxL0=scxL[k*self.nxl:(k+1)*self.nxl],scul0=scul[k*self.nul:(k+1)*self.nul],scuL0=scuL[k*self.nul:(k+1)*self.nul],paral0=weight1)['Qulul_bar_f'].full()
                Quu_hat_k = self.Qulul_hat_fn(xl0=X_nominal[:,k],ul0=U_nominal[:,k],Vxlxl0=Vxx[k+1])['Qulul_hat_f'].full()
                Quu_k     = Quu_bar_k + Quu_hat_k + reg*np.identity(self.nul)
                Quu_inv = LA.inv(Quu_k) # more stable than 'solve'
                
                # compute the control gains
                K_fb[k]  = -Quu_inv@Qxu_k.T
                k_ff[k]  = -Quu_inv@Qu_k
                # compute the derivatives of the value function
                Vx[k]    = Qx_k - Qxu_k@Quu_inv@Qu_k
                Vxx[k]   = Qxx_k - Qxu_k@Quu_inv@Qxu_k.T
                Fx[k]    = self.Fxl_fn(xl0=X_nominal[:,k],ul0=U_nominal[:,k])['Fxl_f'].full()
                Fu[k]    = self.Ful_fn(xl0=X_nominal[:,k],ul0=U_nominal[:,k])['Ful_f'].full()
                Qxx_bar[k]   = Qxx_bar_k
                Qxu_bar[k]   = Qxu_bar_k
                Quu_bar[k]   = Quu_bar_k
                Quuinv[k]    = Quu_inv
                Qxu[k]       = Qxu_k
            # forward pass with adaptive alpha (line search), adaptive alpha makes the DDP more stable!
            alpha = alpha_init
            for i in range(max_line_search_steps):
                X_new = np.zeros((self.nxl,self.N+1))
                U_new = np.zeros((self.nul,self.N))
                X_new[:,0:1] = np.reshape(xl_0,(self.nxl,1))
                cost_new = 0
                for k in range(self.N):
                    delta_x = np.reshape(X_new[:,k] - X_nominal[:,k],(self.nxl,1))
                    u_k     = np.reshape(U_nominal[:,k],(self.nul,1)) + K_fb[k]@delta_x + alpha*k_ff[k]
                    u_k     = np.reshape(u_k,(self.nul,1))
                    X_new[:,k+1:k+2]  = self.model_l_fn(xl0=np.reshape(X_new[:,k],(self.nxl,1)),ul0=u_k)['mdynlf'].full()
                    U_new[:,k:k+1]    = u_k
                    cost_new   += self.Jl_kfn(xl0=X_new[:,k],ul0=u_k,scxl0=scxl[k*self.nxl:(k+1)*self.nxl],scxL0=scxL[k*self.nxl:(k+1)*self.nxl],
                                              scul0=scul[k*self.nul:(k+1)*self.nul],scuL0=scuL[k*self.nul:(k+1)*self.nul],
                                              refxl0=Ref_xl[k*self.nxl:(k+1)*self.nxl],reful0=Ref_ul[k*self.nul:(k+1)*self.nul],paral0=weight1)['Jl_kf'].full()
                cost_new   += self.Jl_Nfn(xl0=X_new[:,-1],refxl0=Ref_xl[self.N*self.nxl:(self.N+1)*self.nxl],scxl0=scxl[self.N*self.nxl:(self.N+1)*self.nxl],
                                          scxL0=scxL[self.N*self.nxl:(self.N+1)*self.nxl], paral0=weight1)['Jl_Nf'].full()
                # Check if the cost decreased
                if cost_new < cost_prev:
                    # update the trajectories
                    X_nominal = X_new
                    U_nominal = U_new
                    break
                alpha = np.clip(alpha*alpha_factor,alpha_min,alpha_init)  # Reduce alpha if cost did not improve

            ratio = np.abs(cost_new-cost_prev)/np.abs(cost_prev)
            print('iteration:',iteration,'ratio=',ratio)
            cost_prev = cost_new
            iteration += 1
        
        opt_sol={"xl_traj":X_nominal.T,
                 "ul_traj":U_nominal.T,
                 "Vxx":Vxx,
                 "Vx":Vx,
                 "K_FB":K_fb,
                 "Hxx":Qxx_bar,
                 "Qxu":Qxu,
                 "Hxu":Qxu_bar,
                 "Huu":Quu_bar,
                 "Quu_inv":Quuinv,
                 "Fx":Fx,
                 "Fu":Fu}
        return opt_sol
    

    def DDP_Cable_ADMM_Subp1(self,xi_0,Ref_xi,Ref_ui,weight2,scxi,scui,scxI,scuI,max_iter,e_tol):
        reg        = 1e-6
        alpha_init = 1 # Initial alpha for line search
        alpha_min = 1e-2  # Minimum allowable alpha
        alpha_factor = 0.5 # 
        max_line_search_steps = 5
        iteration = 1
        ratio = 10
        X_nominal = np.zeros((self.nxi,self.N+1))
        U_nominal = np.zeros((self.nui,self.N))
        X_nominal[:,0:1] = np.reshape(xi_0,(self.nxi,1))
        
        # Initial trajectory and initial cost 
        cost_prev = 0
        for k in range(self.N):
            u_k    = np.reshape(Ref_ui,(self.nui,1))
            X_nominal[:,k+1:k+2] = self.model_i_fn(xi0=X_nominal[:,k],ui0=u_k)['mdynif'].full()
            U_nominal[:,k:k+1]   = u_k
            cost_prev     += self.Ji_k_fn(xi0=X_nominal[:,k],ui0=u_k,scxi0=scxi[k*self.nxi:(k+1)*self.nxi],scxI0=scxI[k*self.nxi:(k+1)*self.nxi],
                                        scui0=scui[k*self.nui:(k+1)*self.nui],scuI0=scuI[k*self.nui:(k+1)*self.nui],
                                        refxi0=Ref_xi[k*self.nxi:(k+1)*self.nxi],refui0=Ref_ui,parai0=weight2)['Ji_kf'].full()
        cost_prev += self.Ji_N_fn(xi0=X_nominal[:,-1],refxi0=Ref_xi[self.N*self.nxi:(self.N+1)*self.nxi],scxi0=scxi[self.N*self.nxi:(self.N+1)*self.nxi],
                                 scxI0=scxI[self.N*self.nxi:(self.N+1)*self.nxi],parai0=weight2)['Ji_Nf'].full()
        
        while ratio>e_tol and iteration<=max_iter:
            Qxx_bar     = self.N*[np.zeros((self.nxi,self.nxi))]
            Qxu_bar     = self.N*[np.zeros((self.nxi,self.nui))]
            Quu_bar     = self.N*[np.zeros((self.nui,self.nui))]
            Qxu         = self.N*[np.zeros((self.nxi,self.nui))]
            Quuinv      = self.N*[np.zeros((self.nui,self.nui))]
            Fx      = self.N*[np.zeros((self.nxi,self.nxi))]
            Fu      = self.N*[np.zeros((self.nxi,self.nui))]
            Vx      = (self.N+1)*[np.zeros((self.nxi,1))]
            Vxx     = (self.N+1)*[np.zeros((self.nxi,self.nxi))]
            Vx[self.N] = self.lxiN_fn(xi0=X_nominal[:,self.N],refxi0=Ref_xi[self.N*self.nxi:(self.N+1)*self.nxi],
                                      scxi0=scxi[self.N*self.nxi:(self.N+1)*self.nxi],parai0=weight2)['lxiN_f'].full()
            Vxx[self.N]= self.lxixiN_fn(parai0=weight2)['lxixiN_f'].full()
            # list of the control gains 
            K_fb    = self.N*[np.zeros((self.nui,self.nxi))] # feedback
            k_ff    = self.N*[np.zeros((self.nui,1))] # feedforward
            # backward pass
            for k in reversed(range(self.N)): # N-1, N-2,...,0
                Qx_k  = self.Qxi_fn(xi0=X_nominal[:,k],ui0=U_nominal[:,k],Vxi0=Vx[k+1],refxi0=Ref_xi[k*self.nxi:(k+1)*self.nxi],refui0=Ref_ui,
                                    scxi0=scxi[k*self.nxi:(k+1)*self.nxi],scxI0=scxI[k*self.nxi:(k+1)*self.nxi],scui0=scui[k*self.nui:(k+1)*self.nui],scuI0=scuI[k*self.nui:(k+1)*self.nui],parai0=weight2)['Qxi_f'].full()
                Qu_k  = self.Qui_fn(xi0=X_nominal[:,k],ui0=U_nominal[:,k],Vxi0=Vx[k+1],refxi0=Ref_xi[k*self.nxi:(k+1)*self.nxi],refui0=Ref_ui,
                                    scxi0=scxi[k*self.nxi:(k+1)*self.nxi],scxI0=scxI[k*self.nxi:(k+1)*self.nxi],scui0=scui[k*self.nui:(k+1)*self.nui],scuI0=scuI[k*self.nui:(k+1)*self.nui],parai0=weight2)['Qui_f'].full()
                Qxx_bar_k = self.Qxixi_bar_fn(xi0=X_nominal[:,k],ui0=U_nominal[:,k],Vxi0=Vx[k+1],refxi0=Ref_xi[k*self.nxi:(k+1)*self.nxi],refui0=Ref_ui,
                                    scxi0=scxi[k*self.nxi:(k+1)*self.nxi],scxI0=scxI[k*self.nxi:(k+1)*self.nxi],scui0=scui[k*self.nui:(k+1)*self.nui],scuI0=scuI[k*self.nui:(k+1)*self.nui],parai0=weight2)['Qxixi_bar_f'].full()
                Qxx_hat_k = self.Qxixi_hat_fn(xi0=X_nominal[:,k],ui0=U_nominal[:,k],Vxixi0=Vxx[k+1])['Qxixi_hat_f'].full()
                Qxx_k     = Qxx_bar_k + Qxx_hat_k
                Qxu_bar_k = self.Qxiui_bar_fn(xi0=X_nominal[:,k],ui0=U_nominal[:,k],Vxi0=Vx[k+1],refxi0=Ref_xi[k*self.nxi:(k+1)*self.nxi],refui0=Ref_ui,
                                    scxi0=scxi[k*self.nxi:(k+1)*self.nxi],scxI0=scxI[k*self.nxi:(k+1)*self.nxi],scui0=scui[k*self.nui:(k+1)*self.nui],scuI0=scuI[k*self.nui:(k+1)*self.nui],parai0=weight2)['Qxiui_bar_f'].full()
                Qxu_hat_k = self.Qxiui_hat_fn(xi0=X_nominal[:,k],ui0=U_nominal[:,k],Vxixi0=Vxx[k+1])['Qxiui_hat_f'].full()
                Qxu_k     = Qxu_bar_k + Qxu_hat_k
                Quu_bar_k = self.Quiui_bar_fn(xi0=X_nominal[:,k],ui0=U_nominal[:,k],Vxi0=Vx[k+1],refxi0=Ref_xi[k*self.nxi:(k+1)*self.nxi],refui0=Ref_ui,
                                    scxi0=scxi[k*self.nxi:(k+1)*self.nxi],scxI0=scxI[k*self.nxi:(k+1)*self.nxi],scui0=scui[k*self.nui:(k+1)*self.nui],scuI0=scuI[k*self.nui:(k+1)*self.nui],parai0=weight2)['Quiui_bar_f'].full()
                Quu_hat_k = self.Quiui_hat_fn(xi0=X_nominal[:,k],ui0=U_nominal[:,k],Vxixi0=Vxx[k+1])['Quiui_hat_f'].full()
                Quu_k     = Quu_bar_k + Quu_hat_k + reg*np.identity(self.nui)
                # Quu_inv = solve(Quu_k+reg*np.identity(self.n_Wl),np.identity(self.n_Wl))
                Quu_inv = LA.inv(Quu_k) # more stable than 'solve'
                
                # compute the control gains
                K_fb[k]  = -Quu_inv@Qxu_k.T
                k_ff[k]  = -Quu_inv@Qu_k
                # compute the derivatives of the value function
                Vx[k]    = Qx_k - Qxu_k@Quu_inv@Qu_k
                Vxx[k]   = Qxx_k - Qxu_k@Quu_inv@Qxu_k.T # Riccati equation
                Fx[k]    = self.Fxi_fn(xi0=X_nominal[:,k],ui0=U_nominal[:,k])['Fxi_f'].full()
                Fu[k]    = self.Fui_fn(xi0=X_nominal[:,k],ui0=U_nominal[:,k])['Fui_f'].full()
                Qxx_bar[k]   = Qxx_bar_k
                Qxu_bar[k]   = Qxu_bar_k
                Quu_bar[k]   = Quu_bar_k
                Quuinv[k]    = Quu_inv
                Qxu[k]       = Qxu_k
            # forward pass with adaptive alpha (line search), adaptive alpha makes the DDP more stable!
            alpha = alpha_init
            for i in range(max_line_search_steps):
                X_new = np.zeros((self.nxi,self.N+1))
                U_new = np.zeros((self.nui,self.N))
                X_new[:,0:1] = np.reshape(xi_0,(self.nxi,1))
                cost_new = 0
                for k in range(self.N):
                    delta_x = np.reshape(X_new[:,k] - X_nominal[:,k],(self.nxi,1))
                    u_k     = np.reshape(U_nominal[:,k],(self.nui,1)) + K_fb[k]@delta_x + alpha*k_ff[k]
                    u_k     = np.reshape(u_k,(self.nui,1))
                    X_new[:,k+1:k+2]  = self.model_i_fn(xi0=np.reshape(X_new[:,k],(self.nxi,1)),ui0=u_k)['mdynif'].full()
                    U_new[:,k:k+1]    = u_k
                    cost_new   += self.Ji_k_fn(xi0=X_new[:,k],ui0=u_k,scxi0=scxi[k*self.nxi:(k+1)*self.nxi],scxI0=scxI[k*self.nxi:(k+1)*self.nxi],
                                              scui0=scui[k*self.nui:(k+1)*self.nui],scuI0=scuI[k*self.nui:(k+1)*self.nui],
                                              refxi0=Ref_xi[k*self.nxi:(k+1)*self.nxi],refui0=Ref_ui,parai0=weight2)['Ji_kf'].full()
                cost_new   += self.Ji_N_fn(xi0=X_new[:,-1],refxi0=Ref_xi[self.N*self.nxi:(self.N+1)*self.nxi],scxi0=scxi[self.N*self.nxi:(self.N+1)*self.nxi],
                                          scxI0=scxI[self.N*self.nxi:(self.N+1)*self.nxi], parai0=weight2)['Ji_Nf'].full()
                # Check if the cost decreased
                if cost_new < cost_prev:
                    # update the trajectories
                    X_nominal = X_new
                    U_nominal = U_new
                    break
                alpha = np.clip(alpha*alpha_factor,alpha_min,alpha_init)  # Reduce alpha if cost did not improve

            ratio = np.abs(cost_new-cost_prev)/np.abs(cost_prev)
            print('iteration:',iteration,'ratio=',ratio)
            cost_prev = cost_new
            iteration += 1
        
        opt_sol={"xi_traj":X_nominal.T,
                 "ui_traj":U_nominal.T,
                 "Vxx":Vxx,
                 "Vx":Vx,
                 "K_FB":K_fb,
                 "Hxx":Qxx_bar,
                 "Qxu":Qxu,
                 "Hxu":Qxu_bar,
                 "Huu":Quu_bar,
                 "Quu_inv":Quuinv,
                 "Fx":Fx,
                 "Fu":Fu}
        return opt_sol


    def MPC_Cable_DDP_Planning_SubP1(self,ParaC): # checked, correct, Apr.1 2025
        xc_traj    = [np.zeros((self.N+1,self.nxi)) for _ in range(int(self.nq))]
        uc_traj    = [np.zeros((self.N,self.nui)) for _ in range(int(self.nq))]
        OPt_sol_c  = []
        max_iter     = 5
        e_tol        = 1e-3
        for i in range(int(self.nq)):
            Parai    = ParaC[i]
            xi_fb    = Parai[0:self.nxi]
            Ref_xi   = Parai[self.nxi:self.nxi*(self.N+2)]
            Ref_ui   = Parai[self.nxi+self.nxi*(self.N+1):self.nxi+self.nxi*(self.N+1)+self.nui]
            # Solve the DDP
            n_scxi_start = self.nxi*(self.N+2)+self.nui
            scxi         = Parai[n_scxi_start:n_scxi_start+self.nxi*(self.N+1)]
            n_scxI_start = n_scxi_start + self.nxi*(self.N+1)
            scxI         = Parai[n_scxI_start:n_scxI_start+self.nxi*(self.N+1)]
            n_scui_start = n_scxI_start + self.nxi*(self.N+1)
            scui         = Parai[n_scui_start:n_scui_start+self.nui*self.N]
            n_scuI_start = n_scui_start + self.nui*self.N
            scuI         = Parai[n_scuI_start:n_scuI_start+self.nui*self.N]
            n_weig_start = n_scuI_start + self.nui*self.N
            weight2   = Parai[n_weig_start:n_weig_start+self.npi]
            opt_sol_i = self.DDP_Cable_ADMM_Subp1(xi_fb,Ref_xi,Ref_ui,weight2,scxi,scui,scxI,scuI,max_iter,e_tol)
            OPt_sol_c += [opt_sol_i]
            xc_traj[i] = opt_sol_i['xi_traj']
            uc_traj[i] = opt_sol_i['ui_traj']
        # output
        opt_solc = {"xc_traj":xc_traj,
                   "uc_traj":uc_traj
                   }
        
        return opt_solc, OPt_sol_c


    def ADMM_SubP2_Init(self):
        # static optimization problem at step k 
        # start with an empty NLP
        w2        = [] # optimal trajectory list
        self.w02  = [] # initial guess list of optimal trajectory 
        self.lbw2 = [] # lower boundary list of optimal variables
        self.ubw2 = [] # upper boundary list of optimal variables
        g2        = [] # equality and inequality constraint list
        self.lbg2 = [] # lower boundary list of constraints
        self.ubg2 = [] # upper boundary list of constraints
        
        # hyperparameters + external signals
        Para2    = SX.sym('P2', (self.nxl # load primal state  
                                +self.nxl # load primal state's Lagrangian multiplier     
                                +self.nul # load primal control
                                +self.nul # load primal control's Lagrangian multuplier
                                +self.nxi*int(self.nq) # all the cable primal states
                                +self.nxi*int(self.nq) # all the cable primal states' Lagrangian multipliers
                                +self.nui*int(self.nq) # all the cable primal controls
                                +self.nui*int(self.nq) # all the cable primal controls' Lagrangian multipliers
                                +1        # ADMM penalty parameter of the load
                                +1)) 

        # formulate the NLP
        p        = Para2[2*(self.nxl+self.nul+self.nxi*int(self.nq)+self.nui*int(self.nq))] # penalty parameter of the load
        pi       = Para2[2*(self.nxl+self.nul+self.nxi*int(self.nq)+self.nui*int(self.nq))+1]
        scxl_k   = SX.sym('scxl',self.nxl)
        w2      += [scxl_k]
        self.lbw2  += self.xl_lb
        self.ubw2  += self.xl_ub
        scul_k   = SX.sym('scul',self.nul)
        w2      += [scul_k]
        self.lbw2  += self.ul_lb
        self.ubw2  += self.ul_ub
        xl_k     = Para2[0:self.nxl]
        scxL_k   = Para2[self.nxl:2*self.nxl]
        ul_k     = Para2[2*self.nxl:2*self.nxl+self.nul]
        scuL_k   = Para2[2*self.nxl+self.nul:2*(self.nxl+self.nul)]
        # total cost at the step k that includes the load and all the cables
        J2       = self.Jl_P2_k_fn(xl0=xl_k,ul0=ul_k,scxl0=scxl_k,scxL0=scxL_k,scul0=scul_k,scuL0=scuL_k,p0=p)['Jl_P2_kf']
        scxc_k   = SX.sym('scxc',self.nxi*int(self.nq))
        scuc_k   = SX.sym('scuc',self.nui*int(self.nq))
        for i in range(int(self.nq)):
            scxi_k  = SX.sym('scx'+str(i),self.nxi)
            w2     += [scxi_k]
            self.lbw2  += self.scxi_lb
            self.ubw2  += self.scxi_ub
            scxc_k[i*self.nxi:(i+1)*self.nxi] = scxi_k
            scui_k  = SX.sym('scu'+str(i),self.nui)
            w2     += [scui_k]
            self.lbw2  += self.ui_lb
            self.ubw2  += self.ui_ub
            scuc_k[i*self.nui:(i+1)*self.nui] = scui_k
            n_start_xi   = 2*(self.nxl+self.nul)
            xi_k    = Para2[n_start_xi+i*self.nxi:n_start_xi+(i+1)*self.nxi] # cable primal state
            n_start_scxI = n_start_xi + self.nxi*int(self.nq)
            scxI_k  = Para2[n_start_scxI+i*self.nxi:n_start_scxI+(i+1)*self.nxi] # cable primal state Lagrangian multiplier
            n_start_ui   = n_start_scxI + self.nxi*int(self.nq)
            ui_k    = Para2[n_start_ui+i*self.nui:n_start_ui+(i+1)*self.nui]
            n_start_scuI = n_start_ui + self.nui*int(self.nq)
            scuI_k  = Para2[n_start_scuI+i*self.nui:n_start_scuI+(i+1)*self.nui]
            J2     += self.Ji_P2_k_fn(xi0=xi_k,scxi0=scxi_k,scxI0=scxI_k,ui0=ui_k,scui0=scui_k,scuI0=scuI_k, pi0=pi)['Ji_P2_kf']
        
        for i in range(int(self.nq)):    
            # safe constriant between the obstacle 1 and the ith quadrotor
            goi1    = self.Gi1[i](scxl0=scxl_k,scxc0=scxc_k)['go1f'+str(i)]
            g2     += [goi1]
            self.lbg2 += [0.1]
            self.ubg2 += [100] # add an upbound for numerical stability
            # safe constriant between the obstacle 2 and the ith quadrotor
            goi2    = self.Gi2[i](scxl0=scxl_k,scxc0=scxc_k)['go2f'+str(i)]
            g2     += [goi2]
            self.lbg2 += [0.1]
            self.ubg2 += [100] # add an upbound for numerical stability
            # quadrotor's thrust limit
            gif     = self.fi[i](scxl0=scxl_k,scul0=scul_k,scxc0=scxc_k,scuc0=scuc_k)['norm_ff'+str(i)]
            g2     += [gif]
            self.lbg2 += [0.1]
            self.ubg2 += [8] # tianchen's parameter
        
        for k in range(len(self.Gij)):
            gij    = self.Gij[k](scxl0=scxl_k,scxc0=scxc_k)['gf'+str(k)]
            g2      += [gij]
            self.lbg2 += [self.rq]
            self.ubg2 += [100]
        
        # control consensus constraint
        g_wc     = self.W_cons_fn(scxl0=scxl_k,scul0=scul_k,scxc0=scxc_k)['W_consf']
        g2       += [g_wc]
        self.lbg2 += self.nul*[0]
        self.ubg2 += self.nul*[0] 

        # create an NLP solver and solve it
        optsi2 = {}
        optsi2['ipopt.tol'] = 1e-8
        optsi2['ipopt.print_level'] = 0
        optsi2['print_time'] = 0
        optsi2['ipopt.warm_start_init_point']='yes'
        optsi2['ipopt.max_iter']=1e3
        optsi2['ipopt.acceptable_tol']=1e-8
        optsi2['ipopt.mu_strategy']='adaptive'
        prob2 = {'f': J2, 
                'x': vertcat(*w2), 
                'p': Para2,
                'g': vertcat(*g2)}
        self.solver2 = nlpsol('solver2', 'ipopt', prob2, optsi2)  


    def ADMM_SubP2_N_Init(self):
        # static optimization problem at step N (terminal) 
        # start with an empty NLP
        w2N        = [] # optimal trajectory list
        self.w02N  = [] # initial guess list of optimal trajectory 
        self.lbw2N = [] # lower boundary list of optimal variables
        self.ubw2N = [] # upper boundary list of optimal variables
        g2N        = [] # equality and inequality constraint list
        self.lbg2N = [] # lower boundary list of constraints
        self.ubg2N = [] # upper boundary list of constraints
        
        # hyperparameters + external signals
        Para2    = SX.sym('P2N', (self.nxl # load primal state at step N
                                +self.nxl # load primal state's Lagrangian multiplier     
                                +self.nxi*int(self.nq) # all the cable primal states
                                +self.nxi*int(self.nq) # all the cable primal states' Lagrangian multipliers
                                +1        # ADMM penalty parameter of the load's state
                                +1)) 

        # formulate the NLP
        p        = Para2[2*(self.nxl+self.nxi*int(self.nq))] # penalty parameter of the load
        pi       = Para2[2*(self.nxl+self.nxi*int(self.nq))+1]
        scxl_k   = SX.sym('scxl',self.nxl)
        w2N      += [scxl_k]
        self.lbw2N  += self.xl_lb
        self.ubw2N  += self.xl_ub
        xl_k     = Para2[0:self.nxl]
        scxL_k   = Para2[self.nxl:2*self.nxl]
        # total cost at the step k that includes the load and all the cables
        J2       = self.Jl_P2_N_fn(xl0=xl_k,scxl0=scxl_k,scxL0=scxL_k,p0=p)['Jl_P2_Nf']
        scxc_k   = SX.sym('scxc',self.nxi*int(self.nq))
        for i in range(int(self.nq)):
            scxi_k  = SX.sym('scx'+str(i),self.nxi)
            w2N     += [scxi_k]
            self.lbw2N  += self.scxi_lb
            self.ubw2N  += self.scxi_ub
            scxc_k[i*self.nxi:(i+1)*self.nxi] = scxi_k
            xi_k    = Para2[2*self.nxl+i*self.nxi:2*self.nxl+(i+1)*self.nxi] # cable primal state
            scxI_k  = Para2[2*self.nxl+self.nxi*int(self.nq)+i*self.nxi:2*self.nxl+self.nxi*int(self.nq)+(i+1)*self.nxi] # cable primal state Lagrangian multiplier
            J2     += self.Ji_P2_N_fn(xi0=xi_k,scxi0=scxi_k,scxI0=scxI_k,pi0=pi)['Jl_P2_Nf']
        
        for i in range(int(self.nq)):
            # safe constriant between the obstacle 1 and the ith quadrotor
            goi1    = self.Gi1[i](scxl0=scxl_k,scxc0=scxc_k)['go1f'+str(i)]
            g2N     += [goi1]
            self.lbg2N += [self.rq]
            self.ubg2N += [100] # add an upbound for numerical stability
            # safe constriant between the obstacle 2 and the ith quadrotor
            goi2    = self.Gi2[i](scxl0=scxl_k,scxc0=scxc_k)['go2f'+str(i)]
            g2N     += [goi2]
            self.lbg2N += [self.rq]
            self.ubg2N += [100] # add an upbound for numerical stability
        
        for k in range(len(self.Gij)):
            gij    = self.Gij[k](scxl0=scxl_k,scxc0=scxc_k)['gf'+str(k)]
            g2N      += [gij]
            self.lbg2N += [self.rq]
            self.ubg2N += [100]
        

        # create an NLP solver and solve it
        optsi2 = {}
        optsi2['ipopt.tol'] = 1e-8
        optsi2['ipopt.print_level'] = 0
        optsi2['print_time'] = 0
        optsi2['ipopt.warm_start_init_point']='yes'
        optsi2['ipopt.max_iter']=1e3
        optsi2['ipopt.acceptable_tol']=1e-8
        optsi2['ipopt.mu_strategy']='adaptive'
        prob2N = {'f': J2, 
                'x': vertcat(*w2N), 
                'p': Para2,
                'g': vertcat(*g2N)}
        self.solver2N = nlpsol('solver2N', 'ipopt', prob2N, optsi2)  

    

    
    def ADMM_SubP2(self,Para2_cable):
        # Para2_cable = SX.sym('p2_cable',(self.nxl*(self.N+1) # load reference state for initialization
        #                                 +self.nul*self.N # load reference control for initialization 
        #                                 +self.nxi*self.nq*(self.N+1) # cables' reference states for initialization
        #                                 +self.nui*self.nq # cables' reference controls for initialization
        #---------------------------------------------------------------------------------------------------
        #                                 +self.nxl*(self.N+1) # load primal state trajectory
        #                                 +self.nxl*(self.N+1) # load primal state's Lagrangian multiplier trajectory
        #                                 +self.nul*self.N # load primal control trajectory
        #                                 +self.nul*self.N # load primal control's Lagrangian multiplier trajectory
        #                                 +self.nxi*self.nq*(self.N+1) # cables' primal state trajectories
        #                                 +self.nxi*self.nq*(self.N+1) # cables' primal state's Lagrangian multiplier trajectories
        #                                 +self.nui*self.nq*self.N # cables' primal control trajectories
        #                                 +self.nui*self.nq*self.N # cables' primal control's Lagrangian multiplier trajectories
        #                                 +2))
        scxl_traj    = np.zeros((self.N+1,self.nxl))
        scul_traj    = np.zeros((self.N,self.nul))
        scxc_traj    = [np.zeros((self.N+1,self.nxi)) for _ in range(int(self.nq))]
        scuc_traj    = [np.zeros((self.N,self.nui)) for _ in range(int(self.nq))]
        n_start_pl   = 3*self.nxl*(self.N+1)+3*self.nul*self.N+self.nui*int(self.nq)+3*self.nxi*int(self.nq)*(self.N+1)+2*self.nui*int(self.nq)*self.N
        p            = Para2_cable[n_start_pl] # load ADMM penalty parameter for load state
        pi           = Para2_cable[n_start_pl+1]
        for k in range(self.N):
            self.w02 = []
            xl_ref   = Para2_cable[k*self.nxl:(k+1)*self.nxl]
            ul_ref   = Para2_cable[self.nxl*(self.N+1)+k*self.nul:self.nxl*(self.N+1)+(k+1)*self.nul]
            scxl0    = []
            for j in range(self.nxl):
                scxl0 += [xl_ref[j]]
            self.w02 += scxl0
            scul0    = []
            for j in range(self.nul):
                scul0 += [ul_ref[j]]
            self.w02 += scul0
            n_start_xl   = self.nxl*(self.N+1)+self.nul*self.N+self.nxi*int(self.nq)*(self.N+1)+self.nui*int(self.nq)
            xl_k         = Para2_cable[n_start_xl+k*self.nxl:n_start_xl+(k+1)*self.nxl]
            n_start_scxL = n_start_xl + self.nxl*(self.N+1)
            scxL_k       = Para2_cable[n_start_scxL+k*self.nxl:n_start_scxL+(k+1)*self.nxl]
            n_start_ul   = n_start_scxL + self.nxl*(self.N+1)
            ul_k         = Para2_cable[n_start_ul+k*self.nul:n_start_ul+(k+1)*self.nul]
            n_start_scuL = n_start_ul + self.nul*self.N
            scuL_k       = Para2_cable[n_start_scuL+k*self.nul:n_start_scuL+(k+1)*self.nul]
            n_start_xc   = n_start_scuL + self.nul*self.N
            xc_k         = Para2_cable[n_start_xc+k*self.nxi*int(self.nq):n_start_xc+(k+1)*self.nxi*int(self.nq)]
            n_start_scxC = n_start_xc + self.nxi*int(self.nq)*(self.N+1)
            scxC_k       = Para2_cable[n_start_scxC+k*self.nxi*int(self.nq):n_start_scxC+(k+1)*self.nxi*int(self.nq)]
            n_start_uc   = n_start_scxC + self.nxi*int(self.nq)*(self.N+1)
            uc_k         = Para2_cable[n_start_uc+k*self.nui*int(self.nq):n_start_uc+(k+1)*self.nui*int(self.nq)]
            n_start_scuC = n_start_uc + self.nui*int(self.nq)*self.N
            scuC_k       = Para2_cable[n_start_scuC+k*self.nui*int(self.nq):n_start_scuC+(k+1)*self.nui*int(self.nq)]
            xq_ref_k     = Para2_cable[self.nxl*(self.N+1)+self.nul*self.N+k*self.nxi*int(self.nq):self.nxl*(self.N+1)+self.nul*self.N+(k+1)*self.nxi*int(self.nq)]
            for i in range(int(self.nq)):
                scxi0   = []
                xi_ref  = xq_ref_k[i*self.nxi:(i+1)*self.nxi]
                for j in range(self.nxi):
                    scxi0 +=[xi_ref[j]]
                self.w02 += scxi0
                scui0   = []
                ui_ref  = Para2_cable[self.nxl*(self.N+1)+self.nul*self.N+self.nxi*int(self.nq)*self.N+i*self.nui:self.nxl*(self.N+1)+self.nul*self.N+self.nxi*int(self.nq)*self.N+(i+1)*self.nui]
                for j in range(self.nui):
                    scui0 +=[ui_ref[j]]
                self.w02 += scui0
            para2   = np.concatenate((xl_k,scxL_k))
            para2   = np.concatenate((para2,ul_k))
            para2   = np.concatenate((para2,scuL_k))
            para2   = np.concatenate((para2,xc_k))
            para2   = np.concatenate((para2,scxC_k))
            para2   = np.concatenate((para2,uc_k))
            para2   = np.concatenate((para2,scuC_k))
            para2   = np.concatenate((para2,[p,pi]))
            # Solve the NLP
            sol2 = self.solver2(x0=self.w02, 
                          lbx=self.lbw2, 
                          ubx=self.ubw2, 
                          p=para2,
                          lbg=self.lbg2, 
                          ubg=self.ubg2)
            w_opt2 = sol2['x'].full().flatten()
            # take the optimal control and state
            sol_traj = np.reshape(w_opt2, (-1, self.nxl + self.nul + (self.nxi+self.nui)*int(self.nq)))
            scxl_opt = sol_traj[:,0:self.nxl]
            scul_opt = sol_traj[:,self.nxl:self.nxl + self.nul]
            scc_opt  = sol_traj[:,self.nxl + self.nul:self.nxl + self.nul + (self.nxi+self.nui)*int(self.nq)]
            scxl_traj[k:k+1,:] = scxl_opt
            scul_traj[k:k+1,:] = scul_opt
            for i in range(int(self.nq)):
                scxc_traj[i][k:k+1,:]=scc_opt[:,i*(self.nxi+self.nui):i*(self.nxi+self.nui)+self.nxi]
                scuc_traj[i][k:k+1,:]=scc_opt[:,i*(self.nxi+self.nui)+self.nxi:(i+1)*(self.nxi+self.nui)]
        
        # terminal cost
        self.w02N = []
        xl_ref   = Para2_cable[self.N*self.nxl:(self.N+1)*self.nxl]
        scxl0N    = []
        for j in range(self.nxl):
            scxl0N += [xl_ref[j]]
        self.w02N += scxl0N
        xq_ref_N  = Para2_cable[self.nxl*(self.N+1)+self.nul*self.N+self.nxi*int(self.nq)*self.N:self.nxl*(self.N+1)+self.nul*self.N+self.nxi*int(self.nq)*(self.N+1)]
        for i in range(int(self.nq)):
            scxi0N   = []
            xi_ref  = xq_ref_N[i*self.nxi:(i+1)*self.nxi]
            for j in range(self.nxi):
                scxi0N +=[xi_ref[j]]
            self.w02N += scxi0N
        xl_N   = Para2_cable[n_start_xl+self.N*self.nxl:n_start_xl+(self.N+1)*self.nxl]
        scxL_N = Para2_cable[n_start_scxL+self.N*self.nxl:n_start_scxL+(self.N+1)*self.nxl]
        xc_N   = Para2_cable[n_start_xc+self.N*self.nxi*int(self.nq):n_start_xc+(self.N+1)*self.nxi*int(self.nq)]
        scxC_N = Para2_cable[n_start_scxC+self.N*self.nxi*int(self.nq):n_start_scxC+(self.N+1)*self.nxi*int(self.nq)]
        para2N  = np.concatenate((xl_N,scxL_N))
        para2N  = np.concatenate((para2N,xc_N))
        para2N  = np.concatenate((para2N,scxC_N))
        para2N  = np.concatenate((para2N,[p,pi]))
        # Solve the NLP
        sol2N = self.solver2N(x0=self.w02N, 
                          lbx=self.lbw2N, 
                          ubx=self.ubw2N, 
                          p=para2N,
                          lbg=self.lbg2N, 
                          ubg=self.ubg2N)
        w_opt2N = sol2N['x'].full().flatten()
        sol_trajN = np.reshape(w_opt2N, (-1, self.nxl + self.nxi*int(self.nq)))
        scxl_optN = sol_trajN[:,0:self.nxl]
        scxc_optN = sol_trajN[:,self.nxl:self.nxl+ self.nxi*int(self.nq)]
        scxl_traj[self.N:self.N+1,:] = scxl_optN
        for i in range(int(self.nq)):
            scxc_traj[i][self.N:self.N+1,:]=scxc_optN[:,i*self.nxi:(i+1)*self.nxi]
        # output
        opt_sol2 = {"scxl_traj":scxl_traj,
                    "scul_traj":scul_traj,
                    "scxc_traj":scxc_traj,
                    "scuc_traj":scuc_traj
                    }
        
        return opt_sol2
    

    def system_derivatives_SubP2_ADMM_k(self):
        # gradients of the Lagrangian (augmented cost function with the soft constraints)
        self.Lscxl          = jacobian(self.J_2_soft_k,self.scxl)
        self.Lscul          = jacobian(self.J_2_soft_k,self.scul)
        self.Lscxc          = jacobian(self.J_2_soft_k,self.scxc)
        self.Lscuc          = jacobian(self.J_2_soft_k,self.scuc)
        # hessians
        self.Lscxlscxl      = jacobian(self.Lscxl,self.scxl)
        self.Lscxlscxl_fn   = Function('Lscxlscxl',[self.xl,self.ul,self.xc,self.uc,self.scxl,self.scxL,self.scul,self.scuL,self.scxc,self.scxC,self.scuc,self.scuC,self.P_auto],[self.Lscxlscxl],
                                       ['xl0','ul0','xc0','uc0','scxl0','scxL0','scul0','scuL0','scxc0','scxC0','scuc0','scuC0','pauto0'],['Lscxlscxl_f'])
        self.Lscxlscul      = jacobian(self.Lscxl,self.scul)
        self.Lscxlscul_fn   = Function('Lscxlscul',[self.xl,self.ul,self.xc,self.uc,self.scxl,self.scxL,self.scul,self.scuL,self.scxc,self.scxC,self.scuc,self.scuC,self.P_auto],[self.Lscxlscul],
                                       ['xl0','ul0','xc0','uc0','scxl0','scxL0','scul0','scuL0','scxc0','scxC0','scuc0','scuC0','pauto0'],['Lscxlscul_f'])
        self.Lscxlscxc      = jacobian(self.Lscxl,self.scxc)
        self.Lscxlscxc_fn   = Function('Lscxlscxc',[self.xl,self.ul,self.xc,self.uc,self.scxl,self.scxL,self.scul,self.scuL,self.scxc,self.scxC,self.scuc,self.scuC,self.P_auto],[self.Lscxlscxc],
                                       ['xl0','ul0','xc0','uc0','scxl0','scxL0','scul0','scuL0','scxc0','scxC0','scuc0','scuC0','pauto0'],['Lscxlscxc_f'])
        self.Lscxlscuc      = jacobian(self.Lscxl,self.scuc)
        self.Lscxlscuc_fn   = Function('Lscxlscuc',[self.xl,self.ul,self.xc,self.uc,self.scxl,self.scxL,self.scul,self.scuL,self.scxc,self.scxC,self.scuc,self.scuC,self.P_auto],[self.Lscxlscuc],
                                       ['xl0','ul0','xc0','uc0','scxl0','scxL0','scul0','scuL0','scxc0','scxC0','scuc0','scuC0','pauto0'],['Lscxlscuc_f'])
        self.Lsculscul      = jacobian(self.Lscul,self.scul)
        self.Lsculscul_fn   = Function('Lsculscul',[self.xl,self.ul,self.xc,self.uc,self.scxl,self.scxL,self.scul,self.scuL,self.scxc,self.scxC,self.scuc,self.scuC,self.P_auto],[self.Lsculscul],
                                       ['xl0','ul0','xc0','uc0','scxl0','scxL0','scul0','scuL0','scxc0','scxC0','scuc0','scuC0','pauto0'],['Lsculscul_f'])
        self.Lsculscxc      = jacobian(self.Lscul,self.scxc)
        self.Lsculscxc_fn   = Function('Lsculscxc',[self.xl,self.ul,self.xc,self.uc,self.scxl,self.scxL,self.scul,self.scuL,self.scxc,self.scxC,self.scuc,self.scuC,self.P_auto],[self.Lsculscxc],
                                       ['xl0','ul0','xc0','uc0','scxl0','scxL0','scul0','scuL0','scxc0','scxC0','scuc0','scuC0','pauto0'],['Lsculscxc_f'])
        self.Lsculscuc      = jacobian(self.Lscul,self.scuc)
        self.Lsculscuc_fn   = Function('Lsculscuc',[self.xl,self.ul,self.xc,self.uc,self.scxl,self.scxL,self.scul,self.scuL,self.scxc,self.scxC,self.scuc,self.scuC,self.P_auto],[self.Lsculscuc],
                                       ['xl0','ul0','xc0','uc0','scxl0','scxL0','scul0','scuL0','scxc0','scxC0','scuc0','scuC0','pauto0'],['Lsculscuc_f'])
        self.Lscxcscxc      = jacobian(self.Lscxc,self.scxc)
        self.Lscxcscxc_fn   = Function('Lscxcscxc',[self.xl,self.ul,self.xc,self.uc,self.scxl,self.scxL,self.scul,self.scuL,self.scxc,self.scxC,self.scuc,self.scuC,self.P_auto],[self.Lscxcscxc],
                                       ['xl0','ul0','xc0','uc0','scxl0','scxL0','scul0','scuL0','scxc0','scxC0','scuc0','scuC0','pauto0'],['Lscxcscxc_f'])
        self.Lscxcscuc      = jacobian(self.Lscxc,self.scuc)
        self.Lscxcscuc_fn   = Function('Lscxcscuc',[self.xl,self.ul,self.xc,self.uc,self.scxl,self.scxL,self.scul,self.scuL,self.scxc,self.scxC,self.scuc,self.scuC,self.P_auto],[self.Lscxcscuc],
                                       ['xl0','ul0','xc0','uc0','scxl0','scxL0','scul0','scuL0','scxc0','scxC0','scuc0','scuC0','pauto0'],['Lscxcscuc_f'])
        self.Lscucscuc      = jacobian(self.Lscuc,self.scuc)
        self.Lscucscuc_fn   = Function('Lscucscuc',[self.xl,self.ul,self.xc,self.uc,self.scxl,self.scxL,self.scul,self.scuL,self.scxc,self.scxC,self.scuc,self.scuC,self.P_auto],[self.Lscucscuc],
                                       ['xl0','ul0','xc0','uc0','scxl0','scxL0','scul0','scuL0','scxc0','scxC0','scuc0','scuC0','pauto0'],['Lscucscuc_f'])
        # hessians w.r.t. the hyperparameters
        self.Lscxlp         = jacobian(self.Lscxl,self.P_auto)
        self.Lscxlp_fn      = Function('Lscxlp',[self.xl,self.ul,self.xc,self.uc,self.scxl,self.scxL,self.scul,self.scuL,self.scxc,self.scxC,self.scuc,self.scuC,self.P_auto],[self.Lscxlp],
                                       ['xl0','ul0','xc0','uc0','scxl0','scxL0','scul0','scuL0','scxc0','scxC0','scuc0','scuC0','pauto0'],['Lscxlp_f'])
        self.Lsculp         = jacobian(self.Lscul,self.P_auto)
        self.Lsculp_fn      = Function('Lsculp',[self.xl,self.ul,self.xc,self.uc,self.scxl,self.scxL,self.scul,self.scuL,self.scxc,self.scxC,self.scuc,self.scuC,self.P_auto],[self.Lsculp],
                                       ['xl0','ul0','xc0','uc0','scxl0','scxL0','scul0','scuL0','scxc0','scxC0','scuc0','scuC0','pauto0'],['Lsculp_f'])
        self.Lscxcp         = jacobian(self.Lscxc,self.P_auto)
        self.Lscxcp_fn      = Function('Lscxcp',[self.xl,self.ul,self.xc,self.uc,self.scxl,self.scxL,self.scul,self.scuL,self.scxc,self.scxC,self.scuc,self.scuC,self.P_auto],[self.Lscxcp],
                                       ['xl0','ul0','xc0','uc0','scxl0','scxL0','scul0','scuL0','scxc0','scxC0','scuc0','scuC0','pauto0'],['Lscxcp_f'])
        self.Lscucp         = jacobian(self.Lscuc,self.P_auto)
        self.Lscucp_fn      = Function('Lscucp',[self.xl,self.ul,self.xc,self.uc,self.scxl,self.scxL,self.scul,self.scuL,self.scxc,self.scxC,self.scuc,self.scuC,self.P_auto],[self.Lscucp],
                                       ['xl0','ul0','xc0','uc0','scxl0','scxL0','scul0','scuL0','scxc0','scxC0','scuc0','scuC0','pauto0'],['Lscucp_f'])

    def system_derivatives_SubP2_ADMM_N(self):
        # gradients of the Lagrangian (augmented cost function with the soft constraints)
        self.Lscxl_N        = jacobian(self.J_2_soft_N,self.scxl)
        self.Lscxc_N        = jacobian(self.J_2_soft_N,self.scxc)
        # hessians
        self.Lscxlscxl_N    = jacobian(self.Lscxl_N,self.scxl)
        self.Lscxlscxl_N_fn = Function('LscxlscxlN',[self.xl,self.xc,self.scxl,self.scxL,self.scxc,self.scxC,self.P_auto],[self.Lscxlscxl_N],
                                       ['xl0','xc0','scxl0','scxL0','scxc0','scxC0','pauto0'],['LscxlscxlN_f'])
        self.Lscxlscxc_N    = jacobian(self.Lscxl_N,self.scxc)
        self.Lscxlscxc_N_fn = Function('LscxlscxcN',[self.xl,self.xc,self.scxl,self.scxL,self.scxc,self.scxC,self.P_auto],[self.Lscxlscxc_N],
                                       ['xl0','xc0','scxl0','scxL0','scxc0','scxC0','pauto0'],['LscxlscxcN_f'])
        self.Lscxcscxc_N    = jacobian(self.Lscxc_N,self.scxc)
        self.Lscxcscxc_N_fn = Function('LscxcscxcN',[self.xl,self.xc,self.scxl,self.scxL,self.scxc,self.scxC,self.P_auto],[self.Lscxcscxc_N],
                                       ['xl0','xc0','scxl0','scxL0','scxc0','scxC0','pauto0'],['LscxcscxcN_f'])
        # hessians w.r.t. the hyperparameters
        self.Lscxlp_N       = jacobian(self.Lscxl_N,self.P_auto)
        self.Lscxlp_N_fn    = Function('LscxlpN',[self.xl,self.xc,self.scxl,self.scxL,self.scxc,self.scxC,self.P_auto],[self.Lscxlp_N],
                                       ['xl0','xc0','scxl0','scxL0','scxc0','scxC0','pauto0'],['LscxlpN_f'])
        self.Lscxcp_N       = jacobian(self.Lscxc_N,self.P_auto)
        self.Lscxcp_N_fn    = Function('LscxcpN',[self.xl,self.xc,self.scxl,self.scxL,self.scxc,self.scxC,self.P_auto],[self.Lscxcp_N],
                                       ['xl0','xc0','scxl0','scxL0','scxc0','scxC0','pauto0'],['LscxcpN_f'])



    def Get_AuxSys_SubP2(self,opt_sol1_l,opt_sol1_c,opt_sol2,scxL,scuL,scxC_list,scuC_list,Pauto):
        xl      = opt_sol1_l['xl_traj']
        ul      = opt_sol1_l['ul_traj']
        xc_list      = opt_sol1_c['xc_traj'] # list that contains all the cables' states
        uc_list      = opt_sol1_c['uc_traj'] # list that contains all the cables' controls
        scxl    = opt_sol2['scxl_traj']
        scul    = opt_sol2['scul_traj']
        scxc_list    = opt_sol2['scxc_traj'] # list that contains all the cables' safe states
        scuc_list    = opt_sol2['scuc_traj'] # list that contains all the cables' safe controls
        Lscxlscxl    = (self.N+1)*[np.zeros((self.nxl,self.nxl))]
        Lscxlscul    = self.N*[np.zeros((self.nxl,self.nul))]
        Lscxlscxc    = (self.N+1)*[np.zeros((self.nxl,self.nxi*int(self.nq)))]
        Lscxlscuc    = self.N*[np.zeros((self.nxl,self.nui*int(self.nq)))]
        Lsculscul    = self.N*[np.zeros((self.nul,self.nul))]
        Lsculscxc    = self.N*[np.zeros((self.nul,self.nxi*int(self.nq)))]
        Lsculscuc    = self.N*[np.zeros((self.nul,self.nui*int(self.nq)))]
        Lscxcscxc    = (self.N+1)*[np.zeros((self.nxi*int(self.nq),self.nxi*int(self.nq)))]
        Lscxcscuc    = self.N*[np.zeros((self.nxi*int(self.nq),self.nui*int(self.nq)))]
        Lscucscuc    = self.N*[np.zeros((self.nui*int(self.nq),self.nui*int(self.nq)))]
        Lscxlp       = (self.N+1)*[np.zeros((self.nxl,self.n_Pauto))]
        Lsculp       = self.N*[np.zeros((self.nul,self.n_Pauto))]
        Lscxcp       = (self.N+1)*[np.zeros((self.nxi*int(self.nq),self.n_Pauto))]
        Lscucp       = self.N*[np.zeros((self.nui*int(self.nq),self.n_Pauto))]
        for k in range(self.N):
            xl_k     = xl[k,:]
            ul_k     = ul[k,:]
            xc_k     = np.concatenate([xc_list[i][k,:] for i in range(int(self.nq))])
            uc_k     = np.concatenate([uc_list[i][k,:] for i in range(int(self.nq))])
            scxl_k   = scxl[k,:]
            scxL_k   = scxL[k,:]
            scul_k   = scul[k,:]
            scuL_k   = scuL[k,:]
            scxc_k   = np.concatenate([scxc_list[i][k,:] for i in range(int(self.nq))])
            scxC_k   = np.concatenate([scxC_list[i][k,:] for i in range(int(self.nq))])
            scuc_k   = np.concatenate([scuc_list[i][k,:] for i in range(int(self.nq))])
            scuC_k   = np.concatenate([scuC_list[i][k,:] for i in range(int(self.nq))])
            Lscxlscxl[k] = self.Lscxlscxl_fn(xl0=xl_k,ul0=ul_k,xc0=xc_k,uc0=uc_k,scxl0=scxl_k,scxL0=scxL_k,scul0=scul_k,scuL0=scuL_k,scxc0=scxc_k,scxC0=scxC_k,scuc0=scuc_k,scuC0=scuC_k,pauto0=Pauto)['Lscxlscxl_f'].full()
            Lscxlscul[k] = self.Lscxlscul_fn(xl0=xl_k,ul0=ul_k,xc0=xc_k,uc0=uc_k,scxl0=scxl_k,scxL0=scxL_k,scul0=scul_k,scuL0=scuL_k,scxc0=scxc_k,scxC0=scxC_k,scuc0=scuc_k,scuC0=scuC_k,pauto0=Pauto)['Lscxlscul_f'].full()
            Lscxlscxc[k] = self.Lscxlscxc_fn(xl0=xl_k,ul0=ul_k,xc0=xc_k,uc0=uc_k,scxl0=scxl_k,scxL0=scxL_k,scul0=scul_k,scuL0=scuL_k,scxc0=scxc_k,scxC0=scxC_k,scuc0=scuc_k,scuC0=scuC_k,pauto0=Pauto)['Lscxlscxc_f'].full()
            Lscxlscuc[k] = self.Lscxlscuc_fn(xl0=xl_k,ul0=ul_k,xc0=xc_k,uc0=uc_k,scxl0=scxl_k,scxL0=scxL_k,scul0=scul_k,scuL0=scuL_k,scxc0=scxc_k,scxC0=scxC_k,scuc0=scuc_k,scuC0=scuC_k,pauto0=Pauto)['Lscxlscuc_f'].full()
            Lsculscul[k] = self.Lsculscul_fn(xl0=xl_k,ul0=ul_k,xc0=xc_k,uc0=uc_k,scxl0=scxl_k,scxL0=scxL_k,scul0=scul_k,scuL0=scuL_k,scxc0=scxc_k,scxC0=scxC_k,scuc0=scuc_k,scuC0=scuC_k,pauto0=Pauto)['Lsculscul_f'].full()
            Lsculscxc[k] = self.Lsculscxc_fn(xl0=xl_k,ul0=ul_k,xc0=xc_k,uc0=uc_k,scxl0=scxl_k,scxL0=scxL_k,scul0=scul_k,scuL0=scuL_k,scxc0=scxc_k,scxC0=scxC_k,scuc0=scuc_k,scuC0=scuC_k,pauto0=Pauto)['Lsculscxc_f'].full()
            Lsculscuc[k] = self.Lsculscuc_fn(xl0=xl_k,ul0=ul_k,xc0=xc_k,uc0=uc_k,scxl0=scxl_k,scxL0=scxL_k,scul0=scul_k,scuL0=scuL_k,scxc0=scxc_k,scxC0=scxC_k,scuc0=scuc_k,scuC0=scuC_k,pauto0=Pauto)['Lsculscuc_f'].full()
            Lscxcscxc[k] = self.Lscxcscxc_fn(xl0=xl_k,ul0=ul_k,xc0=xc_k,uc0=uc_k,scxl0=scxl_k,scxL0=scxL_k,scul0=scul_k,scuL0=scuL_k,scxc0=scxc_k,scxC0=scxC_k,scuc0=scuc_k,scuC0=scuC_k,pauto0=Pauto)['Lscxcscxc_f'].full()
            Lscxcscuc[k] = self.Lscxcscuc_fn(xl0=xl_k,ul0=ul_k,xc0=xc_k,uc0=uc_k,scxl0=scxl_k,scxL0=scxL_k,scul0=scul_k,scuL0=scuL_k,scxc0=scxc_k,scxC0=scxC_k,scuc0=scuc_k,scuC0=scuC_k,pauto0=Pauto)['Lscxcscuc_f'].full()
            Lscucscuc[k] = self.Lscucscuc_fn(xl0=xl_k,ul0=ul_k,xc0=xc_k,uc0=uc_k,scxl0=scxl_k,scxL0=scxL_k,scul0=scul_k,scuL0=scuL_k,scxc0=scxc_k,scxC0=scxC_k,scuc0=scuc_k,scuC0=scuC_k,pauto0=Pauto)['Lscucscuc_f'].full()
            Lscxlp[k]    = self.Lscxlp_fn(xl0=xl_k,ul0=ul_k,xc0=xc_k,uc0=uc_k,scxl0=scxl_k,scxL0=scxL_k,scul0=scul_k,scuL0=scuL_k,scxc0=scxc_k,scxC0=scxC_k,scuc0=scuc_k,scuC0=scuC_k,pauto0=Pauto)['Lscxlp_f'].full()
            Lsculp[k]    = self.Lsculp_fn(xl0=xl_k,ul0=ul_k,xc0=xc_k,uc0=uc_k,scxl0=scxl_k,scxL0=scxL_k,scul0=scul_k,scuL0=scuL_k,scxc0=scxc_k,scxC0=scxC_k,scuc0=scuc_k,scuC0=scuC_k,pauto0=Pauto)['Lsculp_f'].full()
            Lscxcp[k]    = self.Lscxcp_fn(xl0=xl_k,ul0=ul_k,xc0=xc_k,uc0=uc_k,scxl0=scxl_k,scxL0=scxL_k,scul0=scul_k,scuL0=scuL_k,scxc0=scxc_k,scxC0=scxC_k,scuc0=scuc_k,scuC0=scuC_k,pauto0=Pauto)['Lscxcp_f'].full()
            Lscucp[k]    = self.Lscucp_fn(xl0=xl_k,ul0=ul_k,xc0=xc_k,uc0=uc_k,scxl0=scxl_k,scxL0=scxL_k,scul0=scul_k,scuL0=scuL_k,scxc0=scxc_k,scxC0=scxC_k,scuc0=scuc_k,scuC0=scuC_k,pauto0=Pauto)['Lscucp_f'].full()
        # ternimal hessians
        xl_N     = xl[-1,:]
        xc_N     = np.concatenate([xc_list[i][-1,:] for i in range(int(self.nq))])
        scxl_N   = scxl[-1,:]
        scxL_N   = scxL[-1,:]
        scxc_N   = np.concatenate([scxc_list[i][-1,:] for i in range(int(self.nq))])
        scxC_N   = np.concatenate([scxC_list[i][-1,:] for i in range(int(self.nq))])
        Lscxlscxl[self.N] = self.Lscxlscxl_N_fn(xl0=xl_N,xc0=xc_N,scxl0=scxl_N,scxL0=scxL_N,scxc0=scxc_N,scxC0=scxC_N,pauto0=Pauto)['LscxlscxlN_f'].full()
        Lscxlscxc[self.N] = self.Lscxlscxc_N_fn(xl0=xl_N,xc0=xc_N,scxl0=scxl_N,scxL0=scxL_N,scxc0=scxc_N,scxC0=scxC_N,pauto0=Pauto)['LscxlscxcN_f'].full()
        Lscxcscxc[self.N] = self.Lscxcscxc_N_fn(xl0=xl_N,xc0=xc_N,scxl0=scxl_N,scxL0=scxL_N,scxc0=scxc_N,scxC0=scxC_N,pauto0=Pauto)['LscxcscxcN_f'].full()
        Lscxlp[self.N]    = self.Lscxlp_N_fn(xl0=xl_N,xc0=xc_N,scxl0=scxl_N,scxL0=scxL_N,scxc0=scxc_N,scxC0=scxC_N,pauto0=Pauto)['LscxlpN_f'].full()
        Lscxcp[self.N]    = self.Lscxcp_N_fn(xl0=xl_N,xc0=xc_N,scxl0=scxl_N,scxL0=scxL_N,scxc0=scxc_N,scxC0=scxC_N,pauto0=Pauto)['LscxcpN_f'].full()

        auxsys2 = {
            "Lscxlscxl":Lscxlscxl,
            "Lscxlscul":Lscxlscul,
            "Lscxlscxc":Lscxlscxc,
            "Lscxlscuc":Lscxlscuc,
            "Lsculscul":Lsculscul,
            "Lsculscxc":Lsculscxc,
            "Lsculscuc":Lsculscuc,
            "Lscxcscxc":Lscxcscxc,
            "Lscxcscuc":Lscxcscuc,
            "Lscucscuc":Lscucscuc,
            "Lscxlp":Lscxlp,
            "Lsculp":Lsculp,
            "Lscxcp":Lscxcp,
            "Lscucp":Lscucp
        }

        return auxsys2

    
    def ADMM_SubP3(self,xl_traj,scxl_traj,scxL_traj,ul_traj,scul_traj,scuL_traj,xc_traj,scxc_traj,scxC_traj,uc_traj,scuc_traj,scuC_traj,p,pi):
        scxL_traj_new = np.zeros((self.N+1,self.nxl))
        scuL_traj_new = np.zeros((self.N,self.nul))
        scxC_traj_new = [np.zeros((self.N+1,self.nxi)) for _ in range(int(self.nq))]
        scuC_traj_new = [np.zeros((self.N,self.nui)) for _ in range(int(self.nq))]
        for k in range(self.N):
            scxL_new  = scxL_traj[k,:] + p*(xl_traj[k,:] - scxl_traj[k,:])
            scuL_new  = scuL_traj[k,:] + p*(ul_traj[k,:] - scul_traj[k,:])
            scxL_traj_new[k:k+1,:] = scxL_new
            scuL_traj_new[k:k+1,:] = scuL_new
            for i in range(int(self.nq)):
                scxI_new = scxC_traj[i][k,:] + pi*(xc_traj[i][k,:] - scxc_traj[i][k,:])
                scxC_traj_new[i][k:k+1,:] = scxI_new
                scuI_new = scuC_traj[i][k,:] + pi*(uc_traj[i][k,:] - scuc_traj[i][k,:])
                scuC_traj_new[i][k:k+1,:] = scuI_new
        scxL_new  = scxL_traj[self.N,:] + p*(xl_traj[self.N,:] - scxl_traj[self.N,:])
        scxL_traj_new[self.N:self.N+1,:] = scxL_new
        for i in range(int(self.nq)):
            scxI_new = scxC_traj[i][self.N,:] + pi*(xc_traj[i][self.N,:] - scxc_traj[i][self.N,:])
            scxC_traj_new[i][self.N:self.N+1,:] = scxI_new

        opt_sol3 = {"scxL_traj_new":scxL_traj_new,
                    "scuL_traj_new":scuL_traj_new,
                    "scxC_traj_new":scxC_traj_new,
                    "scuC_traj_new":scuC_traj_new
                    }
        
        return opt_sol3
    
    def system_derivatives_SubP3_ADMM(self):
        scxL_update = self.p*(self.xl - self.scxl)
        scuL_update = self.p*(self.ul - self.scul)
        scxC_update = self.pi*(self.xc - self.scxc)
        scuC_update = self.pi*(self.uc - self.scuc)
        self.dscxL_updatedp    = jacobian(scxL_update,self.P_auto)
        self.dscxL_updatedp_fn = Function('dscxL_updatedp',[self.xl,self.scxl],[self.dscxL_updatedp],['xl0','scxl0'],['dscxL_updatedp_f'])
        self.dscuL_updatedp    = jacobian(scuL_update,self.P_auto)
        self.dscuL_updatedp_fn = Function('dscuL_updatedp',[self.ul,self.scul],[self.dscuL_updatedp],['ul0','scul0'],['dscuL_updatedp_f'])
        self.dscxC_updatedp    = jacobian(scxC_update,self.P_auto)
        self.dscxC_updatedp_fn = Function('dscxC_updatedp',[self.xc,self.scxc],[self.dscxC_updatedp],['xc0','scxc0'],['dscxC_updatedp_f'])
        self.dscuC_updatedp    = jacobian(scuC_update,self.P_auto)
        self.dscuC_updatedp_fn = Function('dscuC_updatedp',[self.uc,self.scuc],[self.dscuC_updatedp],['uc0','scuc0'],['dscuC_updatedp_f'])


    def Get_AuxSys_SubP3(self,opt_sol1_l,opt_sol1_c,opt_sol2):
        xl      = opt_sol1_l['xl_traj']
        ul      = opt_sol1_l['ul_traj']
        xc_list = opt_sol1_c['xc_traj']
        uc_list = opt_sol1_c['uc_traj']
        scxl    = opt_sol2['scxl_traj']
        scul    = opt_sol2['scul_traj']
        scxc_list    = opt_sol2['scxc_traj']
        scuc_list    = opt_sol2['scuc_traj']
        dscxL_updatedp = (self.N+1)*[np.zeros((self.nxl,self.n_Pauto))]
        dscuL_updatedp = self.N*[np.zeros((self.nul,self.n_Pauto))]
        dscxC_updatedp = (self.N+1)*[np.zeros((self.nxi*int(self.nq),self.n_Pauto))]
        dscuC_updatedp = self.N*[np.zeros((self.nui*int(self.nq),self.n_Pauto))]
        for k in range(self.N):
            xl_k     = xl[k,:]
            ul_k     = ul[k,:]
            xc_k     = np.concatenate([xc_list[i][k,:] for i in range(int(self.nq))])
            uc_k     = np.concatenate([uc_list[i][k,:] for i in range(int(self.nq))])
            scxl_k   = scxl[k,:]
            scul_k   = scul[k,:]
            scxc_k   = np.concatenate([scxc_list[i][k,:] for i in range(int(self.nq))])
            scuc_k   = np.concatenate([scuc_list[i][k,:] for i in range(int(self.nq))])
            dscxL_updatedp[k] = self.dscxL_updatedp_fn(xl0=xl_k,scxl0=scxl_k)['dscxL_updatedp_f'].full()
            dscuL_updatedp[k] = self.dscuL_updatedp_fn(ul0=ul_k,scul0=scul_k)['dscuL_updatedp_f'].full()
            dscxC_updatedp[k] = self.dscxC_updatedp_fn(xc0=xc_k,scxc0=scxc_k)['dscxC_updatedp_f'].full()
            dscuC_updatedp[k] = self.dscuC_updatedp_fn(uc0=uc_k,scuc0=scuc_k)['dscuC_updatedp_f'].full()
        xl_N     = xl[-1,:]
        scxl_N   = scxl[-1,:]
        xc_N     = np.concatenate([xc_list[i][-1,:] for i in range(int(self.nq))])
        scxc_N   = np.concatenate([scxc_list[i][-1,:] for i in range(int(self.nq))])
        dscxL_updatedp[self.N]= self.dscxL_updatedp_fn(xl0=xl_N,scxl0=scxl_N)['dscxL_updatedp_f'].full()
        dscxC_updatedp[self.N]= self.dscxC_updatedp_fn(xc0=xc_N,scxc0=scxc_N)['dscxC_updatedp_f'].full()

        auxSys3 = {
            "dscxL_updatedp":dscxL_updatedp,
            "dscuL_updatedp":dscuL_updatedp,
            "dscxC_updatedp":dscxC_updatedp,
            "dscuC_updatedp":dscuC_updatedp
        }
        return auxSys3


    
    def Sigmoid_fun(self,x):
        sigmoid = 1/(1+exp(-x))
        return sigmoid
    
    
    def adaptive_penlaty(self,r,s,p_prev):
        eta = 0.01
        e   = 1e-5
        rou_min, rou_max = 1e-2, 1e2
        rou_new = p_prev*np.exp(eta*(r/(s+e)-1))
        rou = np.clip(rou_new,rou_min, rou_max)

        return rou


    def ADMM_forward_MPC(self,Ref_xl,Ref_ul,ref_xq,ref_uq,xl_fb,xq_fb,paral,paraC):
        # initial guess of the safe copy variable trajectories
        scxl_traj_tp = np.zeros(((self.N+1)*self.nxl))
        scul_traj_tp = Ref_ul
        for k in range(self.N):
            scxl_traj_tp[k*self.nxl:(k+1)*self.nxl] = np.reshape(self.model_l_fn(xl0=scxl_traj_tp[k*self.nxl:(k+1)*self.nxl],ul0=Ref_ul[k*self.nul:(k+1)*self.nul])['mdynlf'].full(),self.nxl)
        scxl_traj_tp[self.N*self.nxl:(self.N+1)*self.nxl] = Ref_xl[self.N*self.nxl:(self.N+1)*self.nxl]
        scxc_traj_tp = [np.zeros((self.N+1)*self.nxi) for _ in range(int(self.nq))]
        scuc_traj_tp = [np.zeros(self.N*self.nui)  for _ in range(int(self.nq))]
        for i in range(int(self.nq)):
            for k in range(self.N):
                scxc_traj_tp[i][k*self.nxi:(k+1)*self.nxi] = np.reshape(self.model_i_fn(xi0=scxc_traj_tp[i][k*self.nxi:(k+1)*self.nxi],ui0=ref_uq[i*self.nui:(i+1)*self.nui])['mdynif'].full(),self.nxi)   #ref_xq[i][k*self.nxi:(k+1)*self.nxi]
                scuc_traj_tp[i][k*self.nui:(k+1)*self.nui] = ref_uq[i*self.nui:(i+1)*self.nui]
            scxc_traj_tp[i][self.N*self.nxi:(self.N+1)*self.nxi] = ref_xq[i][self.N*self.nxi:(self.N+1)*self.nxi]
        # initial guess of the Lagrangian multiplier trajectories
        scxL_traj_tp = np.zeros(((self.N+1)*self.nxl)) # 1D array
        scuL_traj_tp = np.zeros((self.N*self.nul)) # 1D array
        scxC_traj_tp = [np.zeros(((self.N+1)*self.nxi)) for _ in range(int(self.nq))] # list of 1D array
        scuC_traj_tp = [np.zeros(self.N*self.nui)  for _ in range(int(self.nq))]
        scxL_traj    = np.zeros((self.N+1,self.nxl))
        scuL_traj    = np.zeros((self.N,self.nul))
        scxC_traj    = [np.zeros((self.N+1,self.nxi)) for _ in range(int(self.nq))]
        scuC_traj    = [np.zeros((self.N,self.nui))  for _ in range(int(self.nq))]
        max_iter     = 5 # 5 for training
        e_tol        = 1e-2 # 1e-2 for training
        Opt_Sol1_l = []
        Opt_Sol1_cddp = []
        Opt_Sol1_c = []
        Opt_Sol2   = []
        Opt_Sol3   = []
        self.max_iter_ADMM = 2
        for i_ADMM in range(self.max_iter_ADMM):
            Paral   = np.concatenate((xl_fb,Ref_xl))
            Paral   = np.concatenate((Paral,Ref_ul))
            Paral   = np.concatenate((Paral,scxl_traj_tp))
            Paral   = np.concatenate((Paral,scxL_traj_tp))
            Paral   = np.concatenate((Paral,scul_traj_tp))
            Paral   = np.concatenate((Paral,scuL_traj_tp))
            Paral   = np.concatenate((Paral,paral))
            # solve Subproblem 1-load (dynamic)
            start_time = TM.time()
            # opt_sol = self.MPC_Load_Planning_SubP1(Paral)
            opt_sol_l = self.DDP_Load_ADMM_Subp1(xl_fb,Ref_xl,Ref_ul,paral,scxl_traj_tp,scul_traj_tp,scxL_traj_tp,scuL_traj_tp,max_iter,e_tol)
            mpctime = (TM.time() - start_time)*1000
            print("subprblem1_load:--- %s ms ---" % format(mpctime,'.2f'))
            xl_traj = opt_sol_l['xl_traj']
            ul_traj = opt_sol_l['ul_traj']
            Kfbl_traj  = opt_sol_l['K_FB']
            xl_traj_tp = np.reshape(xl_traj,(self.N+1)*self.nxl)
            ul_traj_tp = np.reshape(ul_traj,self.N*self.nul)
            # solve Subproblem 1-cable (dynamic, across n cables)
            ParaC   = []
            for i in range(int(self.nq)):
                xi_fb  = xq_fb[i*self.nxi:(i+1)*self.nxi]
                ref_xi = ref_xq[i]
                ref_ui = ref_uq[i*self.nui:(i+1)*self.nui]
                scxi_traj = scxc_traj_tp[i]
                scxI_traj = scxC_traj_tp[i]
                scui_traj = scuc_traj_tp[i]
                scuI_traj = scuC_traj_tp[i]
                parai     = np.concatenate((xi_fb,ref_xi))
                parai     = np.concatenate((parai,ref_ui))
                parai     = np.concatenate((parai,scxi_traj))
                parai     = np.concatenate((parai,scxI_traj))
                parai     = np.concatenate((parai,scui_traj))
                parai     = np.concatenate((parai,scuI_traj))
                parai     = np.concatenate((parai,paraC))
                ParaC  += [parai]
            start_time = TM.time()
            opt_solc, OPt_sol_c = self.MPC_Cable_DDP_Planning_SubP1(ParaC)
            mpctime = (TM.time() - start_time)*1000
            print("subproblem1_cables:--- %s ms ---" % format(mpctime,'.2f'))
            xc_traj  = opt_solc['xc_traj']
            uc_traj  = opt_solc['uc_traj']
            # solve Subproblem 2 (static, N independent steps, each step is a centralized problem)
            xc_traj_tp2 = np.zeros((self.N+1)*int(self.nq)*self.nxi)
            uc_traj_tp2 = np.zeros(self.N*int(self.nq)*self.nui)
            for k in range(self.N):
                xc_traj_k   = np.zeros(int(self.nq)*self.nxi)
                uc_traj_k   = np.zeros(int(self.nq)*self.nui)
                for i in range(int(self.nq)):
                    xc_traj_k[i*self.nxi:(i+1)*self.nxi] = xc_traj[i][k,:]
                    uc_traj_k[i*self.nui:(i+1)*self.nui] = uc_traj[i][k,:]
                xc_traj_tp2[k*int(self.nq)*self.nxi:(k+1)*int(self.nq)*self.nxi] = xc_traj_k
                uc_traj_tp2[k*int(self.nq)*self.nui:(k+1)*int(self.nq)*self.nui] = uc_traj_k
            xc_traj_N   = np.zeros(int(self.nq)*self.nxi)
            for i in range(int(self.nq)):
                xc_traj_N[i*self.nxi:(i+1)*self.nxi] = xc_traj[i][self.N,:]
            xc_traj_tp2[self.N*int(self.nq)*self.nxi:(self.N+1)*int(self.nq)*self.nxi] = xc_traj_N
            scxC_traj_tp2 = np.zeros((self.N+1)*int(self.nq)*self.nxi)
            ref_xc_tp2    = np.zeros((self.N+1)*int(self.nq)*self.nxi)
            scuC_traj_tp2 = np.zeros(self.N*int(self.nq)*self.nui)
            for k in range(self.N):
                scxC_traj_k = np.zeros(int(self.nq)*self.nxi)
                ref_xc_k    = np.zeros(int(self.nq)*self.nxi)
                scuC_traj_k = np.zeros(int(self.nq)*self.nui)
                for i in range(int(self.nq)):
                    scxC_traj_k[i*self.nxi:(i+1)*self.nxi] = scxC_traj_tp[i][k*self.nxi:(k+1)*self.nxi]
                    ref_xc_k[i*self.nxi:(i+1)*self.nxi]    = ref_xq[i][k*self.nxi:(k+1)*self.nxi]
                    scuC_traj_k[i*self.nui:(i+1)*self.nui] = scuC_traj_tp[i][k*self.nui:(k+1)*self.nui]
                scxC_traj_tp2[k*int(self.nq)*self.nxi:(k+1)*int(self.nq)*self.nxi] = scxC_traj_k
                ref_xc_tp2[k*int(self.nq)*self.nxi:(k+1)*int(self.nq)*self.nxi]    = ref_xc_k
                scuC_traj_tp2[k*int(self.nq)*self.nui:(k+1)*int(self.nq)*self.nui] = scuC_traj_k
            scxC_traj_N   = np.zeros(int(self.nq)*self.nxi)
            ref_xc_N      = np.zeros(int(self.nq)*self.nxi)
            for i in range(int(self.nq)):
                scxC_traj_N[i*self.nxi:(i+1)*self.nxi] = scxC_traj_tp[i][self.N*self.nxi:(self.N+1)*self.nxi]
                ref_xc_N[i*self.nxi:(i+1)*self.nxi]    = ref_xq[i][self.N*self.nxi:(self.N+1)*self.nxi]
            scxC_traj_tp2[self.N*int(self.nq)*self.nxi:(self.N+1)*int(self.nq)*self.nxi] = scxC_traj_N
            ref_xc_tp2[self.N*int(self.nq)*self.nxi:(self.N+1)*int(self.nq)*self.nxi]    = ref_xc_N
            Para2_cable = np.concatenate((Ref_xl,Ref_ul))
            Para2_cable = np.concatenate((Para2_cable,ref_xc_tp2))
            Para2_cable = np.concatenate((Para2_cable,ref_uq))
            Para2_cable = np.concatenate((Para2_cable,xl_traj_tp))
            Para2_cable = np.concatenate((Para2_cable,scxL_traj_tp))
            Para2_cable = np.concatenate((Para2_cable,ul_traj_tp))
            Para2_cable = np.concatenate((Para2_cable,scuL_traj_tp))
            Para2_cable = np.concatenate((Para2_cable,xc_traj_tp2))
            Para2_cable = np.concatenate((Para2_cable,scxC_traj_tp2))
            Para2_cable = np.concatenate((Para2_cable,uc_traj_tp2))
            Para2_cable = np.concatenate((Para2_cable,scuC_traj_tp2))
            Para2_cable = np.concatenate((Para2_cable,[paral[-1],paraC[-1]]))
            start_time  = TM.time()
            opt_sol2    = self.ADMM_SubP2(Para2_cable)
            mpctime = (TM.time() - start_time)*1000
            print("subproblem2:--- %s ms ---" % format(mpctime,'.2f'))
            scxl_traj   = opt_sol2['scxl_traj']
            scul_traj   = opt_sol2['scul_traj']
            scxc_traj   = opt_sol2['scxc_traj']
            scuc_traj   = opt_sol2['scuc_traj']
            # solve Subproblem 3
            opt_sol3    = self.ADMM_SubP3(xl_traj,scxl_traj,scxL_traj,ul_traj,scul_traj,scuL_traj,xc_traj,scxc_traj,scxC_traj,uc_traj,scuc_traj,scuC_traj,paral[-1],paraC[-1])
            scxL_traj   = opt_sol3['scxL_traj_new']
            scuL_traj   = opt_sol3['scuL_traj_new']
            scxC_traj   = opt_sol3['scxC_traj_new']
            scuC_traj   = opt_sol3['scuC_traj_new']
            # update trajectories
            scxl_traj_tp_new  = np.reshape(scxl_traj,(self.N+1)*self.nxl) # for Subproblem 1-load
            scul_traj_tp_new  = np.reshape(scul_traj,self.N*self.nul) # for Subproblem 1-load
            scxL_traj_tp  = np.reshape(scxL_traj,(self.N+1)*self.nxl) # for Subproblem 1-load and 2
            scuL_traj_tp  = np.reshape(scuL_traj,self.N*self.nul) # for Subproblem 1-load and 2
            xc_traj_tp    = [np.zeros((self.N+1)*self.nxi)  for _ in range(int(self.nq))] # for Subproblem 2-cable and 3
            uc_traj_tp    = [np.zeros(self.N*self.nui)  for _ in range(int(self.nq))]     # for Subproblem 2-cable and 3
            scxc_traj_tp_new  = [np.zeros((self.N+1)*self.nxi)  for _ in range(int(self.nq))] # for Subproblem 1-cable
            scxC_traj_tp  = [np.zeros((self.N+1)*self.nxi)  for _ in range(int(self.nq))] # for Subproblem 1-cable and 2
            scuc_traj_tp_new  = [np.zeros(self.N*self.nui)  for _ in range(int(self.nq))] # for Subproblem 1-cable
            scuC_traj_tp  = [np.zeros(self.N*self.nui)  for _ in range(int(self.nq))] # for Subproblem 1-cable and 2
            r_xc = [] # primal residual of cables' states
            r_uc = [] # primal residual of cables' controls
            s_xc = [] # dual residual of cables' states
            s_uc = [] # dual residual of cables' contorls
            for i in range(int(self.nq)):
                for k in range(self.N):
                    xc_traj_tp[i][k*self.nxi:(k+1)*self.nxi]       = xc_traj[i][k,:]
                    uc_traj_tp[i][k*self.nui:(k+1)*self.nui]       = uc_traj[i][k,:]
                    scxc_traj_tp_new[i][k*self.nxi:(k+1)*self.nxi] = scxc_traj[i][k,:]
                    scxC_traj_tp[i][k*self.nxi:(k+1)*self.nxi]     = scxC_traj[i][k,:]
                    scuc_traj_tp_new[i][k*self.nui:(k+1)*self.nui] = scuc_traj[i][k,:]
                    scuC_traj_tp[i][k*self.nui:(k+1)*self.nui]     = scuC_traj[i][k,:]
                xc_traj_tp[i][self.N*self.nxi:(self.N+1)*self.nxi]       = xc_traj[i][self.N,:]
                scxc_traj_tp_new[i][self.N*self.nxi:(self.N+1)*self.nxi] = scxc_traj[i][self.N,:]
                scxC_traj_tp[i][self.N*self.nxi:(self.N+1)*self.nxi]     = scxC_traj[i][self.N,:]
                r_xc += [LA.norm(xc_traj_tp[i]-scxc_traj_tp_new[i])]
                s_xc += [paral[-1]*LA.norm(scxc_traj_tp_new[i]-scxc_traj_tp[i])]
                r_uc += [LA.norm(uc_traj_tp[i]-scuc_traj_tp_new[i])]
                s_uc += [paral[-1]*LA.norm(scuc_traj_tp_new[i]-scuc_traj_tp[i])]


            # residuals
            r_xl = LA.norm(xl_traj_tp - scxl_traj_tp_new)
            r_ul = LA.norm(ul_traj_tp - scul_traj_tp_new)
            s_xl = paral[-1]*LA.norm(scxl_traj_tp_new - scxl_traj_tp)
            s_ul = paral[-1]*LA.norm(scul_traj_tp_new - scul_traj_tp)
            # print('ADMM_iteration=',i_ADMM,'p=',paral[-1],'r_xl=',r_xl,'r_ul=',r_ul,'s_xl=',s_xl,'s_ul=',s_ul)
            # for i in range(int(self.nq)):
            #     print('ADMM_iteration=',i_ADMM,'r_xc_'+str(i+1)+'=',r_xc[i],'s_xc_'+str(i+1)+'=',s_xc[i])
            #     print('ADMM_iteration=',i_ADMM,'r_uc_'+str(i+1)+'=',r_uc[i],'s_uc_'+str(i+1)+'=',s_uc[i])
            # update
            scxl_traj_tp = scxl_traj_tp_new
            scul_traj_tp = scul_traj_tp_new
            scxc_traj_tp = scxc_traj_tp_new
            scuc_traj_tp = scuc_traj_tp_new

            Opt_Sol1_l += [opt_sol_l]
            Opt_Sol1_cddp += [OPt_sol_c]
            Opt_Sol1_c += [opt_solc]
            Opt_Sol2   += [opt_sol2]
            Opt_Sol3   += [opt_sol3]
        
        opt_sol = {"xl_traj":xl_traj,
                   "Kfbl_traj":Kfbl_traj,
                   "scxl_traj":scxl_traj,
                   "xc_traj":xc_traj,
                   "uc_traj":uc_traj,
                   "scxc_traj":scxc_traj,
                    }
        
        return opt_sol, Opt_Sol1_l, Opt_Sol1_cddp, Opt_Sol1_c, Opt_Sol2, Opt_Sol3
    

            
    def DDP_Load_Gradient(self,opt_sol,auxSysl, scxl_grad, scxL_grad, scul_grad, scuL_grad, p):
        Quuinv, Qxu, K_fb, F, G  = opt_sol['Quu_inv'], opt_sol['Qxu'], opt_sol['K_FB'], opt_sol['Fx'], opt_sol['Fu']
        HxNp, Hxp, Hup = auxSysl['HxNp'], auxSysl['Hxp'], auxSysl['Hup']
        S          = (self.N+1)*[np.zeros((self.nxl,self.n_Pauto))]
        S[self.N]  = HxNp + scxL_grad[self.N] - p*scxl_grad[self.N] # reduced to HxNp only in the single-agent problem
        v_FF       = self.N*[np.zeros((self.nul,self.n_Pauto))]
        xl_grad    = (self.N+1)*[np.zeros((self.nxl,self.n_Pauto))] 
        ul_grad    = self.N*[np.zeros((self.nul,self.n_Pauto))]
        #-------Backward recursion-------#         
        for k in reversed(range(self.N)): 
            Hxp_k    = Hxp[k] + scxL_grad[k] - p*scxl_grad[k]
            Hup_k    = Hup[k] + scuL_grad[k] - p*scul_grad[k]
            v_FF[k]  = -Quuinv[k]@(Hup_k + G[k].T@S[k+1])
            S[k]     = Hxp_k + F[k].T@S[k+1] + Qxu[k]@v_FF[k] # s[0] not used
        #-------Foreward recursion-------#
        for k in range(self.N):
            ul_grad[k]  = K_fb[k]@xl_grad[k]+v_FF[k]
            xl_grad[k+1]= F[k]@xl_grad[k]+G[k]@ul_grad[k]

        grad_outl ={"xl_grad":xl_grad,
                   "ul_grad":ul_grad
                }
        
        return grad_outl
    

    def DDP_Cable_Gradient(self,opt_sol,auxSysi, scxi_grad, scxI_grad, scui_grad, scuI_grad, pi):
        Quuinv, Qxu, K_fb, F, G  = opt_sol['Quu_inv'], opt_sol['Qxu'], opt_sol['K_FB'], opt_sol['Fx'], opt_sol['Fu']
        HxNp, Hxp, Hup = auxSysi['HxNp'], auxSysi['Hxp'], auxSysi['Hup']
        S          = (self.N+1)*[np.zeros((self.nxi,self.n_Pauto))]
        S[self.N]  = HxNp + scxI_grad[self.N] - pi*scxi_grad[self.N] # reduced to HxNp only in the single-agent problem
        v_FF       = self.N*[np.zeros((self.nui,self.n_Pauto))]
        xi_grad    = (self.N+1)*[np.zeros((self.nxi,self.n_Pauto))] 
        ui_grad    = self.N*[np.zeros((self.nui,self.n_Pauto))]
        #-------Backward recursion-------#         
        for k in reversed(range(self.N)): 
            Hxp_k    = Hxp[k] + scxI_grad[k] - pi*scxi_grad[k]
            Hup_k    = Hup[k] + scuI_grad[k] - pi*scui_grad[k]
            v_FF[k]  = -Quuinv[k]@(Hup_k + G[k].T@S[k+1])
            S[k]     = Hxp_k + F[k].T@S[k+1] + Qxu[k]@v_FF[k] # s[0] not used
        #-------Foreward recursion-------#
        for k in range(self.N):
            ui_grad[k]  = K_fb[k]@xi_grad[k]+v_FF[k]
            xi_grad[k+1]= F[k]@xi_grad[k]+G[k]@ui_grad[k]

        grad_outi ={"xi_grad":xi_grad,
                   "ui_grad":ui_grad
                }
        
        return grad_outi
    
    def min_eigval(self,H):
        H = np.asarray(H, dtype=np.float64)
        H = 0.5 * (H + H.T)  # reinforce symmetry
        # H += 1e-10 * np.eye(H.shape[0])  # regularize to avoid numerical issues
        try:
            val, _ = eigsh(H, k=1, which='SA', tol=1e-4, maxiter=1000)
            return float(val[0])
        except ArpackNoConvergence:
            # Fallback: slow but always works
            print("Warning: eigsh did not converge. Falling back to eigh.")
            val = eigh(H, eigvals_only=True, subset_by_index=[0, 0])
            return float(val[0])


    def SubP2_Gradient(self,auxSys2,grad_outl,grad_outc,scxL_grad,scuL_grad,scxC_grad,scuC_grad,p,pi):
        xl_grad   = grad_outl['xl_grad']
        ul_grad   = grad_outl['ul_grad']
        Lscxlscxl = auxSys2['Lscxlscxl']
        Lscxlscul = auxSys2['Lscxlscul']
        Lscxlscxc = auxSys2['Lscxlscxc']
        Lscxlscuc = auxSys2['Lscxlscuc']
        Lsculscul = auxSys2['Lsculscul']
        Lsculscxc = auxSys2['Lsculscxc']
        Lsculscuc = auxSys2['Lsculscuc']
        Lscxcscxc = auxSys2['Lscxcscxc']
        Lscxcscuc = auxSys2['Lscxcscuc']
        Lscucscuc = auxSys2['Lscucscuc']
        Lscxlp    = auxSys2['Lscxlp']
        Lsculp    = auxSys2['Lsculp']
        Lscxcp    = auxSys2['Lscxcp']
        Lscucp    = auxSys2['Lscucp']
        scxl_grad = (self.N+1)*[np.zeros((self.nxl,self.n_Pauto))]
        scul_grad = self.N*[np.zeros((self.nul,self.n_Pauto))]
        scxc_grad = (self.N+1)*[np.zeros((self.nxi*int(self.nq),self.n_Pauto))]
        scuc_grad = self.N*[np.zeros((self.nui*int(self.nq),self.n_Pauto))]

        for k in range(self.N):
            L_hessian_k = vertcat(
                                horzcat(Lscxlscxl[k],   Lscxlscul[k],   Lscxlscxc[k],   Lscxlscuc[k]),
                                horzcat(Lscxlscul[k].T, Lsculscul[k],   Lsculscxc[k],   Lsculscuc[k]),
                                horzcat(Lscxlscxc[k].T, Lsculscxc[k].T, Lscxcscxc[k],   Lscxcscuc[k]),
                                horzcat(Lscxlscuc[k].T, Lsculscuc[k].T, Lscxcscuc[k].T, Lscucscuc[k])
                                )
            xl_grad_k   = xl_grad[k]
            ul_grad_k   = ul_grad[k]
            scxL_grad_k = scxL_grad[k]
            scuL_grad_k = scuL_grad[k]
            scxC_grad_k = scxC_grad[k]
            scuC_grad_k = scuC_grad[k]
            xc_grad_k   = grad_outc[0]['xi_grad'][k]
            uc_grad_k   = grad_outc[0]['ui_grad'][k]
            for i in range(1,int(self.nq)):
                xc_grad_k = np.vstack((xc_grad_k,grad_outc[i]['xi_grad'][k]))
                uc_grad_k = np.vstack((uc_grad_k,grad_outc[i]['ui_grad'][k]))
            L_trajp_k   = vertcat(
                                horzcat(Lscxlp[k] - p*xl_grad_k - scxL_grad_k),
                                horzcat(Lsculp[k] - p*ul_grad_k - scuL_grad_k),
                                horzcat(Lscxcp[k] - pi*xc_grad_k - scxC_grad_k),
                                horzcat(Lscucp[k] - pi*uc_grad_k - scuC_grad_k)
                                )
            
            L_hessian_k += 1e-5 * np.eye(L_hessian_k.shape[0]) # regularization to enhance numerical stability
            min_eig = np.min(LA.eigvalsh(L_hessian_k))
            # min_eig = self.min_eigval(L_hessian_k)
            if min_eig<0:
                reg = -min_eig + 1e-5
                print('negative_min_eigen_detected!')
            else:
                reg = 0
            
            inv_L_hessian_k = LA.inv(L_hessian_k + reg*np.identity(self.nxl+self.nul+self.nxi*int(self.nq)+self.nui*int(self.nq)))# regularization to enhance numerical stability
            grad_subp2_k    = -inv_L_hessian_k@L_trajp_k
            scxl_grad[k]    = grad_subp2_k[0:self.nxl,:]
            scul_grad[k]    = grad_subp2_k[self.nxl:(self.nxl+self.nul),:]
            scxc_grad[k]    = grad_subp2_k[(self.nxl+self.nul):(self.nxl+self.nul+self.nxi*int(self.nq)),:]
            scuc_grad[k]    = grad_subp2_k[(self.nxl+self.nul+self.nxi*int(self.nq)):(self.nxl+self.nul+self.nxi*int(self.nq)+self.nui*int(self.nq)),:]
        # terminal gradients
        L_hessian_N = vertcat(
                                horzcat(Lscxlscxl[self.N],   Lscxlscxc[self.N]),
                                horzcat(Lscxlscxc[self.N].T, Lscxcscxc[self.N])
                                )
        xl_grad_N   = xl_grad[self.N]
        xc_grad_N   = grad_outc[0]['xi_grad'][self.N]
        for i in range(1,int(self.nq)):
            xc_grad_N = np.vstack((xc_grad_N,grad_outc[i]['xi_grad'][self.N]))
        scxL_grad_N = scxL_grad[self.N]
        scxC_grad_N = scxC_grad[self.N]
        L_trajp_N   = vertcat(
                            horzcat(Lscxlp[self.N] - p*xl_grad_N - scxL_grad_N),
                            horzcat(Lscxcp[self.N] - pi*xc_grad_N - scxC_grad_N)
                            )
        inv_L_hessian_N = LA.inv(L_hessian_N)
        grad_subp2_N    = -inv_L_hessian_N@L_trajp_N
        scxl_grad[self.N] = grad_subp2_N[0:self.nxl,:]
        scxc_grad[self.N] = grad_subp2_N[self.nxl:(self.nxl+self.nxi*int(self.nq)),:]

        grad_out2 = {
                    "scxl_grad":scxl_grad,
                    "scul_grad":scul_grad,
                    "scxc_grad":scxc_grad,
                    "scuc_grad":scuc_grad
                    }
        
        return grad_out2
    

    def SubP3_Gradient(self,auxSys3,grad_outl,grad_outc,grad_out2,scxL_grad,scuL_grad,scxC_grad,scuC_grad,p):
        xl_grad         = grad_outl['xl_grad']
        ul_grad         = grad_outl['ul_grad']
        scxl_grad       = grad_out2['scxl_grad']
        scul_grad       = grad_out2['scul_grad']
        scxc_grad       = grad_out2['scxc_grad']
        scuc_grad       = grad_out2['scuc_grad']
        dscxL_updatedp  = auxSys3['dscxL_updatedp']
        dscuL_updatedp  = auxSys3['dscuL_updatedp']
        dscxC_updatedp  = auxSys3['dscxC_updatedp']
        dscuC_updatedp  = auxSys3['dscuC_updatedp']
        scxL_grad_new   = (self.N+1)*[np.zeros((self.nxl,self.n_Pauto))]
        scuL_grad_new   = self.N*[np.zeros((self.nul,self.n_Pauto))]
        scxC_grad_new   = (self.N+1)*[np.zeros((self.nxi*int(self.nq),self.n_Pauto))]
        scuC_grad_new   = self.N*[np.zeros((self.nui*int(self.nq),self.n_Pauto))]
        for k in range(self.N):
            xc_grad_k   = grad_outc[0]['xi_grad'][k]
            uc_grad_k   = grad_outc[0]['ui_grad'][k]
            for i in range(1,int(self.nq)):
                xc_grad_k = np.vstack((xc_grad_k,grad_outc[i]['xi_grad'][k]))
                uc_grad_k = np.vstack((uc_grad_k,grad_outc[i]['ui_grad'][k]))
            scxL_grad_new[k] = scxL_grad[k] + p*(xl_grad[k] - scxl_grad[k]) + dscxL_updatedp[k]
            scuL_grad_new[k] = scuL_grad[k] + p*(ul_grad[k] - scul_grad[k]) + dscuL_updatedp[k]
            scxC_grad_new[k] = scxC_grad[k] + p*(xc_grad_k - scxc_grad[k]) + dscxC_updatedp[k]
            scuC_grad_new[k] = scuC_grad[k] + p*(uc_grad_k - scuc_grad[k]) + dscuC_updatedp[k]
        # terminal gradients
        xc_grad_N   = grad_outc[0]['xi_grad'][self.N]
        for i in range(1,int(self.nq)):
            xc_grad_N = np.vstack((xc_grad_N,grad_outc[i]['xi_grad'][self.N]))
        scxL_grad_new[self.N]= scxL_grad[self.N] + p*(xl_grad[self.N] - scxl_grad[self.N]) + dscxL_updatedp[self.N]
        scxC_grad_new[self.N]= scxC_grad[self.N] + p*(xc_grad_N - scxc_grad[self.N]) + dscxC_updatedp[self.N]

        grad_out3 = {
            "scxL_grad":scxL_grad_new,
            "scuL_grad":scuL_grad_new,
            "scxC_grad":scxC_grad_new,
            "scuC_grad":scuC_grad_new
        }

        return grad_out3
    

    def ADMM_Gradient_Solver(self,Opt_Sol1_l, Opt_Sol1_cddp, Opt_Sol1_c, Opt_Sol2, Opt_Sol3, Ref_xl, Ref_ul, ref_xq, ref_uq, weight1, weight2):
        # initialize the gradient trajectories of SubP2 and SubP3
        scxl_grad = (self.N+1)*[np.zeros((self.nxl,self.n_Pauto))]
        scul_grad = self.N*[np.zeros((self.nul,self.n_Pauto))]
        scxL_grad = (self.N+1)*[np.zeros((self.nxl,self.n_Pauto))]
        scuL_grad = self.N*[np.zeros((self.nul,self.n_Pauto))]
        scxc_grad = (self.N+1)*[np.zeros((self.nxi*int(self.nq),self.n_Pauto))]
        scuc_grad = self.N*[np.zeros((self.nui*int(self.nq),self.n_Pauto))]
        scxC_grad = (self.N+1)*[np.zeros((self.nxi*int(self.nq),self.n_Pauto))]
        scuC_grad = self.N*[np.zeros((self.nui*int(self.nq),self.n_Pauto))]
        # initial trajectories, same as those used in the ADMM recursion in the forward pass
        scxl      = np.zeros((self.N+1,self.nxl))
        scul      = np.zeros((self.N,self.nul))
        for k in range(self.N):
            scul[k,:] = Ref_ul[k*self.nul:(k+1)*self.nul]
            scxl[k,:] = np.reshape(self.model_l_fn(xl0=scxl[k,:],ul0=scul[k,:])['mdynlf'].full(),self.nxl)
        scxl[self.N,:]= Ref_xl[self.N*self.nxl:(self.N+1)*self.nxl]
        scxc      = [np.zeros((self.N+1,self.nxi)) for _ in range(int(self.nq))] 
        scuc      = [np.zeros((self.N,self.nui)) for _ in range(int(self.nq))] 
        for i in range(int(self.nq)):
            for k in range(self.N):
                scuc[i][k,:] = ref_uq[i*self.nui:(i+1)*self.nui]
                scxc[i][k,:] = np.reshape(self.model_i_fn(xi0=scxc[i][k,:],ui0=scuc[i][k,:])['mdynif'].full(),self.nxi)
            scxc[i][self.N,:] = ref_xq[i][self.N*self.nxi:(self.N+1)*self.nxi]
        scxL      = np.zeros((self.N+1,self.nxl))
        scuL      = np.zeros((self.N,self.nul))
        scxC      = [np.zeros((self.N+1,self.nxi)) for _ in range(int(self.nq))] 
        scuC      = [np.zeros((self.N,self.nui)) for _ in range(int(self.nq))] 
        # lists for storing gradient trajectories
        Grad_Out1l = []
        Grad_Out1c = []
        Grad_Out2 = []
        Grad_Out3 = []
        # self.max_iter_ADMM = 2
        Pauto     = np.concatenate((weight1,weight2))
        for i_ADMM in range(self.max_iter_ADMM):
            # gradients of Subproblem1
            opt_sol   = Opt_Sol1_l[i_ADMM]
            auxSysl   = self.Get_AuxSys_DDP_Load(opt_sol,Ref_xl,Ref_ul,scxl,scul,scxL,scuL,weight1)
            grad_outl = self.DDP_Load_Gradient(opt_sol,auxSysl, scxl_grad, scxL_grad, scul_grad, scuL_grad, weight1[-1])
            grad_outc = []
            for i in range(int(self.nq)):
                opt_solc  = Opt_Sol1_cddp[i_ADMM]
                Ref_xi    = ref_xq[i]
                Ref_ui    = ref_uq[i*self.nui:(i+1)*self.nui]
                scxi      = scxc[i]
                scui      = scuc[i]
                scxI      = scxC[i]
                scuI      = scuC[i]
                auxSysi   = self.Get_AuxSys_DDP_Cable(opt_solc[i],Ref_xi,Ref_ui,scxi,scui,scxI,scuI,weight2)
                scxi_grad = (self.N+1)*[np.zeros((self.nxi, self.n_Pauto))]
                scxI_grad = (self.N+1)*[np.zeros((self.nxi, self.n_Pauto))]
                scui_grad = self.N*[np.zeros((self.nui, self.n_Pauto))]
                scuI_grad = self.N*[np.zeros((self.nui, self.n_Pauto))]
                for k in range(self.N):
                    scxi_grad[k] = np.reshape(scxc_grad[k][i*self.nxi:(i+1)*self.nxi,:],(self.nxi,self.n_Pauto))
                    scxI_grad[k] = np.reshape(scxC_grad[k][i*self.nxi:(i+1)*self.nxi,:],(self.nxi,self.n_Pauto))
                    scui_grad[k] = np.reshape(scuc_grad[k][i*self.nui:(i+1)*self.nui,:],(self.nui,self.n_Pauto))
                    scuI_grad[k] = np.reshape(scuC_grad[k][i*self.nui:(i+1)*self.nui,:],(self.nui,self.n_Pauto))
                scxi_grad[self.N]= np.reshape(scxc_grad[self.N][i*self.nxi:(i+1)*self.nxi,:],(self.nxi,self.n_Pauto))
                scxI_grad[self.N]= np.reshape(scxC_grad[self.N][i*self.nxi:(i+1)*self.nxi,:],(self.nxi,self.n_Pauto))
                grad_outi = self.DDP_Cable_Gradient(opt_solc[i],auxSysi, scxi_grad, scxI_grad, scui_grad, scuI_grad, weight2[-1])
                grad_outc+= [grad_outi]
            # gradients of Subproblem2
            opt_sol1_c = Opt_Sol1_c[i_ADMM]
            opt_sol2   = Opt_Sol2[i_ADMM]
            auxSys2    = self.Get_AuxSys_SubP2(opt_sol,opt_sol1_c,opt_sol2,scxL,scuL,scxC,scuC,Pauto)
            grad_out2  = self.SubP2_Gradient(auxSys2,grad_outl,grad_outc,scxL_grad,scuL_grad,scxC_grad,scuC_grad,weight1[-1],weight2[-1]) 
            # gradients of Subproblem3
            auxSys3    = self.Get_AuxSys_SubP3(opt_sol,opt_sol1_c,opt_sol2)
            grad_out3  = self.SubP3_Gradient(auxSys3,grad_outl,grad_outc,grad_out2,scxL_grad,scuL_grad,scxC_grad,scuC_grad,weight1[-1])
            # update
            scxl       = opt_sol2['scxl_traj']
            scul       = opt_sol2['scul_traj']
            scxc       = opt_sol2['scxc_traj']
            scuc       = opt_sol2['scuc_traj']
            opt_sol3   = Opt_Sol3[i_ADMM]
            scxL       = opt_sol3['scxL_traj_new']
            scuL       = opt_sol3['scuL_traj_new']
            scxC       = opt_sol3['scxC_traj_new']
            scuC       = opt_sol3['scuC_traj_new']
            scxl_grad  = grad_out2['scxl_grad']
            scul_grad  = grad_out2['scul_grad']
            scxc_grad  = grad_out2['scxc_grad']
            scuc_grad  = grad_out2['scuc_grad']
            scxL_grad  = grad_out3['scxL_grad']
            scuL_grad  = grad_out3['scuL_grad']
            scxC_grad  = grad_out3['scxC_grad']
            scuC_grad  = grad_out3['scuC_grad']
            # save the results
            Grad_Out1l += [grad_outl]
            Grad_Out1c += [grad_outc]
            Grad_Out2 += [grad_out2]
            Grad_Out3 += [grad_out3]
        
        return Grad_Out1l, Grad_Out1c, Grad_Out2, Grad_Out3
    

class Gradient_Solver:
    def __init__(self, sysm_para, horizon, xl, ul, scxl, scul, xi, ui, scxi, scui, P_auto, weight1, weight2):
        self.nxl    = xl.numel()
        self.nul    = ul.numel()
        self.nxi    = xi.numel()
        self.nui    = ui.numel()
        self.n_Pauto= P_auto.numel()
        self.npl    = weight1.numel()
        self.npi    = weight2.numel()
        self.nq     = int(sysm_para[8])
        self.N      = horizon
        self.xl     = xl
        self.ul     = ul
        self.xi     = xi
        self.ui     = ui
        self.scxl   = scxl
        self.scul   = scul
        self.scxi   = scxi
        self.scui   = scui
        self.Pauto  = P_auto
        self.xl_ref = SX.sym('xl_ref',self.nxl)
        self.xi_ref = SX.sym('xi_ref',self.nxi)
        # boundaries of the hyperparameters
        self.p_min  = 1e-2 # small value leads to numerical instability
        self.p_max  = 1e3
        #------------- loss definition -------------#
        # tracking loss
        self.w_track  = 1
        track_error_l = self.xl - self.xl_ref
        track_error_i = self.xi - self.xi_ref
        self.loss_track_l = track_error_l.T@track_error_l
        weight_i      = np.diag(np.array([1,1,1,0,0,0,1,0]))
        self.loss_track_i = track_error_i.T@weight_i@track_error_i
        # primal residual loss
        self.w_rp    = 1
        r_primal_xl  = self.xl - self.scxl
        r_primal_ul  = self.ul - self.scul
        self.loss_rpl = r_primal_xl.T@r_primal_xl + r_primal_ul.T@r_primal_ul
        self.loss_rpl_N = r_primal_xl.T@r_primal_xl
        r_primal_xi  = self.xi - self.scxi
        r_primal_ui  = self.ui - self.scui
        self.loss_rpi = r_primal_xi.T@r_primal_xi + r_primal_ui.T@r_primal_ui
        self.loss_rpi_N = r_primal_xi.T@r_primal_xi

    def Set_Parameters(self,tunable_para,p_min):
        weight       = np.zeros(self.n_Pauto)
        for j in range(self.n_Pauto):
            if (j==2*self.nxl+self.nul):
                weight[j]= p_min + (self.p_max - p_min) * 1/(1+np.exp(-tunable_para[j]))
            elif (j== 2*self.nxl+self.nul+2*self.nxi+self.nui + 1):
                weight[j]= p_min + (self.p_max - p_min) * 1/(1+np.exp(-tunable_para[j]))
            else:
                weight[j]= self.p_min + (self.p_max - self.p_min) * 1/(1+np.exp(-tunable_para[j])) # sigmoid boundedness
        return weight
    
    def Set_Parameters_evaluate(self,tunable_para,p_min):
        weight       = np.zeros(self.n_Pauto)
        for j in range(self.n_Pauto):
            if (j==2*self.nxl+self.nul):
                weight[j]= p_min + (self.p_max - p_min) * 1/(1+np.exp(-tunable_para[j]))
            elif (j== 2*self.nxl+self.nul+2*self.nxi+self.nui + 1):
                weight[j]= p_min + (self.p_max - p_min) * 1/(1+np.exp(-tunable_para[j]))
            else:
                weight[j]= self.p_min + (self.p_max - self.p_min) * 1/(1+np.exp(-tunable_para[j])) # sigmoid boundedness
        return weight

    def Set_Parameters_nn_l(self,tunable_para,pl_min):
        weight       = np.zeros(self.npl)
        for k in range(self.npl):
            if (k==self.npl-1):
                weight[k]= pl_min + (self.p_max - pl_min) * tunable_para[0,k]
            else:
                weight[k]= self.p_min + (self.p_max - self.p_min) * tunable_para[0,k] # sigmoid boundedness
        return weight
    
    def Set_Parameters_nn_i(self,tunable_para,pi_min):
        weight       = np.zeros(self.npi)
        for k in range(self.npi):
            if (k== self.npi-1):
                weight[k]= pi_min + (self.p_max - pi_min) * tunable_para[0,k]
            else:
                weight[k]= self.p_min + (self.p_max - self.p_min) * tunable_para[0,k] # sigmoid boundedness
        return weight
    
    def Set_Parameters_nn_l_evaluate(self,tunable_para,pl_min):
        weight       = np.zeros(self.npl)
        for k in range(self.npl):
            if (k==self.npl-1):
                weight[k]= pl_min + (self.p_max - pl_min) * tunable_para[0,k]
            else:
                weight[k]= self.p_min + (self.p_max - self.p_min) * tunable_para[0,k] # sigmoid boundedness
        return weight
    
    def Set_Parameters_nn_i_evaluate(self,tunable_para,pi_min):
        weight       = np.zeros(self.npi)
        for k in range(self.npi):
            if (k== self.npi-1):
                weight[k]= pi_min + (self.p_max - pi_min) * tunable_para[0,k]
            else:
                weight[k]= self.p_min + (self.p_max - self.p_min) * tunable_para[0,k] # sigmoid boundedness
        return weight

    def ChainRule_Gradient(self,tunable_para,p_min):
        Tunable      = SX.sym('Tp',1,self.n_Pauto)
        Weight       = SX.sym('wp',1,self.n_Pauto)
        for k in range(self.n_Pauto):
            if k==2*self.nxl+self.nul:
                Weight[k]= p_min + (self.p_max - p_min) * 1/(1+exp(-Tunable[k]))
            elif k== 2*self.nxl+self.nul+2*self.nxi+self.nui + 1:
                Weight[k]= p_min + (self.p_max - p_min) * 1/(1+exp(-Tunable[k]))
            else:
                Weight[k]= self.p_min + (self.p_max - self.p_min) * 1/(1 + exp(-Tunable[k])) # sigmoid boundedness
        dWdT         = jacobian(Weight,Tunable)
        dWdT_fn      = Function('dWdT',[Tunable],[dWdT],['Tp0'],['dWdT_f'])
        weight_grad  = dWdT_fn(Tp0=tunable_para)['dWdT_f'].full()

        return weight_grad
    
    def ChainRule_Gradient_nn_l(self,tunable_para,pl_min):
        Tunable      = SX.sym('Tp',1,self.npl)
        Weight       = SX.sym('wp',1,self.npl)
        for k in range(self.npl):
            if k==2*self.nxl+self.nul:
                Weight[k]= pl_min + (self.p_max - pl_min) * Tunable[k]
            else:
                Weight[k]= self.p_min + (self.p_max - self.p_min) * Tunable[k] # sigmoid boundedness
        dWdT         = jacobian(Weight,Tunable)
        dWdT_fn      = Function('dWdT',[Tunable],[dWdT],['Tp0'],['dWdT_f'])
        weight_grad  = dWdT_fn(Tp0=tunable_para)['dWdT_f'].full()
        return weight_grad
    
    def ChainRule_Gradient_nn_i(self,tunable_para,pi_min):
        Tunable      = SX.sym('Tp',1,self.npi)
        Weight       = SX.sym('wp',1,self.npi)
        for k in range(self.npi):
            if k== 2*self.nxi+self.nui:
                Weight[k]= pi_min + (self.p_max - pi_min) * Tunable[k]
            else:
                Weight[k]= self.p_min + (self.p_max - self.p_min) * Tunable[k] # sigmoid boundedness
        dWdT         = jacobian(Weight,Tunable)
        dWdT_fn      = Function('dWdT',[Tunable],[dWdT],['Tp0'],['dWdT_f'])
        weight_grad  = dWdT_fn(Tp0=tunable_para)['dWdT_f'].full()
        return weight_grad
    

    def loss(self,Opt_Sol1_l,Opt_Sol1_c,Opt_Sol2,Ref_xl,ref_xq):
        xl_traj   = Opt_Sol1_l[-1]['xl_traj']
        ul_traj   = Opt_Sol1_l[-1]['ul_traj']
        xc_list   = Opt_Sol1_c[-1]['xc_traj']
        uc_list   = Opt_Sol1_c[-1]['uc_traj']
        scxl_traj = Opt_Sol2[-1]['scxl_traj']
        scul_traj = Opt_Sol2[-1]['scul_traj']
        scxc_traj = Opt_Sol2[-1]['scxc_traj'] # list
        scuc_traj = Opt_Sol2[-1]['scuc_traj'] # list
        loss_track = 0
        loss_resid = 0
        for k in range(self.N):
            xl_k        = np.reshape(xl_traj[k,:],(self.nxl,1))
            ul_k        = np.reshape(ul_traj[k,:],(self.nul,1))
            scxl_k      = np.reshape(scxl_traj[k,:],(self.nxl,1))
            scul_k      = np.reshape(scul_traj[k,:],(self.nul,1))
            refxl_k     = np.reshape(Ref_xl[k*self.nxl:(k+1)*self.nxl],(self.nxl,1))
            error_k     = xl_k - refxl_k # load tracking error
            resid_xk    = xl_k - scxl_k  # load primal state residual
            resid_uk    = ul_k - scul_k  # load primal control residual
            loss_track += error_k.T@error_k # load tracking loss at k
            loss_resid += resid_xk.T@resid_xk + resid_uk.T@resid_uk
            for i in range(self.nq):
                xi_k        = np.reshape(xc_list[i][k,:],(self.nxi,1))
                ui_k        = np.reshape(uc_list[i][k,:],(self.nui,1))
                scxi_k      = np.reshape(scxc_traj[i][k,:],(self.nxi,1))
                scui_k      = np.reshape(scuc_traj[i][k,:],(self.nui,1))
                refxi_k     = ref_xq[i][k*self.nxi:(k+1)*self.nxi]
                di_k        = np.reshape(xi_k[0:3,0],(3,1))
                ti_k        = np.reshape(xi_k[6,0],(1,1))
                xdti_k      = np.vstack((di_k,ti_k))
                refdi_k     = np.reshape(refxi_k[0:3],(3,1))
                refti_k     = np.reshape(refxi_k[6],(1,1))
                refdti_k    = np.vstack((refdi_k,refti_k))
                error_ik    = xdti_k - refdti_k
                resid_xik   = xi_k - scxi_k
                resid_uik   = ui_k - scui_k
                loss_track += error_ik.T@error_ik
                loss_resid += resid_xik.T@resid_xik + resid_uik.T@resid_uik
        xl_N        = np.reshape(xl_traj[self.N,:],(self.nxl,1))
        scxl_N      = np.reshape(scxl_traj[self.N,:],(self.nxl,1))
        refxl_N     = np.reshape(Ref_xl[self.N*self.nxl:(self.N+1)*self.nxl],(self.nxl,1))
        error_N     = xl_N - refxl_N
        resid_xN    = xl_N - scxl_N
        loss_track += error_N.T@error_N
        loss_resid += resid_xN.T@resid_xN
        for i in range(self.nq):
            xi_N        = np.reshape(xc_list[i][self.N,:],(self.nxi,1))
            scxi_N      = np.reshape(scxc_traj[i][self.N,:],(self.nxi,1))
            refxi_N     = ref_xq[i][self.N*self.nxi:(self.N+1)*self.nxi]
            di_N        = np.reshape(xi_N[0:3,0],(3,1))
            ti_N        = np.reshape(xi_N[6,0],(1,1))
            xdti_N      = np.vstack((di_N,ti_N))
            refdi_N     = np.reshape(refxi_N[0:3],(3,1))
            refti_N     = np.reshape(refxi_N[6],(1,1))
            refdti_N    = np.vstack((refdi_N,refti_N))
            error_iN    = xdti_N - refdti_N
            resid_xiN   = xi_N - scxi_N
            loss_track += error_iN.T@error_iN
            loss_resid += resid_xiN.T@resid_xiN
        
        loss = self.w_track*loss_track + self.w_rp*loss_resid
        return loss
    

    def ChainRule(self,Opt_Sol1_l,Opt_Sol1_c,Opt_Sol2,Ref_xl,ref_xq,Grad_Out1l,Grad_Out1c,Grad_Out2):
        dltdxl          = jacobian(self.loss_track_l,self.xl)
        dltdxl_fn       = Function('dltdxl',[self.xl,self.xl_ref],[dltdxl],['xl0','refxl0'],['dltdxl_f'])
        dltdxi          = jacobian(self.loss_track_i,self.xi)
        dltdxi_fn       = Function('dltdxi',[self.xi,self.xi_ref],[dltdxi],['xi0','refxi0'],['dltdxi_f'])
        dlrpdxl         = jacobian(self.loss_rpl,self.xl)
        dlrpdxl_fn      = Function('dlrpdxl',[self.xl,self.scxl,self.ul,self.scul],[dlrpdxl],['xl0','scxl0','ul0','scul0'],['dlrpdxl_f'])
        dlrpdul         = jacobian(self.loss_rpl,self.ul)
        dlrpdul_fn      = Function('dlrpdul',[self.xl,self.scxl,self.ul,self.scul],[dlrpdul],['xl0','scxl0','ul0','scul0'],['dlrpdul_f'])
        dlrpdscxl       = jacobian(self.loss_rpl,self.scxl)
        dlrpdscxl_fn    = Function('dlrpdscxl',[self.xl,self.scxl,self.ul,self.scul],[dlrpdscxl],['xl0','scxl0','ul0','scul0'],['dlrpdscxl_f'])
        dlrpdscul       = jacobian(self.loss_rpl,self.scul)
        dlrpdscul_fn    = Function('dlrpdscul',[self.xl,self.scxl,self.ul,self.scul],[dlrpdscul],['xl0','scxl0','ul0','scul0'],['dlrpdscul_f'])
        dlrpdxi         = jacobian(self.loss_rpi,self.xi)
        dlrpdxi_fn      = Function('dlrpdxi',[self.xi,self.scxi,self.ui,self.scui],[dlrpdxi],['xi0','scxi0','ui0','scui0'],['dlrpdxi_f'])
        dlrpdui         = jacobian(self.loss_rpi,self.ui)
        dlrpdui_fn      = Function('dlrpdui',[self.xi,self.scxi,self.ui,self.scui],[dlrpdui],['xi0','scxi0','ui0','scui0'],['dlrpdui_f'])
        dlrpdscxi       = jacobian(self.loss_rpi,self.scxi)
        dlrpdscxi_fn    = Function('dlrpdscxi',[self.xi,self.scxi,self.ui,self.scui],[dlrpdscxi],['xi0','scxi0','ui0','scui0'],['dlrpdscxi_f'])
        dlrpdscui       = jacobian(self.loss_rpi,self.scui)
        dlrpdscui_fn    = Function('dlrpdscui',[self.xi,self.scxi,self.ui,self.scui],[dlrpdscui],['xi0','scxi0','ui0','scui0'],['dlrpdscui_f'])
        dlrpdxlN        = jacobian(self.loss_rpl_N,self.xl)
        dlrpdxlN_fn     = Function('dlrpdxlN',[self.xl,self.scxl],[dlrpdxlN],['xl0','scxl0'],['dlrpdxlN_f'])
        dlrpdscxlN      = jacobian(self.loss_rpl_N,self.scxl)
        dlrpdscxlN_fn   = Function('dlrpdscxlN',[self.xl,self.scxl],[dlrpdscxlN],['xl0','scxl0'],['dlrpdscxlN_f'])
        dlrpdxiN        = jacobian(self.loss_rpi_N,self.xi)
        dlrpdxiN_fn     = Function('dlrpdxiN',[self.xi,self.scxi],[dlrpdxiN],['xi0','scxi0'],['dlrpdxiN_f'])
        dlrpdscxiN      = jacobian(self.loss_rpi_N,self.scxi)
        dlrpdscxiN_fn   = Function('dlrpdscxiN',[self.xi,self.scxi],[dlrpdscxiN],['xi0','scxi0'],['dlrpdscxiN_f'])
        dltdw           = 0
        dlrpdw          = 0
        # load trajectories
        k_admm          = -1
        xl_traj         = Opt_Sol1_l[k_admm]['xl_traj']
        ul_traj         = Opt_Sol1_l[k_admm]['ul_traj']
        scxl_traj       = Opt_Sol2[k_admm]['scxl_traj']
        scul_traj       = Opt_Sol2[k_admm]['scul_traj']
        # load gradient trajectories
        xl_grad         = Grad_Out1l[k_admm]['xl_grad']
        ul_grad         = Grad_Out1l[k_admm]['ul_grad']
        scxl_grad       = Grad_Out2[k_admm]['scxl_grad']
        scul_grad       = Grad_Out2[k_admm]['scul_grad']
        # cable trajectories
        xc_traj         = Opt_Sol1_c[k_admm]['xc_traj'] # a list
        uc_traj         = Opt_Sol1_c[k_admm]['uc_traj'] # a list
        scxc_traj       = Opt_Sol2[k_admm]['scxc_traj'] # a list
        scuc_traj       = Opt_Sol2[k_admm]['scuc_traj'] # a list
        # cable gradient trajectories
        grad_outc       = Grad_Out1c[k_admm] # a list that contains both state and control gradients
        scxc_grad       = Grad_Out2[k_admm]['scxc_grad']
        scuc_grad       = Grad_Out2[k_admm]['scuc_grad']
        # loss
        loss   = self.loss(Opt_Sol1_l,Opt_Sol1_c,Opt_Sol2,Ref_xl,ref_xq)
        for k in range(self.N):
            if k==1:
                test = 1
            # gradient of the load tracking errors
            dltdxl_k    = dltdxl_fn(xl0=xl_traj[k,:],refxl0=Ref_xl[k*self.nxl:(k+1)*self.nxl])['dltdxl_f'].full()
            dltldw      = dltdxl_k@xl_grad[k]
            # print('dltldwr1=',dltldw[0,2*self.nxl],'dltldwpi=',dltldw[0,self.n_Pauto-1])
            dltdw      += dltldw
            # gradient of the load primal residuals
            dlrpdxl_k   = dlrpdxl_fn(xl0=xl_traj[k,:],scxl0=scxl_traj[k,:],ul0=ul_traj[k,:],scul0=scul_traj[k,:])['dlrpdxl_f'].full()
            dlrpdscxl_k = dlrpdscxl_fn(xl0=xl_traj[k,:],scxl0=scxl_traj[k,:],ul0=ul_traj[k,:],scul0=scul_traj[k,:])['dlrpdscxl_f'].full()
            dlrpdul_k   = dlrpdul_fn(xl0=xl_traj[k,:],scxl0=scxl_traj[k,:],ul0=ul_traj[k,:],scul0=scul_traj[k,:])['dlrpdul_f'].full()
            dlrpdscul_k = dlrpdscul_fn(xl0=xl_traj[k,:],scxl0=scxl_traj[k,:],ul0=ul_traj[k,:],scul0=scul_traj[k,:])['dlrpdscul_f'].full()
            dlrpdw     += dlrpdxl_k@xl_grad[k] + dlrpdscxl_k@scxl_grad[k] + dlrpdul_k@ul_grad[k] + dlrpdscul_k@scul_grad[k]
            for i in range(self.nq):
                # gradient of the cable tracking errors
                xi_traj     = xc_traj[i]
                ui_traj     = uc_traj[i]
                scxi_traj   = scxc_traj[i]
                scui_traj   = scuc_traj[i]
                refxi_k     = ref_xq[i][k*self.nxi:(k+1)*self.nxi]
                dltdxi_k    = dltdxi_fn(xi0=xi_traj[k,:],refxi0=refxi_k)['dltdxi_f'].full()
                grad_outi   = grad_outc[i]
                xi_grad     = grad_outi['xi_grad']
                dltidw      = dltdxi_k@xi_grad[k]
                # print('dltidwr4=',dltidw[0,2*self.nxl+self.nul+2*self.nxi+self.nui],'dltidwpl=',dltidw[0,2*self.nxl+self.nul],'dltidwpi=',dltidw[0,2*self.nxl+self.nul+2*self.nxi+self.nui+1])
                dltdw      += dltidw
                # gradient of the cable primal residuals
                ui_grad     = grad_outi['ui_grad']
                scxi_grad_k = scxc_grad[k][i*self.nxi:(i+1)*self.nxi,:]
                scui_grad_k = scuc_grad[k][i*self.nui:(i+1)*self.nui,:]
                dlrpdxi_k   = dlrpdxi_fn(xi0=xi_traj[k,:],scxi0=scxi_traj[k,:],ui0=ui_traj[k,:],scui0=scui_traj[k,:])['dlrpdxi_f'].full()
                dlrpdscxi_k = dlrpdscxi_fn(xi0=xi_traj[k,:],scxi0=scxi_traj[k,:],ui0=ui_traj[k,:],scui0=scui_traj[k,:])['dlrpdscxi_f'].full()
                dlrpdui_k   = dlrpdui_fn(xi0=xi_traj[k,:],scxi0=scxi_traj[k,:],ui0=ui_traj[k,:],scui0=scui_traj[k,:])['dlrpdui_f'].full()
                dlrpdscui_k = dlrpdscui_fn(xi0=xi_traj[k,:],scxi0=scxi_traj[k,:],ui0=ui_traj[k,:],scui0=scui_traj[k,:])['dlrpdscui_f'].full()
                dlrpdw     += dlrpdxi_k@xi_grad[k] + dlrpdscxi_k@scxi_grad_k + dlrpdui_k@ui_grad[k] + dlrpdscui_k@scui_grad_k
        # terminal gradients
        dltdxl_N    = dltdxl_fn(xl0=xl_traj[self.N,:],refxl0=Ref_xl[self.N*self.nxl:(self.N+1)*self.nxl])['dltdxl_f'].full()
        dltdw      += dltdxl_N@xl_grad[self.N]
        dlrpdxl_N   = dlrpdxlN_fn(xl0=xl_traj[self.N,:],scxl0=scxl_traj[self.N,:])['dlrpdxlN_f'].full()
        dlrpdscxl_N = dlrpdscxlN_fn(xl0=xl_traj[self.N,:],scxl0=scxl_traj[self.N,:])['dlrpdscxlN_f'].full()
        dlrpdw     += dlrpdxl_N@xl_grad[self.N] + dlrpdscxl_N@scxl_grad[self.N]
        for i in range(self.nq):
            xi_traj     = xc_traj[i]
            scxi_traj   = scxc_traj[i]
            refxi_N     = ref_xq[i][self.N*self.nxi:(self.N+1)*self.nxi]
            dltdxi_N    = dltdxi_fn(xi0=xi_traj[self.N,:],refxi0=refxi_N)['dltdxi_f'].full()
            grad_outi   = grad_outc[i]
            xi_grad     = grad_outi['xi_grad']
            dltdw      += dltdxi_N@xi_grad[self.N]
            scxi_grad_N = scxc_grad[self.N][i*self.nxi:(i+1)*self.nxi,:]
            dlrpdxi_N   = dlrpdxiN_fn(xi0=xi_traj[self.N,:],scxi0=scxi_traj[self.N,:])['dlrpdxiN_f'].full()
            dlrpdscxi_N = dlrpdscxiN_fn(xi0=xi_traj[self.N,:],scxi0=scxi_traj[self.N,:])['dlrpdscxiN_f'].full()
            dlrpdw     += dlrpdxi_N@xi_grad[self.N] + dlrpdscxi_N@scxi_grad_N
        # total gradient
        dldw        = self.w_track*dltdw + self.w_rp*dlrpdw


        return dldw, loss
  





















    





        

    

                


                    







    


        



    


    

    

    

        
        
            





    

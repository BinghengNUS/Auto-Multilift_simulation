"""
Main function of the load planner (Tension Allocation)
horizontal obstacles
------------------------------------------------------
1st version, Dr. Wang Bingheng, 19-Dec-2024
2nd version, Dr. Wang Bingheng, 17-June-2025
3rd version, Dr. Wang Bingheng, 02-Dec-2025
"""

from casadi import *
import numpy as np
from numpy import linalg as LA
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import Dynamics_meta_learning_COM_Dyn
import Optimal_Allocation_DDP_quaternion_autotuning_ADMM_COM_Dyn
import math
import time as TM
from scipy.spatial.transform import Rotation as Rot
import os
import Neural_network
import torch

if not os.path.exists("Planning_plots_meta_evaluation_COM_Dyn_H"):
    os.makedirs("Planning_plots_meta_evaluation_COM_Dyn_H")

"""--------------------------------------Load Environment---------------------------------------"""
m1        = 0.4  # the load's net weight [kg], a circular basket with uniform mass distribution
m2        = 0.05  # the added mass [kg]
# the total weight remains the same as in training, but the ratio between m1 and m2 changes.
mtot      = m1+m2 # the total weight [kg]
nq        = 3     # the number of quadrotors, different from that in training, try 3, 4, 6
cl0       = 1     # the cable length [m]
rq        = 0.15  # the radius of quadrotor [m]
rl        = 0.25  # the radius of the load [m]
ro        = 0.65  # the radius of obstacle [m]
"""--------------------------------------Load Environment---------------------------------------"""
sysm_para = np.array([m1, m2, 
                      1/4*m1*rl**2, 1/4*m1*rl**2, 1/2*m1*rl**2, 
                      rl, nq, rq, cl0, ro])
dt        = 0.05 
sysm      = Dynamics_meta_learning_COM_Dyn.multilift_model(sysm_para,dt)
# set the coordinate of the added mass in the load body frame
print("Generate a random offset")
max_radius = 0.15  # reference length [m]
radiusp   = np.random.uniform(0,max_radius) 
alpha     = np.random.uniform(0,2*np.pi)
rp        = np.array([[radiusp*np.cos(alpha),radiusp*np.sin(alpha),0]]).T # unit: [m]
sysm.Rotational_Inertia(rp)
sysm.model()
nxl       = sysm.nxl # dimension of the load's state
nul       = 3*nq # total dimension of the load's control = 6 (wrench) + 3*6-6 (null-space vector)
nWl       = sysm.nWl


"""--------------------------------------Define Planner---------------------------------------"""
horizon   = 120
e_abs, e_rel = 1e-4, 1e-3
MPC_load  = Optimal_Allocation_DDP_quaternion_autotuning_ADMM_COM_Dyn.MPC_Planner(sysm_para,dt,horizon,e_abs,e_rel)
pob1, pob2 = np.array([[-1.1,-0.4]]).T, np.array([[1.1,0.4]]).T
print('obstacle_distance=',LA.norm(pob1-pob2))
rg_task    = m2/mtot*rp
np.save('Planning_plots_meta_evaluation_COM_Dyn_H/rp_task',rp)
np.save('Planning_plots_meta_evaluation_COM_Dyn_H/nq',nq)
print('random rg_task for evaluation=',rg_task,'rp=',rp)
MPC_load.allocation_martrix(rg_task)
MPC_load.SetStateVariable(sysm.xl)
MPC_load.SetCtrlVariable(sysm.Wl)
MPC_load.SetDyn(sysm.model_l)
MPC_load.SetLearnablePara()
MPC_load.SetConstraints_ADMM_Subp2(pob1,pob2)
MPC_load.SetCostDyn_ADMM()
MPC_load.ADMM_SubP2_Init()
MPC_load.system_derivatives_DDP_ADMM()
MPC_load.system_derivatives_SubP2_ADMM()
MPC_load.system_derivatives_SubP3_ADMM()

# define the network size
D_in, D_h1, D_h2, D_out = 1, 16, 32, MPC_load.n_Pauto 
def convert_nn(nn_i_outcolumn):
    # convert a column tensor to a row np.array
    nn_i_row = np.zeros((1,D_out))
    for i in range(D_out):
        nn_i_row[0,i] = nn_i_outcolumn[i,0]
    return nn_i_row


"""--------------------------------------Redefine Gradient Solver---------------------------------------"""
Grad_Solver = Optimal_Allocation_DDP_quaternion_autotuning_ADMM_COM_Dyn.Gradient_Solver(horizon,sysm.xl,sysm.Wl,MPC_load.sc_xl,MPC_load.sc_Wl,MPC_load.P_auto) #horizon, xl, Wl, scxl, scWl, P_auto

"""--------------------------------------Define Load Reference---------------------------------------"""
Coeffx        = np.zeros((4,8))
Coeffy        = np.zeros((4,8))
Coeffz        = np.zeros((4,8))
for k in range(4):
    Coeffx[k,:] = np.load('Reference_traj_6_S_shape_evaluation/coeffx'+str(k+1)+'.npy')
    Coeffy[k,:] = np.load('Reference_traj_6_S_shape_evaluation/coeffy'+str(k+1)+'.npy')
    Coeffz[k,:] = np.load('Reference_traj_6_S_shape_evaluation/coeffz'+str(k+1)+'.npy')
Ref_xl = np.zeros(nxl*(horizon+1))
Ref_ul = np.zeros(nul*horizon)
Ref_pl = np.zeros((3,horizon))
Ref_Wl = np.zeros(nWl*horizon)
Time   = []
time   = 0
for k in range(horizon):
    Time  += [time]
    ref_xl, ref_Wl= sysm.minisnap_load_S_shape(Coeffx,Coeffy,Coeffz,time,rg_task)
    Ref_xl[k*nxl:(k+1)*nxl] = ref_xl
    Ref_Wl[k*nWl:(k+1)*nWl] = ref_Wl
    Ref_pl[:,k:(k+1)]       = np.reshape(ref_xl[0:3],(3,1))
    time += dt
# Time  += [time]
ref_xl, ref_Wl = sysm.minisnap_load_S_shape(Coeffx,Coeffy,Coeffz,time,rg_task)
Ref_xl[horizon*nxl:(horizon+1)*nxl] = ref_xl
# fig0, ax0 = plt.subplots(figsize=(5,5),dpi=300)
# obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
# obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
# ax0.add_patch(obs1)
# ax0.add_patch(obs2)
# ax0.plot(Ref_pl[0,:],Ref_pl[1,:],label='Ref',linewidth=1,linestyle='--')
# ax0.set_xlabel('x [m]')
# ax0.set_ylabel('y [m]')
# ax0.set_aspect('equal')
# ax0.legend()
# ax0.grid(True)
# plt.show()
# initial palyload's state
x0         = np.random.normal(-2,0.01)
y0         = np.random.normal(-2,0.01) 
z0         = np.random.normal(0.5,0.01)
pl         = np.array([[x0,y0,z0]]).T
vl         = np.reshape(np.random.normal(0,0.01,3),(3,1)) # initial velocity of CO in {I}
Eulerl     = np.clip(np.reshape(np.random.normal(0,0.01,3),(3,1)),-2/57.3,2/57.3)
Rl0        = sysm.dir_cosine(Eulerl)
r          = Rot.from_matrix(Rl0)  
# quaternion in the format of x, y, z, w 
# (https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.as_quat.html)
ql0        = r.as_quat() 
ql         = np.array([[ql0[3], ql0[0], ql0[1], ql0[2]]]).T
wl         = np.reshape(np.random.normal(0,0.01,3),(3,1))
xl_init    = np.reshape(np.vstack((pl,vl,ql,wl)),nxl)
xl_init[0] += rg_task[0]
xl_init[1] += rg_task[1]

np.save('Planning_plots_meta_evaluation_COM_Dyn_H/xl_init',xl_init)
# MPC weights
radius      = np.sqrt(rg_task[0]**2+rg_task[1]**2)
print("Please choose ADMM penalty policy")
adaptiveADMM = input("enter 'a' or 'f' without the quotation mark, a: iteration-adaptive; f: iteration-fixed")
print("Please choose ADMM truncation number")
max_iter_ADMM= int(input("enter '2', '3', '4', or '5' without the quotation mark"))
print("Please choose initial model")
initial_model = int(input("enter '0', '1', '2', '3' or '4' without the quotation mark"))
PATH2       = "trained_data_meta_COM_Dyn/trained_nn_waypoint_"+str(initial_model)+'_'+str(max_iter_ADMM)+"_n_"+str(adaptiveADMM)+".pt"
nn_waypoint = torch.load(PATH2, weights_only=False)
nn_input   = np.reshape(radius/max_radius,(1,1)) # dimensionless
nn_output  = convert_nn(nn_waypoint(nn_input))
weight     = Grad_Solver.Set_Parameters_nn(nn_output)
p_weight1  = weight[0:MPC_load.n_P1]
p_weight2  = weight[MPC_load.n_P1:MPC_load.n_P1 + MPC_load.n_P2]
p1         = weight[-1]
print('Q1=',p_weight1[0:MPC_load.n_xl],'R1=',p_weight1[2*MPC_load.n_xl:],'nv_w=',p_weight2[0:MPC_load.n_P2],'p1=',p1,'rg_radius (cm) =',radius)
start_time = TM.time()

Opt_Sol1, Opt_Sol2, Opt_Y, Opt_Eta  = MPC_load.ADMM_forward_MPC_DDP(xl_init,Ref_xl,Ref_Wl,p_weight1,p_weight2,p1,max_iter_ADMM,adaptiveADMM)
mpctime    = (TM.time() - start_time)*1000
print("a:--- %s ms ---" % format(mpctime,'.2f'))
xl_opt     = Opt_Sol1[-1]['xl_opt']
Wl_opt     = Opt_Sol1[-1]['Wl_opt']
Tl_opt     = Opt_Sol2[-1]['Tl_opt']
scxl_opt   = Opt_Sol2[-1]['scxl_opt']
# System open-loop predicted trajectories
P_pinv     = MPC_load.P_pinv # pseudo-inverse of P matrix
P_ns       = MPC_load.P_ns # null-space of P matrix
Pl         = np.zeros((3,horizon))
scPl       = np.zeros((3,horizon))
Euler_l    = np.zeros((3,horizon))
norm_2_Ql  = np.zeros(horizon)
for k in range(horizon):
    Pl[:,k:k+1] = np.reshape(xl_opt[k,0:3],(3,1))
    scPl[:,k:k+1] = np.reshape(scxl_opt[k,0:3],(3,1))
    ql_k  = np.reshape(xl_opt[k,6:10],(4,1))
    norm_2_Ql[k] = LA.norm(ql_k)
    Rl_k  = sysm.q_2_rotation(ql_k)
    rk    = Rot.from_matrix(Rl_k)
    euler_k = np.reshape(rk.as_euler('xyz',degrees=True),(3,1))
    Euler_l[:,k:k+1] = euler_k 
Xq         = [] # list that stores all quadrotors' predicted trajectories
DI         = [] # list that stores all cables' direction trajectories
Aq         = [] # list that stores all cable attachments' trajectories in the world frame
Tq         = np.zeros((nq,horizon))
for i in range(nq):
    Pi     = np.zeros((3,horizon))
    di     = np.zeros((3,horizon))
    ri     = np.reshape(MPC_load.ra[:,i],(3,1))
    ai     = np.zeros((3,horizon))
    for k in range(horizon):
        wl_k  = np.reshape(Wl_opt[k,:],(6,1)) # 6-D wrench at the kth step
        nv_k  = np.reshape(Tl_opt[k,:],(3*nq-6,1)) # 3-D null-space vector at the kth step
        t_k   = P_pinv@wl_k + P_ns@nv_k # 9-D tension vector at the kth step in the load's body frame
        ti_k  = np.reshape(t_k[3*i:3*(i+1)],(3,1))
        pl_k  = np.reshape(xl_opt[k,0:3],(3,1))
        ql_k  = np.reshape(xl_opt[k,6:10],(4,1))
        Rl_k  = sysm.q_2_rotation(ql_k)
        pi_k  = pl_k + Rl_k@(ri + cl0*ti_k/LA.norm(ti_k))
        di_k  = Rl_k@ti_k/LA.norm(ti_k)
        ai_k  = pl_k + Rl_k@ri
        Pi[:,k:k+1] = pi_k
        di[:,k:k+1] = di_k
        ai[:,k:k+1] = ai_k
        Tq[i,k] = LA.norm(ti_k)
    Xq += [Pi]
    DI += [di]
    Aq += [ai]

# Save data
np.save('Planning_plots_meta_evaluation_COM_Dyn_H/tension_magnitude_'+str(max_iter_ADMM)+'_'+str(adaptiveADMM),Tq)
np.save('Planning_plots_meta_evaluation_COM_Dyn_H/cable_direction_'+str(max_iter_ADMM)+'_'+str(adaptiveADMM),DI)
    

print('norm of quaternion=',norm_2_Ql)
    
# Plots

fig1, ax1 = plt.subplots(figsize=(5,5),dpi=300)
obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
ax1.add_patch(obs1)
ax1.add_patch(obs2)
ax1.plot(Xq[0][0,:],Xq[0][1,:],label='1st quadrotor',linewidth=1)
for k in range(horizon):
    quad  = Circle((Xq[0][0,k],Xq[0][1,k]),rq,color='blue',fill=False)
    ax1.add_patch(quad)
ax1.set_xlabel('x [m]')
ax1.set_ylabel('y [m]')
ax1.legend()
ax1.set_aspect('equal')
ax1.grid(True)
fig1.savefig('Planning_plots_meta_evaluation_COM_Dyn_H/quadrotor1_traj_'+str(nq)+'.png',dpi=400)
plt.show()

fig2, ax2 = plt.subplots(figsize=(5,5),dpi=300)
obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
ax2.add_patch(obs1)
ax2.add_patch(obs2)
ax2.plot(Xq[1][0,:],Xq[1][1,:],label='2nd quadrotor',linewidth=1)
for k in range(horizon):
    quad  = Circle((Xq[1][0,k],Xq[1][1,k]),rq,color='blue',fill=False)
    ax2.add_patch(quad)
ax2.set_xlabel('x [m]')
ax2.set_ylabel('y [m]')
ax2.legend()
ax2.set_aspect('equal')
ax2.grid(True)
fig2.savefig('Planning_plots_meta_evaluation_COM_Dyn_H/quadrotor2_traj_'+str(nq)+'.png',dpi=400)
plt.show()

fig3, ax3 = plt.subplots(figsize=(5,5),dpi=300)
obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
ax3.add_patch(obs1)
ax3.add_patch(obs2)
ax3.plot(Xq[2][0,:],Xq[2][1,:],label='3rd quadrotor',linewidth=1)
for k in range(horizon):
    quad  = Circle((Xq[2][0,k],Xq[2][1,k]),rq,color='blue',fill=False)
    ax3.add_patch(quad)
ax3.set_xlabel('x [m]')
ax3.set_ylabel('y [m]')
ax3.set_aspect('equal')
ax3.legend()
ax3.grid(True)
fig3.savefig('Planning_plots_meta_evaluation_COM_Dyn_H/quadrotor3_traj_'+str(nq)+'.png',dpi=400)
plt.show()

if nq==4:
    fig4, ax4 = plt.subplots(figsize=(5,5),dpi=300)
    obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
    obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
    ax4.add_patch(obs1)
    ax4.add_patch(obs2)
    ax4.plot(Xq[3][0,:],Xq[3][1,:],label='4th quadrotor',linewidth=1)
    for k in range(horizon):
        quad  = Circle((Xq[3][0,k],Xq[3][1,k]),rq,color='blue',fill=False)
        ax4.add_patch(quad)
    ax4.set_xlabel('x [m]')
    ax4.set_ylabel('y [m]')
    ax4.set_aspect('equal')
    ax4.legend()
    ax4.grid(True)
    fig4.savefig('Planning_plots_meta_evaluation_COM_Dyn_H/quadrotor4_traj_'+str(nq)+'.png',dpi=400)
    plt.show()

if nq==6:
    fig4, ax4 = plt.subplots(figsize=(5,5),dpi=300)
    obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
    obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
    ax4.add_patch(obs1)
    ax4.add_patch(obs2)
    ax4.plot(Xq[3][0,:],Xq[3][1,:],label='4th quadrotor',linewidth=1)
    for k in range(horizon):
        quad  = Circle((Xq[3][0,k],Xq[3][1,k]),rq,color='blue',fill=False)
        ax4.add_patch(quad)
    ax4.set_xlabel('x [m]')
    ax4.set_ylabel('y [m]')
    ax4.set_aspect('equal')
    ax4.legend()
    ax4.grid(True)
    fig4.savefig('Planning_plots_meta_evaluation_COM_Dyn_H/quadrotor4_traj_'+str(nq)+'.png',dpi=400)
    plt.show()

    fig5, ax5 = plt.subplots(figsize=(5,5),dpi=300)
    obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
    obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
    ax5.add_patch(obs1)
    ax5.add_patch(obs2)
    ax5.plot(Xq[4][0,:],Xq[4][1,:],label='5th quadrotor',linewidth=1)
    for k in range(horizon):
        quad  = Circle((Xq[4][0,k],Xq[4][1,k]),rq,color='blue',fill=False)
        ax5.add_patch(quad)
    ax5.set_xlabel('x [m]')
    ax5.set_ylabel('y [m]')
    ax5.set_aspect('equal')
    ax5.legend()
    ax5.grid(True)
    fig5.savefig('Planning_plots_meta_evaluation_COM_Dyn_H/quadrotor5_traj_'+str(nq)+'.png',dpi=400)
    plt.show()

    fig6, ax6 = plt.subplots(figsize=(5,5),dpi=300)
    obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
    obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
    ax6.add_patch(obs1)
    ax6.add_patch(obs2)
    ax6.plot(Xq[5][0,:],Xq[5][1,:],label='6th quadrotor',linewidth=1)
    for k in range(horizon):
        quad  = Circle((Xq[5][0,k],Xq[5][1,k]),rq,color='blue',fill=False)
        ax6.add_patch(quad)
    ax6.set_xlabel('x [m]')
    ax6.set_ylabel('y [m]')
    ax6.set_aspect('equal')
    ax6.legend()
    ax6.grid(True)
    fig6.savefig('Planning_plots_meta_evaluation_COM_Dyn_H/quadrotor6_traj_'+str(nq)+'.png',dpi=400)
    plt.show()


fig9, ax9 = plt.subplots(figsize=(5,5),dpi=300)
obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
ax9.add_patch(obs1)
ax9.add_patch(obs2)
ax9.plot(Ref_pl[0,:],Ref_pl[1,:],label='Ref',linewidth=1,linestyle='--')
ax9.plot(Pl[0,:],Pl[1,:],label='Planned_SubP1',linewidth=1)
ax9.plot(scPl[0,:],scPl[1,:],label='Planned_SubP2',linewidth=1)
kt = 46
ratio = (horizon/100)
for k in range(horizon):
    if k==2 or k==int(kt*ratio) or k==int(99*ratio):
        if nq==6:
            quad1  = Circle((Xq[0][0,k],Xq[0][1,k]),rq,fill=False)
            ax9.add_patch(quad1)
            quad2  = Circle((Xq[1][0,k],Xq[1][1,k]),rq,fill=False)
            ax9.add_patch(quad2)
            quad3  = Circle((Xq[2][0,k],Xq[2][1,k]),rq,fill=False)
            ax9.add_patch(quad3)
            quad4  = Circle((Xq[3][0,k],Xq[3][1,k]),rq,fill=False)
            ax9.add_patch(quad4)
            quad5  = Circle((Xq[4][0,k],Xq[4][1,k]),rq,fill=False)
            ax9.add_patch(quad5)
            quad6  = Circle((Xq[5][0,k],Xq[5][1,k]),rq,fill=False)
            ax9.add_patch(quad6)
            ax9.plot((Xq[0][0,k],Aq[0][0,k]),[Xq[0][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Xq[1][0,k],Aq[1][0,k]],[Xq[1][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Xq[2][0,k],Aq[2][0,k]],[Xq[2][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Xq[3][0,k],Aq[3][0,k]],[Xq[3][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Xq[4][0,k],Aq[4][0,k]],[Xq[4][1,k],Aq[4][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Xq[5][0,k],Aq[5][0,k]],[Xq[5][1,k],Aq[5][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Aq[0][0,k],Aq[1][0,k]],[Aq[0][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Aq[1][0,k],Aq[2][0,k]],[Aq[1][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Aq[2][0,k],Aq[3][0,k]],[Aq[2][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Aq[3][0,k],Aq[4][0,k]],[Aq[3][1,k],Aq[4][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Aq[4][0,k],Aq[5][0,k]],[Aq[4][1,k],Aq[5][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Aq[5][0,k],Aq[0][0,k]],[Aq[5][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
        elif nq==4: 
            quad1  = Circle((Xq[0][0,k],Xq[0][1,k]),rq,fill=False)
            ax9.add_patch(quad1)
            quad2  = Circle((Xq[1][0,k],Xq[1][1,k]),rq,fill=False)
            ax9.add_patch(quad2)
            quad3  = Circle((Xq[2][0,k],Xq[2][1,k]),rq,fill=False)
            ax9.add_patch(quad3)
            quad4  = Circle((Xq[3][0,k],Xq[3][1,k]),rq,fill=False)
            ax9.add_patch(quad4)
            ax9.plot((Xq[0][0,k],Aq[0][0,k]),[Xq[0][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Xq[1][0,k],Aq[1][0,k]],[Xq[1][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Xq[2][0,k],Aq[2][0,k]],[Xq[2][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Xq[3][0,k],Aq[3][0,k]],[Xq[3][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Aq[0][0,k],Aq[1][0,k]],[Aq[0][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Aq[1][0,k],Aq[2][0,k]],[Aq[1][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Aq[2][0,k],Aq[3][0,k]],[Aq[2][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Aq[3][0,k],Aq[0][0,k]],[Aq[3][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
        else: # nq=3
            quad1  = Circle((Xq[0][0,k],Xq[0][1,k]),rq,fill=False)
            ax9.add_patch(quad1)
            quad2  = Circle((Xq[1][0,k],Xq[1][1,k]),rq,fill=False)
            ax9.add_patch(quad2)
            quad3  = Circle((Xq[2][0,k],Xq[2][1,k]),rq,fill=False)
            ax9.add_patch(quad3)
            ax9.plot((Xq[0][0,k],Aq[0][0,k]),[Xq[0][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Xq[1][0,k],Aq[1][0,k]],[Xq[1][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Xq[2][0,k],Aq[2][0,k]],[Xq[2][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Aq[0][0,k],Aq[1][0,k]],[Aq[0][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Aq[1][0,k],Aq[2][0,k]],[Aq[1][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax9.plot([Aq[2][0,k],Aq[0][0,k]],[Aq[2][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)

    
ax9.set_xlabel('x [m]')
ax9.set_ylabel('y [m]')
ax9.set_aspect('equal')
ax9.legend()
ax9.grid(True)
# plt.axis('equal')
fig9.savefig('Planning_plots_meta_evaluation_COM_Dyn_H/system_traj_quadrotor_num_'+str(nq)+'_'+str(kt)+'.png',dpi=400)
plt.show()


fig10, ax10 = plt.subplots(figsize=(5,5),dpi=300)
obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
ax10.add_patch(obs1)
ax10.add_patch(obs2)
ax10.plot(Ref_pl[0,:],Ref_pl[1,:],label='Ref',linewidth=1,linestyle='--')
ax10.plot(Pl[0,:],Pl[1,:],label='Planned_SubP1',linewidth=1)
ax10.plot(scPl[0,:],scPl[1,:],label='Planned_SubP2',linewidth=1)
kt = 48
ratio = (horizon/100)
for k in range(horizon):
    if k==2 or k==int(kt*ratio) or k==int(99*ratio):
        if nq==6:
            quad1  = Circle((Xq[0][0,k],Xq[0][1,k]),rq,fill=False)
            ax10.add_patch(quad1)
            quad2  = Circle((Xq[1][0,k],Xq[1][1,k]),rq,fill=False)
            ax10.add_patch(quad2)
            quad3  = Circle((Xq[2][0,k],Xq[2][1,k]),rq,fill=False)
            ax10.add_patch(quad3)
            quad4  = Circle((Xq[3][0,k],Xq[3][1,k]),rq,fill=False)
            ax10.add_patch(quad4)
            quad5  = Circle((Xq[4][0,k],Xq[4][1,k]),rq,fill=False)
            ax10.add_patch(quad5)
            quad6  = Circle((Xq[5][0,k],Xq[5][1,k]),rq,fill=False)
            ax10.add_patch(quad6)
            ax10.plot((Xq[0][0,k],Aq[0][0,k]),[Xq[0][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Xq[1][0,k],Aq[1][0,k]],[Xq[1][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Xq[2][0,k],Aq[2][0,k]],[Xq[2][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Xq[3][0,k],Aq[3][0,k]],[Xq[3][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Xq[4][0,k],Aq[4][0,k]],[Xq[4][1,k],Aq[4][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Xq[5][0,k],Aq[5][0,k]],[Xq[5][1,k],Aq[5][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Aq[0][0,k],Aq[1][0,k]],[Aq[0][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Aq[1][0,k],Aq[2][0,k]],[Aq[1][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Aq[2][0,k],Aq[3][0,k]],[Aq[2][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Aq[3][0,k],Aq[4][0,k]],[Aq[3][1,k],Aq[4][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Aq[4][0,k],Aq[5][0,k]],[Aq[4][1,k],Aq[5][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Aq[5][0,k],Aq[0][0,k]],[Aq[5][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
        elif nq==4: 
            quad1  = Circle((Xq[0][0,k],Xq[0][1,k]),rq,fill=False)
            ax10.add_patch(quad1)
            quad2  = Circle((Xq[1][0,k],Xq[1][1,k]),rq,fill=False)
            ax10.add_patch(quad2)
            quad3  = Circle((Xq[2][0,k],Xq[2][1,k]),rq,fill=False)
            ax10.add_patch(quad3)
            quad4  = Circle((Xq[3][0,k],Xq[3][1,k]),rq,fill=False)
            ax10.add_patch(quad4)
            ax10.plot((Xq[0][0,k],Aq[0][0,k]),[Xq[0][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Xq[1][0,k],Aq[1][0,k]],[Xq[1][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Xq[2][0,k],Aq[2][0,k]],[Xq[2][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Xq[3][0,k],Aq[3][0,k]],[Xq[3][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Aq[0][0,k],Aq[1][0,k]],[Aq[0][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Aq[1][0,k],Aq[2][0,k]],[Aq[1][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Aq[2][0,k],Aq[3][0,k]],[Aq[2][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Aq[3][0,k],Aq[0][0,k]],[Aq[3][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
        else:
            quad1  = Circle((Xq[0][0,k],Xq[0][1,k]),rq,fill=False)
            ax10.add_patch(quad1)
            quad2  = Circle((Xq[1][0,k],Xq[1][1,k]),rq,fill=False)
            ax10.add_patch(quad2)
            quad3  = Circle((Xq[2][0,k],Xq[2][1,k]),rq,fill=False)
            ax10.add_patch(quad3)
            ax10.plot((Xq[0][0,k],Aq[0][0,k]),[Xq[0][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Xq[1][0,k],Aq[1][0,k]],[Xq[1][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Xq[2][0,k],Aq[2][0,k]],[Xq[2][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Aq[0][0,k],Aq[1][0,k]],[Aq[0][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Aq[1][0,k],Aq[2][0,k]],[Aq[1][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax10.plot([Aq[2][0,k],Aq[0][0,k]],[Aq[2][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
    
ax10.set_xlabel('x [m]')
ax10.set_ylabel('y [m]')
ax10.set_aspect('equal')
ax10.legend()
ax10.grid(True)
# plt.axis('equal')
fig10.savefig('Planning_plots_meta_evaluation_COM_Dyn_H/system_traj_quadrotor_num_'+str(nq)+'_'+str(kt)+'.png',dpi=400)
plt.show()


fig11, ax11 = plt.subplots(figsize=(5,5),dpi=300)
obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
ax11.add_patch(obs1)
ax11.add_patch(obs2)
ax11.plot(Ref_pl[0,:],Ref_pl[1,:],label='Ref',linewidth=1,linestyle='--')
ax11.plot(Pl[0,:],Pl[1,:],label='Planned_SubP1',linewidth=1)
ax11.plot(scPl[0,:],scPl[1,:],label='Planned_SubP2',linewidth=1)
kt = 50
ratio = (horizon/100)
for k in range(horizon):
    if k==2 or k==int(kt*ratio) or k==int(99*ratio):
        if nq==6:
            quad1  = Circle((Xq[0][0,k],Xq[0][1,k]),rq,fill=False)
            ax11.add_patch(quad1)
            quad2  = Circle((Xq[1][0,k],Xq[1][1,k]),rq,fill=False)
            ax11.add_patch(quad2)
            quad3  = Circle((Xq[2][0,k],Xq[2][1,k]),rq,fill=False)
            ax11.add_patch(quad3)
            quad4  = Circle((Xq[3][0,k],Xq[3][1,k]),rq,fill=False)
            ax11.add_patch(quad4)
            quad5  = Circle((Xq[4][0,k],Xq[4][1,k]),rq,fill=False)
            ax11.add_patch(quad5)
            quad6  = Circle((Xq[5][0,k],Xq[5][1,k]),rq,fill=False)
            ax11.add_patch(quad6)
            ax11.plot((Xq[0][0,k],Aq[0][0,k]),[Xq[0][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Xq[1][0,k],Aq[1][0,k]],[Xq[1][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Xq[2][0,k],Aq[2][0,k]],[Xq[2][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Xq[3][0,k],Aq[3][0,k]],[Xq[3][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Xq[4][0,k],Aq[4][0,k]],[Xq[4][1,k],Aq[4][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Xq[5][0,k],Aq[5][0,k]],[Xq[5][1,k],Aq[5][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Aq[0][0,k],Aq[1][0,k]],[Aq[0][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Aq[1][0,k],Aq[2][0,k]],[Aq[1][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Aq[2][0,k],Aq[3][0,k]],[Aq[2][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Aq[3][0,k],Aq[4][0,k]],[Aq[3][1,k],Aq[4][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Aq[4][0,k],Aq[5][0,k]],[Aq[4][1,k],Aq[5][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Aq[5][0,k],Aq[0][0,k]],[Aq[5][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
        elif nq==4: 
            quad1  = Circle((Xq[0][0,k],Xq[0][1,k]),rq,fill=False)
            ax11.add_patch(quad1)
            quad2  = Circle((Xq[1][0,k],Xq[1][1,k]),rq,fill=False)
            ax11.add_patch(quad2)
            quad3  = Circle((Xq[2][0,k],Xq[2][1,k]),rq,fill=False)
            ax11.add_patch(quad3)
            quad4  = Circle((Xq[3][0,k],Xq[3][1,k]),rq,fill=False)
            ax11.add_patch(quad4)
            ax11.plot((Xq[0][0,k],Aq[0][0,k]),[Xq[0][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Xq[1][0,k],Aq[1][0,k]],[Xq[1][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Xq[2][0,k],Aq[2][0,k]],[Xq[2][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Xq[3][0,k],Aq[3][0,k]],[Xq[3][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Aq[0][0,k],Aq[1][0,k]],[Aq[0][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Aq[1][0,k],Aq[2][0,k]],[Aq[1][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Aq[2][0,k],Aq[3][0,k]],[Aq[2][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Aq[3][0,k],Aq[0][0,k]],[Aq[3][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
        else:
            quad1  = Circle((Xq[0][0,k],Xq[0][1,k]),rq,fill=False)
            ax11.add_patch(quad1)
            quad2  = Circle((Xq[1][0,k],Xq[1][1,k]),rq,fill=False)
            ax11.add_patch(quad2)
            quad3  = Circle((Xq[2][0,k],Xq[2][1,k]),rq,fill=False)
            ax11.add_patch(quad3)
            ax11.plot((Xq[0][0,k],Aq[0][0,k]),[Xq[0][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Xq[1][0,k],Aq[1][0,k]],[Xq[1][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Xq[2][0,k],Aq[2][0,k]],[Xq[2][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Aq[0][0,k],Aq[1][0,k]],[Aq[0][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Aq[1][0,k],Aq[2][0,k]],[Aq[1][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax11.plot([Aq[2][0,k],Aq[0][0,k]],[Aq[2][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
        
ax11.set_xlabel('x [m]')
ax11.set_ylabel('y [m]')
ax11.set_aspect('equal')
ax11.legend()
ax11.grid(True)
# plt.axis('equal')
fig11.savefig('Planning_plots_meta_evaluation_COM_Dyn_H/system_traj_quadrotor_num_'+str(nq)+'_'+str(kt)+'.png',dpi=400)
plt.show()

fig12, ax12 = plt.subplots(figsize=(5,5),dpi=300)
obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
ax12.add_patch(obs1)
ax12.add_patch(obs2)
ax12.plot(Ref_pl[0,:],Ref_pl[1,:],label='Ref',linewidth=1,linestyle='--')
ax12.plot(Pl[0,:],Pl[1,:],label='Planned_SubP1',linewidth=1)
ax12.plot(scPl[0,:],scPl[1,:],label='Planned_SubP2',linewidth=1)
kt = 52
ratio = (horizon/100)
for k in range(horizon):
    if k==2 or k==int(kt*ratio) or k==int(99*ratio):
        if nq==6:
            quad1  = Circle((Xq[0][0,k],Xq[0][1,k]),rq,fill=False)
            ax12.add_patch(quad1)
            quad2  = Circle((Xq[1][0,k],Xq[1][1,k]),rq,fill=False)
            ax12.add_patch(quad2)
            quad3  = Circle((Xq[2][0,k],Xq[2][1,k]),rq,fill=False)
            ax12.add_patch(quad3)
            quad4  = Circle((Xq[3][0,k],Xq[3][1,k]),rq,fill=False)
            ax12.add_patch(quad4)
            quad5  = Circle((Xq[4][0,k],Xq[4][1,k]),rq,fill=False)
            ax12.add_patch(quad5)
            quad6  = Circle((Xq[5][0,k],Xq[5][1,k]),rq,fill=False)
            ax12.add_patch(quad6)
            ax12.plot((Xq[0][0,k],Aq[0][0,k]),[Xq[0][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Xq[1][0,k],Aq[1][0,k]],[Xq[1][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Xq[2][0,k],Aq[2][0,k]],[Xq[2][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Xq[3][0,k],Aq[3][0,k]],[Xq[3][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Xq[4][0,k],Aq[4][0,k]],[Xq[4][1,k],Aq[4][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Xq[5][0,k],Aq[5][0,k]],[Xq[5][1,k],Aq[5][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Aq[0][0,k],Aq[1][0,k]],[Aq[0][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Aq[1][0,k],Aq[2][0,k]],[Aq[1][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Aq[2][0,k],Aq[3][0,k]],[Aq[2][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Aq[3][0,k],Aq[4][0,k]],[Aq[3][1,k],Aq[4][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Aq[4][0,k],Aq[5][0,k]],[Aq[4][1,k],Aq[5][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Aq[5][0,k],Aq[0][0,k]],[Aq[5][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
        elif nq==4: 
            quad1  = Circle((Xq[0][0,k],Xq[0][1,k]),rq,fill=False)
            ax12.add_patch(quad1)
            quad2  = Circle((Xq[1][0,k],Xq[1][1,k]),rq,fill=False)
            ax12.add_patch(quad2)
            quad3  = Circle((Xq[2][0,k],Xq[2][1,k]),rq,fill=False)
            ax12.add_patch(quad3)
            quad4  = Circle((Xq[3][0,k],Xq[3][1,k]),rq,fill=False)
            ax12.add_patch(quad4)
            ax12.plot((Xq[0][0,k],Aq[0][0,k]),[Xq[0][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Xq[1][0,k],Aq[1][0,k]],[Xq[1][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Xq[2][0,k],Aq[2][0,k]],[Xq[2][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Xq[3][0,k],Aq[3][0,k]],[Xq[3][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Aq[0][0,k],Aq[1][0,k]],[Aq[0][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Aq[1][0,k],Aq[2][0,k]],[Aq[1][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Aq[2][0,k],Aq[3][0,k]],[Aq[2][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Aq[3][0,k],Aq[0][0,k]],[Aq[3][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
        else:
            quad1  = Circle((Xq[0][0,k],Xq[0][1,k]),rq,fill=False)
            ax12.add_patch(quad1)
            quad2  = Circle((Xq[1][0,k],Xq[1][1,k]),rq,fill=False)
            ax12.add_patch(quad2)
            quad3  = Circle((Xq[2][0,k],Xq[2][1,k]),rq,fill=False)
            ax12.add_patch(quad3)
            ax12.plot((Xq[0][0,k],Aq[0][0,k]),[Xq[0][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Xq[1][0,k],Aq[1][0,k]],[Xq[1][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Xq[2][0,k],Aq[2][0,k]],[Xq[2][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Aq[0][0,k],Aq[1][0,k]],[Aq[0][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Aq[1][0,k],Aq[2][0,k]],[Aq[1][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax12.plot([Aq[2][0,k],Aq[0][0,k]],[Aq[2][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
    
ax12.set_xlabel('x [m]')
ax12.set_ylabel('y [m]')
ax12.set_aspect('equal')
ax12.legend()
ax12.grid(True)
# plt.axis('equal')
fig12.savefig('Planning_plots_meta_evaluation_COM_Dyn_H/system_traj_quadrotor_num_'+str(nq)+'_'+str(kt)+'.png',dpi=400)
plt.show()

fig13, ax13 = plt.subplots(figsize=(5,5),dpi=300)
obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
ax13.add_patch(obs1)
ax13.add_patch(obs2)
ax13.plot(Ref_pl[0,:],Ref_pl[1,:],label='Ref',linewidth=1,linestyle='--')
ax13.plot(Pl[0,:],Pl[1,:],label='Planned_SubP1',linewidth=1)
ax13.plot(scPl[0,:],scPl[1,:],label='Planned_SubP2',linewidth=1)
kt = 54
ratio = (horizon/100)
for k in range(horizon):
    if k==2 or k==int(kt*ratio) or k==int(99*ratio):
        if nq==6:
            quad1  = Circle((Xq[0][0,k],Xq[0][1,k]),rq,fill=False)
            ax13.add_patch(quad1)
            quad2  = Circle((Xq[1][0,k],Xq[1][1,k]),rq,fill=False)
            ax13.add_patch(quad2)
            quad3  = Circle((Xq[2][0,k],Xq[2][1,k]),rq,fill=False)
            ax13.add_patch(quad3)
            quad4  = Circle((Xq[3][0,k],Xq[3][1,k]),rq,fill=False)
            ax13.add_patch(quad4)
            quad5  = Circle((Xq[4][0,k],Xq[4][1,k]),rq,fill=False)
            ax13.add_patch(quad5)
            quad6  = Circle((Xq[5][0,k],Xq[5][1,k]),rq,fill=False)
            ax13.add_patch(quad6)
            ax13.plot((Xq[0][0,k],Aq[0][0,k]),[Xq[0][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Xq[1][0,k],Aq[1][0,k]],[Xq[1][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Xq[2][0,k],Aq[2][0,k]],[Xq[2][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Xq[3][0,k],Aq[3][0,k]],[Xq[3][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Xq[4][0,k],Aq[4][0,k]],[Xq[4][1,k],Aq[4][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Xq[5][0,k],Aq[5][0,k]],[Xq[5][1,k],Aq[5][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Aq[0][0,k],Aq[1][0,k]],[Aq[0][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Aq[1][0,k],Aq[2][0,k]],[Aq[1][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Aq[2][0,k],Aq[3][0,k]],[Aq[2][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Aq[3][0,k],Aq[4][0,k]],[Aq[3][1,k],Aq[4][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Aq[4][0,k],Aq[5][0,k]],[Aq[4][1,k],Aq[5][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Aq[5][0,k],Aq[0][0,k]],[Aq[5][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
        elif nq==4: 
            quad1  = Circle((Xq[0][0,k],Xq[0][1,k]),rq,fill=False)
            ax13.add_patch(quad1)
            quad2  = Circle((Xq[1][0,k],Xq[1][1,k]),rq,fill=False)
            ax13.add_patch(quad2)
            quad3  = Circle((Xq[2][0,k],Xq[2][1,k]),rq,fill=False)
            ax13.add_patch(quad3)
            quad4  = Circle((Xq[3][0,k],Xq[3][1,k]),rq,fill=False)
            ax13.add_patch(quad4)
            ax13.plot((Xq[0][0,k],Aq[0][0,k]),[Xq[0][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Xq[1][0,k],Aq[1][0,k]],[Xq[1][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Xq[2][0,k],Aq[2][0,k]],[Xq[2][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Xq[3][0,k],Aq[3][0,k]],[Xq[3][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Aq[0][0,k],Aq[1][0,k]],[Aq[0][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Aq[1][0,k],Aq[2][0,k]],[Aq[1][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Aq[2][0,k],Aq[3][0,k]],[Aq[2][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Aq[3][0,k],Aq[0][0,k]],[Aq[3][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
        else:
            quad1  = Circle((Xq[0][0,k],Xq[0][1,k]),rq,fill=False)
            ax13.add_patch(quad1)
            quad2  = Circle((Xq[1][0,k],Xq[1][1,k]),rq,fill=False)
            ax13.add_patch(quad2)
            quad3  = Circle((Xq[2][0,k],Xq[2][1,k]),rq,fill=False)
            ax13.add_patch(quad3)
            ax13.plot((Xq[0][0,k],Aq[0][0,k]),[Xq[0][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Xq[1][0,k],Aq[1][0,k]],[Xq[1][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Xq[2][0,k],Aq[2][0,k]],[Xq[2][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Aq[0][0,k],Aq[1][0,k]],[Aq[0][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Aq[1][0,k],Aq[2][0,k]],[Aq[1][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
            ax13.plot([Aq[2][0,k],Aq[0][0,k]],[Aq[2][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
    
ax13.set_xlabel('x [m]')
ax13.set_ylabel('y [m]')
ax13.set_aspect('equal')
ax13.legend()
ax13.grid(True)
# plt.axis('equal')
fig13.savefig('Planning_plots_meta_evaluation_COM_Dyn_H/system_traj_quadrotor_num_'+str(nq)+'_'+str(kt)+'.png',dpi=400)
plt.show()


plt.figure(8,figsize=(6,4),dpi=300)
plt.plot(Time,Tq[0,:],linewidth=1,label='1st cable')
plt.plot(Time,Tq[1,:],linewidth=1,label='2nd cable')
plt.plot(Time,Tq[2,:],linewidth=1,label='3rd cable')
if nq==4:
    plt.plot(Time,Tq[3,:],linewidth=1,label='4th cable')
if nq==6:
    plt.plot(Time,Tq[4,:],linewidth=1,label='5th cable')
    plt.plot(Time,Tq[5,:],linewidth=1,label='6th cable')
plt.legend()
plt.xlabel('Time [s]')
plt.ylabel('MPC tension force [N]')
plt.grid()
plt.savefig('Planning_plots_meta_evaluation_COM_Dyn_H/cable_MPC_tensions_'+str(nq)+'.png',dpi=400)
plt.show()


plt.figure(9,figsize=(6,4),dpi=300)
plt.plot(Time,Euler_l[0,:],linewidth=1,label='roll')
plt.plot(Time,Euler_l[1,:],linewidth=1,label='pitch')
plt.plot(Time,Euler_l[2,:],linewidth=1,label='yaw')
plt.legend()
plt.xlabel('Time [s]')
plt.ylabel('Euler angle [deg]')
plt.grid()
plt.savefig('Planning_plots_meta_evaluation_COM_Dyn_H/euler_'+str(nq)+'.png',dpi=400)
plt.show()


plt.figure(10,figsize=(6,4),dpi=300)
plt.plot(Time,Pl[2,:],linewidth=1,label='planned')
plt.plot(Time,Ref_pl[2,:],label='Ref',linewidth=1,linestyle='--')
plt.legend()
plt.xlabel('Time [s]')
plt.ylabel('Height [m]')
plt.grid()
plt.savefig('Planning_plots_meta_evaluation_COM_Dyn_H/Height_'+str(nq)+'.png',dpi=400)
plt.show()




"""
Main function of the load planner (Tension Dynamic Allocation)
------------------------------------------------------
1st version, Dr. Wang Bingheng, 07-Mar-2025
2nd version, Dr. Wang Bingheng, 17-June-2025
"""

from casadi import *
import numpy as np
from numpy import linalg as LA
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import Dynamics_load_cable_autotuning_2nd_evaluation
import Kinodynamic_Planning_ADMM_quaternion_DDP_autotuning_2nd_evaluation
import math
import time as TM
from scipy.spatial.transform import Rotation as Rot
import os
import Neural_network
import torch
import random


"""--------------------------------------Load Environment---------------------------------------"""
sysm_para = np.array([1.5, 0.25, 0.15,0.15,0.2, 0.02,-0.01,0, 6, 1, 0.125, 0.75, 0.25]) 
# the larger the rg, the larger the deviation of the 5th quadrotor away from the center will be
# the last one is the quadrotor mass
# for a load with a biased CoM, different quadrotors need different weights
dt        = 0.04 # step size 0.1s
ml        = sysm_para[0]
rl        = sysm_para[1]
rg        = np.reshape(np.array([[sysm_para[5],sysm_para[6],sysm_para[7]]]).T,(3,1))
rq        = sysm_para[10]
ro        = sysm_para[11]
nq        = int(sysm_para[8])
cl0       = sysm_para[9] # cable length
sysm      = Dynamics_load_cable_autotuning_2nd_evaluation.multilift_model(sysm_para,dt)
sysm.model()
nxl       = sysm.nxl # dimension of the load's state
nul       = sysm.nul # dimension of the load's control
nxi       = sysm.nxi # dimension of the cable's state
nui       = sysm.nui # dimension of the cable's control

"""--------------------------------------Define Planner---------------------------------------"""
horizon   = 125
# pob1, pob2 = np.array([[1.5,1.3]]).T, np.array([[0.4,2.9]]).T  # planar positions of the two obstacle in the world frame
pob1, pob2 = np.array([[1.6,1.1]]).T, np.array([[0.35,3.1]]).T # planar positions of the two obstacle in the world frame
MPC_load  = Kinodynamic_Planning_ADMM_quaternion_DDP_autotuning_2nd_evaluation.MPC_Planner(sysm_para,dt,horizon)
MPC_load.SetStateVariables(sysm.xl,sysm.xi)
MPC_load.SetCtrlVariables(sysm.ul,sysm.ui)
MPC_load.SetDyns(sysm.model_l,sysm.model_i)
MPC_load.SetWeightPara()
MPC_load.SetPayloadCostDyn()
MPC_load.SetCableCostDyn()
MPC_load.SetConstriants(pob1,pob2)
MPC_load.SetADMMSubP2_SoftCost_k()
MPC_load.SetADMMSubP2_SoftCost_N()
MPC_load.ADMM_SubP2_Init()
MPC_load.ADMM_SubP2_N_Init()
MPC_load.Load_derivatives_DDP_ADMM()
MPC_load.Cable_derivatives_DDP_ADMM()
MPC_load.system_derivatives_SubP2_ADMM_k()
MPC_load.system_derivatives_SubP2_ADMM_N()
MPC_load.system_derivatives_SubP3_ADMM()

npl       = MPC_load.npl
npi       = MPC_load.npi
npauto    = MPC_load.n_Pauto

D_inl, D_h1l, D_h2l, D_outl = 2, 16, 32, MPC_load.npl 
def convert_nn_l(nn_l_outcolumn):
    # convert a column tensor to a row np.array
    nn_l_row = np.zeros((1,D_outl))
    for i in range(D_outl):
        nn_l_row[0,i] = nn_l_outcolumn[i,0]
    return nn_l_row

D_ini, D_h1i, D_h2i, D_outi = 2, 10, 20, MPC_load.npi
def convert_nn_i(nn_i_outcolumn):
    # convert a column tensor to a row np.array
    nn_i_row = np.zeros((1,D_outi))
    for i in range(D_outi):
        nn_i_row[0,i] = nn_i_outcolumn[i,0]
    return nn_i_row

num_task  = 10
rg_task   = sysm_para[5:7]

# parameters of ADAM
coefficient = 1e2 # reduced for the multi-agent system, unit: cm * note that if 1e3 is used, some initial model will lead to very biased weights which are diffcult to train
epsilon   = 1e-8
"""--------------------------------------Define Gradient Solver--------------------------------------"""
Grad_Solver   = Kinodynamic_Planning_ADMM_quaternion_DDP_autotuning_2nd_evaluation.Gradient_Solver(sysm_para, horizon, MPC_load.xl, MPC_load.ul, MPC_load.scxl, MPC_load.scul, MPC_load.xi, MPC_load.ui, MPC_load.scxi, MPC_load.scui, MPC_load.P_auto, MPC_load.para_l, MPC_load.para_i)


"""--------------------------------------Define Load Reference---------------------------------------"""
Coeffx        = np.zeros((2,8))
Coeffy        = np.zeros((2,8))
Coeffz        = np.zeros((2,8))
for k in range(2):
    Coeffx[k,:] = np.load('Reference_traj_5/coeffx'+str(k+1)+'.npy')
    Coeffy[k,:] = np.load('Reference_traj_5/coeffy'+str(k+1)+'.npy')
    Coeffz[k,:] = np.load('Reference_traj_5/coeffz'+str(k+1)+'.npy')
Ref_xl = np.zeros(nxl*(horizon+1))
Ref_ul = np.zeros(nul*horizon)
Ref_pl = np.zeros((3,horizon+1))
Time   = []
time   = 0

# initial palyload's state (randomly initialized, so the training results can be different even with the same network model!)
x0         = np.random.normal(0,0.01)
y0         = np.random.normal(0,0.01)
z0         = np.random.normal(0.5,0.01) 
pl         = np.array([[x0,y0,z0]]).T 
vl         = np.reshape(np.random.normal(0,0.01,3),(3,1)) # initial velocity of CO in {Bl}
Eulerl     = np.clip(np.reshape(np.random.normal(0,0.01,3),(3,1)),-1/57.3,1/57.3)
Rl0        = sysm.dir_cosine(Eulerl)
r          = Rot.from_matrix(Rl0)  
# quaternion in the format of x, y, z, w 
# (https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.as_quat.html)
ql0        = r.as_quat() 
ql         = np.array([[ql0[3], ql0[0], ql0[1], ql0[2]]]).T
wl         = np.reshape(np.random.normal(0,0.01,3),(3,1))
xl_init    = np.reshape(np.vstack((pl,vl,ql,wl)),nxl)
np.save('trained_data_multiagent_meta/xl_init',xl_init)
# xl_init = np.load('trained_data_multiagent_meta/xl_init.npy')
if not os.path.exists("Planning_plots_multiagent_meta_evaluation"):
    os.makedirs("Planning_plots_multiagent_meta_evaluation")
# initial cables' states
alpha  = 2*np.pi/nq
beta   = np.random.normal(1/2*np.pi,0.01) # need to be collision-free
d_quad = (rl+cl0*math.cos(beta))*alpha
while d_quad<(3*rq+0.125):
    beta  -= 0.01
    d_quad = (rl+cl0*math.cos(beta))*alpha

PATH2   = "trained_data_multiagent_meta/trained_nn_l.pt"
PATH3   = "trained_data_multiagent_meta/trained_nn_i.pt"
nn_l    = torch.load(PATH2)
nn_i    = torch.load(PATH3)
pl_min  = np.load('trained_data_multiagent_meta/pl_min.npy')
pi_min  = np.load('trained_data_multiagent_meta/pi_min.npy')
"""--------------------------------------Redefine Planner---------------------------------------"""
Ref_xl = np.zeros(nxl*(horizon+1))
Ref_ul = np.zeros(nul*horizon)
Ref_pl = np.zeros((3,horizon+1))
Time   = []
time   = 0
DI     = np.load('Planning_plots_meta_evaluation/cable_direction.npy')
TI     = np.load('Planning_plots_meta_evaluation/tension_magnitude.npy')
ref_xc = [np.zeros((horizon+1)*nxi) for _ in range(nq)]
for k in range(horizon):
    Time  += [time]
    ref_xl, ref_ul, ref_p, ref_xq, ref_uq = sysm.minisnap_load_circle(Coeffx,Coeffy,Coeffz,time)
    Ref_xl[k*nxl:(k+1)*nxl] = ref_xl
    Ref_ul[k*nul:(k+1)*nul] = ref_ul
    Ref_pl[:,k:k+1] = ref_p
    time += dt
    for i in range(nq):
        ref_di_k = np.reshape(DI[i][:,k],(3,1))
        ref_wi = np.zeros((3,1))
        ref_ti_k = np.reshape(TI[i,k],(1,1))
        ref_dti_k= np.zeros((1,1))
        ref_xi_k = np.reshape(np.vstack((ref_di_k,ref_wi,ref_ti_k,ref_dti_k)),nxi)
        ref_xc[i][k*nxi:(k+1)*nxi] = ref_xi_k
ref_xl, ref_ul, ref_p, ref_xq, ref_uq = sysm.minisnap_load_circle(Coeffx,Coeffy,Coeffz,time)
for i in range(nq):
    ref_xc[i][horizon*nxi:(horizon+1)*nxi]=ref_xq[i*nxi:(i+1)*nxi]
Ref_xl[horizon*nxl:(horizon+1)*nxl] = ref_xl
Ref_pl[:,horizon:horizon+1] = ref_p
xq_init = np.zeros(nq*nxi) # in the world frame
for j in range(nq):
    di_init   = np.array([[math.cos(beta)*math.cos(j*alpha),math.cos(beta)*math.sin(j*alpha),math.sin(beta)]]).T # reference direction vector
    wi_init   = np.zeros((3,1))
    ti_init   = np.array([[TI[j,0]]])
    dti_init  = np.zeros((1,1))
    xi_init   = np.reshape(np.vstack((di_init,wi_init,ti_init,dti_init)),nxi)
    xq_init[j*nxi:(j+1)*nxi] = xi_init
nn_input         = np.reshape(coefficient*rg_task,(2,1)) # unit cm, or mm
nn_l_output_task = convert_nn_l(nn_l(nn_input))     
P_weight1  = Grad_Solver.Set_Parameters_nn_l(nn_l_output_task,pl_min)
nn_i_output_task = convert_nn_i(nn_i(nn_input))
P_weight2  = Grad_Solver.Set_Parameters_nn_i(nn_i_output_task,pi_min)        
print('Ql=',P_weight1[0:nxl],'QlN=',P_weight1[nxl:2*nxl],'Rl=',P_weight1[2*nxl:2*nxl+nul],'p=',P_weight1[-1])
print('Qi=',P_weight2[0:nxi],'QiN=',P_weight2[nxi:2*nxi],'Ri=',P_weight2[2*nxi:2*nxi+nui],'pi=',P_weight2[-1])
start_time = TM.time()
opt_sol, Opt_Sol1_l, Opt_Sol1_cddp, Opt_Sol1_c, Opt_Sol2, Opt_Sol3 = MPC_load.ADMM_forward_MPC(Ref_xl,Ref_ul,ref_xc,ref_uq,xl_init,xq_init,P_weight1,P_weight2)
mpctime    = (TM.time() - start_time)*1000
print("forward mpc:--- %s ms ---" % format(mpctime,'.2f'))
start_time = TM.time()
                
xl_traj    = opt_sol['xl_traj']    
scxl_traj  = opt_sol['scxl_traj']
xq_traj    = opt_sol['xc_traj']
uq_traj    = opt_sol['uc_traj']
scxq_traj  = opt_sol['scxc_traj']
Kfb_traj   = opt_sol['Kfbl_traj']
np.save('Planning_plots_multiagent_meta_evaluation/xl_traj',xl_traj)
np.save('Planning_plots_multiagent_meta_evaluation/Kfb_traj',Kfb_traj)
np.save('Planning_plots_multiagent_meta_evaluation/xq_traj',xq_traj)
np.save('Planning_plots_multiagent_meta_evaluation/uq_traj',uq_traj)

# System open-loop predicted trajectories
Pl         = np.zeros((3,horizon))
scPl       = np.zeros((3,horizon))
Eulerl     = np.zeros((3,horizon))
norm_2_Ql  = np.zeros(horizon)
for k in range(horizon):
    Pl[:,k:k+1] = np.reshape(xl_traj[k,0:3],(3,1))
    scPl[:,k:k+1]=np.reshape(scxl_traj[k,0:3],(3,1))
    ql_k        = np.reshape(xl_traj[k,6:10],(4,1))
    norm_2_Ql[k] = LA.norm(ql_k)
    Rl_k        = sysm.q_2_rotation(ql_k)
    rl_k        = Rot.from_matrix(Rl_k)
    eulerl_k    = np.reshape(rl_k.as_euler('zyx',degrees=True),(3,1))
    Eulerl[:,k:k+1] = eulerl_k

Xq         = [] # list that stores all quadrotors' predicted trajectories
Aq         = [] # list that stores all cable attachments' trajectories in the world frame
scXq       = [] # list that stores all quadrotors' safe copy predicted trajectories
refXq      = [] # list that stores all quadrotors' reference trajectories
alpha      = 2*np.pi/nq
Tq         = np.zeros((nq,horizon))
scTq       = np.zeros((nq,horizon))

 
for i in range(nq):
    Pi     = np.zeros((3,horizon))
    scPi   = np.zeros((3,horizon))
    refPi  = np.zeros((3,horizon))
    ri     = np.array([[rl*math.cos(i*alpha),rl*math.sin(i*alpha),0]]).T
    ai     = np.zeros((3,horizon))
    for k in range(horizon):
        pl_k   = np.reshape(xl_traj[k,0:3],(3,1))
        ql_k   = np.reshape(xl_traj[k,6:10],(4,1))
        Rl_k   = sysm.q_2_rotation(ql_k)
        di_k   = np.reshape(xq_traj[i][k,0:3],(3,1))
        ti_k   = xq_traj[i][k,6]
        scdi_k = np.reshape(scxq_traj[i][k,0:3],(3,1))
        scti_k = scxq_traj[i][k,6]
        ai_k   = pl_k + Rl_k@ri
        pi_k   = ai_k + cl0*di_k
        scpi_k = ai_k + cl0*scdi_k
        ref_plk= np.reshape(Ref_pl[:,k],(3,1)) + ri
        refpi_k= ref_plk + cl0*np.reshape(DI[i][:,k],(3,1))
        ai[:,k:k+1] = ai_k
        Tq[i,k]= ti_k
        scTq[i,k] = scti_k
        Pi[:,k:k+1] = pi_k
        scPi[:,k:k+1] = scpi_k
        refPi[:,k:k+1]= refpi_k
    Xq    += [Pi]
    scXq  += [scPi]
    refXq += [refPi]
    Aq    += [ai]
    

#Plots
# fig1, ax1 = plt.subplots(figsize=(5,5),dpi=300)
# obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
# obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
# ax1.add_patch(obs1)
# ax1.add_patch(obs2)
# ax1.plot(Xq[0][0,:],Xq[0][1,:],label='1st quadrotor',linewidth=1)
# ax1.plot(scXq[0][0,:],scXq[0][1,:],label='1st quadrotor_safe copy',color='black',marker='.',markersize=1,linewidth=1)
# ax1.plot(refXq[0][0,:],refXq[0][1,:],label='1st quadrotor_ref',color='orange',marker='.',markersize=1,linewidth=1)
# for k in range(horizon):
#     quad  = Circle((Xq[0][0,k],Xq[0][1,k]),rq,color='blue',fill=False)
#     ax1.add_patch(quad)
# ax1.set_xlabel('x [m]')
# ax1.set_ylabel('y [m]')
# ax1.legend()
# ax1.set_aspect('equal')
# ax1.grid(True)
# fig1.savefig('Planning_plots_multiagent_meta/quadrotor1_traj.png',dpi=400)
# plt.show()

# fig2, ax2 = plt.subplots(figsize=(5,5),dpi=300)
# obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
# obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
# ax2.add_patch(obs1)
# ax2.add_patch(obs2)
# ax2.plot(Xq[1][0,:],Xq[1][1,:],label='2nd quadrotor',linewidth=1)
# ax2.plot(scXq[1][0,:],scXq[1][1,:],label='2nd quadrotor_safe copy',color='black',marker='.',markersize=1,linewidth=1)
# ax2.plot(refXq[1][0,:],refXq[1][1,:],label='2nd quadrotor_ref',color='orange',marker='.',markersize=1,linewidth=1)
# for k in range(horizon):
#     quad  = Circle((Xq[1][0,k],Xq[1][1,k]),rq,color='blue',fill=False)
#     ax2.add_patch(quad)
# ax2.set_xlabel('x [m]')
# ax2.set_ylabel('y [m]')
# ax2.legend()
# ax2.set_aspect('equal')
# ax2.grid(True)
# fig2.savefig('Planning_plots_multiagent_meta/quadrotor2_traj.png',dpi=400)
# plt.show()

# fig3, ax3 = plt.subplots(figsize=(5,5),dpi=300)
# obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
# obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
# ax3.add_patch(obs1)
# ax3.add_patch(obs2)
# ax3.plot(Xq[2][0,:],Xq[2][1,:],label='3rd quadrotor',linewidth=1)
# ax3.plot(scXq[2][0,:],scXq[2][1,:],label='3rd quadrotor_safe copy',color='black',marker='.',markersize=1,linewidth=1)
# ax3.plot(refXq[2][0,:],refXq[2][1,:],label='3rd quadrotor_ref',color='orange',marker='.',markersize=1,linewidth=1)
# for k in range(horizon):
#     quad  = Circle((Xq[2][0,k],Xq[2][1,k]),rq,color='blue',fill=False)
#     ax3.add_patch(quad)
# ax3.set_xlabel('x [m]')
# ax3.set_ylabel('y [m]')
# ax3.set_aspect('equal')
# ax3.legend()
# ax3.grid(True)
# fig3.savefig('Planning_plots_multiagent_meta/quadrotor3_traj.png',dpi=400)
# plt.show()

# fig4, ax4 = plt.subplots(figsize=(5,5),dpi=300)
# obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
# obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
# ax4.add_patch(obs1)
# ax4.add_patch(obs2)
# ax4.plot(Xq[3][0,:],Xq[3][1,:],label='4th quadrotor',linewidth=1)
# ax4.plot(scXq[3][0,:],scXq[3][1,:],label='4th quadrotor_safe copy',color='black',marker='.',markersize=1,linewidth=1)
# ax4.plot(refXq[3][0,:],refXq[3][1,:],label='4th quadrotor_ref',color='orange',marker='.',markersize=1,linewidth=1)
# for k in range(horizon):
#     quad  = Circle((Xq[3][0,k],Xq[3][1,k]),rq,color='blue',fill=False)
#     ax4.add_patch(quad)
# ax4.set_xlabel('x [m]')
# ax4.set_ylabel('y [m]')
# ax4.set_aspect('equal')
# ax4.legend()
# ax4.grid(True)
# fig4.savefig('Planning_plots_multiagent_meta/quadrotor4_traj.png',dpi=400)
# plt.show()

# fig5, ax5 = plt.subplots(figsize=(5,5),dpi=300)
# obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
# obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
# ax5.add_patch(obs1)
# ax5.add_patch(obs2)
# ax5.plot(Xq[4][0,:],Xq[4][1,:],label='5th quadrotor',linewidth=1)
# ax5.plot(scXq[4][0,:],scXq[4][1,:],label='5th quadrotor_safe copy',color='black',marker='.',markersize=1,linewidth=1)
# ax5.plot(refXq[4][0,:],refXq[4][1,:],label='5th quadrotor_ref',color='orange',marker='.',markersize=1,linewidth=1)
# for k in range(horizon):
#     quad  = Circle((Xq[4][0,k],Xq[4][1,k]),rq,color='blue',fill=False)
#     ax5.add_patch(quad)
# ax5.set_xlabel('x [m]')
# ax5.set_ylabel('y [m]')
# ax5.set_aspect('equal')
# ax5.legend()
# ax5.grid(True)
# fig5.savefig('Planning_plots_multiagent_meta/quadrotor5_traj.png',dpi=400)
# plt.show()

# fig6, ax6 = plt.subplots(figsize=(5,5),dpi=300)
# obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
# obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
# ax6.add_patch(obs1)
# ax6.add_patch(obs2)
# ax6.plot(Xq[5][0,:],Xq[5][1,:],label='6th quadrotor',linewidth=1)
# ax6.plot(scXq[5][0,:],scXq[5][1,:],label='6th quadrotor_safe copy',color='black',marker='.',markersize=1,linewidth=1)
# ax6.plot(refXq[5][0,:],refXq[5][1,:],label='6th quadrotor_ref',color='orange',marker='.',markersize=1,linewidth=1)
# for k in range(horizon):
#     quad  = Circle((Xq[5][0,k],Xq[5][1,k]),rq,color='blue',fill=False)
#     ax6.add_patch(quad)
# ax6.set_xlabel('x [m]')
# ax6.set_ylabel('y [m]')
# ax6.set_aspect('equal')
# ax6.legend()
# ax6.grid(True)
# fig6.savefig('Planning_plots_multiagent_meta/quadrotor6_traj.png',dpi=400)
# plt.show()

fig7, ax7 = plt.subplots(figsize=(5,5),dpi=300)
obs1      = Circle((pob1[0,0],pob1[1,0]),ro,color='red',alpha=0.5)
obs2      = Circle((pob2[0,0],pob2[1,0]),ro,color='red',alpha=0.5)
ax7.add_patch(obs1)
ax7.add_patch(obs2)
ax7.plot(Ref_pl[0,:],Ref_pl[1,:],label='Ref',linewidth=1,linestyle='--')
ax7.plot(Pl[0,:],Pl[1,:],label='Planned',linewidth=1)
ax7.plot(scPl[0,:],scPl[1,:],label='Planned_safe_copy',color='black',marker='.',markersize=1,linewidth=1)
kt= 51
ratio = (horizon/100)
for k in range(horizon):
    if k==2 or k==int(kt*ratio) or k==int(99*ratio):
        #6 quadrotors
        quad1  = Circle((Xq[0][0,k],Xq[0][1,k]),rq,fill=False)
        ax7.add_patch(quad1)
        quad2  = Circle((Xq[1][0,k],Xq[1][1,k]),rq,fill=False)
        ax7.add_patch(quad2)
        quad3  = Circle((Xq[2][0,k],Xq[2][1,k]),rq,fill=False)
        ax7.add_patch(quad3)
        quad4  = Circle((Xq[3][0,k],Xq[3][1,k]),rq,fill=False)
        ax7.add_patch(quad4)
        quad5  = Circle((Xq[4][0,k],Xq[4][1,k]),rq,fill=False)
        ax7.add_patch(quad5)
        quad6  = Circle((Xq[5][0,k],Xq[5][1,k]),rq,fill=False)
        ax7.add_patch(quad6) 
        ax7.plot((Xq[0][0,k],Aq[0][0,k]),[Xq[0][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)
        ax7.plot([Xq[1][0,k],Aq[1][0,k]],[Xq[1][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
        ax7.plot([Xq[2][0,k],Aq[2][0,k]],[Xq[2][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
        ax7.plot([Xq[3][0,k],Aq[3][0,k]],[Xq[3][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
        ax7.plot([Xq[4][0,k],Aq[4][0,k]],[Xq[4][1,k],Aq[4][1,k]],color='blue',linewidth=0.5)
        ax7.plot([Xq[5][0,k],Aq[5][0,k]],[Xq[5][1,k],Aq[5][1,k]],color='blue',linewidth=0.5)
        ax7.plot([Aq[0][0,k],Aq[1][0,k]],[Aq[0][1,k],Aq[1][1,k]],color='blue',linewidth=0.5)
        ax7.plot([Aq[1][0,k],Aq[2][0,k]],[Aq[1][1,k],Aq[2][1,k]],color='blue',linewidth=0.5)
        ax7.plot([Aq[2][0,k],Aq[3][0,k]],[Aq[2][1,k],Aq[3][1,k]],color='blue',linewidth=0.5)
        ax7.plot([Aq[3][0,k],Aq[4][0,k]],[Aq[3][1,k],Aq[4][1,k]],color='blue',linewidth=0.5)
        ax7.plot([Aq[4][0,k],Aq[5][0,k]],[Aq[4][1,k],Aq[5][1,k]],color='blue',linewidth=0.5)
        ax7.plot([Aq[5][0,k],Aq[0][0,k]],[Aq[5][1,k],Aq[0][1,k]],color='blue',linewidth=0.5)

ax7.set_xlabel('x [m]')
ax7.set_ylabel('y [m]')
ax7.set_aspect('equal')
ax7.legend()
ax7.grid(True)
# fig7.savefig('Planning_plots_multiagent_meta/system_traj_quadrotor_num6_'+str(i_train)+'_'+str(task_idx)+'.png',dpi=400)
fig7.savefig('Planning_plots_multiagent_meta_evaluation/system_traj_quadrotor_num6_'+str(kt)+'.png',dpi=400)
plt.show()


plt.figure(8,figsize=(6,4),dpi=300)
plt.plot(Time,Tq[0,:],linewidth=1,label='1st cable')
plt.plot(Time,Tq[1,:],linewidth=1,label='2nd cable')
plt.plot(Time,Tq[2,:],linewidth=1,label='3rd cable')
plt.plot(Time,Tq[3,:],linewidth=1,label='4th cable')
plt.plot(Time,Tq[4,:],linewidth=1,label='5th cable')
plt.plot(Time,Tq[5,:],linewidth=1,label='6th cable')
plt.legend()
plt.xlabel('Time [s]')
plt.ylabel('MPC tension force [N]')
plt.grid()
plt.savefig('Planning_plots_multiagent_meta_evaluation/cable_MPC_tensions_6.png',dpi=400)
plt.show()

# plt.figure(10,figsize=(6,4),dpi=300)
# plt.plot(Time,Eulerl[0,:],linewidth=1,label='roll')
# plt.plot(Time,Eulerl[1,:],linewidth=1,label='pitch')
# plt.plot(Time,Eulerl[2,:],linewidth=1,label='yaw')
# plt.legend()
# plt.xlabel('Time [s]')
# plt.ylabel('Euler angle [deg]')
# plt.grid()
# plt.savefig('Planning_plots_multiagent_meta_evaluation/euler_6.png',dpi=400)
# plt.show()


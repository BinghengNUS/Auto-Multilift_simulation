bool mpc_traj # mark the start and end of distributed mpc task
float32 time_traj # control time in current trajectory, each update in 'opt_system', += dt_ctrl
bool[] rec_temp # mark the received temp_traj from drones, each iter need all Qnodes & Lnode to run at least once

float32[] xq_traj  # shape = (nq, horizon+1, nxi)
float32[] uq_traj  # shape = (nq, horizon, nui)
float32[] xl_traj  # shape = (horizon+1, nxl)
float32[] ul_traj  # shape = (horizon, nul)
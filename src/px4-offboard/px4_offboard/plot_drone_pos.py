#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from mpl_toolkits.mplot3d import Axes3D   

# 1)  Load reference data

from get_data import DataLoader
data = DataLoader()


def equal_axis(ax, size=4):
    """Force cubic axes of given `size` (length in metres)."""
    xm, ym, zm = (np.mean(lim) for lim in
                  (ax.get_xlim(), ax.get_ylim(), ax.get_zlim()))
    half = size / 2
    ax.set_xlim(xm - half, xm + half)
    ax.set_ylim(ym - half, ym + half)
    ax.set_zlim(zm - half, zm + half)

N_DRONES = 6
DT       = 0.01                          # 25 Hz  → 0.04 s
N        = data.payload_x.shape[0]       # samples
T        = np.arange(N) * DT             # time axis


# 2)  Reference positions (x_ref) & velocities (v_ref)

if all(hasattr(data, k) for k in ('drone_x', 'drone_v')):
    x_ref = data.drone_x[:N_DRONES]      # shape (6,N,3)
    v_ref = data.drone_v[:N_DRONES]      # shape (6,N,3)
else:
    # --- rebuild like controller (no frame conversion here) --------
    L, R_p = 1.0, 0.25
    rho, offs = [], []
    for i in range(N_DRONES):
        ang = 2*np.pi*i/N_DRONES
        rho.append([R_p*np.sin(ang),  R_p*np.cos(ang), 0])
        offs.append([(L+R_p)*np.sin(ang), (L+R_p)*np.cos(ang), 0])
    rho  = np.array(rho)
    offs = np.array(offs)

    x_ref = np.zeros((N_DRONES, N, 3))
    for k in range(N):
        x_d  = data.payload_x[k]
        dirs = data.cable_direction[:, k, :].T
        q_d  = -dirs / np.linalg.norm(dirs, axis=0, keepdims=True)
        for i in range(N_DRONES):
            x_ref[i, k] = x_d + rho[i] - L*q_d[:, i] - offs[i]

    v_ref = np.gradient(x_ref, DT, axis=1)   # finite-diff velocity

# colour map
cmap = plt.cm.tab10(np.linspace(0, 1, N_DRONES))


# 3A) 2-D XY trajectories

fig_xy, ax_xy = plt.subplots(figsize=(7, 7))
for i, c in enumerate(cmap):
    ax_xy.plot(x_ref[i, :, 0], x_ref[i, :, 1],
               lw=1.2, color=c, label=f'Drone {i}')
ax_xy.set_xlabel('X [m]'); ax_xy.set_ylabel('Y [m]')
ax_xy.set_title('UAV XY Reference Trajectories')
ax_xy.axis('equal'); ax_xy.grid(ls='--', alpha=.4)
ax_xy.legend(ncol=3, fontsize=8)
fig_xy.tight_layout()


# 3B) 3-D static trajectories  (NEW)

fig_xyz = plt.figure(figsize=(7, 6))
ax_xyz  = fig_xyz.add_subplot(111, projection='3d')
for i, c in enumerate(cmap):
    ax_xyz.plot(x_ref[i, :, 0],          # X
                x_ref[i, :, 1],          # Y
                x_ref[i, :, 2],          # Z (height)
                lw=1.2, color=c, label=f'Drone {i}')
# mark final positions
ax_xyz.scatter(x_ref[:, -1, 0], x_ref[:, -1, 1], x_ref[:, -1, 2],
               s=35, c=cmap)
ax_xyz.set_xlabel('X [m]'); ax_xyz.set_ylabel('Y [m]'); ax_xyz.set_zlabel('Z [m]')
ax_xyz.set_title('UAV 3-D Reference Trajectories')
ax_xyz.legend(ncol=3, fontsize=8)
ax_xyz.set_box_aspect([1, 1, 1])          # equal aspect
equal_axis(ax_xyz, size=4)  # 4-metre cube
# little margin
margin = 0.2
pts = x_ref.reshape(-1, 3)
for setter, lim in zip((ax_xyz.set_xlim, ax_xyz.set_ylim, ax_xyz.set_zlim),
                       zip(pts.min(0) - margin, pts.max(0) + margin)):
    setter(*lim)
fig_xyz.tight_layout()


# 4)  Speed-time curves

fig_spd, ax_spd = plt.subplots(figsize=(9, 4))
for i, c in enumerate(cmap):
    speed = np.linalg.norm(v_ref[i], axis=1)   # |v|
    ax_spd.plot(T, speed, lw=1.2, color=c, label=f'Drone {i}')
ax_spd.set_xlabel('Time [s]'); ax_spd.set_ylabel('Speed [m/s]')
ax_spd.set_title('UAV Reference Speeds')
ax_spd.grid(ls='--', alpha=.4)
ax_spd.legend(ncol=3, fontsize=8)
fig_spd.tight_layout()


# 5)  3-D animation with trail 

try:
    import matplotlib.animation as anim

    fig_a = plt.figure(figsize=(6, 6))
    ax_a  = fig_a.add_subplot(111, projection='3d')
    ax_a.set_xlabel('X'); ax_a.set_ylabel('Y'); ax_a.set_zlabel('Z')
    ax_a.set_title('UAV reference trajectories (animated)')
    pts_all = x_ref.reshape(-1, 3)
    for fn, lim in zip((ax_a.set_xlim, ax_a.set_ylim, ax_a.set_zlim),
                       zip(pts_all.min(0) - .2, pts_all.max(0) + .2)):
        fn(*lim)
    equal_axis(ax_a, size=4)
    scat = [ax_a.scatter([], [], [], s=500, color=cmap[i]) for i in range(N_DRONES)]
    lines = [ax_a.plot([], [], [], color=cmap[i], lw=1)[0] for i in range(N_DRONES)]

    def init():
        for s in scat: s._offsets3d = ([], [], [])
        for ln in lines: ln.set_data_3d([], [], [])
        return scat + lines

    def update(frame):
        for i in range(N_DRONES):
            x, y, z = x_ref[i, frame]
            scat[i]._offsets3d = ([x], [y], [z])
            lines[i].set_data_3d(x_ref[i, :frame+1, 0],
                                 x_ref[i, :frame+1, 1],
                                 x_ref[i, :frame+1, 2])
        ax_a.set_title(f't = {T[frame]:.2f} s')
        return scat + lines

    ani = anim.FuncAnimation(fig_a, update, frames=N,
                             init_func=init, interval=40, blit=False)
    out = Path('uav_traj_3d.mp4')

    plt.show()

except Exception as e:
    print("animation skipped:", e)

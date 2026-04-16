#!/usr/bin/env python3
# tiny plot script

from pathlib import Path
import sys
import numpy as np
import matplotlib.pyplot as plt

def newest_file(dir_path: Path, pattern: str = "*.npz") -> Path:
    """Return the newest file (by mtime) matching pattern in dir_path."""
    files = list(dir_path.glob(pattern))
    if not files:
        raise FileNotFoundError(f"No files matching {pattern} in {dir_path}")
    return max(files, key=lambda p: p.stat().st_mtime)

# ----------- choose log_path -----------
if len(sys.argv) > 1:
    log_path = Path(sys.argv[1]).expanduser().resolve()
else:
    log_path = newest_file(Path("/home/carlson/lift_log"))

arg_id   = sys.argv[2] if len(sys.argv) > 2 else "all"   # "all" or an int

# ---------- rest of your original plotting code ----------
print(f"Loading data from: {log_path}")
data = np.load(log_path)
t = data["t"]
labels = ["x", "y", "z"]

# log_path = Path(sys.argv[1]) #if len(sys.argv) > 1 #else Path("/home/carlson/lift_log/pd_errors_20250725_131726.npz")
# arg_id   = sys.argv[2] if len(sys.argv) > 2 else "all"   # "all" or an int

# data = np.load(log_path)
# t = data["t"]
# labels = ["x", "y", "z"]

# ---------- fig1: error ----------
fig1, ax1 = plt.subplots(3, 2, figsize=(14, 10), sharex="col")
ax1 = ax1.flatten()

vec_keys  = ["ex", "ev", "eR", "eO"]
rope_keys = ["eq", "ew"]

for k, key in enumerate(vec_keys):
    arr = data[key]
    for i in range(3):
        ax1[k].plot(t, arr[:, i], label=labels[i])
    ax1[k].set_title(key)
    ax1[k].grid(True, ls="--", lw=0.5)
ax1[0].legend(loc="upper right")

for j, key in enumerate(rope_keys, start=4):
    arr = data[key]
    if arr.ndim == 3:  # shape (T, n, 3) -> norm
        arr = np.linalg.norm(arr, axis=2)
    for i in range(arr.shape[1]):
        ax1[j].plot(t, arr[:, i], label=f"d{i}")
    ax1[j].set_title(key + "_norm")
    ax1[j].grid(True, ls="--", lw=0.5)
ax1[4].legend(loc="upper right")

fig1.supxlabel("time [s]")
fig1.supylabel("error")
fig1.tight_layout()

# ---------- fig2 q / w ----------
if all(k in data for k in ("q_ref", "q_act", "w_ref", "w_act")):

    n_drones = data["q_ref"].shape[1]

    if arg_id != "all":
        ids = [int(arg_id)]
    else:
        ids = range(n_drones)

    for did in ids:
        q_ref = data["q_ref"][:, did, :]
        q_act = data["q_act"][:, did, :]
        w_ref = data["w_ref"][:, did, :]
        w_act = data["w_act"][:, did, :]

        fig2, ax2 = plt.subplots(3, 2, figsize=(14, 9), sharex=True, num=f"Drone {did} q/w")
        ax2 = ax2.reshape(3, 2)

        for i in range(3):
            ax2[i, 0].plot(t, q_ref[:, i], "k--", label="q_ref" if i == 0 else "")
            ax2[i, 0].plot(t, q_act[:, i], label="q_act" if i == 0 else "")
            ax2[i, 0].set_ylabel(f"q_{labels[i]}")
            ax2[i, 0].grid(True, ls="--", lw=0.5)

            ax2[i, 1].plot(t, w_ref[:, i], "k--", label="w_ref" if i == 0 else "")
            ax2[i, 1].plot(t, w_act[:, i], label="w_act" if i == 0 else "")
            ax2[i, 1].set_ylabel(f"w_{labels[i]}")
            ax2[i, 1].grid(True, ls="--", lw=0.5)

        ax2[0, 0].set_title(f"Drone {did} q")
        ax2[0, 1].set_title(f"Drone {did} w")
        ax2[0, 0].legend()
        ax2[0, 1].legend()
        fig2.supxlabel("time [s]")
        fig2.tight_layout()

plt.show()

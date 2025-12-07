import argparse
import signal
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    print("\nExiting...")
    plt.close("all")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def load_traj(csv_path: Path, label: str):
    """Load a trajectory CSV and compute finite-difference velocities."""
    if not csv_path.exists():
        raise FileNotFoundError(f"{label} CSV not found: {csv_path}")

    df = pd.read_csv(csv_path, sep=",")
    time = df["time"].to_numpy()
    x = df["x"].to_numpy()
    y = df["y"].to_numpy()
    z = df["z"].to_numpy()
    vx = df["vx"].to_numpy()
    vy = df["vy"].to_numpy()
    vz = df["vz"].to_numpy()

    vx_from_pos = np.gradient(x, time)
    vy_from_pos = np.gradient(y, time)
    vz_from_pos = np.gradient(z, time)

    err_vx = vx - vx_from_pos
    err_vy = vy - vy_from_pos
    err_vz = vz - vz_from_pos

    rmse_vx = np.sqrt(np.mean(err_vx**2))
    rmse_vy = np.sqrt(np.mean(err_vy**2))
    rmse_vz = np.sqrt(np.mean(err_vz**2))

    dt_mean = float(np.mean(np.diff(time)))

    print(f"{label}: {len(time)} samples, mean dt={dt_mean:.4f} s")
    print(f"  RMSE(vx): {rmse_vx:.6e}, RMSE(vy): {rmse_vy:.6e}, RMSE(vz): {rmse_vz:.6e}")

    return {
        "label": label,
        "time": time,
        "x": x,
        "y": y,
        "z": z,
        "vx": vx,
        "vy": vy,
        "vz": vz,
        "vx_from_pos": vx_from_pos,
        "vy_from_pos": vy_from_pos,
        "vz_from_pos": vz_from_pos,
    }


def compute_dynamics(traj: dict):
    """Compute accel, jerk, attitude, and body rates from a trajectory."""
    t = traj["time"]
    vx_fd = traj["vx_from_pos"]
    vy_fd = traj["vy_from_pos"]
    vz_fd = traj["vz_from_pos"]
    ax_fd = np.gradient(vx_fd, t)
    ay_fd = np.gradient(vy_fd, t)
    az_fd = np.gradient(vz_fd, t)

    jx_fd = np.gradient(ax_fd, t)
    jy_fd = np.gradient(ay_fd, t)
    jz_fd = np.gradient(az_fd, t)

    # Attitude reconstruction
    g = 9.81
    e3 = np.array([0.0, 0.0, 1.0])
    a_vec = np.column_stack((ax_fd, ay_fd, az_fd))
    f_vec = g * e3 - a_vec
    f_norm = np.linalg.norm(f_vec, axis=1)
    f_norm = np.maximum(f_norm, 1e-6)
    z_b = f_vec / f_norm[:, None]

    psi_des = 0.0
    x_c = np.array([np.cos(psi_des), np.sin(psi_des), 0.0])
    y_c = np.array([-np.sin(psi_des), np.cos(psi_des), 0.0])

    R_list = []
    for zb in z_b:
        x_b = np.cross(y_c, zb)
        x_b_norm = np.linalg.norm(x_b)
        if x_b_norm < 1e-6:
            x_b = np.array([1.0, 0.0, 0.0])
            x_b_norm = 1.0
        x_b /= x_b_norm
        y_b = np.cross(zb, x_b)
        y_b /= np.linalg.norm(y_b)
        R_list.append(np.column_stack((x_b, y_b, zb)))

    R = np.stack(R_list)
    roll = np.arctan2(R[:, 2, 1], R[:, 2, 2])
    pitch = np.arcsin(-R[:, 2, 0])
    yaw = np.arctan2(R[:, 1, 0], R[:, 0, 0])

    T = f_norm
    f_dot = -np.column_stack((jx_fd, jy_fd, jz_fd))
    T_dot = np.sum(f_dot * z_b, axis=1)
    z_b_dot = (f_dot - T_dot[:, None] * z_b) / T[:, None]
    omega_world_perp = np.cross(z_b, z_b_dot)
    omega_body = np.einsum("nij,nj->ni", np.transpose(R, (0, 2, 1)), omega_world_perp)

    traj["ax"] = ax_fd
    traj["ay"] = ay_fd
    traj["az"] = az_fd
    traj["jx"] = jx_fd
    traj["jy"] = jy_fd
    traj["jz"] = jz_fd
    traj["roll"] = roll
    traj["pitch"] = pitch
    traj["yaw"] = yaw
    traj["p"] = omega_body[:, 0]
    traj["q"] = omega_body[:, 1]
    traj["r"] = omega_body[:, 2]
    return traj


def main():
    parser = argparse.ArgumentParser(description="Compare 20 Hz and 100 Hz trajectories.")
    parser.add_argument(
        "--drone-id", type=int, default=0, help="Drone ID to plot (default: 0)"
    )
    parser.add_argument(
        "--base-dir",
        type=Path,
        default=Path("./3drone_trajectories_new"),
        help="Directory containing *_raw_20hz.csv and *_smoothed_100hz.csv files",
    )
    args = parser.parse_args()

    raw_path = args.base_dir / f"drone_{args.drone_id}_traj_raw_20hz.csv"
    smoothed_path = args.base_dir / f"drone_{args.drone_id}_traj_smoothed_100hz.csv"

    traj_raw = load_traj(raw_path, "raw_20hz")
    traj_smooth = load_traj(smoothed_path, "smoothed_100hz")
    traj_raw = compute_dynamics(traj_raw)
    traj_smooth = compute_dynamics(traj_smooth)

    # Resample the 20 Hz data onto the 100 Hz timeline to visualize what linear interpolation looks like.
    t_ref = traj_smooth["time"]
    raw_resampled = {
        "x": np.interp(t_ref, traj_raw["time"], traj_raw["x"]),
        "y": np.interp(t_ref, traj_raw["time"], traj_raw["y"]),
        "z": np.interp(t_ref, traj_raw["time"], traj_raw["z"]),
        "vx": np.interp(t_ref, traj_raw["time"], traj_raw["vx"]),
        "vy": np.interp(t_ref, traj_raw["time"], traj_raw["vy"]),
        "vz": np.interp(t_ref, traj_raw["time"], traj_raw["vz"]),
    }

    # === 3D trajectory overlay ===
    fig1 = plt.figure(figsize=(8, 6))
    ax1 = fig1.add_subplot(111, projection="3d")
    ax1.plot(traj_raw["x"], traj_raw["y"], traj_raw["z"], label="raw 20 Hz")
    ax1.plot(traj_smooth["x"], traj_smooth["y"], traj_smooth["z"], label="smoothed 100 Hz")
    ax1.set_title("3D Trajectory (NED)")
    ax1.set_xlabel("North [m]")
    ax1.set_ylabel("East [m]")
    ax1.set_zlabel("Down [m]")
    ax1.legend()
    ax1.grid(True)

    # === Position vs time ===
    fig2, axs2 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for axis, key in enumerate(["x", "y", "z"]):
        axs2[axis].plot(traj_raw["time"], traj_raw[key], "C1", label="raw 20 Hz")
        axs2[axis].plot(traj_smooth["time"], traj_smooth[key], "C0", label="smoothed 100 Hz")
        axs2[axis].set_ylabel(f"{key} [m]")
        axs2[axis].grid(True)
    axs2[0].set_title("Position (NED)")
    axs2[-1].set_xlabel("time [s]")
    axs2[0].legend()

    # === Velocity vs time (CSV vs from position) ===
    fig3, axs3 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    vel_keys = [("vx", "vx_from_pos"), ("vy", "vy_from_pos"), ("vz", "vz_from_pos")]
    labels = ["x", "y", "z"]
    for axis, (vel_key, vel_pos_key) in enumerate(vel_keys):
        axs3[axis].plot(traj_raw["time"], traj_raw[vel_key], "C1", label="raw 20 Hz (csv)")
        axs3[axis].plot(
            traj_raw["time"], traj_raw[vel_pos_key], "C1--", label="raw 20 Hz (from pos)"
        )
        axs3[axis].plot(traj_smooth["time"], traj_smooth[vel_key], "C0", label="smoothed 100 Hz (csv)")
        axs3[axis].plot(
            traj_smooth["time"],
            traj_smooth[vel_pos_key],
            "C0--",
            label="smoothed 100 Hz (from pos)",
        )
        axs3[axis].set_ylabel(f"v{labels[axis]} [m/s]")
        axs3[axis].grid(True)
    axs3[0].set_title("Velocity comparison")
    axs3[-1].set_xlabel("time [s]")
    axs3[0].legend()

    # === Resampled 20 Hz vs 100 Hz on a common time grid ===
    fig4, axs4 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for axis, key in enumerate(["x", "y", "z"]):
        axs4[axis].plot(t_ref, traj_smooth[key], "C0", label="smoothed 100 Hz")
        axs4[axis].plot(
            t_ref, raw_resampled[key], "C1--", label="raw 20 Hz resampled (linear interp)"
        )
        axs4[axis].set_ylabel(f"{key} [m]")
        axs4[axis].grid(True)
    axs4[0].set_title("Raw 20 Hz interpolated to 100 Hz timeline")
    axs4[-1].set_xlabel("time [s]")
    axs4[0].legend()

    # === Acceleration, jerk, attitude, body rates (using smoothed 100 Hz) ===
    fig5, axs5 = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for traj, color, name in [(traj_raw, "C1", "raw 20 Hz"), (traj_smooth, "C0", "smoothed 100 Hz")]:
        axs5[0].plot(traj["time"], traj["ax"], color, label=f"ax ({name})")
        axs5[0].plot(traj["time"], traj["ay"], color, linestyle="--", label=f"ay ({name})")
        axs5[0].plot(traj["time"], traj["az"], color, linestyle=":", label=f"az ({name})")
    axs5[0].set_ylabel("Acceleration [m/s²]")
    axs5[0].set_title("Acceleration (from position)")
    axs5[0].legend()
    axs5[0].grid(True)

    for traj, color, name in [(traj_raw, "C1", "raw 20 Hz"), (traj_smooth, "C0", "smoothed 100 Hz")]:
        axs5[1].plot(traj["time"], traj["jx"], color, label=f"jx ({name})")
        axs5[1].plot(traj["time"], traj["jy"], color, linestyle="--", label=f"jy ({name})")
        axs5[1].plot(traj["time"], traj["jz"], color, linestyle=":", label=f"jz ({name})")
    axs5[1].set_ylabel("Jerk [m/s³]")
    axs5[1].set_xlabel("time [s]")
    axs5[1].set_title("Jerk (from position)")
    axs5[1].legend()
    axs5[1].grid(True)

    fig6, axs6 = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for traj, color, name in [(traj_raw, "C1", "raw 20 Hz"), (traj_smooth, "C0", "smoothed 100 Hz")]:
        axs6[0].plot(traj["time"], traj["roll"], color, label=f"roll ({name})")
        axs6[0].plot(traj["time"], traj["pitch"], color, linestyle="--", label=f"pitch ({name})")
        axs6[0].plot(traj["time"], traj["yaw"], color, linestyle=":", label=f"yaw ({name})")
    axs6[0].set_ylabel("Angle [rad]")
    axs6[0].set_title("Attitude (relative to NED)")
    axs6[0].legend()
    axs6[0].grid(True)

    for traj, color, name in [(traj_raw, "C1", "raw 20 Hz"), (traj_smooth, "C0", "smoothed 100 Hz")]:
        axs6[1].plot(traj["time"], traj["p"], color, label=f"p ({name})")
        axs6[1].plot(traj["time"], traj["q"], color, linestyle="--", label=f"q ({name})")
        axs6[1].plot(traj["time"], traj["r"], color, linestyle=":", label=f"r ({name})")
    axs6[1].set_ylabel("Rate [rad/s]")
    axs6[1].set_xlabel("time [s]")
    axs6[1].set_title("Body rates")
    axs6[1].legend()
    axs6[1].grid(True)

    print("Displaying plots. Press Ctrl+C to exit or close all plot windows.")
    plt.tight_layout()
    plt.show(block=True)


if __name__ == "__main__":
    main()

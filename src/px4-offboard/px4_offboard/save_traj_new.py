#!/usr/bin/env python3
"""
Save the new 3-quad trajectory to per-drone CSV files.
- Raw 20 Hz data from get_data_new.DataLoader.
- Smoothed 100 Hz version via local high-order polynomial fits to the offline data
  (positions preserved at raw samples) with analytic derivatives up to snap,
  plus attitude and body rates from differential flatness.
"""

import csv
import sys
from pathlib import Path

import numpy as np
try:
    import matplotlib.pyplot as plt
except Exception as exc:  # matplotlib may be missing or mismatched locally
    plt = None
    _PLOT_IMPORT_ERROR = exc
else:
    _PLOT_IMPORT_ERROR = None

# Allow running as script
FILE_DIR = Path(__file__).resolve().parent
REPO_ROOT = FILE_DIR.parents[3]
sys.path.append(str(FILE_DIR.parent))  # add px4_offboard package root

from px4_offboard.get_data_new import DataLoader


class TrajectorySaverNew:
    def __init__(self, output_dir: str | Path = "./3drone_trajectories_new"):
        self.loader = DataLoader()
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)

        self.num_drones = self.loader.num_drones
        self.dt = self.loader.dt
        self.target_dt = 0.01  # 100 Hz output
        self.rl = self.loader.rl
        self.cable_length = self.loader.cable_length
        self.alpha = self.loader.alpha
        self.T_enu2ned = np.array([[0, 1, 0],
                                   [1, 0, 0],
                                   [0, 0, -1]])
        self.g = self.loader.g
        self.ez = self.loader.ez.reshape(3)

    def compute_positions(self) -> np.ndarray:
        """Compute drone positions over time in NED (shape: num_drones x N x 3)."""
        N = self.loader.payload_x.shape[0]
        pos = np.zeros((self.num_drones, N, 3))

        # precompute payload attachment offsets
        ri = np.array(
            [
                [
                    self.rl * np.cos(i * self.alpha),
                    self.rl * np.sin(i * self.alpha),
                    0.0,
                ]
                for i in range(self.num_drones)
            ]
        )

        for i in range(self.num_drones):
            # transform payload pos and cable dir to NED
            payload_ned = (self.T_enu2ned @ self.loader.payload_x.T).T
            cable_dir_ned = (self.T_enu2ned @ self.loader.cable_direction[i].T).T
            pos[i] = payload_ned + ri[i] + self.cable_length * cable_dir_ned
        return pos

    def compute_velocities(self, pos: np.ndarray) -> np.ndarray:
        """Finite-difference velocities from positions."""
        return np.gradient(pos, self.dt, axis=1)

    def compute_accelerations(self, vel: np.ndarray) -> np.ndarray:
        """Finite-difference accelerations from velocities."""
        return np.gradient(vel, self.dt, axis=1)

    @staticmethod
    def _moving_average(series: np.ndarray, window: int = 3) -> np.ndarray:
        """
        Simple moving-average smoothing along the time axis.
        Expects array shaped [num_drones, N, 3].
        """
        if window <= 1:
            return series

        pad = window // 2
        kernel = np.ones(window) / window
        padded = np.pad(series, ((0, 0), (pad, pad), (0, 0)), mode="edge")
        smoothed = np.zeros_like(series, dtype=float)
        for axis in range(series.shape[2]):
            smoothed[..., axis] = np.apply_along_axis(
                lambda m: np.convolve(m, kernel, mode="valid"),
                axis=1,
                arr=padded[..., axis],
            )
        return smoothed

    def resample_polynomial(
        self,
        pos_raw: np.ndarray,
        time_raw: np.ndarray,
        time_dense: np.ndarray,
        window: int = 8,
        order: int = 7,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Local high-order polynomial fit per axis/drone.
        - Uses up to `window` raw samples (default 8) around each target time.
        - Fits up to `order` (default 7); reduced if fewer points available.
        - Preserves raw positions exactly at raw timestamps.
        Returns pos/vel/acc/jerk/snap on time_dense.
        """
        num_drones, _, _ = pos_raw.shape
        pos_out = np.zeros((num_drones, len(time_dense), 3))
        vel_out = np.zeros_like(pos_out)
        acc_out = np.zeros_like(pos_out)
        jerk_out = np.zeros_like(pos_out)
        snap_out = np.zeros_like(pos_out)

        half_w = window // 2
        for i in range(num_drones):
            for axis in range(3):
                values = pos_raw[i, :, axis]
                for idx_t, t in enumerate(time_dense):
                    # If this is exactly a raw sample, lock position to it.
                    close_mask = np.isclose(time_raw, t)
                    raw_idx = int(np.argmax(close_mask)) if np.any(close_mask) else np.searchsorted(time_raw, t)
                    is_raw = raw_idx < len(time_raw) and np.isclose(time_raw[raw_idx], t)
                    # Build window
                    center = raw_idx
                    start = max(0, center - half_w)
                    end = min(len(time_raw), start + window)
                    start = max(0, end - window)
                    t_win = time_raw[start:end]
                    v_win = values[start:end]
                    order_fit = min(order, len(t_win) - 1)
                    if order_fit < 1:
                        # Not enough points; fallback to nearest value
                        pos_out[i, idx_t, axis] = values[center if center < len(values) else -1]
                        continue
                    # Condition with time shift
                    t_shift = t_win - t
                    coeffs = np.polyfit(t_shift, v_win, order_fit)
                    # Evaluate polynomial and derivatives at zero (since shifted)
                    pos_val = np.polyval(coeffs, 0.0)
                    if is_raw:
                        pos_val = values[raw_idx]  # enforce exact raw position
                    vel_coeffs = np.polyder(coeffs, 1)
                    acc_coeffs = np.polyder(coeffs, 2)
                    jerk_coeffs = np.polyder(coeffs, 3)
                    snap_coeffs = np.polyder(coeffs, 4)
                    vel_val = np.polyval(vel_coeffs, 0.0)
                    acc_val = np.polyval(acc_coeffs, 0.0)
                    jerk_val = np.polyval(jerk_coeffs, 0.0)
                    snap_val = np.polyval(snap_coeffs, 0.0)

                    pos_out[i, idx_t, axis] = pos_val
                    vel_out[i, idx_t, axis] = vel_val
                    acc_out[i, idx_t, axis] = acc_val
                    jerk_out[i, idx_t, axis] = jerk_val
                    snap_out[i, idx_t, axis] = snap_val

        return pos_out, vel_out, acc_out, jerk_out, snap_out

    def compute_cable_derivatives(
        self,
        cable_dir: np.ndarray,
        cable_omega: np.ndarray,
        cable_omega_dot: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute r, rdot, rddot from cable direction and its angular rates.
        ṙ = ω × r
        r̈ = ω̇ × r + ω × ṙ
        jerk/snap will be obtained from polynomial fits on position directly.
        """
        rdot = np.cross(cable_omega, cable_dir, axis=2)
        rddot = np.cross(cable_omega_dot, cable_dir, axis=2) + np.cross(cable_omega, rdot, axis=2)
        return cable_dir, rdot, rddot


    def save_csv(
        self,
        pos: np.ndarray,
        vel: np.ndarray,
        time: np.ndarray,
        suffix: str,
        accel: np.ndarray | None = None,
        jerk: np.ndarray | None = None,
        snap: np.ndarray | None = None,
        roll: np.ndarray | None = None,
        pitch: np.ndarray | None = None,
        yaw: np.ndarray | None = None,
        p_rates: np.ndarray | None = None,
        q_rates: np.ndarray | None = None,
        r_rates: np.ndarray | None = None,
    ) -> None:
        """Save per-drone CSV with provided timebase. Accel/Jerk/Snap optional."""
        N = pos.shape[1]

        for i in range(self.num_drones):
            out_file = self.output_dir / f"drone_{i}_traj_{suffix}.csv"
            with out_file.open("w", newline="") as f:
                writer = csv.writer(f)
                headers = ["time", "x", "y", "z", "vx", "vy", "vz"]
                cols = [
                    time,
                    pos[i, :, 0],
                    pos[i, :, 1],
                    pos[i, :, 2],
                    vel[i, :, 0],
                    vel[i, :, 1],
                    vel[i, :, 2],
                ]

                if accel is not None:
                    headers += ["ax", "ay", "az"]
                    cols += [accel[i, :, 0], accel[i, :, 1], accel[i, :, 2]]
                if jerk is not None:
                    headers += ["jx", "jy", "jz"]
                    cols += [jerk[i, :, 0], jerk[i, :, 1], jerk[i, :, 2]]
                if snap is not None:
                    headers += ["sx", "sy", "sz"]
                    cols += [snap[i, :, 0], snap[i, :, 1], snap[i, :, 2]]
                if roll is not None and pitch is not None and yaw is not None:
                    headers += ["roll", "pitch", "yaw"]
                    cols += [roll[i, :], pitch[i, :], yaw[i, :]]
                if p_rates is not None and q_rates is not None and r_rates is not None:
                    headers += ["p", "q", "r"]
                    cols += [p_rates[i, :], q_rates[i, :], r_rates[i, :]]

                writer.writerow(headers)
                data = np.column_stack(cols)
                for row in data:
                    writer.writerow([f"{val:.6f}" for val in row])
            print(f"Saved {out_file}")

    def plot_comparison(
        self,
        pos_raw: np.ndarray,
        vel_raw: np.ndarray,
        time_raw: np.ndarray,
        pos_smooth: np.ndarray,
        vel_smooth: np.ndarray,
        time_smooth: np.ndarray,
        drone_idx: int = 0,
    ) -> None:
        """Plot raw 20 Hz vs smoothed 100 Hz position/velocity for one drone."""
        if plt is None:
            print(f"Skipping plots; matplotlib not available: {_PLOT_IMPORT_ERROR}")
            return

        labels = ["x", "y", "z"]
        fig, axes = plt.subplots(2, 3, figsize=(12, 6), sharex="row")

        for j, label in enumerate(labels):
            axes[0, j].plot(time_smooth, pos_smooth[drone_idx, :, j], label="smoothed 100 Hz")
            axes[0, j].plot(
                time_raw,
                pos_raw[drone_idx, :, j],
                "o",
                markersize=3,
                alpha=0.6,
                label="raw 20 Hz" if j == 0 else None,
            )
            axes[0, j].set_title(f"{label}-pos [m]")

            axes[1, j].plot(time_smooth, vel_smooth[drone_idx, :, j], label="smoothed 100 Hz")
            axes[1, j].plot(
                time_raw,
                vel_raw[drone_idx, :, j],
                "o",
                markersize=3,
                alpha=0.6,
                label="raw 20 Hz" if j == 0 else None,
            )
            axes[1, j].set_title(f"{label}-vel [m/s]")

        axes[0, 0].set_ylabel("Position")
        axes[1, 0].set_ylabel("Velocity")
        axes[1, 1].set_xlabel("Time [s]")
        axes[1, 2].set_xlabel("Time [s]")
        handles, labels_legend = axes[0, 0].get_legend_handles_labels()
        if handles:
            fig.legend(handles, labels_legend, loc="upper center", ncol=2)
        fig.tight_layout()
        out_file = self.output_dir / f"traj_compare_drone{drone_idx}.png"
        fig.savefig(out_file, dpi=200)
        plt.close(fig)
        print(f"Saved comparison plot to {out_file}")

    def compute_attitude_and_body_rates(
        self, accel: np.ndarray, jerk: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute attitude (roll, pitch, yaw) and body rates (p, q, r) from
        acceleration and jerk using differential flatness relationships.
        """
        N = accel.shape[1]
        roll = np.zeros((self.num_drones, N))
        pitch = np.zeros_like(roll)
        yaw = np.zeros_like(roll)
        p_rates = np.zeros_like(roll)
        q_rates = np.zeros_like(roll)
        r_rates = np.zeros_like(roll)

        g_vec = self.g * self.ez
        psi_des = 0.0
        x_c = np.array([np.cos(psi_des), np.sin(psi_des), 0.0])
        y_c = np.array([-np.sin(psi_des), np.cos(psi_des), 0.0])

        for i in range(self.num_drones):
            for k in range(N):
                a_k = accel[i, k, :]
                j_k = jerk[i, k, :]

                f_vec = g_vec - a_k
                T = np.linalg.norm(f_vec)
                if T < 1e-6:
                    T = 1e-6
                z_b = f_vec / T

                x_b = np.cross(y_c, z_b)
                norm_xb = np.linalg.norm(x_b)
                if norm_xb < 1e-6:
                    x_b = np.array([1.0, 0.0, 0.0])
                    norm_xb = 1.0
                x_b /= norm_xb
                y_b = np.cross(z_b, x_b)
                y_b /= np.linalg.norm(y_b)
                R = np.column_stack((x_b, y_b, z_b))

                roll[i, k] = np.arctan2(R[2, 1], R[2, 2])
                pitch[i, k] = np.arcsin(-R[2, 0])
                yaw[i, k] = np.arctan2(R[1, 0], R[0, 0])

                f_dot = -j_k
                T_dot = np.dot(f_dot, z_b)
                z_b_dot = (f_dot - T_dot * z_b) / T
                omega_world = np.cross(z_b, z_b_dot)
                omega_body = R.T @ omega_world

                p_rates[i, k] = omega_body[0]
                q_rates[i, k] = omega_body[1]
                r_rates[i, k] = omega_body[2]

        return roll, pitch, yaw, p_rates, q_rates, r_rates

    def run(self) -> None:
        # Timebases
        time_raw = np.arange(self.loader.payload_x.shape[0]) * self.dt
        time_dense_end = time_raw[-1]
        time_dense = np.arange(0.0, time_dense_end + self.target_dt / 2, self.target_dt)

        # Raw positions/velocities from offline data (NED)
        pos_raw = self.compute_positions()
        vel_raw = self.compute_velocities(pos_raw)
        acc_raw = self.compute_accelerations(vel_raw)
        jerk_raw = np.gradient(acc_raw, self.dt, axis=1)
        snap_raw = np.gradient(jerk_raw, self.dt, axis=1)

        # Attitude/body rates for raw
        self.accel = acc_raw
        self.jerk = jerk_raw
        self.snap = snap_raw
        (
            self.roll,
            self.pitch,
            self.yaw,
            self.p_rates,
            self.q_rates,
            self.r_rates,
        ) = self.compute_attitude_and_body_rates(acc_raw, jerk_raw)
        self.save_csv(
            pos_raw,
            vel_raw,
            time_raw,
            suffix="raw_20hz",
            accel=acc_raw,
            jerk=jerk_raw,
            snap=snap_raw,
            roll=self.roll,
            pitch=self.pitch,
            yaw=self.yaw,
            p_rates=self.p_rates,
            q_rates=self.q_rates,
            r_rates=self.r_rates,
        )

        # Smooth 100 Hz via local polynomial fits on the raw positions (per-axis, per-drone)
        pos_smooth, vel_smooth, acc_smooth, jerk_smooth, snap_smooth = self.resample_polynomial(
            pos_raw, time_raw, time_dense
        )

        # Attitude/body rates for smooth trajectory
        self.accel = acc_smooth
        self.jerk = jerk_smooth
        self.snap = snap_smooth
        (
            self.roll,
            self.pitch,
            self.yaw,
            self.p_rates,
            self.q_rates,
            self.r_rates,
        ) = self.compute_attitude_and_body_rates(acc_smooth, jerk_smooth)
        self.save_csv(
            pos_smooth,
            vel_smooth,
            time_dense,
            suffix="smoothed_100hz",
            accel=acc_smooth,
            jerk=jerk_smooth,
            snap=snap_smooth,
            roll=self.roll,
            pitch=self.pitch,
            yaw=self.yaw,
            p_rates=self.p_rates,
            q_rates=self.q_rates,
            r_rates=self.r_rates,
        )

        self.plot_comparison(pos_raw, vel_raw, time_raw, pos_smooth, vel_smooth, time_dense)


def main():
    saver = TrajectorySaverNew()
    saver.run()


if __name__ == "__main__":
    main()

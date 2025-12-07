# Reader for the updated offline trajectory (20 Hz, 6 s, 3-quad S-shape case)
# Class name kept as DataLoader for drop-in use. Missing legacy params are zeroed.

import math
from pathlib import Path

import numpy as np
from casadi import SX, Function, jacobian, mtimes, vertcat, horzcat

# Repository root
BASE_DIR = Path(__file__).resolve().parents[3]


class DataLoader:
    """
    Load planned trajectory data for the new COM_Dyn_H evaluation.
    - 3 drones, 20 Hz (dt = 0.05 s), 6 s horizon.
    - cable_omega_dot is stored inside xq_traj (indices 6:9).
    - Control inputs and feedback gains are provided as zeros for compatibility.
    """

    def __init__(self):
        self.num_drones = 3
        self.dt = 0.05  # 20 Hz
        self.traj_duration = 6.0
        self.train_idx = -1
        self.task_idx = 0
        self.rl = 0.25
        self.alpha = 2 * math.pi / self.num_drones
        self.cable_length = 1.0
        self.g = 9.81
        self.ml = 0.45  # m1 + m2 from setting.txt
        self.ez = np.array([0.0, 0.0, 1.0]).reshape(3, 1)
        self.rg = np.zeros(3)  # no offset saved

        # Paths
        self.path = BASE_DIR / "3quad_traj" / "Planning_plots_multiagent_meta_evaluation_COM_Dyn_H"
        reference_dir = self.path / "Reference_traj_6_S_shape_evaluation"

        # Reference trajectory coefficients (S-shape, 4 segments)
        self.Coeffx = np.zeros((4, 8))
        self.Coeffy = np.zeros((4, 8))
        self.Coeffz = np.zeros((4, 8))
        for k in range(4):
            self.Coeffx[k, :] = np.load(reference_dir / f"coeffx{k+1}.npy")
            self.Coeffy[k, :] = np.load(reference_dir / f"coeffy{k+1}.npy")
            self.Coeffz[k, :] = np.load(reference_dir / f"coeffz{k+1}.npy")

        # Payload trajectory (shape: [121, 13])
        xl_file = self.path / "xl_traj_0_3_a.npy"
        self.xl_traj = np.load(xl_file, allow_pickle=True)
        self.payload_x = self.xl_traj[:, 0:3]
        self.payload_v = self.xl_traj[:, 3:6]
        self.payload_q = self.xl_traj[:, 6:10]
        self.payload_w = self.xl_traj[:, 10:13]

        # Cable states (shape: [3, 121, 14])
        self.xq_traj = np.load(self.path / "xq_traj_0_3_a.npy", allow_pickle=True)
        self.cable_direction = self.xq_traj[:, :, 0:3]
        self.cable_omega = self.xq_traj[:, :, 3:6]
        self.cable_omega_dot = self.xq_traj[:, :, 6:9]  # now part of the state
        self.cable_mu = self.xq_traj[:, :, 12]
        self.cable_mu_dot = self.xq_traj[:, :, 13]

        # Legacy placeholders (zeros) for compatibility with previous code paths
        N = self.xq_traj.shape[1]
        self.uq_traj = np.zeros((self.num_drones, N, 3))
        self.Kb = np.zeros((N, 6, 13))

        # CasADi symbols for reference generation
        self.polyc = SX.sym("c", 1, 8)
        self.time = SX.sym("t")
        self.time0 = SX.sym("t0")
        self.pl = SX.sym("pl", 3, 1)
        self.vl = SX.sym("vl", 3, 1)
        self.ql = SX.sym("ql", 4, 1)
        self.wl = SX.sym("wl", 3, 1)
        self.xl = vertcat(self.pl, self.vl, self.ql, self.wl)
        self.nxl = self.xl.numel()
        self.Fl = SX.sym("Fl", 3, 1)
        self.Ml = SX.sym("Ml", 3, 1)
        self.Wl = vertcat(self.Fl, self.Ml)
        self.nWl = self.Wl.numel()

    def get_drone_pos(self):
        """
        Compute initial drone positions using the first payload pose and cable directions.
        """
        drone_pos = np.zeros((self.num_drones, 3))
        for i in range(self.num_drones):
            ri = np.array(
                [
                    self.rl * math.cos(i * self.alpha),
                    self.rl * math.sin(i * self.alpha),
                    0.0,
                ]
            )
            Rl = np.eye(3)
            drone_pos[i, :] = (
                self.payload_x[0]
                + Rl @ ri
                + self.cable_length * self.cable_direction[i, 0, :].reshape(3, 1).T
            )
        return drone_pos

    # Polynomial trajectory helpers
    def polytraj(self, coeff, time, time0):
        time_vec = vertcat(
            1,
            self.time - self.time0,
            (self.time - self.time0) ** 2,
            (self.time - self.time0) ** 3,
            (self.time - self.time0) ** 4,
            (self.time - self.time0) ** 5,
            (self.time - self.time0) ** 6,
            (self.time - self.time0) ** 7,
        )
        polyp = mtimes(self.polyc, time_vec)
        polyp_fn = Function(
            "ref_p",
            [self.polyc, self.time, self.time0],
            [polyp],
            ["pc0", "t0", "ti0"],
            ["ref_pf"],
        )
        ref_p = polyp_fn(pc0=coeff, t0=time, ti0=time0)["ref_pf"].full()
        polyv = jacobian(polyp, self.time)
        polyv_fn = Function(
            "ref_v",
            [self.polyc, self.time, self.time0],
            [polyv],
            ["pc0", "t0", "ti0"],
            ["ref_vf"],
        )
        ref_v = polyv_fn(pc0=coeff, t0=time, ti0=time0)["ref_vf"].full()
        polya = jacobian(polyv, self.time)
        polya_fn = Function(
            "ref_a",
            [self.polyc, self.time, self.time0],
            [polya],
            ["pc0", "t0", "ti0"],
            ["ref_af"],
        )
        ref_a = polya_fn(pc0=coeff, t0=time, ti0=time0)["ref_af"].full()
        polyj = jacobian(polya, self.time)
        polyj_fn = Function(
            "ref_j",
            [self.polyc, self.time, self.time0],
            [polyj],
            ["pc0", "t0", "ti0"],
            ["ref_jf"],
        )
        ref_j = polyj_fn(pc0=coeff, t0=time, ti0=time0)["ref_jf"].full()
        polys = jacobian(polyj, self.time)
        polys_fn = Function(
            "ref_s",
            [self.polyc, self.time, self.time0],
            [polys],
            ["pc0", "t0", "ti0"],
            ["ref_sf"],
        )
        ref_s = polys_fn(pc0=coeff, t0=time, ti0=time0)["ref_sf"].full()
        return ref_p, ref_v, ref_a, ref_j, ref_s

    def minisnap_load_S_shape(self, Coeffx, Coeffy, Coeffz, time, rg):
        """
        Reference generator mirrored from Dynamics_meta_learning_COM_Dyn.minisnap_load_S_shape.
        """
        t_switch = 0
        t1 = 2
        t2 = 1
        t3 = 1
        if time < t1:
            ref_px, ref_vx, ref_ax, ref_jx, ref_sx = self.polytraj(
                Coeffx[0, :], time, t_switch
            )
            ref_py, ref_vy, ref_ay, ref_jy, ref_sy = self.polytraj(
                Coeffy[0, :], time, t_switch
            )
            ref_pz, ref_vz, ref_az, ref_jz, ref_sz = self.polytraj(
                Coeffz[0, :], time, t_switch
            )
        elif time < (t1 + t2):
            ref_px, ref_vx, ref_ax, ref_jx, ref_sx = self.polytraj(
                Coeffx[1, :], time, t1 + t_switch
            )
            ref_py, ref_vy, ref_ay, ref_jy, ref_sy = self.polytraj(
                Coeffy[1, :], time, t1 + t_switch
            )
            ref_pz, ref_vz, ref_az, ref_jz, ref_sz = self.polytraj(
                Coeffz[1, :], time, t1 + t_switch
            )
        elif time < (t1 + t2 + t3):
            ref_px, ref_vx, ref_ax, ref_jx, ref_sx = self.polytraj(
                Coeffx[2, :], time, t1 + t2 + t_switch
            )
            ref_py, ref_vy, ref_ay, ref_jy, ref_sy = self.polytraj(
                Coeffy[2, :], time, t1 + t2 + t_switch
            )
            ref_pz, ref_vz, ref_az, ref_jz, ref_sz = self.polytraj(
                Coeffz[2, :], time, t1 + t2 + t_switch
            )
        else:
            ref_px, ref_vx, ref_ax, ref_jx, ref_sx = self.polytraj(
                Coeffx[3, :], time, t1 + t2 + t3 + t_switch
            )
            ref_py, ref_vy, ref_ay, ref_jy, ref_sy = self.polytraj(
                Coeffy[3, :], time, t1 + t2 + t3 + t_switch
            )
            ref_pz, ref_vz, ref_az, ref_jz, ref_sz = self.polytraj(
                Coeffz[3, :], time, t1 + t2 + t3 + t_switch
            )

        ref_p = (
            np.reshape(np.vstack((ref_px, ref_py, ref_pz)), (3, 1))
            + np.reshape(np.vstack((rg[0], rg[1], 0)), (3, 1))
        )
        ref_v = np.reshape(np.vstack((ref_vx, ref_vy, ref_vz)), (3, 1))
        ref_q = np.array([[1, 0, 0, 0]]).T
        ref_w = np.zeros((3, 1))
        ref_xl = np.reshape(np.vstack((ref_p, ref_v, ref_q, ref_w)), self.nxl)
        ref_a = np.reshape(np.vstack((ref_ax, ref_ay, ref_az)), (3, 1))
        ref_Fl = self.ml * (ref_a + self.g * self.ez)
        ref_ml = np.zeros((3, 1))
        ref_Wl = np.reshape(np.vstack((ref_Fl, ref_ml)), self.nWl)
        return ref_xl, ref_Wl


def main():
    loader = DataLoader()
    print("Payload trajectory:", loader.payload_x.shape)
    print("Cable direction:", loader.cable_direction.shape)
    print("Cable omega dot sample:", loader.cable_omega_dot[:, 0, :])
    print("Kb shape (zeros):", loader.Kb.shape)
    print("Initial drone positions:\n", loader.get_drone_pos())


if __name__ == "__main__":
    main()

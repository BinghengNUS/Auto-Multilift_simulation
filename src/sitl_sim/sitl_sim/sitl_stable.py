#!/usr/bin/env python3
"""payload_sitl_ros.py – drones init + payload pose pub (50 Hz) – fixed pub list"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

import omni.timeline as tl
from omni.isaac.core import World
import omni.isaac.core.utils.prims as prim_utils
from pxr import UsdGeom, UsdPhysics, Gf, PhysxSchema
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend, PX4MavlinkBackendConfig)
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from cable_model import RigidBodyRopes
from scipy.spatial.transform import Rotation
import rclpy
from geometry_msgs.msg import TransformStamped
from time import sleep

NUM_DRONES = 6
PUB_HZ     = 50.0

# ───────────────────────────────── world ────────────────────────────
class Sim:
    def __init__(self):
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        phys = self.world.get_physics_context(); phys.enable_gpu_dynamics(True)
        PhysxSchema.PhysxSceneAPI.Apply(self.world.stage.GetPrimAtPath("/physicsScene"))
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Flat Plane"])
        prim_utils.create_prim("/World/Light_Key", "SphereLight",
                               position=Gf.Vec3f(0, 0, 55),
                               attributes={"inputs:radius": 30.0,
                                           "inputs:intensity": 1e4})

# ─────────────────────────────── scene ─────────────────────────────
def spawn(sim, node, init_pubs):
    stg = sim.world.stage
    RigidBodyRopes().create(stg, num_ropes=NUM_DRONES,
                            rope_length=1.8, payload_mass=6, load_height=0.03)
    payload = stg.GetPrimAtPath("/World/CommonPayload/Payload")
    xf = UsdGeom.XformCache(); sleep(1)               # wait for USD

    for i in range(NUM_DRONES):
        box  = f"/World/Rope{i}/box{i}Actor"
        pos  = xf.GetLocalToWorldTransform(stg.GetPrimAtPath(box)).ExtractTranslation()
        dpos = pos + Gf.Vec3d(0, 0, 0.05)

        cfg = MultirotorConfig()
        cfg.backends = [PX4MavlinkBackend(PX4MavlinkBackendConfig({
            "vehicle_id": i,
            "px4_autolaunch": True,
            "px4_dir": sim.pg.px4_path,
            "px4_vehicle_model": sim.pg.px4_default_airframe,
        }))]

        prim = "/World/quadrotor" if i == 0 else f"/World/quadrotor_{i:02d}"
        Multirotor(prim, ROBOTS["Iris"], i, dpos, Rotation.identity().as_quat(),
                   config=cfg)

        joint = UsdPhysics.FixedJoint.Define(stg, f"/World/Rope{i}/droneJoint")
        joint.CreateBody0Rel().SetTargets([box])
        joint.CreateBody1Rel().SetTargets([f"{prim}/body"])
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, -0.06))

        # publish init pose immediately
        msg = TransformStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "world"; msg.child_frame_id = f"drone_{i}"
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, dpos)
        msg.transform.rotation.w = 1.0
        init_pubs[i].publish(msg)

    # ensure ROS2 sends
    rclpy.spin_once(node, timeout_sec=0.0)
    return payload

# ─────────────────────────────── main ──────────────────────────────
def main():
    rclpy.init(); node = rclpy.create_node("sitl_pose_pub")

    init_pubs = [node.create_publisher(TransformStamped,
                                       f"/drone_{i}_init_pos", 1)
                 for i in range(NUM_DRONES)]
    pay_pub   = node.create_publisher(TransformStamped, "/payload_pose", 1)

    sim     = Sim()
    payload = spawn(sim, node, init_pubs)

    dt          = sim.world.get_physics_dt()
    step_target = max(1, round(1.0 / (PUB_HZ * dt)))
    steps       = 0
    xf_cache    = UsdGeom.XformCache()          # reuse, Clear() each pub
    pay_msg     = TransformStamped(); pay_msg.header.frame_id = "world"; pay_msg.child_frame_id = "payload"

    def cb(_):                                  # physics callback
        nonlocal steps
        steps += 1
        if steps < step_target:
            return
        steps = 0

        xf_cache.Clear()
        tf   = xf_cache.GetLocalToWorldTransform(payload)
        pos  = tf.ExtractTranslation()
        quat = tf.ExtractRotation().GetQuat()

        pay_msg.header.stamp = node.get_clock().now().to_msg()
        pay_msg.transform.translation.x = float(pos[0])
        pay_msg.transform.translation.y = float(pos[1])
        pay_msg.transform.translation.z = float(pos[2])
        imag = quat.GetImaginary()
        pay_msg.transform.rotation.x = float(imag[0])
        pay_msg.transform.rotation.y = float(imag[1])
        pay_msg.transform.rotation.z = float(imag[2])
        pay_msg.transform.rotation.w = float(quat.GetReal())
        pay_pub.publish(pay_msg)

        rclpy.spin_once(node, timeout_sec=0.0)

    sim.world.add_physics_callback("payload_pub", cb)

    w = sim.world; tli = tl.get_timeline_interface()
    w.reset(); tli.play()
    while simulation_app.is_running(): w.step(render=True)
    tli.stop(); simulation_app.close()

    node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()

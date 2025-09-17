#!/usr/bin/env python3
"""
payload_sitl_ros.py
• ONE-SHOT  /drone_i_init_pos   (i = 0…5)
• 50 Hz     /payload_pose       (physics callback)
"""

# ── Isaac-Sim bootstrap ─────────────────────────────────────────────
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

# ── Core / Pegasus / USD ────────────────────────────────────────────
import omni.timeline, omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core import World
from pxr import UsdGeom, UsdPhysics, Gf, PhysxSchema
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend, PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from cable_model import RigidBodyRopes
from scipy.spatial.transform import Rotation
from time import sleep

# ── ROS 2 ───────────────────────────────────────────────────────────
import rclpy
from geometry_msgs.msg import TransformStamped

# ===================================================================
class PegasusApp:
    def __init__(self):
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        phys = self.world.get_physics_context()
        phys.enable_gpu_dynamics(True)
        PhysxSchema.PhysxSceneAPI.Apply(self.world.stage.GetPrimAtPath("/physicsScene"))
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Flat Plane"])

    def run(self):
        tl = omni.timeline.get_timeline_interface()
        self.world.reset(); tl.play()
        while simulation_app.is_running():
            self.world.step(render=True)
        tl.stop(); simulation_app.close()

# -------------------------------------------------------------------
def spawn_scene(app: PegasusApp, node, init_pubs):
    """Spawn payload/drones and publish init poses. Return payload prim."""
    stg = app.world.stage
    prim_utils.create_prim("/World/Light_Key", "SphereLight",
                           position=Gf.Vec3f(0, 0, 55),
                           attributes={"inputs:radius": 30.0,
                                       "inputs:intensity": 1e4})

    # ropes + payload
    RigidBodyRopes().create(stg, num_ropes=6, rope_length=1.8,
                            payload_mass=6, load_height=0.03)
    payload = stg.GetPrimAtPath("/World/CommonPayload/Payload")

    xf = UsdGeom.XformCache()
    sleep(1.0)                       # wait for USD update

    for i in range(6):
        box = f"/World/Rope{i}/box{i}Actor"
        box_pos = xf.GetLocalToWorldTransform(stg.GetPrimAtPath(box)).ExtractTranslation()
        drone_pos = box_pos + Gf.Vec3d(0, 0, 0.05)

        cfg = MultirotorConfig()
        cfg.backends = [PX4MavlinkBackend(PX4MavlinkBackendConfig({
            "vehicle_id": i,
            "px4_autolaunch": True,
            "px4_dir": app.pg.px4_path,
            "px4_vehicle_model": app.pg.px4_default_airframe,
        }))]

        prim = "/World/quadrotor" if i == 0 else f"/World/quadrotor_0{i}"
        Multirotor(prim, ROBOTS["Iris"], i, drone_pos,
                   Rotation.identity().as_quat(), config=cfg)

        joint = UsdPhysics.FixedJoint.Define(stg, f"/World/Rope{i}/droneJoint")
        joint.CreateBody0Rel().SetTargets([box])
        joint.CreateBody1Rel().SetTargets([f"{prim}/body"])
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, -0.06))

        # publish init pose once
        msg = TransformStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id  = f"drone_{i}"
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, drone_pos)
        msg.transform.rotation.w = 1.0
        init_pubs[i].publish(msg)

    return payload

# -------------------------------------------------------------------
def main():
    # ROS
    rclpy.init()
    node = rclpy.create_node("sitl_pose_pub")

    init_pubs   = [node.create_publisher(TransformStamped,
                                         f"/drone_{i}_init_pos", 10) for i in range(6)]
    payload_pub = node.create_publisher(TransformStamped, "/payload_pose", 10)

    # world + scene
    app       = PegasusApp()
    payload   = spawn_scene(app, node, init_pubs)
    rclpy.spin_once(node, timeout_sec=0.1)            # flush init messages

    # physics callback at 50 Hz
    physics_dt    = app.world.get_physics_dt() 
    step_target   = max(1, round(1.0 / (50.0 * physics_dt)))  # steps per pub
    step_counter  = 0

    def cb(_):                                           # _ = simulationStep
        nonlocal step_counter
        step_counter += 1
        if step_counter < step_target:
            rclpy.spin_once(node, timeout_sec=0.0)                   # keep ROS alive
            return
        step_counter = 0
        xf_cache = UsdGeom.XformCache()
        tf  = xf_cache.GetLocalToWorldTransform(payload)
        pos = tf.ExtractTranslation()
        quat = tf.ExtractRotation().GetQuat()            # Gf.Quatf / Quatd

        msg = TransformStamped()
        msg.header.stamp    = node.get_clock().now().to_msg()
        msg.header.frame_id = "world"; msg.child_frame_id = "payload"
        msg.transform.translation.x = float(pos[0])
        msg.transform.translation.y = float(pos[1])
        msg.transform.translation.z = float(pos[2])

        imag = quat.GetImaginary()                       # Vec3d
        msg.transform.rotation.x = float(imag[0])
        msg.transform.rotation.y = float(imag[1])
        msg.transform.rotation.z = float(imag[2])
        msg.transform.rotation.w = float(quat.GetReal())
        payload_pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)

    app.world.add_physics_callback("pub_payload", cb)

    try:
        app.run()
    finally:
        node.destroy_node(); rclpy.shutdown()

# -------------------------------------------------------------------
if __name__ == "__main__":
    main()

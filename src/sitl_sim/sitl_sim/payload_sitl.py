#!/usr/bin/env python
"""
| File: payload_sitl.py
| Description: SITL payload example with initial-pos publishing
| Author: Yichao Gao
"""
import carb
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})
import omni.timeline
from omni.isaac.core.world import World
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.dynamic_control import _dynamic_control as dc
from pxr import UsdGeom, PhysxSchema, Gf, UsdPhysics
from time import sleep

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
PX4MavlinkBackend,
PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped

from cable_model import RigidBodyRopes

class PegasusApp:
    """Initialize IsaacSim + Pegasus world and run stepping loop."""
    def __init__(self):
        self.sim_app = simulation_app
        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        stage = self.world.stage
        phys = self.world.get_physics_context()
        phys.enable_gpu_dynamics(True)
        phys.enable_stablization(False)
        phys.enable_ccd(False)
        phys.set_gpu_max_num_partitions(32)
        PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))

        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Flat Plane"])

    def run(self):
        """Start the simulation loop until the window closes."""
        self.world.reset()
        self.timeline.play()
        while self.sim_app.is_running():
            self.world.step(render=True)
        carb.log_warn("Simulation closing.")
        self.timeline.stop()
        self.sim_app.close()



def spawn_and_publish(pg_app: PegasusApp, node: Node, pubs):
    """
    Create ropes + drones, attach joints, then publish each drone's init pos.
    - pg_app: initialized PegasusApp
    - node: ROS2 Node for publishing
    - pubs: list of TransformStamped publishers
    """
    stage = pg_app.world.stage

    # lighting
    prim_utils.create_prim(
        "/World/Light_1", "SphereLight",
        position=Gf.Vec3f(0, 0, 55),
        attributes={
            "inputs:radius": 35.0,
            "inputs:intensity": 1e4,
            "inputs:color": (1, 1, 1)
        }
    )

    # build payload + cables
    ropes = RigidBodyRopes()
    ropes.create(
        stage,
        num_ropes=6,
        rope_length=1.8,
        payload_mass=6,
        load_height=0.03,
        elevation_angle=0
    )

    xf = UsdGeom.XformCache()
    sleep(1)  # allow all prims to settle

    # for each rope, spawn a drone and publish its start pose
    for i in range(6):
        box_path = f"/World/Rope{i}/box{i}Actor"
        box_prim = stage.GetPrimAtPath(box_path)
        pos = xf.GetLocalToWorldTransform(box_prim).ExtractTranslation()
        drone_pos = pos + Gf.Vec3d(0, 0, 0.05)

        # configure PX4 backend
        mcfg = MultirotorConfig()
        mbcfg = PX4MavlinkBackendConfig({
            "vehicle_id": i,
            "px4_autolaunch": True,
            "px4_dir": pg_app.pg.px4_path,
            "px4_vehicle_model": pg_app.pg.px4_default_airframe,
        })
        mcfg.backends = [PX4MavlinkBackend(mbcfg)]

        prim_path = "/World/quadrotor" if i == 0 else f"/World/quadrotor_0{i}"
        Multirotor(
            prim_path,
            ROBOTS["Iris"],
            i,
            drone_pos,
            Rotation.from_euler("XYZ", [0, 0, 0], degrees=True).as_quat(),
            config=mcfg
        )

        # attach physics joint
        joint = UsdPhysics.Joint.Define(stage, f"/World/Rope{i}/droneJoint")
        joint.CreateBody0Rel().SetTargets([box_path])
        joint.CreateBody1Rel().SetTargets([f"{prim_path}/body"])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, -0.06))

        # publish initial position
        msg = TransformStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id = f"drone_{i}"
        msg.transform.translation.x = float(drone_pos[0])
        msg.transform.translation.y = float(drone_pos[1])
        msg.transform.translation.z = float(drone_pos[2])
        msg.transform.rotation.w = 1.0  # identity quaternion
        pubs[i].publish(msg)



def main():
    rclpy.init()
    node = Node("drone_init_pos_publisher")

    # create one publisher per drone
    num = 6
    publishers = [
        node.create_publisher(TransformStamped, f"drone_{i}_init_pos", 10)
        for i in range(num)
    ]

    # setup simulation and spawn everything
    pg_app = PegasusApp()
    spawn_and_publish(pg_app, node, publishers)

    # allow ROS2 to send the messages
    rclpy.spin_once(node, timeout_sec=0.1)

    # start simulation loop
    pg_app.run()

    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
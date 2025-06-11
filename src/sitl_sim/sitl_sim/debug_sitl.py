#!/usr/bin/env python3
"""
payload_sitl_ros_graph.py
Isaac Sim + Pegasus multi-drone payload demo.
"""

#  std / 3-party 
import os, numpy as np, time
from scipy.spatial.transform import Rotation

#  Isaac Sim core 
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})
import omni.timeline as tl
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")                   # ROS 2 bridge

import omni.graph.core as og
g = og.Controller                                           # graph helper
from pxr import UsdGeom, UsdPhysics, Gf
from omni.isaac.core import World
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.prims import RigidPrim

#  Pegasus helpers 
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend, PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS

#  ROS 2 (only for one-shot TF) 
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import TransformStamped, TwistStamped

#  local rope model 
from cable_model import RigidBodyRopes
from time import sleep
import time
# from omni.isaac.sensor import IMUSensor

#  constants 
NUM_DRONES   = 3
ROS_PUB_HZ   = 50.0
PAYLOAD_PRIM = "/World/CommonPayload/Payload"

# 
#  world helper
# 
class Sim:
    def __init__(self):
        self.pg        = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world     = self.pg.world

        phys_ctx = self.world.get_physics_context()
        phys_ctx.enable_gpu_dynamics(True)
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Flat Plane"])

        prim_utils.create_prim(
            "/World/Light_Key", "SphereLight",
            position=Gf.Vec3f(0, 0, 55),
            attributes={"inputs:radius": 30.0,
                        "inputs:intensity": 1e4},
        )

# 
#  Scene builder
# 
def spawn_scene(sim: Sim, node, init_pubs) -> None:
    stage = sim.world.stage
    RigidBodyRopes().create(stage, NUM_DRONES, 1.2, 3, 0.01)
    xf_cache = UsdGeom.XformCache()
    time_spawn = time.time()
    # tl.get_timeline_interface().play()
    # while (time.time() - time_spawn ) < 1.0:
    #     sim.world.step()
    sleep(1.0)                                             # wait for USD build

    for i in range(NUM_DRONES):
        box_path  = f"/World/Rope{i}/box{i}Actor"
        box_pos   = xf_cache.GetLocalToWorldTransform(stage.GetPrimAtPath(box_path)).ExtractTranslation()
        drone_pos = box_pos + Gf.Vec3d(0, 0, 0.05)

        cfg = MultirotorConfig()
        cfg.backends = [PX4MavlinkBackend(PX4MavlinkBackendConfig({
            "vehicle_id":        i,
            "px4_autolaunch":    True,
            "px4_dir":           sim.pg.px4_path,
            "px4_vehicle_model": sim.pg.px4_default_airframe,
            # "px4_vehicle_model": "raynor",
        }))]

        prim_name = "/World/quadrotor" if i == 0 else f"/World/quadrotor_{i:02d}"
        Multirotor(prim_name, ROBOTS["Iris"], i,
                   drone_pos, Rotation.identity().as_quat(), config=cfg)

        # weld rope <-> drone
        joint = UsdPhysics.FixedJoint.Define(stage, f"/World/Rope{i}/droneJoint")
        joint.CreateBody0Rel().SetTargets([box_path])
        joint.CreateBody1Rel().SetTargets([f"{prim_name}/body"])
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, -0.05))

        tf = TransformStamped()
        tf.header.stamp    = node.get_clock().now().to_msg()
        tf.header.frame_id = "world"
        tf.child_frame_id  = f"drone_{i}"
        x, y, z = map(float, drone_pos)
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = x, y, z
        tf.transform.rotation.w = 1.0
        init_pubs[i].publish(tf)

# 
#  OmniGraph – 50 Hz publishers
# 
def build_ros_graph() -> None:
    graph = "/ActionGraph"
    og.Controller.edit(
        {
            "graph_path": graph, 
            "evaluator_name": "execution", 
            # "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
        {
            og.Controller.Keys.CREATE_NODES: [
                # ("OnTick",        "omni.graph.action.OnTick"),
                ("OnPhysics",   "omni.isaac.core_nodes.OnPhysicsStep"),
                ("Gate",         "omni.isaac.core_nodes.IsaacSimulationGate"),
                ("ReadSimTime",  "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("Odom",         "omni.isaac.core_nodes.IsaacComputeOdometry"),
                # ("IMUsensor",    "omni.isaac.sensor.IsaacReadIMU"),
                ("Ctx",          "omni.isaac.ros2_bridge.ROS2Context"),
                ("PubOdom",      "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PubOdom.inputs:topicName",     "/payload_odom"),
                ("PubOdom.inputs:odomFrameId",   "world"),     
                ("PubOdom.inputs:chassisFrameId","world"), 
                ("Gate.inputs:step", 2),    # 50
                ("Odom.inputs:chassisPrim", "/World/CommonPayload/Payload"),
                # ("IMUsensor.inputs:imuPrim", "/World/CommonPayload/Payload/imu_sensor"),
            ],
            og.Controller.Keys.CONNECT: [
                # ("OnTick.outputs:tick", "Gate.inputs:execIn"),
                ("OnPhysics.outputs:step", "Gate.inputs:execIn"),
                ("Gate.outputs:execOut",    "Odom.inputs:execIn"),
                ("Gate.outputs:execOut",    "PubOdom.inputs:execIn"),   
                # ("Gate.outputs:execOut",    "IMUsensor.inputs:execIn"),   
                ("ReadSimTime.outputs:simulationTime",  "PubOdom.inputs:timeStamp"),
                ("Odom.outputs:position",        "PubOdom.inputs:position"),
                ("Odom.outputs:orientation",     "PubOdom.inputs:orientation"),
                ("Odom.outputs:linearVelocity",  "PubOdom.inputs:linearVelocity"),
                ("Odom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
                # ("IMUsensor.outputs:angVel",     "PubOdom.inputs:angularVelocity"),
                # ROS2 context
                ("Ctx.outputs:context",          "PubOdom.inputs:context"),
            ],
        },
    )
    return graph

#  main
def main() -> None:
    # keep a few cores free for PX4 / DDS
    try:
        os.sched_setaffinity(0, set(range(max(1, os.cpu_count() - 4))))
    except AttributeError:
        pass

    sim = Sim()
    #  one-shot “/drone_i_init_pos” TFs 
    rclpy.init()
    node = rclpy.create_node("sim_tf_publisher")
    init_tf = [node.create_publisher(TransformStamped, f"/drone_{i}_init_pos", 1)
               for i in range(NUM_DRONES)]
    spawn_scene(sim, node, init_tf)

    #  build graph & run sim 
    graph_path = build_ros_graph()
    og.Controller.evaluate_sync(graph_path)
    simulation_app.update()                           # instantiate graph
    sim.world.reset()
    tl.get_timeline_interface().play()
    while simulation_app.is_running():
        # sim.world.step(render = False)                  # PhysX step
        sim.world.step()                     # PhysX step + render

    # clean shutdown
    simulation_app.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

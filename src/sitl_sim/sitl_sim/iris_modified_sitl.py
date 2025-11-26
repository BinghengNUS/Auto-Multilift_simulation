#!/usr/bin/env python3
"""
payload_sitl_ros_graph.py
Isaac Sim + Pegasus multi-drone payload demo.
"""

#  std / 3-party 
import os
from datetime import datetime
from scipy.spatial.transform import Rotation
import threading

#  Isaac Sim core 
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})
# simulation_app = SimulationApp({"headless": True})
import omni.timeline as tl
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")                   # ROS 2 bridge

import omni.graph.core as og
g = og.Controller                                           # omni graph helper
from pxr import UsdGeom, UsdPhysics, Gf, PhysxSchema
from omni.isaac.core import World
import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.core.prims import RigidPrim
from omni.kit.viewport.utility import get_active_viewport
from omni.isaac.core.objects import VisualCylinder, VisualSphere, DynamicSphere

import numpy as np

#  Pegasus
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend, PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS, ROBOTS_CONFIG

#  ROS 2 (only for one-shot TF) 
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import TransformStamped, TwistStamped

#  local rope model 
from cable_model import RigidBodyRopes
from time import sleep
# from omni.isaac.sensor import IMUSensor
# create Effort Sensor for cable tension measurement
# from omni.isaac.sensor.scripts.effort_sensor import EffortSensor
# for screen shot
from omni.isaac.sensor import Camera
import cv2
import rclpy
import usdrt.Sdf

#  constants 
NUM_DRONES   = 6
ROS_PUB_HZ   = 100.0
PAYLOAD_PRIM = "/World/CommonPayload/Payload"
CAMERA_STAGE_PATH = "/World/camera"
ROS_CAMERA_GRAPH_PATH = '/ROS_cam_graph'
PAYLOAD_GRAPH_PATH = "/ActionGraph"

# 
#  world helper
# 
class Sim:
    def __init__(self):
        self.pg        = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world     = self.pg.world

        phys_ctx = self.world.get_physics_context()
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])


        prim_utils.create_prim(
            "/World/Light_Key", "SphereLight",
            position=Gf.Vec3f(1.0, 2.10, 55),
            attributes={"inputs:radius": 30.0,
                        "inputs:intensity": 4e3},
        )

class CameraCapture:
    def __init__(self, sim, camera_path,sim_time, start_time) -> None:
        # get the simulation interface
        self.sim = sim
        # time config
        self.period = 0.50
        self.sim_time = sim_time
        self.start_time = start_time
        # ROS2 config (Threading to avoid blocking)
        self._ros_spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._ros_spin_thread.start()
        # save config
        time_str = datetime.now().strftime("%Y%m%d-%H%M%S")
        save_dir = f"/home/carlson/lift_log/captures/{NUM_DRONES}drones/{time_str}/"
        self.folder_path = os.path.join(save_dir)
        os.makedirs(self.folder_path, exist_ok=True)

    def _spin(self):
        rclpy.spin(self.node)
    

    def capture(self) -> None:
        pass

    def save_img(self) -> None:
        pass
        

def create_marker(sim, name, radius, position, path, color) -> None:
    """Create a marker at `position` with `radius`."""
    marker_prim = DynamicSphere(
            prim_path = path,
            name = name,
            position = position,
            radius = radius,
            color=color,
            mass=0.00001,
        )
    # marker_prim.disable_rigid_body_physics()
    sim.world.scene.add(
        marker_prim
    )
    marker_collisionAPI = UsdPhysics.CollisionAPI.Apply(marker_prim.prim)
    marker_collisionAPI.CreateCollisionEnabledAttr().Set(False)

#  Scene builder
def spawn_scene(sim: Sim, node, init_pubs) -> None:
    stage = sim.world.stage
    # RigidBodyRopes().create(stage, NUM_DRONES, 1.0, 2.0, 0.05)
    RigidBodyRopes().create(stage, NUM_DRONES, 1.0, 1.50, 0.05)
    xf_cache = UsdGeom.XformCache()
    sleep(1.0)                                             # wait for USD build
    angle = 2 * np.pi / NUM_DRONES
    # cable_effort_sensor_path = []

    for i in range(NUM_DRONES):
        # box_path  = f"/World/Rope{i}/Rope{i}HookSphere"
        box_path  = f"/World/Rope{i}/box{i}Actor"
        box_pos   = xf_cache.GetLocalToWorldTransform(stage.GetPrimAtPath(box_path)).ExtractTranslation()
        drone_pos = box_pos + Gf.Vec3d(0, 0, 0.017)

        cfg = MultirotorConfig()
        # cfg = sim.pg.generate_quadrotor_config_from_yaml(ROBOTS_CONFIG["Raynor"])
        cfg.backends = [PX4MavlinkBackend(PX4MavlinkBackendConfig({
            "vehicle_id":        i,
            "px4_autolaunch":    True,
            "px4_dir":           sim.pg.px4_path,
            # "px4_vehicle_model": sim.pg.px4_default_airframe,
            "px4_vehicle_model": "iris_modified",
            # "input_scaling": [5400, 5400, 5400, 5400], # For raynor
        }))]

        prim_name = f"/World/quadrotor" if i == 0 else f"/World/quadrotor_{i:02d}"
        Multirotor(prim_name, ROBOTS["Iris_modified"], i,
                   drone_pos, Rotation.from_euler("XYZ", [0.0, 0.0, angle*i], degrees=False).as_quat(), config=cfg)
        # # weld rope <-> drone
        joint = UsdPhysics.SphericalJoint.Define(stage, f"/World/Rope{i}/droneJoint")
        # # joint.CreateAxisAttr("Y")
        joint.CreateBody1Rel().SetTargets([f"{prim_name}/body"])
        joint.CreateBody0Rel().SetTargets([box_path])
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, -0.017))

        tf = TransformStamped()
        tf.header.stamp    = node.get_clock().now().to_msg()
        tf.header.frame_id = "world"
        tf.child_frame_id  = f"drone_{i}"
        x, y, z = map(float, drone_pos)
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = x, y, z
        tf.transform.rotation.w = 1.0
        init_pubs[i].publish(tf)


    # generate obsticle
    obsticle_1_pos = np.array([1.60,1.15,1])
    obsticle_2_pos = np.array([0.35,3.20,1])
    obsticle_1 = sim.world.scene.add(
        VisualCylinder(
            prim_path = '/World/Obstical_1',
            name = 'VisualCylinder1',
            position = obsticle_1_pos,
            radius = 0.70,
            height = 2.0,
            color=(np.array([255, 32, 32], dtype=float) / 255.0),  # light coral color
        )
    )
    obsticle_2 = sim.world.scene.add(
        VisualCylinder(
            prim_path = '/World/Obstical_2',
            name = 'VisualCylinder2',
            position = obsticle_2_pos,
            radius = 0.70,
            height = 2.0,
            color=(np.array([255, 32, 32], dtype=float) / 255.0),
        )
    )

    # generate marker for CoM 
    CoM_marker_path = '/World/CommonPayload/CoM_marker'
    CoM_marker_color = (np.array([128, 0, 0], dtype=float) / 255.0)  # -> (1.0, 0.0, 0.0)
    CoM_marker_name = "CoM_marker"
    # CoG_marker_path = '/World/CommonPayload/CoG_marker'
    # CoG_marker_color = (np.array([0, 100, 0], dtype=float) / 255.0) # -> (0.133, 0.545, 0.133)
    # CoG_marker_name = "CoG_marker"
    payload_path = '/World/CommonPayload/Payload'
    payload_prim = stage.GetPrimAtPath(payload_path)
    payload_massAPI = UsdPhysics.MassAPI.Apply(payload_prim)
    CoM_pos = payload_massAPI.GetCenterOfMassAttr().Get()
    height_offset = Gf.Vec3f(0, 0, 0.04)
    CoM_marker_pos = CoM_pos + height_offset
    # CoG_marker_pos = height_offset
    create_marker(sim=sim, radius=0.015, name=CoM_marker_name, path=CoM_marker_path, position=CoM_marker_pos, color=CoM_marker_color)
    # create_marker(sim=sim, radius=0.005, name=CoG_marker_name, path=CoG_marker_path, position=CoG_marker_pos, color=CoG_marker_color)
    CoM_joint = UsdPhysics.FixedJoint.Define(stage, f'{CoM_marker_path}/CoMmarkerJoint')
    CoM_joint.CreateBody0Rel().SetTargets([CoM_marker_path])
    CoM_joint.CreateBody1Rel().SetTargets([payload_path])
    # CoG_joint = UsdPhysics.FixedJoint.Define(stage, f'{CoG_marker_path}/CoGmarkerJoint')
    # CoG_joint.CreateBody0Rel().SetTargets([CoG_marker_path])
    # CoG_joint.CreateBody1Rel().SetTargets([payload_path])

    # setup the camera model
    camera_pos = (obsticle_1_pos + obsticle_2_pos) / 2 + np.array([0.0, 0.0, 8.0])
    camera = Camera(
        prim_path=CAMERA_STAGE_PATH,
        position=camera_pos,  # 2 meter away from the side of the cube
        resolution=(2560, 1440),
        orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, -55]), degrees=True),
    )
    camera.set_focal_length(3.0)
    camera.initialize()

    
    

#  OmniGraph
def build_payload_ros_graph():
    og.Controller.edit(
        {
            "graph_path": PAYLOAD_GRAPH_PATH, 
            "evaluator_name": "execution",  # use execution for sim
            # "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
        {
            og.Controller.Keys.CREATE_NODES: [
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
                ("Gate.inputs:step", 2),   
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
    # return graph

def build_capture_graph():
    keys = og.Controller.Keys
    (ros_camera_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": ROS_CAMERA_GRAPH_PATH, 
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                ("getRenderProduct", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
                ("setCamera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
                ("cameraHelperRgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
                ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("createViewport.inputs:viewportId", 0),
                ("cameraHelperRgb.inputs:frameId", "sim_camera"),
                ("cameraHelperRgb.inputs:topicName", "rgb"),
                ("cameraHelperRgb.inputs:type", "rgb"),
                ("setCamera.inputs:cameraPrim", [usdrt.Sdf.Path(CAMERA_STAGE_PATH)]),
            ],
        },
    )


def main() -> None:
    sim = Sim()
    rclpy.init()
    node = rclpy.create_node("sim_tf_publisher")
    init_tf = [node.create_publisher(TransformStamped, f"/drone_{i}_init_pos", 1)
               for i in range(NUM_DRONES)]
    spawn_scene(sim, node, init_tf)

    #  build graph & run sim 
    build_payload_ros_graph()
    build_capture_graph()
    og.Controller.evaluate_sync(PAYLOAD_GRAPH_PATH)
    og.Controller.evaluate_sync(ROS_CAMERA_GRAPH_PATH)
    simulation_app.update()                           # instantiate graph
    sim.world.reset()
    tl.get_timeline_interface().play()
    viewport_api = get_active_viewport()
    viewport_api.set_texture_resolution((2560, 1440))
    # test Effort Sensor
    # cable_effort_sensor_path = f"/World/Rope0/Rope0EffortSensorArticulation/sensorJoint"
    # cable_effort_sensor = EffortSensor(cable_effort_sensor_path)
    while simulation_app.is_running():
        # sim.world.step(render = False)                  # PhysX step
        sim.world.step()                     # PhysX step + render
        # reading = cable_effort_sensor.get_sensor_reading()
        # print(f"Sensor Time: {reading.time}   Value: {reading.value}   Validity: {reading.is_valid}")


    # clean shutdown
    simulation_app.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

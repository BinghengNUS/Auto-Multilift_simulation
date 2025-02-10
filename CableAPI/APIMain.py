"""
Developer: Li Yunchao
Date: 2025-02-07
Description: 
    This standalone script is the main program for APICable.py, used to launch the model of single or multiple cables.
    
    Single Rope (Vertical): Set num_ropes=1. As the baseline.
    The adjusted (physics, D6joints) parameters ensure model stability and high stiffness accuracy for payloads between 1~6Kg. 
    Payloads of 7Kg or higher may cause precision errors of rope stiffness.
    
    Multiple Ropes (Elevation angle): Set num_ropes > 1 and customed elevation angle.
    *Fix the translation axis of the D6Joints, solve the issue of unstable jitter.
    *Control the elevation angle of the ropes, ease the testing of multiple ropes.
"""

import math
import sys
import time
import signal
import csv
from isaacsim import SimulationApp

units_in_meters = 1.0
real_frame_per_second = 1000.0
internal_frame_per_second = 60.0

# Global flags and references
is_processing = False
kit = None

def scheduler(signum, frame):
    """
    This function is triggered by the SIGALRM signal.
    It checks if an update is already in progress.
    If not, it updates the SimulationApp (kit).
    """
    global is_processing, kit
    if not is_processing:
        is_processing = True
        kit.update()
        is_processing = False

def main():
    global kit
    # Initialize the SimulationApp
    kit = SimulationApp({"renderer": "RayTracedLighting", "headless": False})

    import omni
    from omni.isaac.core.world import World
    from pxr import Sdf, PhysxSchema, Gf, PhysicsSchemaTools, PhysxSchema
    from APICable import RigidBodyRopes

    # Initialize the World instance
    world = World(
        stage_units_in_meters=units_in_meters,
        physics_dt=1.0 / real_frame_per_second,
        rendering_dt=1.0 / internal_frame_per_second,
        physics_prim_path="/World/physicsScene"
        )
    # Get the stage from the World instance
    stage = world.stage

    PhysicsSchemaTools.addGroundPlane(
    stage, "/World/groundPlane", "Z", 
    50, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.7)
    )

    world.initialize_physics()
    physics_context = world.get_physics_context()
    physics_context.set_solver_type("TGS")
    physics_context.set_broadphase_type("GPU")
    physics_context.enable_gpu_dynamics(True)
    physics_context.enable_stablization(True)
    physics_context.enable_ccd(True)
    physics_context.set_friction_offset_threshold(0.01)
    physics_context.set_gpu_max_num_partitions(32)

    # Set Solver iteration count
    PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/World/physicsScene"))
    physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/World/physicsScene")
    ## IMPORTANT: Set the solver iteration count ##############################
    # physxSceneAPI.CreateMinPositionIterationCountAttr(25) # For 1~6Kg. 7Kg or higher mass got issues 
    # physxSceneAPI.CreateMinVelocityIterationCountAttr(0)
    ###########################################################################
    

    # Create an instance of the RigidBodyRopesDemo class
    model_instance = RigidBodyRopes()

    defaultPrimPath = Sdf.Path("/World")
    stage.DefinePrim(defaultPrimPath)
    stage.SetDefaultPrim(stage.GetPrimAtPath(defaultPrimPath))

    # Call the create method with the desired parameters
    loadOn1Rope = 6.0

    # To test 1 rope in vertical, set num_ropes = 1.
    # num_ropes = 1
    # To test multiple ropes, set num_ropes > 1 and customed elevation angle.
    num_ropes = 6
    rope_length = 1.0
    load_height = 2.0
    elevation_angle = math.pi / 2.0

    # Calculate the mass of the payload
    if elevation_angle != 0.0:
        calMass = loadOn1Rope / math.sin(elevation_angle) * num_ropes 
    else:
        calMass = 48.0

    model_instance.create(
        stage, 
        num_ropes=num_ropes, 
        rope_length=rope_length,
        payload_mass=3,
        load_height=load_height,
        elevation_angle=elevation_angle
        )
    
    world.reset()
    world.stop()

    # Set up the signal alarm for scheduled updates
    signal.signal(signal.SIGALRM, scheduler)
    signal.setitimer(signal.ITIMER_REAL, 1.0 / real_frame_per_second, 1.0 / real_frame_per_second)

    try:
        # Keep this script running, so the scheduler can trigger updates
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        # On keyboard interrupt, stop and close everything
        omni.timeline.get_timeline_interface().stop()
        kit.close()

if __name__ == "__main__":
    main()
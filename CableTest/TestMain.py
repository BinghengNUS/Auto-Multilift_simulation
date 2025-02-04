import sys
import time
import signal
import csv
from isaacsim import SimulationApp


# ==== Other Mannual Settings for the simulation =====
# 1. physicsScene: Minimum Pos/Vel Iterations = 51, 50
# 2. Window -> Physics -> Debug -> Disable sleeping
# 3. Time Step per second = 500


# Frame rates
real_frame_per_second = 1000.0
internal_frame_per_second = 60.0

# Path to the USD file
# usd_path = "./Assets/FEMCable.usd"
# usd_path = "./Assets/Cable3Kg.usd"
# usd_path = "./Assets/Cable5Kg.usd"
usd_path = "/home/carlson/isaac_ws/Feb4-1kg.usd"

# Global flags and references
is_processing = False
kit = None
units_in_meters = 1
min_step_index = float('inf')
initial_pos = None
start_time = None
stiffness = None
mass = None
initial_pos0 = None
initial_pos10 = None

def scheduler(signum, frame):
    """
    This function is triggered by the SIGALRM signal.
    It checks if an update is already in progress.
    If not, it updates the SimulationApp (kit).
    """
    global is_processing
    global kit
    if not is_processing:
        is_processing = True
        kit.update()
        is_processing = False


def step_callback(step_size):
    """
    Callback function for physics updates.
    Logs the payload's position and velocity to a CSV file.
    """
    global dc, object, capsule0, capsule10, csv_writer, world, units_in_meters, min_step_index, initial_pos0, initial_pos10, start_time, stiffness, mass

    # Constants
    gravity = 9.81

    # Get current step index and time
    step_index = world.current_time_step_index
    current_time = world.current_time

    # Get pose and velocity
    # pose = dc.get_rigid_body_pose(object)
    # velocity = dc.get_rigid_body_linear_velocity(object)
    pose0 = dc.get_rigid_body_pose(capsule0)
    pose10 = dc.get_rigid_body_pose(capsule10)

    # Print information
    print(f"Current payload info @ step {step_index} :\n",
        f"Position0: {pose0.p}\n",
        f"Position10: {pose10.p}\n",
        )
    
    # print(f"Current Simulation TIME: {current_time}")

    # Record the real-world start time
    # Update the minimum step index and record the real-world start time (Only Once!)
    if step_index < min_step_index:
        min_step_index = step_index
        start_time = time.time()
        initial_pos0 = pose0.p.z 
        initial_pos10 = pose10.p.z 
    
    initial_length  = initial_pos10 - initial_pos0

    # # Calculate and print the real-world elapsed time
    elapsed_real_time = time.time() - start_time
    # print(f"Current Real TIME: {elapsed_real_time}\n")

    # Calculate stiffness based on position
    stiffness = (gravity * mass) / (((pose10.p.z - pose0.p.z) - initial_length) * units_in_meters)

    if stiffness is not None and mass is not None:
        print(f"Calculated stiffness: {stiffness} N/m\n", 
            f"Mass: {mass} Kg\n")

    # Save to CSV file
    csv_writer.writerow([
        step_index,
        current_time,
        elapsed_real_time,
        pose0.p.z,
        pose10.p.z,
        stiffness
        ])


def render_callback(event):
    """
    Callback function for rendering updates.
    """
    # print(f"=== Render Frame ===")
    

# Main function and other parts of the script remain unchanged
def main():
    global kit, dc, object, capsule0, capsule10, csv_writer, world, units_in_meters, mass

    # Create the SimulationApp with required arguments
    kit = SimulationApp({"renderer": "RayTracedLighting", "headless": False, "open_usd": usd_path})

    import omni
    from pxr import Usd, UsdGeom, Sdf, PhysxSchema
    from omni.isaac.core.world import World
    from omni.isaac.dynamic_control import _dynamic_control as dc
    from omni.isaac.core.prims import RigidPrim

    # Create a World instance and specify physics and rendering step sizes
    world = World(
        stage_units_in_meters=units_in_meters,
        physics_dt=1.0 / real_frame_per_second,
        rendering_dt=1.0 / internal_frame_per_second
    )
    # Get the stage from the World instance
    stage = world.stage

    # IMPORTANT: Initialize physics after creating the World instance
    world.initialize_physics()

    # # For FEM mesh type Cube, set physics solver to TGS and broadphase to GPU
    physics_context = world.get_physics_context()
    # physics_context.set_solver_type("TGS")
    # physics_context.set_broadphase_type("GPU")
    physics_context.enable_gpu_dynamics(True)
    # physics_context.enable_stablization(True)
    # physics_context.enable_ccd(False)
    # # Set Solver iteration count
    # PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
    # physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
    # physxSceneAPI.CreateMinPositionIterationCountAttr(51)
    # physxSceneAPI.CreateMinVelocityIterationCountAttr(50)

    # Get payload object
    payload_path = "/World/Payload"
    capsule0_path = "/World/Capsule0"
    capsule10_path = "/World/Capsule10"
    # For FEM
    # payload_path = "/World/Cube"

    # Acquire dynamic control interface
    dc = dc.acquire_dynamic_control_interface()
    object = dc.get_rigid_body(payload_path)
    capsule0 = dc.get_rigid_body(capsule0_path)
    capsule10 = dc.get_rigid_body(capsule10_path)

    # Acquire prim of payload, to get mass
    prim = RigidPrim(payload_path)
    mass = prim.get_mass()

    # Open CSV file for logging
    csv_file = open(f"./Feb4/payload_{mass}Kg_log.csv", mode="w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["step",
                        "simulation_time", "real_time",
                        "0z",
                        "10z",
                        "stiffness"])

    # Add callbacks
    world.add_physics_callback("physics_callback", step_callback)
    world.add_render_callback("render_callback", render_callback)

    # Stop the simulation after acquiring the payload, manually start the simulation
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
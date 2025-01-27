import sys
import time
import signal
import csv
from isaacsim import SimulationApp


# ==== Settings for the simulation =====
# 1. physicsScene: Minimum Pos/Vel Iterations = 51, 50
# 2. Window -> Physics -> Debug -> Disable sleeping
# 3. Time Step per second = 500


# Frame rates
real_frame_per_second = 500.0
internal_frame_per_second = 60.0

# Path to the USD file
# usd_path = "./Assets/FEMCable.usd"
# usd_path = "./Assets/Cable3Kg.usd"
# usd_path = "./Assets/Cable5Kg.usd"
usd_path = "./Assets/Cable8Kg.usd"


# Global flags and references
is_processing = False
kit = None
units_in_meters = 0.01
min_step_index = float('inf')
initial_pos = None
start_time = None
stiffness = None
mass = None

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
    global dc, object, csv_writer, world, units_in_meters, min_step_index, initial_pos, start_time, stiffness, mass

    # Constants
    gravity = 9.81

    # Get current step index and time
    step_index = world.current_time_step_index
    current_time = world.current_time

    # Get pose and velocity
    pose = dc.get_rigid_body_pose(object)
    velocity = dc.get_rigid_body_linear_velocity(object)

    # Print information
    print(f"Current payload info @ step {step_index} :\n",
        f"Position: {pose.p}\n",
        f"Velocity: {velocity}\n",
        f"Rotation (quaternion): {pose.r}\n")
    
    print(f"Current Simulation TIME: {current_time}")

    # Record the real-world start time
    # Update the minimum step index and record the real-world start time (Only Once!)
    if step_index < min_step_index:
        min_step_index = step_index
        start_time = time.time()
        initial_pos = pose.p.z # Initial position of the payload

    # Calculate and print the real-world elapsed time
    elapsed_real_time = time.time() - start_time
    print(f"Current Real TIME: {elapsed_real_time}\n")

    # Calculate stiffness based on position
    stiffness = (gravity * mass) / ((initial_pos - pose.p.z) * units_in_meters)

    if stiffness is not None and mass is not None:
        print(f"Calculated stiffness: {stiffness} N/m\n", 
            f"Mass: {mass} Kg\n")

    # Save to CSV file
    csv_writer.writerow([
        step_index,
        current_time,
        elapsed_real_time,
        pose.p.x, pose.p.y, pose.p.z,
        velocity.x, velocity.y, velocity.z,
        stiffness
        ])


def render_callback(event):
    """
    Callback function for rendering updates.
    """
    # print(f"=== Render Frame, Current time {time.time()} ===")
    print(f"=== Render Frame ===")
    

# Main function and other parts of the script remain unchanged
def main():
    global kit, dc, object, csv_writer, world, units_in_meters, mass

    # Create the SimulationApp with required arguments
    kit = SimulationApp({"renderer": "RayTracedLighting", "headless": False, "open_usd": usd_path})

    import omni
    from omni.isaac.core.world import World
    from omni.isaac.dynamic_control import _dynamic_control as dc
    from omni.isaac.core.prims import RigidPrim
    from pxr import Usd, UsdGeom, Sdf, PhysxSchema

    # Create a World instance and specify physics and rendering step sizes
    world = World(
        stage_units_in_meters=units_in_meters,
        physics_dt=1.0 / real_frame_per_second,
        rendering_dt=1.0 / internal_frame_per_second
    )
    # Get the stage from the World instance
    stage = world.stage

    # IMPORTANT: Initialize physics after creating the World instance, 不初始化就读不到payload!
    world.initialize_physics()

    # For FEM mesh type Cube, set physics solver to TGS and broadphase to GPU
    physics_context = world.get_physics_context()
    physics_context.set_solver_type("TGS")
    physics_context.set_broadphase_type("GPU")
    physics_context.enable_gpu_dynamics(True)
    physics_context.enable_stablization(True)
    physics_context.enable_ccd(False)

    # Set Solver iteration count
    PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
    physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
    physxSceneAPI.CreateMinPositionIterationCountAttr(51)
    physxSceneAPI.CreateMinVelocityIterationCountAttr(50)

    # Get payload object
    payload_path = "/vechicle/Payload/payload0"
    # For FEM
    # payload_path = "/World/Cube"

    # Acquire dynamic control interface
    dc = dc.acquire_dynamic_control_interface()
    object = dc.get_rigid_body(payload_path)

    # Acquire prim of payload, to get mass
    prim = RigidPrim(payload_path)
    mass = prim.get_mass()

    # Open CSV file for logging
    csv_file = open(f"./CsvLog/payload_{mass}Kg_log.csv", mode="w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["step",
                        "simulation_time", "real_time",
                        "px", "py", "pz", 
                        "vx", "vy", "vz", 
                        "stiffness"])

    # Add callbacks
    world.add_physics_callback("physics_callback", step_callback)
    world.add_render_callback("render_callback", render_callback)

    # 获取完payload后再stop, 手动启动模拟
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
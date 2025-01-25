from pxr import UsdPhysics,PhysxSchema,Gf,PhysicsSchemaTools,UsdGeom,Vt,Sdf
import omni
from omni.isaac.kit import SimulationApp
from omni.isaac.core.world import World

world = World()
world.clear()
# world.reset()
stage = omni.usd.get_context().get_stage()

gravity = 9.81
scene = UsdPhysics.Scene.Define(stage, "/World/physics")
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(gravity)
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/World/physics"))
physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/World/physics")
physxSceneAPI.CreateEnableCCDAttr(True)
physxSceneAPI.CreateEnableStabilizationAttr(True)
physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
physxSceneAPI.CreateSolverTypeAttr("TGS")

# physxSceneAPI.CreateSolverPositionIterationCountAttr(30)
# physxSceneAPI.CreateSolverVelocityIterationCountAttr(2)


PhysicsSchemaTools.addGroundPlane(
    stage,
    "/World/groundPlane",
    "Z",
    15,
    Gf.Vec3f(0, 0, 0),
    Gf.Vec3f(0.7)
)

# payload
payload_path = "/World/Payload"
cubeGeom = UsdGeom.Cube.Define(stage, payload_path)
cubePrim = stage.GetPrimAtPath(payload_path)
size = 0.2
initial_height = 1.0
offset = Gf.Vec3f(0.0, 0.0, initial_height)
cubeGeom.CreateSizeAttr(size)
cubeGeom.AddTranslateOp().Set(offset)
# Mark the cube as a rigid body
rigid_api = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
rigid_api.CreateRigidBodyEnabledAttr(True)
UsdPhysics.CollisionAPI.Apply(cubePrim)

# Cable
tot = 11
for num in range(tot):
    capsule_path = f"/World/Capsule{num}"
    capsuleGeom = UsdGeom.Capsule.Define(stage, capsule_path)
    
    half_length = 0.02
    capsuleGeom.CreateHeightAttr(half_length)
    capsuleGeom.CreateRadiusAttr(0.01)
    capsuleGeom.CreateAxisAttr("Z")
    
    # Set display color
    color_array = Vt.Vec3fArray([Gf.Vec3f(0.0, 0.1, 0.8)])
    capsuleGeom.CreateDisplayColorAttr().Set(color_array)
    
    # Position each capsule above the ground, stacked along Z
    cap_height = initial_height + size/2 + half_length + 2 * half_length * num
    capsule_pose = Gf.Vec3f(0.0, 0.0, cap_height)
    capsuleGeom.AddTranslateOp().Set(capsule_pose)
    
    # Rigid body & collision
    UsdPhysics.CollisionAPI.Apply(capsuleGeom.GetPrim())
    # UsdPhysics.RigidBodyAPI.Apply(capsuleGeom.GetPrim())
    rigidbodyAPI = UsdPhysics.RigidBodyAPI.Apply(capsuleGeom.GetPrim())
    physx_rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(capsuleGeom.GetPrim())
    physx_rigid_api.CreateAngularDampingAttr(1)
    massAPI = UsdPhysics.MassAPI.Apply(capsuleGeom.GetPrim())
    massAPI.CreateDensityAttr().Set(0.01)

# Joints
for jointnum in range(tot - 1):
    joint_path = f"/World/joint{jointnum}"
    joint = UsdPhysics.Joint.Define(stage, joint_path)
    d6Prim = joint.GetPrim()
    
    parent_path = f"/World/Capsule{jointnum}"
    child_path = f"/World/Capsule{jointnum + 1}"
    
    # Attach the bodies to the joint
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
    
    # Local positions at the top and bottom of each capsule
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0,  0.02))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, -0.02))
    
    # Rotations
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    
    # Allow some motion in transZ
    for axis in ["transZ"]:
        limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, axis)
        limitAPI.CreateLowAttr(0)
        limitAPI.CreateHighAttr(0)
        physx_limit_api = PhysxSchema.PhysxLimitAPI.Apply(d6Prim, axis)
        # physx_limit_api.CreateStiffnessAttr(100000.0)  # how "firm" the limit feels
        # physx_limit_api.CreateDampingAttr(1000.0)   
        driveAPI = UsdPhysics.DriveAPI.Apply(d6Prim, axis)
        driveAPI.CreateTypeAttr("force")
        driveAPI.CreateDampingAttr(1000)
        driveAPI.CreateStiffnessAttr(100000)
    
    # Lock out transX, transY, rotX
    for axis in ["transX", "transY", "rotX"]:
        limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, axis)
        limitAPI.CreateLowAttr(0.0)
        limitAPI.CreateHighAttr(0.0)
    
    # Limit rotY, rotZ
    for axis in ["rotY", "rotZ"]:
        limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, axis)
        limitAPI.CreateLowAttr(-170)
        limitAPI.CreateHighAttr(170)

# ----------------------------------------------------------------
# 6) Create a joint between the first Capsule and the Payload
# ----------------------------------------------------------------
payload_joint_path = f"/World/payload_joint"
payload_joint = UsdPhysics.Joint.Define(stage, payload_joint_path)
payload_d6Prim = payload_joint.GetPrim()

parent_path = f"/World/Capsule0"
child_path = f"/World/Payload"

payload_joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
payload_joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])

payload_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, -0.02))
payload_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, size/2))

payload_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
payload_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

for axis in ["transZ"]:
    limitAPI = UsdPhysics.LimitAPI.Apply(payload_d6Prim, axis)
    limitAPI.CreateLowAttr(-1)
    limitAPI.CreateHighAttr(1)

for axis in ["transX", "transY", "rotX"]:
    limitAPI = UsdPhysics.LimitAPI.Apply(payload_d6Prim, axis)
    limitAPI.CreateLowAttr(0.0)
    limitAPI.CreateHighAttr(0.0)

for axis in ["rotY", "rotZ"]:
    limitAPI = UsdPhysics.LimitAPI.Apply(payload_d6Prim, axis)
    limitAPI.CreateLowAttr(-160)
    limitAPI.CreateHighAttr(160)


# collision_group_path = "/World/CapsuleCollisionGroup"
# collision_group = UsdPhysics.CollisionGroup.Define(stage, collision_group_path)
# collision_group_prim = collision_group.GetPrim()
# capsules = [f"/World/Capsule{i}" for i in range(tot)]  
# includes_rel = collision_group_prim.CreateRelationship("includes", custom=False)
# includes_rel.SetTargets([Sdf.Path(x) for x in capsules])


world.play()
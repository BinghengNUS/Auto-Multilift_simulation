#!/usr/bin/env python
import carb
from isaacsim import SimulationApp

# Creat SimulationApp
simulation_app = SimulationApp({"headless": False})

import math
import numpy as np
import omni.timeline
import omni.physxdemos as demo
from omni.isaac.core.world import World
from omni.isaac.dynamic_control import _dynamic_control as dc
from pxr import UsdGeom, Sdf, Gf, UsdPhysics, PhysxSchema, Vt
from omni.physx import  acquire_physx_interface
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge") # enable ROS2 bridge extension

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS, ROBOTS_CONFIG
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped


class PegasusApp:
    def __init__(self):
        # Get the simulation timeline
        self.timeline = omni.timeline.get_timeline_interface()

        # Initialize the simulation app
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        stage = self.world.stage
        physics_context = self.world.get_physics_context()
        physics_context.enable_gpu_dynamics(True)
        physics_context.enable_stablization(False)
        physics_context.enable_ccd(False)
        physics_context.set_gpu_max_num_partitions(32)
        PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")

        # Using GPU in Isaac Sim
        physx = acquire_physx_interface()
        physx.overwrite_gpu_setting(1) 

        # Load the environment
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        config_multirotor = MultirotorConfig()
        

    def run(self, node):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:
            
            # run the ros2 node, non-blocking
            rclpy.spin_once(node)

            # Update the UI of the app and perform the physics step
            self.world.step(render=True)
        
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

class RigidBodyRopes(demo.Base):
    title = "Elastic Cable Model"

    def __init__(self):
        return

    ## Main Call Funtion to build the scene ##
    def create(self, stage, num_ropes=6, rope_length=1.0, ropd_stiffness=1e4, rope_damping=1e2, 
               payload_mass=7.0, payload_radius=0.05, payload_height=0.05, 
               Jl=0.7*np.array([[2, 2, 2.5]]).T, 
               rg=np.array([[0.1, 0.1, -0.1]]).T,
               load_height=2.0, elevation_angle=0.0):
        self._stage = stage
        # Ensure "/World" exists
        if not stage.GetPrimAtPath("/World"):
            stage.DefinePrim("/World", "Xform")

        # Make "/World" the default prim explicitly
        stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
        self._defaultPrimPath = stage.GetDefaultPrim().GetPath()
        
        ## Payload config:
        # self._payloadRadius = 0.24
        self._payloadRadius = payload_radius
        # self._payloadHight = self._payloadRadius / 4
        self._payloadHight = payload_height
        self._payloadMass = payload_mass
        self._payloadInertia = Jl
        self._payloadCOM = rg
        self._payloadColor = [0.22, 0.43, 0.55]

        self._initLoadHeight = load_height
        self._payloadPos = Gf.Vec3f(0.0, 0.0, self._initLoadHeight)

        self._payloadXform = self._defaultPrimPath.AppendChild(f"CommonPayload")
        self._payloadPath = self._payloadXform.AppendChild("Payload")

        ## Ropes config:
        # Each capsule is 0.1m long = 0.08 + 0.01*2
        self._linkHalfLength = 0.08
        self._linkRadius = 0.01
        self._linkLength = self._linkHalfLength + 2 * self._linkRadius
        self.segment_num = int(rope_length / self._linkLength)

        self._ropeLength = rope_length
        self._numRopes = num_ropes
        self._ropeSpacing = 15.0
        self._ropeColor = demo.get_primary_color()

        self._coneAngleLimit = 160
        self._slideLimit = 0.1
        # self._slideMaxforceLimit = 10.0 * self._payloadMass * 9.81
        self._slideMaxforceLimit = 1000

        # Joint stiffness and damping settings
        # self._slide_stiffness = 1e5 # stiffness for Prismatic joint 
        # self._slide_damping = 1e3 # damping for Prismatic joint 
        # NOTE: we assum each joint's stiffness and damping is the same, thus:
        # Stiffness: \frac{1}{k_total} = \sum_{i=1}^{n} \frac{1}{k_i} = \frac{n}{k_i}, k_total = k_i/n, k_i = k_total*n
        # Damping: \frac{1}{d_total} = \sum_{i=1}^{n} \frac{1}{d_i} = \frac{n}{d_i}, d_total = d_i/n, d_i = d_total*n
        self._slide_stiffness = ropd_stiffness * self.segment_num # stiffness for Prismatic joint 
        self._slide_damping = rope_damping * self.segment_num # damping for Prismatic joint 

        ##### IMPORTANT: The LIMITS also influence the TRUE stiffness and damping!!! ###############################
        self._slide_stiffness_limit = 4 * self._slide_stiffness
        self._slide_damping_limit = 3 * self._slide_damping
        ############################################################################################################

        ## Table / Box Config
        self._scaleFactor = 1.0 / (UsdGeom.GetStageMetersPerUnit(stage) * 100.0)

        self._tableThickness = 6.0
        self._boxSize = 1.0
        self._tableHeight = self._initLoadHeight + self._payloadHight/2 + rope_length + self._tableThickness*self._scaleFactor
        
        floorOffset = 0.0
        self._floorOffset = floorOffset - self._tableHeight
        self._tableSurfaceDim = Gf.Vec2f(200.0, 100.0)
        self._tableColor = Gf.Vec3f(168.0, 142.0, 119.0) / 255.0

        self._tableXform = self._defaultPrimPath.AppendChild(f"Table")
        self._tableTopPath = self._tableXform.AppendChild("tableTopActor")

        self._upAxis = UsdGeom.GetStageUpAxis(stage)
        if self._upAxis == UsdGeom.Tokens.z:
            self._orientation = [0,1,2]
        if self._upAxis == UsdGeom.Tokens.y:
            self._orientation = [1,2,0]
        if self._upAxis == UsdGeom.Tokens.x:
            self._orientation = [2,1,0]

        ## Create the scene 
        # Create the common payload
        self._createPayload()
        
        if (num_ropes < 2):
            # Create a single rope as the baseline template
            self.create_table()
            self._createVerticalRopes()
        else:
            # Create multiple ropes with a elevation angle
            self._createMultiRopes(elevation_angle)
            

    ## Scene Object Functions ##
    def _createCapsule(self, path: Sdf.Path, axis="Z"):
        capsuleGeom = UsdGeom.Capsule.Define(self._stage, path)

        capsuleGeom.CreateHeightAttr(self._linkHalfLength)
        capsuleGeom.CreateRadiusAttr(self._linkRadius)
        capsuleGeom.CreateAxisAttr(axis)
        capsuleGeom.CreateDisplayColorAttr().Set([self._ropeColor])

        UsdPhysics.RigidBodyAPI.Apply(capsuleGeom.GetPrim())
        physx_rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(capsuleGeom.GetPrim())
        physx_rigid_api.CreateLinearDampingAttr(1)
        # physx_rigid_api.CreateAngularDampingAttr(0.1)
        # physx_rigid_api.GetSolverPositionIterationCountAttr(20)
        # physx_rigid_api.CreateCfmScaleAttr(0.2)

        massAPI = UsdPhysics.MassAPI.Apply(capsuleGeom.GetPrim())
        massAPI.CreateMassAttr().Set(0.008)

        UsdPhysics.CollisionAPI.Apply(capsuleGeom.GetPrim())

    def _createPayload(self):
        UsdGeom.Xform.Define(self._stage, self._payloadXform)

        payloadGeom = UsdGeom.Cylinder.Define(self._stage, self._payloadPath)
        payloadGeom.AddTranslateOp().Set(self._payloadPos)
        payloadGeom.CreateRadiusAttr(self._payloadRadius)
        payloadGeom.CreateHeightAttr(self._payloadHight)
        payloadGeom.CreateDisplayColorAttr().Set([self._payloadColor])

        rigidAPI = UsdPhysics.RigidBodyAPI.Apply(payloadGeom.GetPrim())
        rigidAPI.CreateRigidBodyEnabledAttr(True)

        physx_rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(payloadGeom.GetPrim())
        # physx_rigid_api.CreateLinearDampingAttr(0.01)

        massAPI = UsdPhysics.MassAPI.Apply(payloadGeom.GetPrim())
        massAPI.CreateMassAttr().Set(self._payloadMass)

         # Inertia tensor
        # inertia_tensor = 0.7 * np.array([[2, 2, 2.5]]).T  # Jl 
        inertia_tensor = self._payloadInertia
        massAPI.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*inertia_tensor.flatten()))

        # COM offset
        # center_of_mass_offset = np.array([[0.1, 0.1, -0.1]]).T  #  rg 
        center_of_mass_offset = self._payloadCOM
        massAPI.CreateCenterOfMassAttr().Set(Gf.Vec3f(*center_of_mass_offset.flatten()))

        UsdPhysics.CollisionAPI.Apply(payloadGeom.GetPrim())

    def create_box(self, rootPath, primPath, dimensions, position, color, orientation = Gf.Quatf(1.0), positionMod = None):
        boxActorPath = self.get_path(rootPath, primPath)
        newPosition = Gf.Vec3f(0.0)

        # deep copy to avoid modifying reference
        for i in range(3):
            newPosition[i] = position[i]
            if positionMod:
                newPosition[i] *= positionMod[i]

        cubeGeom = UsdGeom.Cube.Define(self._stage, boxActorPath)
        cubePrim = self._stage.GetPrimAtPath(boxActorPath)

        cubeGeom.AddTranslateOp().Set(newPosition)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(self.orient_dim(dimensions))
        cubeGeom.CreateSizeAttr(1.0)
        cubeGeom.CreateDisplayColorAttr().Set([color]) 

        half_extent = 0.5
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])

        rigidAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        # rigidAPI.CreateRigidBodyEnabledAttr(False)

        UsdPhysics.CollisionAPI.Apply(cubePrim)
            
    def create_table(self):
        tableDim = Gf.Vec3f(self._tableSurfaceDim[0], self._tableSurfaceDim[1], self._tableHeight)

        # Only create the table top
        self.create_box(
            "Table", "tableTopActor",
            Gf.Vec3f(tableDim[0], tableDim[1], self._tableThickness),
            Gf.Vec3f(0.0, 0.0, tableDim[2] - self._tableThickness*self._scaleFactor*0.5),
            self._tableColor
        )


    ## Joint Functions ##
    ## Feb 4th: Setting limits for stiffness & damping, setting maxForce (10 * mass* gravity).
    def _createCableJoint(self, jointPath, axis="Z"):
        rotatedDOFs = ["rotX", "rotY"]
        if (axis == "Z"):
            slideDOF = "transZ",
            lockedDOFs = ["transX", "transY"]
            rotatedDOFs = ["rotX", "rotY", "rotZ"]
        elif (axis == "X"):
            slideDOF = "transX",
            lockedDOFs = ["transY", "transZ"]
            rotatedDOFs = ["rotY", "rotZ", "rotX"]
        else:
            slideDOF = "transY",
            lockedDOFs = ["transX", "transZ"]
            rotatedDOFs = ["rotX", "rotZ", "rotY"]

        joint = UsdPhysics.Joint.Define(self._stage, jointPath)
        d6Prim = joint.GetPrim()

        # Slided DOF (transZ) with limits:
        for prim in slideDOF:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, prim)
            # limitAPI.CreateLowAttr(-self._slideLimit)  
            limitAPI.CreateLowAttr(-0.0001)  
            limitAPI.CreateHighAttr(0.0001) # debug
            
            physx_limit_api = PhysxSchema.PhysxLimitAPI.Apply(d6Prim, prim)
            physx_limit_api.CreateStiffnessAttr(self._slide_stiffness_limit)  
            physx_limit_api.CreateDampingAttr(self._slide_damping_limit)
            physx_limit_api.CreateRestitutionAttr(1)
            physx_limit_api.CreateContactDistanceAttr(0.0001)

            driveAPI = UsdPhysics.DriveAPI.Apply(d6Prim, prim)
            driveAPI.CreateTypeAttr("force")
            driveAPI.CreateMaxForceAttr(self._slideMaxforceLimit)
            driveAPI.CreateDampingAttr(self._slide_damping)
            driveAPI.CreateStiffnessAttr(self._slide_stiffness)

        # Locked DOF (lock - low is greater than high) transY/Z and rotX:
        for axis in lockedDOFs:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, axis)
            limitAPI.CreateLowAttr(0.0)
            limitAPI.CreateHighAttr(0.0)

        # Rotated DOF rotY, rotZ with limits:
        for d in rotatedDOFs:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, d)
            physx_limit_api = PhysxSchema.PhysxLimitAPI.Apply(d6Prim, d)
            driveAPI = UsdPhysics.DriveAPI.Apply(d6Prim, d)
            driveAPI.CreateTypeAttr("force")
            # driveAPI.CreateMaxForceAttr(self._slideMaxforceLimit)
            driveAPI.CreateDampingAttr(0.01)
            # driveAPI.CreateStiffnessAttr(self._slide_stiffness)
            # limitAPI.CreateLowAttr(-self._coneAngleLimit)
            # limitAPI.CreateHighAttr(self._coneAngleLimit)

    def _createFixJoint(self, jointPath):
        joint = UsdPhysics.Joint.Define(self._stage, jointPath)
        d6Prim = joint.GetPrim()
        # Lock All 6 DOFs:
        for axis in ["transX", "transY", "transZ", "rotX", "rotY", "rotZ"]:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, axis)
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)  
    
    def _createUniJoint(self, jointPath):
        joint = UsdPhysics.Joint.Define(self._stage, jointPath)
        d6Prim = joint.GetPrim()
        # Lock All 3 tran and 1 rot DOFs:
        for axis in ["transX", "transY", "transZ"]:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, axis)
            limitAPI.CreateLowAttr(0.0)
            limitAPI.CreateHighAttr(0.0)  
        for axis in ["rotX", "rotY", "rotZ"]:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, axis)
            # limitAPI.CreateLowAttr(-self._coneAngleLimit)
            # limitAPI.CreateHighAttr(self._coneAngleLimit)
   

    ## Rope Functions ##
    # ONE rope along Z, as the baseline to tune the params of physics, D6Joints
    def _createVerticalRopes(self):
        linkLength = self._linkHalfLength + 2 * self._linkRadius
        numLinks = int(self._ropeLength / linkLength)

        payloadHightHalf = self._payloadHight * 0.5
        capsuleHalf = linkLength * 0.5
        
        # Model from payload, The first link is the end of the rope
        xStart = 0.0
        yStart = - (self._numRopes // 2) * self._ropeSpacing
        zStart = self._initLoadHeight + payloadHightHalf + capsuleHalf

        # Create each rope from the payload
        for ropeInd in range(self._numRopes):
            scopePath = self._defaultPrimPath.AppendChild(f"Rope{ropeInd}")
            UsdGeom.Xform.Define(self._stage, scopePath)

            instancerPath = scopePath.AppendChild("rigidBodyInstancer")
            rboInstancer = UsdGeom.PointInstancer.Define(self._stage, instancerPath)

            capsulePath = instancerPath.AppendChild("capsule")
            self._createCapsule(capsulePath)

            meshIndices = []
            positions = []
            orientations = []

            # Rope offset in Y for each rope
            yPos = yStart + ropeInd * self._ropeSpacing

            # 1) Add rope links (all capsules)
            for linkInd in range(numLinks):
                meshIndices.append(0)  # capsule
                z = zStart + linkInd * linkLength
                positions.append(Gf.Vec3f(xStart, yPos, z))
                orientations.append(Gf.Quath(1.0, 0.0, 0.0, 0.0))

            # 2) Set up instancer attributes
            meshList = rboInstancer.GetPrototypesRel()
            meshList.AddTarget(capsulePath)

            rboInstancer.GetProtoIndicesAttr().Set(Vt.IntArray(meshIndices))
            rboInstancer.GetPositionsAttr().Set(Vt.Vec3fArray(positions))
            rboInstancer.GetOrientationsAttr().Set(Vt.QuathArray(orientations))

            # 3) Create a D6 joint prototype for table-link, link–link and link–payload connections
            jointInstancerPath = scopePath.AppendChild("jointInstancer")
            jointInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, jointInstancerPath)
            
            jointPath = jointInstancerPath.AppendChild("chainJoints")
            self._createCableJoint(jointPath) 

            jointMeshIndices = []
            jointBody0Indices = []
            jointBody1Indices = []
            jointLocalPos0 = []
            jointLocalPos1 = []
            jointLocalRot0 = []
            jointLocalRot1 = []

            # Link to Link joint connections
            for linkInd in range(numLinks - 1):
                body0Index = linkInd
                body1Index = linkInd + 1
                jointMeshIndices.append(0)
                jointBody0Indices.append(body0Index)
                jointBody1Indices.append(body1Index)

                jointLocalPos0.append(Gf.Vec3f(0.0, 0.0, capsuleHalf))
                jointLocalPos1.append(Gf.Vec3f(0.0, 0.0, -capsuleHalf))
                jointLocalRot0.append(Gf.Quath(1.0, 0.0, 0.0, 0.0))
                jointLocalRot1.append(Gf.Quath(1.0, 0.0, 0.0, 0.0))

            # Add the joint path to the physics prototypes relationship
            jointInstancer.GetPhysicsPrototypesRel().AddTarget(jointPath)

            # Set the targets for PhysicsBody0s and PhysicsBody1s relationships to the instancer path
            jointInstancer.GetPhysicsBody0sRel().SetTargets([instancerPath])
            jointInstancer.GetPhysicsBody1sRel().SetTargets([instancerPath])

            # Set the physics prototype indices attribute with the joint mesh indices
            jointInstancer.GetPhysicsProtoIndicesAttr().Set(Vt.IntArray(jointMeshIndices))

            # Set the physics body indices attributes with the joint body indices
            jointInstancer.GetPhysicsBody0IndicesAttr().Set(Vt.IntArray(jointBody0Indices))
            jointInstancer.GetPhysicsBody1IndicesAttr().Set(Vt.IntArray(jointBody1Indices))

            # Set the local positions and rotations for the physics bodies
            jointInstancer.GetPhysicsLocalPos0sAttr().Set(Vt.Vec3fArray(jointLocalPos0))
            jointInstancer.GetPhysicsLocalPos1sAttr().Set(Vt.Vec3fArray(jointLocalPos1))
            jointInstancer.GetPhysicsLocalRot0sAttr().Set(Vt.QuathArray(jointLocalRot0))
            jointInstancer.GetPhysicsLocalRot1sAttr().Set(Vt.QuathArray(jointLocalRot1))

            # 4) Create **Cable** joint for link–payload.
            # Last link – Payload
            payloadAttachScopePath = scopePath.AppendChild("ropePayloadCon")
            payloadAttachInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, payloadAttachScopePath)

            PayloadJointPath = payloadAttachScopePath.AppendChild("PayloadJoint")
            # self._createUniJoint(PayloadJointPath)
            self._createCableJoint(PayloadJointPath)

            payloadAttachInstancer.GetPhysicsPrototypesRel().AddTarget(PayloadJointPath)

            payloadAttachInstancer.GetPhysicsBody0sRel().SetTargets([self._payloadPath])
            payloadAttachInstancer.GetPhysicsBody1sRel().SetTargets([instancerPath])

            payloadAttachInstancer.GetPhysicsProtoIndicesAttr().Set(Vt.IntArray([0]))

            payloadAttachInstancer.GetPhysicsBody0IndicesAttr().Set(Vt.IntArray([0]))
            payloadAttachInstancer.GetPhysicsBody1IndicesAttr().Set(Vt.IntArray([0]))

            payloadAttachInstancer.GetPhysicsLocalPos0sAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.0, 0.0, payloadHightHalf)]))
            payloadAttachInstancer.GetPhysicsLocalPos1sAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.0, 0.0, -capsuleHalf)]))
            payloadAttachInstancer.GetPhysicsLocalRot0sAttr().Set(Vt.QuathArray([Gf.Quath(1.0)]))
            payloadAttachInstancer.GetPhysicsLocalRot1sAttr().Set(Vt.QuathArray([Gf.Quath(1.0)]))
            
            # 5) Create universal joint for table-link.
            tableAttachScopePath = scopePath.AppendChild("ropeTableCon")
            tableAttachInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, tableAttachScopePath)

            TableJointPath = tableAttachScopePath.AppendChild("TableJoint")
            self._createUniJoint(TableJointPath) 
            # self._createFixJoint(TableJointPath)

            tableAttachInstancer.GetPhysicsPrototypesRel().AddTarget(TableJointPath)

            tableAttachInstancer.GetPhysicsBody0sRel().SetTargets([instancerPath])
            tableAttachInstancer.GetPhysicsBody1sRel().SetTargets([self._tableTopPath])

            tableAttachInstancer.GetPhysicsProtoIndicesAttr().Set(Vt.IntArray([0]))

            tableAttachInstancer.GetPhysicsBody0IndicesAttr().Set(Vt.IntArray([numLinks - 1]))
            tableAttachInstancer.GetPhysicsBody1IndicesAttr().Set(Vt.IntArray([0]))

            tableAttachInstancer.GetPhysicsLocalPos0sAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.0, 0.0, capsuleHalf)]))
            tableAttachInstancer.GetPhysicsLocalPos1sAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.0, 0.0, -self._tableThickness * self._scaleFactor * 0.5)]))
            tableAttachInstancer.GetPhysicsLocalRot0sAttr().Set(Vt.QuathArray([Gf.Quath(1.0)]))
            tableAttachInstancer.GetPhysicsLocalRot1sAttr().Set(Vt.QuathArray([Gf.Quath(1.0)]))

    # MULTIPLE ropes with elevation angle. Connect to a common payload, each rope fixed to a box
    def _createMultiRopes(self, elevation_angle):
        translationAxis = "X"
        linkLength = self._linkHalfLength + 2 * self._linkRadius
        numLinks = int(self._ropeLength / linkLength)

        payloadHightHalf = self._payloadHight * 0.5
        capsuleHalf = linkLength * 0.5

        # Calculate the angle increment for each rope, seperate 2pi into numRopes
        angle_increment = 2 * math.pi / self._numRopes

        # Calculate the elevation angle components
        cos_elevation = math.cos(elevation_angle)
        sin_elevation = math.sin(elevation_angle)

        # Create each rope from the payload
        for ropeInd in range(self._numRopes):
            scopePath = self._defaultPrimPath.AppendChild(f"Rope{ropeInd}")
            UsdGeom.Xform.Define(self._stage, scopePath)

            instancerPath = scopePath.AppendChild("rigidBodyInstancer")
            rboInstancer = UsdGeom.PointInstancer.Define(self._stage, instancerPath)

            capsulePath = instancerPath.AppendChild("capsule")
            self._createCapsule(capsulePath, translationAxis)

            meshIndices = []
            positions = []
            orientations = []

            # Calculate the position of the END link
            angle = angle_increment * ropeInd
            xstartPos = (self._payloadRadius + capsuleHalf * cos_elevation) * math.cos(angle)
            ystartPos = (self._payloadRadius + capsuleHalf * cos_elevation) * math.sin(angle)
            zstartPos = self._initLoadHeight + capsuleHalf * sin_elevation
            ## IMPORTANT: Calculate the orientation based on the angle and the elevation_angle
            # The first rotation based on the seperated angle and the axis of rotation (Z), on XY plane #####
            q_z = self.calculate_orientation(angle, Gf.Vec3f(0.0, 0.0, 1.0))
            # The second rotation based on the elevation angle and the axis of rotation (Y), on XZ plane #####
            q_y = self.calculate_orientation(elevation_angle, Gf.Vec3f(0.0, -1.0, 0.0))
            # The final orientation is the multiplication of the two quaternions
            orientation = q_z * q_y

            # 1) Add rope links (all capsules)
            for linkInd in range(numLinks):
                meshIndices.append(0)  # capsule
                x = xstartPos + linkInd * linkLength * math.cos(angle) * cos_elevation
                y = ystartPos + linkInd * linkLength * math.sin(angle) * cos_elevation
                z = zstartPos + linkInd * linkLength * sin_elevation
                positions.append(Gf.Vec3f(x, y, z))
                orientations.append(orientation)

            # 2) Set up instancer attributes
            meshList = rboInstancer.GetPrototypesRel()
            meshList.AddTarget(capsulePath)

            rboInstancer.GetProtoIndicesAttr().Set(Vt.IntArray(meshIndices))
            rboInstancer.GetPositionsAttr().Set(Vt.Vec3fArray(positions))
            rboInstancer.GetOrientationsAttr().Set(Vt.QuathArray(orientations))

            # 3) Create a D6 joint prototype for table-link, link–link and link–payload connections
            jointInstancerPath = scopePath.AppendChild("jointInstancer")
            jointInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, jointInstancerPath)
            
            jointPath = jointInstancerPath.AppendChild("chainJoints")
            # *Fix the translation axis of the D6Joints*
            self._createCableJoint(jointPath, translationAxis) 

            jointMeshIndices = []
            jointBody0Indices = []
            jointBody1Indices = []
            jointLocalPos0 = []
            jointLocalPos1 = []
            jointLocalRot0 = []
            jointLocalRot1 = []

            # Link to Link joint connections
            for linkInd in range(numLinks - 1):
                body0Index = linkInd
                body1Index = linkInd + 1
                jointMeshIndices.append(0)
                jointBody0Indices.append(body0Index)
                jointBody1Indices.append(body1Index)

                jointLocalPos0.append(Gf.Vec3f(capsuleHalf, 0.0, 0.0))
                jointLocalPos1.append(Gf.Vec3f(-capsuleHalf, 0.0, 0.0))
                jointLocalRot0.append(Gf.Quath(1.0, 0.0, 0.0, 0.0))
                jointLocalRot1.append(Gf.Quath(1.0, 0.0, 0.0, 0.0))

            # Add the joint path to the physics prototypes relationship
            jointInstancer.GetPhysicsPrototypesRel().AddTarget(jointPath)

            # Set the targets for PhysicsBody0s and PhysicsBody1s relationships to the instancer path
            jointInstancer.GetPhysicsBody0sRel().SetTargets([instancerPath])
            jointInstancer.GetPhysicsBody1sRel().SetTargets([instancerPath])

            # Set the physics prototype indices attribute with the joint mesh indices
            jointInstancer.GetPhysicsProtoIndicesAttr().Set(Vt.IntArray(jointMeshIndices))

            # Set the physics body indices attributes with the joint body indices
            jointInstancer.GetPhysicsBody0IndicesAttr().Set(Vt.IntArray(jointBody0Indices))
            jointInstancer.GetPhysicsBody1IndicesAttr().Set(Vt.IntArray(jointBody1Indices))

            # Set the local positions and rotations for the physics bodies
            jointInstancer.GetPhysicsLocalPos0sAttr().Set(Vt.Vec3fArray(jointLocalPos0))
            jointInstancer.GetPhysicsLocalPos1sAttr().Set(Vt.Vec3fArray(jointLocalPos1))
            jointInstancer.GetPhysicsLocalRot0sAttr().Set(Vt.QuathArray(jointLocalRot0))
            jointInstancer.GetPhysicsLocalRot1sAttr().Set(Vt.QuathArray(jointLocalRot1))

            # 4) Create **Cable** joint for link–payload.
            payloadAttachScopePath = scopePath.AppendChild("ropePayloadCon")
            payloadAttachInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, payloadAttachScopePath)

            PayloadJointPath = payloadAttachScopePath.AppendChild("PayloadJoint")
            # self._createFixJoint(PayloadJointPath) # Used to debug, test if the joint successfully connect body0 and body1
            self._createCableJoint(PayloadJointPath)

            payloadAttachInstancer.GetPhysicsPrototypesRel().AddTarget(PayloadJointPath)

            payloadAttachInstancer.GetPhysicsBody0sRel().SetTargets([self._payloadPath])
            payloadAttachInstancer.GetPhysicsBody1sRel().SetTargets([instancerPath])

            payloadAttachInstancer.GetPhysicsProtoIndicesAttr().Set(Vt.IntArray([0]))

            payloadAttachInstancer.GetPhysicsBody0IndicesAttr().Set(Vt.IntArray([0]))
            payloadAttachInstancer.GetPhysicsBody1IndicesAttr().Set(Vt.IntArray([0]))

            ## IMPORTANT: The payload has NO orientation (Local Coordinate System): the LocalPos is calculated based on the angle!!! #####
            # payloadAttachInstancer.GetPhysicsLocalPos0sAttr().Set(Vt.Vec3fArray([Gf.Vec3f(payload, 0.0, self._payloadRadius)]))
            payloadAttachInstancer.GetPhysicsLocalPos0sAttr().Set(Vt.Vec3fArray([Gf.Vec3f(self._payloadRadius*math.cos(angle), self._payloadRadius*math.sin(angle), 0.0)]))
            payloadAttachInstancer.GetPhysicsLocalPos1sAttr().Set(Vt.Vec3fArray([Gf.Vec3f(-capsuleHalf, 0.0, 0.0)]))

            ## IMPORTANT: payload has NO orientation, rotate the LocalRot0(payload) to make it align with the rope direction!!! #####
            # payloadAttachInstancer.GetPhysicsLocalRot0sAttr().Set(Vt.QuathArray([Gf.Quath(1.0)]))
            payloadAttachInstancer.GetPhysicsLocalRot0sAttr().Set(Vt.QuathArray([orientation]))
            payloadAttachInstancer.GetPhysicsLocalRot1sAttr().Set(Vt.QuathArray([Gf.Quath(1.0)]))

            # 5) Create universal joint for link–box
            boxPath = scopePath.AppendChild(f"box{ropeInd}Actor")
            
            # Create a box at the end of the rope, with the same orientation as the rope
            self.create_box(
                f"Rope{ropeInd}", f"box{ropeInd}Actor", 
                Gf.Vec3f(self._boxSize, self._boxSize, self._boxSize), 
                Gf.Vec3f(x + (capsuleHalf + self._boxSize*self._scaleFactor*0.5) * math.cos(angle) * cos_elevation, 
                         y + (capsuleHalf + self._boxSize*self._scaleFactor*0.5) * math.sin(angle) * cos_elevation, 
                         z + (capsuleHalf + self._boxSize*self._scaleFactor*0.5) * sin_elevation),
                self._tableColor,
                orientation=Gf.Quatf(orientation)
                )

            boxAttachScopePath = scopePath.AppendChild("ropeBoxCon")
            boxAttachInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, boxAttachScopePath)

            BoxJointPath = boxAttachScopePath.AppendChild("BoxJoint")
            # self._createFixJoint(BoxJointPath) # Used to debug, test if the joint successfully connect body0 and body1
            self._createUniJoint(BoxJointPath)

            boxAttachInstancer.GetPhysicsPrototypesRel().AddTarget(BoxJointPath)

            boxAttachInstancer.GetPhysicsBody0sRel().SetTargets([instancerPath])
            boxAttachInstancer.GetPhysicsBody1sRel().SetTargets([boxPath])

            boxAttachInstancer.GetPhysicsProtoIndicesAttr().Set(Vt.IntArray([0]))

            boxAttachInstancer.GetPhysicsBody0IndicesAttr().Set(Vt.IntArray([numLinks - 1]))
            boxAttachInstancer.GetPhysicsBody1IndicesAttr().Set(Vt.IntArray([0]))

            boxAttachInstancer.GetPhysicsLocalPos0sAttr().Set(Vt.Vec3fArray([Gf.Vec3f(capsuleHalf + self._boxSize*self._scaleFactor*0.5, 0.0, 0.0)]))
            boxAttachInstancer.GetPhysicsLocalPos1sAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0, 0.0, 0.0)]))
            boxAttachInstancer.GetPhysicsLocalRot0sAttr().Set(Vt.QuathArray([Gf.Quath(1.0)]))
            boxAttachInstancer.GetPhysicsLocalRot1sAttr().Set(Vt.QuathArray([Gf.Quath(1.0)]))


    ### Helper functions ##
    def calculate_orientation(self, angle, axis):
        """
        Calculate the quaternion representing a rotation around an arbitrary axis.
        
        Parameters:
        angle (float): The rotation angle in radians.
        axis (Gf.Vec3f): The unit vector representing the rotation axis.
        
        Returns:
        Gf.Quath: The quaternion representing the rotation.
        """
        half_angle = angle / 2
        sin_half_angle = math.sin(half_angle)
        cos_half_angle = math.cos(half_angle)

        # Make sure the axis is normalized
        axis = axis.GetNormalized()

        return Gf.Quath(cos_half_angle, axis[0] * sin_half_angle, axis[1] * sin_half_angle, axis[2] * sin_half_angle)

    def get_path(self, rootPath, primPath):
        # Start from an empty path or root slash.
        # Here we use an empty path so that the first AppendChild will become "/roomScene".
        finalPathRoot = self._defaultPrimPath

        # Split the root path by "/"
        segments = [seg for seg in rootPath.split("/") if seg]  # skip empty strings

        # Build up the path piece by piece and define scopes
        for seg in segments:
            finalPathRoot = finalPathRoot.AppendChild(seg)
            if not self._stage.GetPrimAtPath(finalPathRoot):
                # UsdGeom.Scope.Define(self._stage, finalPathRoot)
                UsdGeom.Xform.Define(self._stage, finalPathRoot)

        # Finally, append the primPath to get the "leaf" path (e.g., tableTopActor)
        finalPrimPath = finalPathRoot.AppendChild(primPath)
        if not self._stage.GetPrimAtPath(finalPrimPath):
            # UsdGeom.Scope.Define(self._stage, finalPrimPath)
            UsdGeom.Xform.Define(self._stage, finalPrimPath)

        return finalPrimPath

    def orient_dim(self, vec):
        newVec = Gf.Vec3f(0.0)
        for i in range(3):
            newVec[i] = vec[self._orientation[i]] * self._scaleFactor
        return newVec

    def orient_pos(self, vec):
        newVec = Gf.Vec3f(0.0)
        for i in range(3):
            curOrientation = self._orientation[i]
            newVec[i] = vec[curOrientation]
            if curOrientation == 2:
                newVec[i] += self._floorOffset # shift everything downwards so that the table is at the origin
            newVec[i] *= self._scaleFactor
        return newVec


class SpawnerPublisher(Node):
    def __init__(self, pg_app, payload_path, num_drones, dt_pub=2e-2):
        super().__init__('payload_quadrotors_state_publisher')

        self.world = pg_app.world
        self.stage = self.world.stage
        self.payload_path = payload_path
        self.num_drones = num_drones

        # Initialize XformCache and dynamic control
        self.xform_cache = UsdGeom.XformCache()
        self.dc = dc.acquire_dynamic_control_interface()

        # Find prim and DC handle
        self.payload_prim = self.stage.GetPrimAtPath(payload_path)
        self.payload_handle = self.dc.get_rigid_body(payload_path)

        if not self.payload_prim or not self.payload_handle:
            self.get_logger().error(f"Payload '{payload_path}' not found in stage or DC interface!")
            return
        
        # QoS setup
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.pose_publisher = self.create_publisher(PoseStamped, '/payload_pose', self.qos_profile)
        self.twist_publisher = self.create_publisher(TwistStamped, '/payload_twist', self.qos_profile)

        # Publishers for drones
        self.drone_publishers = []
        for i in range(num_drones):
            pose_pub = self.create_publisher(PoseStamped, f'/drone_{i}/pose', self.qos_profile)
            twist_pub = self.create_publisher(TwistStamped, f'/drone_{i}/twist', self.qos_profile)
            self.drone_publishers.append((pose_pub, twist_pub))

        self.timer = self.create_timer(dt_pub, self.publish_states)

    def spawn_model(pg_app, 
                uav_para = np.array([1.5, 0.02912, 0.02912, 0.05522, 3.0, 0.2]),
                load_para = np.array([3.0, 1.0]),
                cable_para = np.array([1e9, 8e-6, 1e-2, 2.0]),
                Jl = 0.7*np.array([[2.0, 2.0, 2.5]]).T,
                rg = np.array([[0.1, 0.1, -0.1]]).T
                ):
        model_instance = RigidBodyRopes()

        # Initialize the World instance
        world = pg_app.world
        # Get the stage from the World instance
        stage = world.stage

        # To test 1 rope in vertical, set num_ropes = 1.
        # num_ropes = 1
        # To test multiple ropes, set num_ropes > 1 and customed elevation angle.
        num_ropes = int(uav_para[4])

        elevation_angle = 0 # set zero for SITL, since initially the drones are on the ground
        
        # Update to align with Auto-Multilift
        payload_mass = load_para[0]
        payload_radius = load_para[1]
        payload_height = 0.1
        load_height = 0.03

        # E=1 Gpa, A=7mm^2 (pi*1.5^2), c=10, L0=2, Nylon-HD
        ropd_stiffness = cable_para[0] * cable_para[1] / cable_para[3] # 4e3 is the factor to convert to N/m
        rope_damping = cable_para[2] * 1e3
        rope_length = cable_para[3]

        model_instance.create(
            stage, 
            num_ropes=num_ropes, 
            rope_length=rope_length,
            ropd_stiffness=ropd_stiffness,
            rope_damping=rope_damping,
            payload_mass=payload_mass,
            payload_radius=payload_radius,
            payload_height=payload_height,
            Jl=Jl,
            rg=rg,
            load_height=load_height,
            elevation_angle=elevation_angle
            )
        
        # config_multirotor = MultirotorConfig()
        xformCache = UsdGeom.XformCache()
        
        # spawn the drones
        for vehicle_id in range(0, num_ropes):
            # Create the multirotor configuration
            config_multirotor = MultirotorConfig()
            # breakpoint()
            box_path = f"/World/Rope{vehicle_id}/box{vehicle_id}Actor"
            box_prim = stage.GetPrimAtPath(box_path)
            box_xform = xformCache.GetLocalToWorldTransform(box_prim)
            box_pos = box_xform.ExtractTranslation()
            drone_pos = box_pos + Gf.Vec3d(0.0, 0.0, 0.07)
            mavlink_config = PX4MavlinkBackendConfig({
                "vehicle_id": vehicle_id,
                "px4_autolaunch": True,
                "px4_dir": pg_app.pg.px4_path,
                # "px4_vehicle_model": "iris" # CHANGE this line to 'iris' if using PX4 version bellow v1.14
                "px4_vehicle_model": "none_iris" # CHANGE this line to 'iris' if using PX4 version bellow v1.14
            })
            config_multirotor.backends = [PX4MavlinkBackend(mavlink_config)]

            # Create the drone in the world
            Multirotor(
                "/World/quadrotor",
                ROBOTS['Iris'],
                vehicle_id,
                drone_pos,
                Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
                config=config_multirotor)
            
            # Create joints between loads and drones
            if vehicle_id==0 :
                droneJointPath = f"/World/Rope{vehicle_id}/droneJoint"
                droneBodyPath = f"/World/quadrotor/body"
                droneJoint = UsdPhysics.Joint.Define(stage, droneJointPath)
                droneJoint.CreateBody0Rel().SetTargets([box_path])
                droneJoint.CreateBody1Rel().SetTargets([droneBodyPath])
                joint_pos = Gf.Vec3f(0.0, 0.0, -0.08)
                droneJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
                droneJoint.CreateLocalPos1Attr().Set(joint_pos)
            else:
                droneJointPath = f"/World/Rope{vehicle_id}/droneJoint"
                droneBodyPath = f"/World/quadrotor_0{vehicle_id}/body"
                droneJoint = UsdPhysics.Joint.Define(stage, droneJointPath)
                droneJoint.CreateBody0Rel().SetTargets([box_path])
                droneJoint.CreateBody1Rel().SetTargets([droneBodyPath])
                joint_pos = Gf.Vec3f(0.0, 0.0, -0.08)
                droneJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
                droneJoint.CreateLocalPos1Attr().Set(joint_pos)


    def publish_states(self):
        # Publish payload state
        self.publish_payload_state()

        # Publish drone states BUG not suitable for Xform!!
        # for i in range(self.num_drones):
        #     if i == 0:
        #         parent_path = "/World/quadrotor"
        #     else:
        #         parent_path = f"/World/quadrotor_0{i}"
        #     body_path = f"{parent_path}/body"
        #     self.publish_drone_state(i, parent_path, body_path)

    def publish_payload_state(self):
        pose_msg = PoseStamped()
        twist_msg = TwistStamped()

        # header
        now = self.get_clock().now().to_msg()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = "world"
        twist_msg.header = pose_msg.header

        # --- Use dynamic_control for real-time pose ---
        pose = self.dc.get_rigid_body_pose(self.payload_handle)
        if pose is not None:
            position = pose.p
            rotation = pose.r

            pose_msg.pose.position.x = position.x
            pose_msg.pose.position.y = position.y
            pose_msg.pose.position.z = position.z

            pose_msg.pose.orientation.x = rotation.x
            pose_msg.pose.orientation.y = rotation.y
            pose_msg.pose.orientation.z = rotation.z
            pose_msg.pose.orientation.w = rotation.w
        else:
            self.get_logger().warn("Failed to get payload pose from dynamic_control")
        
        # --- Use dynamic_control for real-time twist ---
        # linear_velocity = self.dc.get_rigid_body_linear_velocity(self.payload_handle)
        body_frame_velocity = self.dc.get_rigid_body_local_linear_velocity(self.payload_handle)
        angular_velocity = self.dc.get_rigid_body_angular_velocity(self.payload_handle)

        # if linear_velocity is not None:
        if body_frame_velocity is not None:
            # NOTE Convert payload linear velocity to body frame for Auto-Multilift
            # rotation_matrix = Rotation.from_quat([rotation.x, rotation.y, rotation.z, rotation.w]).as_matrix()
            # body_frame_velocity = np.dot(rotation_matrix.T, np.array([linear_velocity[0], linear_velocity[1], linear_velocity[2]]))

            twist_msg.twist.linear.x = body_frame_velocity[0]
            twist_msg.twist.linear.y = body_frame_velocity[1]
            twist_msg.twist.linear.z = body_frame_velocity[2]

        if angular_velocity is not None:
            twist_msg.twist.angular.x = angular_velocity[0]
            twist_msg.twist.angular.y = angular_velocity[1]
            twist_msg.twist.angular.z = angular_velocity[2]

        # publish
        self.pose_publisher.publish(pose_msg)
        self.twist_publisher.publish(twist_msg)

    def publish_drone_state(self, drone_id, parent_path, body_path):
        pose_msg = PoseStamped()
        twist_msg = TwistStamped()

        # header
        now = self.get_clock().now().to_msg()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = "world"
        twist_msg.header = pose_msg.header

        # Get handles
        parent_handle = self.dc.get_rigid_body(parent_path)
        body_handle = self.dc.get_rigid_body(body_path)

        if not parent_handle or not body_handle:
            self.get_logger().warn(f"Drone {drone_id} handles not found!")
            return

        # Get world pose and velocity
        T_body_world, q_body_world, lv_world, av_local = self.get_world_pose_velocity(self.dc, body_handle, parent_handle)

        # Fill pose message
        pose_msg.pose.position.x = T_body_world[0]
        pose_msg.pose.position.y = T_body_world[1]
        pose_msg.pose.position.z = T_body_world[2]

        pose_msg.pose.orientation.x = q_body_world[0]
        pose_msg.pose.orientation.y = q_body_world[1]
        pose_msg.pose.orientation.z = q_body_world[2]
        pose_msg.pose.orientation.w = q_body_world[3]

        # Fill twist message
        twist_msg.twist.linear.x = lv_world[0]
        twist_msg.twist.linear.y = lv_world[1]
        twist_msg.twist.linear.z = lv_world[2]

        twist_msg.twist.angular.x = av_local[0]
        twist_msg.twist.angular.y = av_local[1]
        twist_msg.twist.angular.z = av_local[2]

        # Publish
        pose_pub, twist_pub = self.drone_publishers[drone_id]
        pose_pub.publish(pose_msg)
        twist_pub.publish(twist_msg)
    
    def get_world_pose_velocity(dc, body_handle, parent_handle):
        # Get the parent handle (/World/quadrotor) in the world coordinate system
        parent_pose = dc.get_rigid_body_pose(parent_handle)
        T_world = np.array([parent_pose.p.x, parent_pose.p.y, parent_pose.p.z])
        q_world = np.array([parent_pose.r.x, parent_pose.r.y, parent_pose.r.z, parent_pose.r.w])
        R_world = Rotation.from_quat(q_world)

        # Get the body handle (World/quadrotor/body) in the parent coordinate system
        body_pose = dc.get_rigid_body_pose(body_handle)
        T_local = np.array([body_pose.p.x, body_pose.p.y, body_pose.p.z])
        q_local = np.array([body_pose.r.x, body_pose.r.y, body_pose.r.z, body_pose.r.w])
        R_local = Rotation.from_quat(q_local)

        # Convert the local position to the world coordinate system
        T_body_world = T_world + R_world.apply(T_local)
        R_body_world = R_world * R_local
        q_body_world = R_body_world.as_quat()

        # Get the local velocity in the parent coordinate system
        lv_local = dc.get_rigid_body_linear_velocity(body_handle)
        av_local = dc.get_rigid_body_angular_velocity(body_handle)
        lv_local = np.array([lv_local.x, lv_local.y, lv_local.z])
        av_local = np.array([av_local.x, av_local.y, av_local.z])

        # Transform the local velocity to the world coordinate system
        lv_world = R_world.apply(lv_local)
        # av_world = R_world.apply(av_local)
        
        return T_body_world, q_body_world, lv_world, av_local


def spawn_model(pg_app, 
                uav_para = np.array([1.5, 0.02912, 0.02912, 0.05522, 3.0, 0.2]),
                load_para = np.array([3.0, 1.0]),
                cable_para = np.array([1e9, 8e-6, 1e-2, 2.0]),
                Jl = 0.7*np.array([[2.0, 2.0, 2.5]]).T,
                rg = np.array([[0.1, 0.1, -0.1]]).T
                ):
    model_instance = RigidBodyRopes()

    # Initialize the World instance
    world = pg_app.world
    # Get the stage from the World instance
    stage = world.stage

    # To test 1 rope in vertical, set num_ropes = 1.
    # num_ropes = 1
    # To test multiple ropes, set num_ropes > 1 and customed elevation angle.
    num_ropes = int(uav_para[4])

    elevation_angle = 0 # set zero for SITL, since initially the drones are on the ground
    
    # Update to align with Auto-Multilift
    payload_mass = load_para[0]
    payload_radius = load_para[1]
    payload_height = 0.1
    load_height = 0.03

    # E=1 Gpa, A=7mm^2 (pi*1.5^2), c=10, L0=2, Nylon-HD
    ropd_stiffness = cable_para[0] * cable_para[1] / cable_para[3] # 4e3 is the factor to convert to N/m
    rope_damping = cable_para[2] * 1e3
    rope_length = cable_para[3]

    model_instance.create(
        stage, 
        num_ropes=num_ropes, 
        rope_length=rope_length,
        ropd_stiffness=ropd_stiffness,
        rope_damping=rope_damping,
        payload_mass=payload_mass,
        payload_radius=payload_radius,
        payload_height=payload_height,
        Jl=Jl,
        rg=rg,
        load_height=load_height,
        elevation_angle=elevation_angle
        )
    
    # config_multirotor = MultirotorConfig()
    xformCache = UsdGeom.XformCache()
    
    # spawn the drones
    for vehicle_id in range(0, num_ropes):
        # Create the multirotor configuration
        config_multirotor = MultirotorConfig()
        # breakpoint()
        box_path = f"/World/Rope{vehicle_id}/box{vehicle_id}Actor"
        box_prim = stage.GetPrimAtPath(box_path)
        box_xform = xformCache.GetLocalToWorldTransform(box_prim)
        box_pos = box_xform.ExtractTranslation()
        drone_pos = box_pos + Gf.Vec3d(0.0, 0.0, 0.07)
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": vehicle_id,
            "px4_autolaunch": True,
            "px4_dir": pg_app.pg.px4_path,
            # "px4_vehicle_model": "iris" # CHANGE this line to 'iris' if using PX4 version bellow v1.14
            "px4_vehicle_model": "none_iris" # PX4 version v1.14.3    
            })
        config_multirotor.backends = [PX4MavlinkBackend(mavlink_config)]

        # Create the drone in the world
        Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            vehicle_id,
            drone_pos,
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor)
        
        # Create joints between loads and drones
        if vehicle_id==0 :
            droneJointPath = f"/World/Rope{vehicle_id}/droneJoint"
            droneBodyPath = f"/World/quadrotor/body"
            droneJoint = UsdPhysics.Joint.Define(stage, droneJointPath)
            droneJoint.CreateBody0Rel().SetTargets([box_path])
            droneJoint.CreateBody1Rel().SetTargets([droneBodyPath])
            joint_pos = Gf.Vec3f(0.0, 0.0, -0.08)
            droneJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            droneJoint.CreateLocalPos1Attr().Set(joint_pos)
        else:
            droneJointPath = f"/World/Rope{vehicle_id}/droneJoint"
            droneBodyPath = f"/World/quadrotor_0{vehicle_id}/body"
            droneJoint = UsdPhysics.Joint.Define(stage, droneJointPath)
            droneJoint.CreateBody0Rel().SetTargets([box_path])
            droneJoint.CreateBody1Rel().SetTargets([droneBodyPath])
            joint_pos = Gf.Vec3f(0.0, 0.0, -0.08)
            droneJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            droneJoint.CreateLocalPos1Attr().Set(joint_pos)


def main(args=None):
    rclpy.init(args=args)
    # Instantiate the template app
    pg_app = PegasusApp()
    
    uav_para     = np.array([1.5, 0.02912, 0.02912, 0.05522, 3.0, 0.2]) # L quadrotors XXX
    load_para    = np.array([3.0, 1.0]) # 2.5 kg for 3 quadrotors, 7.5 kg for 6 quadrotors
    cable_para   = np.array([1e9, 8e-6, 1e-2, 2.0]) # E=1 Gpa, A=7mm^2 (pi*1.5^2), c=10, L0=2, Nylon-HD, [5], np.array([5e3, 1e-2, 2])
    Jl           = 0.5*np.array([[2.0, 2.0, 2.5]]).T # payload's moment of inertia, 0.5*Jl for 3 quadrotors, Jl for 6 quadrotors
    rg           = np.array([[0.1, 0.1, -0.1]]).T # coordinate of the payload's CoM in {Bl}

    spawn_model(pg_app, uav_para, load_para, cable_para, Jl, rg)
    pg_app.world.reset() # Register the model after spawning

    dt_pub = 2e-2 # 50Hz
    payload_path = "/World/CommonPayload/Payload"
    # payload_path = "/World/quadrotor/body/body"
    node = SpawnerPublisher(pg_app, payload_path, int(uav_para[4]), dt_pub)
    # node.spawn_model(pg_app, uav_para, load_para, cable_para, Jl, rg)
    
    # Run the application loop, spin_once(node) in its while loop
    pg_app.run(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
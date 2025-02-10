""" RigidBodyRopes Class
    This class is responsible for creating a scene with elastic cable models using the USD (Universal Scene Description) framework. 
    It supports creating both single and multiple ropes with various configurations.
    Author: Li Yunchao
    Date: 2025-02-07

    Methods:
    --------
    Scene Creation:
    ---------------
    create(stage, num_ropes, rope_length, payload_mass, load_height=2.0, elevation_angle=0.0):
        Main function to build the scene with the specified number of ropes, rope length, payload mass, load height, and elevation angle.
    _createCapsule(path: Sdf.Path, axis="Z"):
        Creates a capsule geometry at the specified path with the given axis.
    _createPayload():
        Creates the payload geometry and applies the necessary physics properties.
    create_box(rootPath, primPath, dimensions, position, color, orientation=Gf.Quatf(1.0), positionMod=None):
        Creates a box geometry at the specified path with the given dimensions, position, color, and orientation.
    create_table():
        Creates a table geometry with the specified dimensions and color.

    Joint Creation:
    ---------------
    _createCableJoint(jointPath, axis="Z"):
        Creates a cable joint with the specified path and axis, applying the necessary physics properties and limits.
    _createFixJoint(jointPath):
        Creates a fixed joint with the specified path, locking all degrees of freedom.
    _createUniJoint(jointPath):
        Creates a universal joint with the specified path, locking all translational and one rotational degrees of freedom.

    Rope Creation:
    --------------
    _createVerticalRopes():
        Creates a single vertical rope as the baseline template, attaching it to the payload and table.
    _createMultiRopes(elevation_angle):
        Creates multiple ropes with the specified elevation angle, attaching them to the payload and individual boxes.

    Helper Functions:
    -----------------
    calculate_orientation(angle, axis):
        Calculates the quaternion representing a rotation around an arbitrary axis.
    get_path(rootPath, primPath):
        Constructs and returns the final path by appending the root path and prim path, defining scopes if necessary.
    orient_dim(vec):
        Adjusts the dimensions vector based on the stage's orientation and scale factor.
    orient_pos(vec):
        Adjusts the position vector based on the stage's orientation, scale factor, and floor offset.
"""

import math
from pxr import UsdLux, UsdGeom, Sdf, Gf, UsdPhysics, UsdShade, PhysxSchema, Vt
import omni.physxdemos as demo
import omni.physx.bindings._physx as physx_bindings

class RigidBodyRopes(demo.Base):
    title = "Elastic Cable Model"

    ## Main Call Funtion to build the scene ##
    def create(self, stage, num_ropes, rope_length, payload_mass, load_height=2.0, elevation_angle=0.0):
        self._stage = stage
        self._defaultPrimPath = stage.GetDefaultPrim().GetPath()
        
        ## Payload config:
        self._payloadRadius = 0.24
        self._payloadHight = self._payloadRadius / 4
        self._payloadMass = payload_mass
        self._payloadColor = [0.22, 0.43, 0.55]

        self._initLoadHeight = load_height
        self._payloadPos = Gf.Vec3f(0.0, 0.0, self._initLoadHeight)

        self._payloadXform = self._defaultPrimPath.AppendChild(f"CommonPayload")
        self._payloadPath = self._payloadXform.AppendChild("Payload")

        ## Ropes config:
        self._linkHalfLength = 0.08
        self._linkRadius = 0.01

        self._ropeLength = rope_length
        self._numRopes = num_ropes
        self._ropeSpacing = 15.0
        self._ropeColor = demo.get_primary_color()

        self._coneAngleLimit = 160
        self._slideLimit = 0.1
        # self._slideMaxforceLimit = 10.0 * self._payloadMass * 9.81
        self._slideMaxforceLimit = 1000

        self._slide_stiffness = 1e5 # stiffness for Prismatic joint 
        self._slide_damping = 1e3 # damping for Prismatic joint 

        ##### IMPORTANT: The LIMITS also influence the TRUE stiffness and damping!!! ###############################
        self._slide_stiffness_limit = 4 * self._slide_stiffness
        self._slide_damping_limit = 3*self._slide_damping
        ############################################################################################################

        ## Table / Box Config
        self._scaleFactor = 1.0 / (UsdGeom.GetStageMetersPerUnit(stage) * 100.0)

        self._tableThickness = 6.0
        self._boxSize = 12.0
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
        physx_rigid_api.CreateLinearDampingAttr(0.01)

        massAPI = UsdPhysics.MassAPI.Apply(payloadGeom.GetPrim())
        massAPI.CreateMassAttr().Set(self._payloadMass)

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
        rigidAPI.CreateRigidBodyEnabledAttr(False)

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
            physx_limit_api.CreateDampingAttr(0.1)
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

            boxAttachInstancer.GetPhysicsLocalPos0sAttr().Set(Vt.Vec3fArray([Gf.Vec3f(capsuleHalf, 0.0, 0.0)]))
            boxAttachInstancer.GetPhysicsLocalPos1sAttr().Set(Vt.Vec3fArray([Gf.Vec3f(-self._boxSize*self._scaleFactor*0.5, 0.0, 0.0)]))
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




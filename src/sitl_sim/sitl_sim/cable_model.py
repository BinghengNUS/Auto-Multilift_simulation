#!/usr/bin/env python
"""
| File: cable_model.py
| Description: Cable model for the Pegasus simulator.
| Author: Yichao Gao
"""

from isaacsim import SimulationApp
# simulation_app = SimulationApp({"headless": False})
import math
from pxr import UsdLux, UsdGeom, Sdf, Gf, UsdPhysics, UsdShade, PhysxSchema, Vt
import omni.physxdemos as demo
import omni.kit.commands


class RigidBodyRopes(demo.Base):
    title = "Elastic Cable Model"

    def __init__(self):
        return

    def create(
        self, stage, num_ropes, rope_length, payload_mass, load_height=2.0, elevation_angle=0.0
    ):
        self._stage = stage
        # Ensure "/World" exists
        if not stage.GetPrimAtPath("/World"):
            stage.DefinePrim("/World", "Xform")

        # Make "/World" the default prim explicitly
        stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
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
        # self._ropeColor = demo.get_primary_color()
        self._ropeColor = Gf.Vec3f(1.0, 1.0, 1.0)

        self._coneAngleLimit = 160
        self._slideLimit = 0.1
        self._slideMaxforceLimit = 1000

        self._slide_stiffness = 1e5  # stiffness for Prismatic joint
        self._slide_damping = 1e3  # damping for Prismatic joint

        # The LIMITS also influence the effective stiffness/damping
        self._slide_stiffness_limit = 4 * self._slide_stiffness
        self._slide_damping_limit = 3 * self._slide_damping

        # Table / Box Config
        self._scaleFactor = 1.0 / (UsdGeom.GetStageMetersPerUnit(stage) * 100.0)
        self._tableThickness = 6.0
        self._boxSize = 1.0
        self._tableHeight = (
            self._initLoadHeight
            + self._payloadHight / 2
            + rope_length
            + self._tableThickness * self._scaleFactor
        )

        floorOffset = 0.0
        self._floorOffset = floorOffset - self._tableHeight
        self._tableSurfaceDim = Gf.Vec2f(200.0, 100.0)
        self._tableColor = Gf.Vec3f(168.0, 142.0, 119.0) / 255.0

        self._tableXform = self._defaultPrimPath.AppendChild(f"Table")
        self._tableTopPath = self._tableXform.AppendChild("tableTopActor")

        self._upAxis = UsdGeom.GetStageUpAxis(stage)
        if self._upAxis == UsdGeom.Tokens.z:
            self._orientation = [0, 1, 2]
        elif self._upAxis == UsdGeom.Tokens.y:
            self._orientation = [1, 2, 0]
        else:  # self._upAxis == UsdGeom.Tokens.x
            self._orientation = [2, 1, 0]

        # Create the scene
        self._createPayload()  # Defines the payload geometry and applies an iron-like look

        if num_ropes < 2:
            self.create_table()
            self._createVerticalRopes()
        else:
            self._createMultiRopes(elevation_angle)

    ## Scene Object Functions ##
    def _createCapsule(self, path: Sdf.Path, axis="Z"):
        capsuleGeom = UsdGeom.Capsule.Define(self._stage, path)
        capsuleGeom.CreateHeightAttr(self._linkHalfLength)
        capsuleGeom.CreateRadiusAttr(self._linkRadius)
        capsuleGeom.CreateAxisAttr(axis)
        capsuleGeom.CreateDisplayColorAttr().Set([self._ropeColor])
        UsdPhysics.RigidBodyAPI.Apply(capsuleGeom.GetPrim())
        PhysxSchema.PhysxRigidBodyAPI.Apply(capsuleGeom.GetPrim())
        massAPI = UsdPhysics.MassAPI.Apply(capsuleGeom.GetPrim())
        massAPI.CreateMassAttr().Set(0.008)
        UsdPhysics.CollisionAPI.Apply(capsuleGeom.GetPrim())

    def _createPayload(self):
        """
        Defines the payload geometry and sets its mass.
        Applies a metallic 'iron-like' material for visualization only.
        """
        UsdGeom.Xform.Define(self._stage, self._payloadXform)
        payloadGeom = UsdGeom.Cylinder.Define(self._stage, self._payloadPath)
        payloadGeom.AddTranslateOp().Set(self._payloadPos)
        payloadGeom.CreateRadiusAttr(self._payloadRadius)
        payloadGeom.CreateHeightAttr(self._payloadHight)
        payloadGeom.CreateDisplayColorAttr().Set([self._payloadColor])
        # Apply physics
        rigidAPI = UsdPhysics.RigidBodyAPI.Apply(payloadGeom.GetPrim())
        rigidAPI.CreateRigidBodyEnabledAttr(True)
        PhysxSchema.PhysxRigidBodyAPI.Apply(payloadGeom.GetPrim())
        massAPI = UsdPhysics.MassAPI.Apply(payloadGeom.GetPrim())
        massAPI.CreateMassAttr().Set(self._payloadMass)
        UsdPhysics.CollisionAPI.Apply(payloadGeom.GetPrim())

        # ---------------------------
        # Create an iron-like material (visual only)
        # ---------------------------
        materialPath = Sdf.Path("/World/Looks/IronMaterial")
        ironMaterial = UsdShade.Material.Define(self._stage, materialPath)

        pbrShader = UsdShade.Shader.Define(self._stage, materialPath.AppendChild("PBRShader"))
        pbrShader.CreateIdAttr("UsdPreviewSurface")
        # Set the shader inputs to achieve a metallic appearance
        pbrShader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(1.0)
        pbrShader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.3)
        pbrShader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.2, 0.2, 0.2))
        # Create an output on the PBR shader for the "surface"
        pbrShaderOutput = pbrShader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
        # Create the material's surface output using the default render context
        materialSurfaceOutput = ironMaterial.CreateSurfaceOutput()
        materialSurfaceOutput.ConnectToSource(pbrShaderOutput)
        # Bind the material to the payload geometry (visual only)
        UsdShade.MaterialBindingAPI(payloadGeom.GetPrim()).Bind(ironMaterial)

    def create_box(self, rootPath, primPath, dimensions, position, color, orientation=Gf.Quatf(1.0), positionMod=None):
        boxActorPath = self.get_path(rootPath, primPath)
        newPosition = Gf.Vec3f(0.0)
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
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        UsdPhysics.CollisionAPI.Apply(cubePrim)

    def create_table(self):
        tableDim = Gf.Vec3f(self._tableSurfaceDim[0], self._tableSurfaceDim[1], self._tableThickness)
        self.create_box(
            "Table",
            "tableTopActor",
            Gf.Vec3f(tableDim[0], tableDim[1], self._tableThickness),
            Gf.Vec3f(0.0, 0.0, tableDim[2] - self._tableThickness * self._scaleFactor * 0.5),
            self._tableColor,
        )

    ## Joint Functions ##
    def _createCableJoint(self, jointPath, axis="Z"):
        rotatedDOFs = ["rotX", "rotY"]
        if axis == "Z":
            slideDOF = ("transZ",)
            lockedDOFs = ["transX", "transY"]
            rotatedDOFs = ["rotX", "rotY", "rotZ"]
        elif axis == "X":
            slideDOF = ("transX",)
            lockedDOFs = ["transY", "transZ"]
            rotatedDOFs = ["rotY", "rotZ", "rotX"]
        else:
            slideDOF = ("transY",)
            lockedDOFs = ["transX", "transZ"]
            rotatedDOFs = ["rotX", "rotZ", "rotY"]

        joint = UsdPhysics.Joint.Define(self._stage, jointPath)
        d6Prim = joint.GetPrim()
        for prim in slideDOF:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, prim)
            limitAPI.CreateLowAttr(-0.0001)
            limitAPI.CreateHighAttr(0.0001)
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
        for axis_name in lockedDOFs:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, axis_name)
            limitAPI.CreateLowAttr(0.0)
            limitAPI.CreateHighAttr(0.0)
        for d in rotatedDOFs:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, d)
            physx_limit_api = PhysxSchema.PhysxLimitAPI.Apply(d6Prim, d)
            driveAPI = UsdPhysics.DriveAPI.Apply(d6Prim, d)
            driveAPI.CreateTypeAttr("force")
            driveAPI.CreateDampingAttr(0.0005)

    def _createFixJoint(self, jointPath):
        joint = UsdPhysics.Joint.Define(self._stage, jointPath)
        d6Prim = joint.GetPrim()
        for axis in ["transX", "transY", "transZ", "rotX", "rotY", "rotZ"]:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, axis)
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)

    def _createUniJoint(self, jointPath):
        joint = UsdPhysics.Joint.Define(self._stage, jointPath)
        d6Prim = joint.GetPrim()
        for axis in ["transX", "transY", "transZ"]:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, axis)
            limitAPI.CreateLowAttr(0.0)
            limitAPI.CreateHighAttr(0.0)

    ## Rope Functions ##
    def _createVerticalRopes(self):
        linkLength = self._linkHalfLength + 2 * self._linkRadius
        numLinks = int(self._ropeLength / linkLength)
        payloadHightHalf = self._payloadHight * 0.5
        capsuleHalf = linkLength * 0.5
        xStart = 0.0
        yStart = -(self._numRopes // 2) * self._ropeSpacing
        zStart = self._initLoadHeight + payloadHightHalf + capsuleHalf

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
            yPos = yStart + ropeInd * self._ropeSpacing
            for linkInd in range(numLinks):
                meshIndices.append(0)
                z = zStart + linkInd * linkLength
                positions.append(Gf.Vec3f(xStart, yPos, z))
                orientations.append(Gf.Quath(1.0, 0.0, 0.0, 0.0))
            meshList = rboInstancer.GetPrototypesRel()
            meshList.AddTarget(capsulePath)
            rboInstancer.GetProtoIndicesAttr().Set(Vt.IntArray(meshIndices))
            rboInstancer.GetPositionsAttr().Set(Vt.Vec3fArray(positions))
            rboInstancer.GetOrientationsAttr().Set(Vt.QuathArray(orientations))
            jointInstancerPath = scopePath.AppendChild("jointInstancer")
            jointInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, jointInstancerPath)
            chainJointPath = jointInstancerPath.AppendChild("chainJoints")
            self._createCableJoint(chainJointPath)
            jointMeshIndices = []
            jointBody0Indices = []
            jointBody1Indices = []
            jointLocalPos0 = []
            jointLocalPos1 = []
            jointLocalRot0 = []
            jointLocalRot1 = []
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
            jointInstancer.GetPhysicsPrototypesRel().AddTarget(chainJointPath)
            jointInstancer.GetPhysicsBody0sRel().SetTargets([instancerPath])
            jointInstancer.GetPhysicsBody1sRel().SetTargets([instancerPath])
            jointInstancer.GetPhysicsProtoIndicesAttr().Set(Vt.IntArray(jointMeshIndices))
            jointInstancer.GetPhysicsBody0IndicesAttr().Set(Vt.IntArray(jointBody0Indices))
            jointInstancer.GetPhysicsBody1IndicesAttr().Set(Vt.IntArray(jointBody1Indices))
            jointInstancer.GetPhysicsLocalPos0sAttr().Set(Vt.Vec3fArray(jointLocalPos0))
            jointInstancer.GetPhysicsLocalPos1sAttr().Set(Vt.Vec3fArray(jointLocalPos1))
            jointInstancer.GetPhysicsLocalRot0sAttr().Set(Vt.QuathArray(jointLocalRot0))
            jointInstancer.GetPhysicsLocalRot1sAttr().Set(Vt.QuathArray(jointLocalRot1))
            payloadAttachScopePath = scopePath.AppendChild("ropePayloadCon")
            payloadAttachInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, payloadAttachScopePath)
            PayloadJointPath = payloadAttachScopePath.AppendChild("PayloadJoint")
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
            tableAttachScopePath = scopePath.AppendChild("ropeTableCon")
            tableAttachInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, tableAttachScopePath)
            TableJointPath = tableAttachScopePath.AppendChild("TableJoint")
            self._createUniJoint(TableJointPath)
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

    def _createMultiRopes(self, elevation_angle):
        translationAxis = "X"
        linkLength = self._linkHalfLength + 2 * self._linkRadius
        numLinks = int(self._ropeLength / linkLength)
        payloadHightHalf = self._payloadHight * 0.5
        capsuleHalf = linkLength * 0.5
        angle_increment = 2 * math.pi / self._numRopes
        cos_elevation = math.cos(elevation_angle)
        sin_elevation = math.sin(elevation_angle)
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
            angle = angle_increment * ropeInd
            xstartPos = (self._payloadRadius + capsuleHalf * cos_elevation) * math.cos(angle)
            ystartPos = (self._payloadRadius + capsuleHalf * cos_elevation) * math.sin(angle)
            zstartPos = self._initLoadHeight + capsuleHalf * sin_elevation
            q_z = self.calculate_orientation(angle, Gf.Vec3f(0.0, 0.0, 1.0))
            q_y = self.calculate_orientation(elevation_angle, Gf.Vec3f(0.0, -1.0, 0.0))
            orientation = q_z * q_y
            for linkInd in range(numLinks):
                meshIndices.append(0)
                x = xstartPos + linkInd * linkLength * math.cos(angle) * cos_elevation
                y = ystartPos + linkInd * linkLength * math.sin(angle) * cos_elevation
                z = zstartPos + linkInd * linkLength * sin_elevation
                positions.append(Gf.Vec3f(x, y, z))
                orientations.append(orientation)
            meshList = rboInstancer.GetPrototypesRel()
            meshList.AddTarget(capsulePath)
            rboInstancer.GetProtoIndicesAttr().Set(Vt.IntArray(meshIndices))
            rboInstancer.GetPositionsAttr().Set(Vt.Vec3fArray(positions))
            rboInstancer.GetOrientationsAttr().Set(Vt.QuathArray(orientations))
            jointInstancerPath = scopePath.AppendChild("jointInstancer")
            jointInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, jointInstancerPath)
            chainJointPath = jointInstancerPath.AppendChild("chainJoints")
            self._createCableJoint(chainJointPath, translationAxis)
            jointMeshIndices = []
            jointBody0Indices = []
            jointBody1Indices = []
            jointLocalPos0 = []
            jointLocalPos1 = []
            jointLocalRot0 = []
            jointLocalRot1 = []
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
            jointInstancer.GetPhysicsPrototypesRel().AddTarget(chainJointPath)
            jointInstancer.GetPhysicsBody0sRel().SetTargets([instancerPath])
            jointInstancer.GetPhysicsBody1sRel().SetTargets([instancerPath])
            jointInstancer.GetPhysicsProtoIndicesAttr().Set(Vt.IntArray(jointMeshIndices))
            jointInstancer.GetPhysicsBody0IndicesAttr().Set(Vt.IntArray(jointBody0Indices))
            jointInstancer.GetPhysicsBody1IndicesAttr().Set(Vt.IntArray(jointBody1Indices))
            jointInstancer.GetPhysicsLocalPos0sAttr().Set(Vt.Vec3fArray(jointLocalPos0))
            jointInstancer.GetPhysicsLocalPos1sAttr().Set(Vt.Vec3fArray(jointLocalPos1))
            jointInstancer.GetPhysicsLocalRot0sAttr().Set(Vt.QuathArray(jointLocalRot0))
            jointInstancer.GetPhysicsLocalRot1sAttr().Set(Vt.QuathArray(jointLocalRot1))
            payloadAttachScopePath = scopePath.AppendChild("ropePayloadCon")
            payloadAttachInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, payloadAttachScopePath)
            PayloadJointPath = payloadAttachScopePath.AppendChild("PayloadJoint")
            self._createCableJoint(PayloadJointPath)
            payloadAttachInstancer.GetPhysicsPrototypesRel().AddTarget(PayloadJointPath)
            payloadAttachInstancer.GetPhysicsBody0sRel().SetTargets([self._payloadPath])
            payloadAttachInstancer.GetPhysicsBody1sRel().SetTargets([instancerPath])
            payloadAttachInstancer.GetPhysicsProtoIndicesAttr().Set(Vt.IntArray([0]))
            payloadAttachInstancer.GetPhysicsBody0IndicesAttr().Set(Vt.IntArray([0]))
            payloadAttachInstancer.GetPhysicsBody1IndicesAttr().Set(Vt.IntArray([0]))
            payloadAttachInstancer.GetPhysicsLocalPos0sAttr().Set(
                Vt.Vec3fArray([Gf.Vec3f(self._payloadRadius * math.cos(angle), self._payloadRadius * math.sin(angle), 0.0)])
            )
            payloadAttachInstancer.GetPhysicsLocalPos1sAttr().Set(Vt.Vec3fArray([Gf.Vec3f(-capsuleHalf, 0.0, 0.0)]))
            payloadAttachInstancer.GetPhysicsLocalRot0sAttr().Set(Vt.QuathArray([orientation]))
            payloadAttachInstancer.GetPhysicsLocalRot1sAttr().Set(Vt.QuathArray([Gf.Quath(1.0)]))
            boxPath = scopePath.AppendChild(f"box{ropeInd}Actor")
            x = positions[-1][0]
            y = positions[-1][1]
            z = positions[-1][2]
            self.create_box(
                f"Rope{ropeInd}",
                f"box{ropeInd}Actor",
                Gf.Vec3f(self._boxSize, self._boxSize, self._boxSize),
                Gf.Vec3f(
                    x + (capsuleHalf + self._boxSize * self._scaleFactor * 0.5) * math.cos(angle) * cos_elevation,
                    y + (capsuleHalf + self._boxSize * self._scaleFactor * 0.5) * math.sin(angle) * cos_elevation,
                    z + (capsuleHalf + self._boxSize * self._scaleFactor * 0.5) * sin_elevation,
                ),
                self._tableColor,
                orientation=Gf.Quatf(orientation),
            )
            boxAttachScopePath = scopePath.AppendChild("ropeBoxCon")
            boxAttachInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, boxAttachScopePath)
            BoxJointPath = boxAttachScopePath.AppendChild("BoxJoint")
            self._createUniJoint(BoxJointPath)
            boxAttachInstancer.GetPhysicsPrototypesRel().AddTarget(BoxJointPath)
            boxAttachInstancer.GetPhysicsBody0sRel().SetTargets([instancerPath])
            boxAttachInstancer.GetPhysicsBody1sRel().SetTargets([boxPath])
            boxAttachInstancer.GetPhysicsProtoIndicesAttr().Set(Vt.IntArray([0]))
            boxAttachInstancer.GetPhysicsBody0IndicesAttr().Set(Vt.IntArray([numLinks - 1]))
            boxAttachInstancer.GetPhysicsBody1IndicesAttr().Set(Vt.IntArray([0]))
            boxAttachInstancer.GetPhysicsLocalPos0sAttr().Set(
                Vt.Vec3fArray([Gf.Vec3f(capsuleHalf + self._boxSize * self._scaleFactor * 0.5, 0.0, 0.0)])
            )
            boxAttachInstancer.GetPhysicsLocalPos1sAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0, 0.0, 0.0)]))
            boxAttachInstancer.GetPhysicsLocalRot0sAttr().Set(Vt.QuathArray([Gf.Quath(1.0)]))
            boxAttachInstancer.GetPhysicsLocalRot1sAttr().Set(Vt.QuathArray([Gf.Quath(1.0)]))

    def calculate_orientation(self, angle, axis):
        half_angle = angle / 2
        sin_half_angle = math.sin(half_angle)
        cos_half_angle = math.cos(half_angle)
        axis = axis.GetNormalized()
        return Gf.Quath(cos_half_angle, axis[0] * sin_half_angle, axis[1] * sin_half_angle, axis[2] * sin_half_angle)

    def get_path(self, rootPath, primPath):
        finalPathRoot = self._defaultPrimPath
        segments = [seg for seg in rootPath.split("/") if seg]
        for seg in segments:
            finalPathRoot = finalPathRoot.AppendChild(seg)
            if not self._stage.GetPrimAtPath(finalPathRoot):
                UsdGeom.Xform.Define(self._stage, finalPathRoot)
        finalPrimPath = finalPathRoot.AppendChild(primPath)
        if not self._stage.GetPrimAtPath(finalPrimPath):
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
                newVec[i] += self._floorOffset
            newVec[i] *= self._scaleFactor
        return newVec



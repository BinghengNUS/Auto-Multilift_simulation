import carb
from pxr import UsdLux, UsdGeom, Sdf, Gf, UsdPhysics, UsdShade, PhysxSchema, Vt
import omni.physxdemos as demo
from omni.physx.scripts import physicsUtils
import omni.physx.bindings._physx as physx_bindings
from omni.isaac.dynamic_control import _dynamic_control
import math

class RigidBodyRopesDemo(demo.Base):
    title = "test Cable with load"
    category = demo.Categories.JOINTS
    short_description = "Self made cable with a small load by Gao"
    description = "Cable with elasticity"

    kit_settings = {
        physx_bindings.SETTING_MOUSE_PICKING_FORCE: 10.0,
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
    }
    
    params = {
        "num_ropes": demo.IntParam(3, 1, 100, 1),
        "rope_length": demo.IntParam(100, 100, 1000, 10)
    }

    def create(self, stage, num_ropes, rope_length):
        self._stage = stage
        self._defaultPrimPath, scene = demo.setup_physics_scene(self, stage, primPathAsString=False)
        room = demo.get_demo_room(self, stage, zoom=0.3, hasTable=False, hasWalls=False, onlyUsePlane=True)

        # payload config
        self._payloadSize = 7
        self._paylaodDensity = 2
        self._payloadColor = [0.56, 0.57, 0.58]
        self._initHangHeight = rope_length + 100.0

        # Rope configuration:
        self.capsuleDensity = 0.1
        self._linkHalfLength = 4
        self._linkRadius = 1      # radius = 1
        # Capsule total length = height(2) + 2*radius(1) = 2 + 2 = 4 units
        self._ropeLength = rope_length
        self._numRopes = num_ropes
        self._ropeSpacing = 15.0
        self._ropeColor = demo.get_primary_color()
        self._coneAngleLimit = 160
        self._rope_damping = 0.1   # rotation damping
        self._rope_stiffness = 0.1 # rotation stiffness
        self._slideLimit = 10
        rope_stiffness = 100000
        self._slide_damping = 1000
        self._slide_stiffness = rope_stiffness

        # Collider capsule configuration:
        self._capsuleZ = 590.0
        self._capsuleHeight = 400.0
        self._capsuleRadius = 10.0
        self._capsuleRestOffset = -2.0
        self._capsuleColor = demo.get_static_color()

        # Table Config
        self._tableHeight = self._initHangHeight
        self._tableSurfaceDim = Gf.Vec2f(200.0, 100.0)
        self._tableColor = Gf.Vec3f(168.0, 142.0, 119.0) / 255.0

        # Physics material:
        self._contactOffset = 2.0
        self._physicsMaterialPath = self._defaultPrimPath.AppendChild("PhysicsMaterial")
        UsdShade.Material.Define(self._stage, self._physicsMaterialPath)
        material = UsdPhysics.MaterialAPI.Apply(self._stage.GetPrimAtPath(self._physicsMaterialPath))
        material.CreateStaticFrictionAttr().Set(0.5)
        material.CreateDynamicFrictionAttr().Set(0.5)
        material.CreateRestitutionAttr().Set(0)


        # self.hookPath = self._defaultPrimPath.AppendChild("hook")

        # self._createRopes()
        # self._createHook(self.hookPath)
        # self._createVerticalRopes()
        # self._createColliderCapsule()
        self.create_table()

    def _createCapsule(self, path: Sdf.Path):
        capsuleGeom = UsdGeom.Capsule.Define(self._stage, path)
        # height=2, radius=1, axis=X
        capsuleGeom.CreateHeightAttr(self._linkHalfLength)
        capsuleGeom.CreateRadiusAttr(self._linkRadius)
        capsuleGeom.CreateAxisAttr("X")
        capsuleGeom.CreateDisplayColorAttr().Set([self._ropeColor])

        UsdPhysics.CollisionAPI.Apply(capsuleGeom.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(capsuleGeom.GetPrim())
        massAPI = UsdPhysics.MassAPI.Apply(capsuleGeom.GetPrim())
        massAPI.CreateDensityAttr().Set(self.capsuleDensity)
        physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(capsuleGeom.GetPrim())
        physxCollisionAPI.CreateRestOffsetAttr().Set(0.0)
        physxCollisionAPI.CreateContactOffsetAttr().Set(self._contactOffset)
        physicsUtils.add_physics_material_to_prim(self._stage, capsuleGeom.GetPrim(), self._physicsMaterialPath)

    def _createJoint(self, jointPath):
        joint = UsdPhysics.Joint.Define(self._stage, jointPath)
        d6Prim = joint.GetPrim()

        for prim in ["transX"]:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, prim)
            limitAPI.CreateLowAttr(-self._slideLimit)  
            limitAPI.CreateHighAttr(self._slideLimit)  
            driveAPI = UsdPhysics.DriveAPI.Apply(d6Prim, prim)
            driveAPI.CreateTypeAttr("force")
            driveAPI.CreateDampingAttr(self._slide_damping)  # damping
            driveAPI.CreateStiffnessAttr(self._slide_stiffness)  # stiffness

        # Lock transX/Y/Z and rotX:
        for axis in ["transY", "transZ", "rotX"]:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, axis)
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)

        # rotY, rotZ free with limits:
        for d in ["rotY", "rotZ"]:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, d)
            limitAPI.CreateLowAttr(-self._coneAngleLimit)
            limitAPI.CreateHighAttr(self._coneAngleLimit)
            driveAPI = UsdPhysics.DriveAPI.Apply(d6Prim, d)
            driveAPI.CreateTypeAttr("force")
            # High stiffness and damping to keep rope taut
            driveAPI.CreateStiffnessAttr().Set(self._rope_stiffness)
            driveAPI.CreateDampingAttr().Set(self._rope_damping)
        
    def _createFixJoint(self, jointPath):
        joint = UsdPhysics.Joint.Define(self._stage, jointPath)
        d6Prim = joint.GetPrim()
        # Lock joint:
        for axis in ["transX", "transY", "transZ", "rotX", "rotY", "rotZ"]:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, axis)
            limitAPI.CreateLowAttr(1.0)
            limitAPI.CreateHighAttr(-1.0)  

    def _createColliderCapsule(self):
        capsulePath = self._defaultPrimPath.AppendChild("CapsuleCollider")
        capsuleGeom = UsdGeom.Capsule.Define(self._stage, capsulePath)
        capsuleGeom.CreateHeightAttr(self._capsuleHeight)   # setup the pos
        capsuleGeom.CreateRadiusAttr(self._capsuleRadius)
        capsuleGeom.CreateAxisAttr("Y")
        capsuleGeom.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, self._capsuleZ))
        capsuleGeom.CreateDisplayColorAttr().Set([self._capsuleColor])

        capsulePrim = capsuleGeom.GetPrim()
        capsulePrim.CreateAttribute("refinementEnableOverride", Sdf.ValueTypeNames.Bool, True).Set(True)
        capsulePrim.CreateAttribute("refinementLevel", Sdf.ValueTypeNames.Int, True).Set(2)

        UsdPhysics.CollisionAPI.Apply(capsulePrim)
        physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(capsulePrim)
        physxCollisionAPI.CreateRestOffsetAttr().Set(self._capsuleRestOffset)
        physxCollisionAPI.CreateContactOffsetAttr().Set(self._contactOffset)
        physicsUtils.add_physics_material_to_prim(self._stage, capsulePrim, self._physicsMaterialPath)

        
    def _createPayload(self, path: Sdf.Path):
        payloadGeom = UsdGeom.Cube.Define(self._stage, path)
        payloadGeom.CreateSizeAttr(self._payloadSize)
        payloadGeom.CreateDisplayColorAttr().Set([self._payloadColor])

        UsdPhysics.CollisionAPI.Apply(payloadGeom.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(payloadGeom.GetPrim())
        massAPI = UsdPhysics.MassAPI.Apply(payloadGeom.GetPrim())
        massAPI.CreateDensityAttr().Set(self._paylaodDensity)  # Set density
        # physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(payloadGeom.GetPrim())
        # physxCollisionAPI.CreateRestOffsetAttr().Set(0.1)
        # physxCollisionAPI.CreateContactOffsetAttr().Set(0.2)
        physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(payloadGeom.GetPrim())
        physxCollisionAPI.CreateRestOffsetAttr().Set(self._capsuleRestOffset)
        physxCollisionAPI.CreateContactOffsetAttr().Set(self._contactOffset)
        physicsUtils.add_physics_material_to_prim(self._stage, payloadGeom.GetPrim(), self._physicsMaterialPath)


    def _createRopes(self):
        linkLength = 4.0
        numLinks = int(self._ropeLength / linkLength)

        # Center the rope along X so that the rope is under the red capsule if capsule at X=0
        xStart = -numLinks * linkLength * 0.5  
        yStart = -(self._numRopes // 2) * self._ropeSpacing

        payloadHalf = self._payloadSize * 0.5
        jointX = linkLength/2  # half of link length

        for ropeInd in range(self._numRopes):
            scopePath = self._defaultPrimPath.AppendChild(f"Rope{ropeInd}")
            UsdGeom.Scope.Define(self._stage, scopePath)
            
            instancerPath = scopePath.AppendChild("rigidBodyInstancer")
            rboInstancer = UsdGeom.PointInstancer.Define(self._stage, instancerPath)

            # Create prototypes: capsule(0) and payload(1)
            capsulePath = instancerPath.AppendChild("capsule")
            self._createCapsule(capsulePath)
            payloadPath = instancerPath.AppendChild("payload")
            self._createPayload(payloadPath)

            meshIndices = []
            positions = []
            orientations = []
            
            y = yStart + ropeInd * self._ropeSpacing
            z = self._capsuleZ + self._capsuleRadius + self._linkRadius * 1.4

            # Rope links
            for linkInd in range(numLinks):
                meshIndices.append(0)  # capsule
                x = xStart + linkInd * linkLength
                positions.append(Gf.Vec3f(x, y, z))
                orientations.append(Gf.Quath(1.0))

            # Add payload flush to the last link end
            lastLinkCenterX = xStart + (numLinks - 1)*linkLength
            payloadCenterX = lastLinkCenterX + jointX + payloadHalf
            payload2CenterX = xStart - jointX - payloadHalf
            meshIndices.append(1)  # payload
            positions.append(Gf.Vec3f(payloadCenterX, y, z))
            orientations.append(Gf.Quath(1.0))
            meshIndices.append(1)  # payload2
            positions.append(Gf.Vec3f(payload2CenterX, y, z))
            orientations.append(Gf.Quath(1.0))

            # Set instancer attributes
            meshList = rboInstancer.GetPrototypesRel()
            meshList.AddTarget(capsulePath)
            meshList.AddTarget(payloadPath)

            rboInstancer.GetProtoIndicesAttr().Set(Vt.IntArray(meshIndices))
            rboInstancer.GetPositionsAttr().Set(Vt.Vec3fArray(positions))
            rboInstancer.GetOrientationsAttr().Set(Vt.QuathArray(orientations))

            # Joint instancer (only D6 joints)
            jointInstancerPath = scopePath.AppendChild("jointInstancer")
            jointInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, jointInstancerPath)
            jointPath = jointInstancerPath.AppendChild("joint")
            self._createJoint(jointPath)

            # Prepare joint arrays
            jointMeshIndices = []
            jointBody0Indices = []
            jointBody1Indices = []
            jointLocalPos0 = []
            jointLocalPos1 = []
            jointLocalRot0 = []
            jointLocalRot1 = []

            # Connect links with joints
            # Each joint between linkInd and linkInd+1
            for linkInd in range(numLinks - 1):
                body0Index = linkInd
                body1Index = linkInd + 1
                # end-to-end: link end at +2, next link start at -2
                jointMeshIndices.append(0)
                jointBody0Indices.append(body0Index)
                jointBody1Indices.append(body1Index)
                jointLocalPos0.append(Gf.Vec3f(jointX, 0, 0))
                jointLocalPos1.append(Gf.Vec3f(-jointX, 0, 0))
                jointLocalRot0.append(Gf.Quath(1.0))
                jointLocalRot1.append(Gf.Quath(1.0))

            # Final joint from last link to payload
            finalLinkIndex = numLinks - 1
            payloadIndex = numLinks
            jointMeshIndices.append(0)
            jointBody0Indices.append(finalLinkIndex)
            jointBody1Indices.append(payloadIndex)
            # link end at +2, payload start face at -50
            jointLocalPos0.append(Gf.Vec3f(jointX, 0, 0))
            jointLocalPos1.append(Gf.Vec3f(-payloadHalf, 0, 0))
            jointLocalRot0.append(Gf.Quath(1.0))
            jointLocalRot1.append(Gf.Quath(1.0))
            # Inital joint from first link to payload
            firstLinkIndex = 0
            payload2Index = numLinks + 1
            jointMeshIndices.append(0)
            jointBody0Indices.append(firstLinkIndex)
            jointBody1Indices.append(payload2Index)
            jointLocalPos0.append(Gf.Vec3f(-jointX, 0, 0))
            jointLocalPos1.append(Gf.Vec3f(payloadHalf, 0, 0))
            jointLocalRot0.append(Gf.Quath(1.0))
            jointLocalRot1.append(Gf.Quath(1.0))

            # Set joint instancer data
            jointInstancer.GetPhysicsPrototypesRel().AddTarget(jointPath)
            jointInstancer.GetPhysicsBody0sRel().SetTargets([instancerPath])
            jointInstancer.GetPhysicsBody1sRel().SetTargets([instancerPath])
            jointInstancer.GetPhysicsProtoIndicesAttr().Set(Vt.IntArray(jointMeshIndices))
            jointInstancer.GetPhysicsBody0IndicesAttr().Set(Vt.IntArray(jointBody0Indices))
            jointInstancer.GetPhysicsBody1IndicesAttr().Set(Vt.IntArray(jointBody1Indices))
            jointInstancer.GetPhysicsLocalPos0sAttr().Set(Vt.Vec3fArray(jointLocalPos0))
            jointInstancer.GetPhysicsLocalPos1sAttr().Set(Vt.Vec3fArray(jointLocalPos1))
            jointInstancer.GetPhysicsLocalRot0sAttr().Set(Vt.QuathArray(jointLocalRot0))
            jointInstancer.GetPhysicsLocalRot1sAttr().Set(Vt.QuathArray(jointLocalRot1))


    # def _createVerticalRopes(self):
    #     import math
    #     linkLength = 4.0
    #     numLinks = int(self._ropeLength / linkLength)

    #     # We'll build the rope along decreasing Z from top to bottom.
    #     xStart = 0.0
    #     # Stagger ropes along Y
    #     yStart = -(self._numRopes // 2) * self._ropeSpacing

    #     payloadHalf = self._payloadSize * 0.5  # 5 -> 2.5
    #     initHangHeight = self._initHangHeight

    #     jointX = linkLength * 0.5

    #     # The top of the rope starts at zStart
    #     zStart = initHangHeight - self._capsuleRadius - self._linkRadius * 1.4

    #     # Rotate each capsule (and payload) by 90° around Y so that local X -> world Z
    #     half_pi = math.pi * 0.5
    #     rot90_y = Gf.Quath(math.cos(half_pi / 2), 0.0, math.sin(half_pi / 2), 0.0)

    #     hookPath = self._defaultPrimPath.AppendChild("Hook") 
    #     self._createHook(hookPath)

    #     for ropeInd in range(self._numRopes):
    #         scopePath = self._defaultPrimPath.AppendChild(f"Rope{ropeInd}")
    #         UsdGeom.Scope.Define(self._stage, scopePath)

    #         instancerPath = scopePath.AppendChild("rigidBodyInstancer")
    #         rboInstancer = UsdGeom.PointInstancer.Define(self._stage, instancerPath)

    #         # Create prototypes: capsule(0) and payload(1)
    #         capsulePath = instancerPath.AppendChild("capsule")
    #         self._createCapsule(capsulePath)
    #         payloadPath = instancerPath.AppendChild("payload")
    #         self._createPayload(payloadPath)

    #         meshIndices = []
    #         positions = []
    #         orientations = []

    #         # Rope offset in Y for each rope
    #         yPos = yStart + ropeInd * self._ropeSpacing

    #         for linkInd in range(numLinks):
    #             meshIndices.append(0)  # capsule
    #             x = xStart
    #             # Decreasing Z each time
    #             z = zStart - linkInd * linkLength
    #             positions.append(Gf.Vec3f(x, yPos, z))
    #             orientations.append(rot90_y)  # Rotate link so local X -> world Z

    #         # Last link is at index (numLinks - 1)
    #         lastLinkCenterZ = zStart - (numLinks - 1) * linkLength
    #         # Move payload below that by half a link + half the cube
    #         payloadCenterZ = lastLinkCenterZ - (jointX + payloadHalf)

    #         meshIndices.append(1)  # payload
    #         positions.append(Gf.Vec3f(xStart, yPos, payloadCenterZ))
    #         orientations.append(rot90_y)  # Payload also oriented local X -> world Z

    #         # Set instancer data
    #         meshList = rboInstancer.GetPrototypesRel()
    #         meshList.AddTarget(capsulePath)
    #         meshList.AddTarget(payloadPath)
    #         meshList.AddTarget(hookPath)

    #         rboInstancer.GetProtoIndicesAttr().Set(Vt.IntArray(meshIndices))
    #         rboInstancer.GetPositionsAttr().Set(Vt.Vec3fArray(positions))
    #         rboInstancer.GetOrientationsAttr().Set(Vt.QuathArray(orientations))

    #         jointInstancerPath = scopePath.AppendChild("jointInstancer")
    #         jointInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, jointInstancerPath)
    #         jointPath = jointInstancerPath.AppendChild("joint")
    #         self._createJoint(jointPath)

    #         # Prepare joint arrays
    #         jointMeshIndices = []
    #         jointBody0Indices = []
    #         jointBody1Indices = []
    #         jointLocalPos0 = []
    #         jointLocalPos1 = []
    #         jointLocalRot0 = []
    #         jointLocalRot1 = []

    #         for linkInd in range(numLinks - 1):
    #             body0Index = linkInd
    #             body1Index = linkInd + 1
    #             jointMeshIndices.append(0)  # We only have one prototype joint
    #             jointBody0Indices.append(body0Index)
    #             jointBody1Indices.append(body1Index)

    #             # Each capsule is 4.0 in local X, so the "tips" are ±(2,0,0).
    #             jointLocalPos0.append(Gf.Vec3f(+jointX, 0.0, 0.0))
    #             jointLocalPos1.append(Gf.Vec3f(-jointX, 0.0, 0.0))

    #             # Keep these local rotations as identity because
    #             # the entire geometry is already rotated by rot90_y at the instancer level.
    #             jointLocalRot0.append(Gf.Quath(1.0))
    #             jointLocalRot1.append(Gf.Quath(1.0))

    #         finalLinkIndex = numLinks - 1
    #         payloadIndex = numLinks
    #         jointMeshIndices.append(0)
    #         jointBody0Indices.append(finalLinkIndex)
    #         jointBody1Indices.append(payloadIndex)
    #         # Last link end is local +2 in X, payload near face is local -2.5 in X (if size=5)
    #         jointLocalPos0.append(Gf.Vec3f(+jointX, 0.0, 0.0))
    #         jointLocalPos1.append(Gf.Vec3f(-payloadHalf, 0.0, 0.0))
    #         jointLocalRot0.append(Gf.Quath(1.0))
    #         jointLocalRot1.append(Gf.Quath(1.0))
            
    #         initLinkIndex = 0
    #         HookIndex = numLinks + 1
    #         jointMeshIndices.append(0)
    #         jointBody0Indices.append(HookIndex)
    #         jointBody1Indices.append(initLinkIndex)
    #         jointLocalPos0.append(Gf.Vec3f(0.0, 0.0, 0.0))
    #         jointLocalPos1.append(Gf.Vec3f(0.0, 0.0, 0.0))
    #         jointLocalRot0.append(Gf.Quath(1.0))
    #         jointLocalRot1.append(Gf.Quath(1.0))

    #         jointInstancer.GetPhysicsPrototypesRel().AddTarget(jointPath)
    #         jointInstancer.GetPhysicsBody0sRel().SetTargets([instancerPath])
    #         jointInstancer.GetPhysicsBody1sRel().SetTargets([instancerPath])

    #         jointInstancer.GetPhysicsProtoIndicesAttr().Set(Vt.IntArray(jointMeshIndices))
    #         jointInstancer.GetPhysicsBody0IndicesAttr().Set(Vt.IntArray(jointBody0Indices))
    #         jointInstancer.GetPhysicsBody1IndicesAttr().Set(Vt.IntArray(jointBody1Indices))
    #         jointInstancer.GetPhysicsLocalPos0sAttr().Set(Vt.Vec3fArray(jointLocalPos0))
    #         jointInstancer.GetPhysicsLocalPos1sAttr().Set(Vt.Vec3fArray(jointLocalPos1))
    #         jointInstancer.GetPhysicsLocalRot0sAttr().Set(Vt.QuathArray(jointLocalRot0))
    #         jointInstancer.GetPhysicsLocalRot1sAttr().Set(Vt.QuathArray(jointLocalRot1))


    def _createVerticalRopes(self):
        linkLength = 4.0
        numLinks = int(self._ropeLength / linkLength)

        xStart = 0.0
        # Stagger ropes along Y
        yStart = -(self._numRopes // 2) * self._ropeSpacing

        payloadHalf = self._payloadSize * 0.5  # e.g. 2.5 if payloadSize=5
        initHangHeight = self._initHangHeight

        jointX = linkLength * 0.5

        # The top of the rope starts at zStart
        zStart = initHangHeight - self._capsuleRadius - self._linkRadius * 1.4

        # Rotate each capsule (and payload) by 90° around Y so that local X -> world Z
        half_pi = math.pi * 0.5
        rot90_y = Gf.Quath(math.cos(half_pi / 2), 0.0, math.sin(half_pi / 2), 0.0)

        # Now create each rope
        for ropeInd in range(self._numRopes):
            scopePath = self._defaultPrimPath.AppendChild(f"Rope{ropeInd}")
            UsdGeom.Scope.Define(self._stage, scopePath)

            instancerPath = scopePath.AppendChild("rigidBodyInstancer")
            rboInstancer = UsdGeom.PointInstancer.Define(self._stage, instancerPath)

            # Create prototypes: capsule(0) and payload(1)
            capsulePath = instancerPath.AppendChild("capsule")
            self._createCapsule(capsulePath)
            payloadPath = instancerPath.AppendChild("payload")
            self._createPayload(payloadPath)

            meshIndices = []
            positions = []
            orientations = []

            # Rope offset in Y for each rope
            yPos = yStart + ropeInd * self._ropeSpacing

            # 1) Add rope links (all capsules)
            for linkInd in range(numLinks):
                meshIndices.append(0)  # capsule
                x = xStart
                z = zStart - linkInd * linkLength
                positions.append(Gf.Vec3f(x, yPos, z))
                orientations.append(rot90_y)  # rotate link so local X -> world Z

            # 2) Add one payload
            lastLinkCenterZ = zStart - (numLinks - 1) * linkLength
            payloadCenterZ = lastLinkCenterZ - (jointX + payloadHalf)

            meshIndices.append(1)  # payload
            positions.append(Gf.Vec3f(xStart, yPos, payloadCenterZ))
            orientations.append(rot90_y)

            # 3) Set up instancer attributes
            meshList = rboInstancer.GetPrototypesRel()
            meshList.AddTarget(capsulePath)
            meshList.AddTarget(payloadPath)
            # Do NOT add hookPath here, so remove: meshList.AddTarget(hookPath)

            rboInstancer.GetProtoIndicesAttr().Set(Vt.IntArray(meshIndices))
            rboInstancer.GetPositionsAttr().Set(Vt.Vec3fArray(positions))
            rboInstancer.GetOrientationsAttr().Set(Vt.QuathArray(orientations))

            # 4) Create a D6 joint prototype for all rope link–link and link–payload connections
            jointInstancerPath = scopePath.AppendChild("jointInstancer")
            jointInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, jointInstancerPath)
            jointPath = jointInstancerPath.AppendChild("joint")
            self._createJoint(jointPath)  # your D6 joint setup

            jointMeshIndices = []
            jointBody0Indices = []
            jointBody1Indices = []
            jointLocalPos0 = []
            jointLocalPos1 = []
            jointLocalRot0 = []
            jointLocalRot1 = []

            # Link–Link
            for linkInd in range(numLinks - 1):
                body0Index = linkInd
                body1Index = linkInd + 1
                jointMeshIndices.append(0)
                jointBody0Indices.append(body0Index)
                jointBody1Indices.append(body1Index)

                jointLocalPos0.append(Gf.Vec3f(+jointX, 0.0, 0.0))  # tip of capsule
                jointLocalPos1.append(Gf.Vec3f(-jointX, 0.0, 0.0))
                jointLocalRot0.append(Gf.Quath(1.0))
                jointLocalRot1.append(Gf.Quath(1.0))

            # Last link – Payload
            finalLinkIndex = numLinks - 1
            payloadIndex   = numLinks
            jointMeshIndices.append(0)
            jointBody0Indices.append(finalLinkIndex)
            jointBody1Indices.append(payloadIndex)

            jointLocalPos0.append(Gf.Vec3f(+jointX, 0.0, 0.0))
            jointLocalPos1.append(Gf.Vec3f(-payloadHalf, 0.0, 0.0))
            jointLocalRot0.append(Gf.Quath(1.0))
            jointLocalRot1.append(Gf.Quath(1.0))

            # initialLinkIndex = 0
            # hookIndex   = numLinks + 1 
            # jointMeshIndices.append(0)
            # jointBody0Indices.append(initialLinkIndex)
            # jointBody1Indices.append(hookIndex)

            # jointLocalPos0.append(Gf.Vec3f(+jointX, 0.0, 0.0))
            # jointLocalPos1.append(Gf.Vec3f(-payloadHalf, 0.0, 0.0))
            # jointLocalRot0.append(Gf.Quath(1.0))
            # jointLocalRot1.append(Gf.Quath(1.0))

            jointInstancer.GetPhysicsPrototypesRel().AddTarget(jointPath)
            jointInstancer.GetPhysicsBody0sRel().SetTargets([instancerPath])
            jointInstancer.GetPhysicsBody1sRel().SetTargets([instancerPath])
            jointInstancer.GetPhysicsProtoIndicesAttr().Set(Vt.IntArray(jointMeshIndices))
            jointInstancer.GetPhysicsBody0IndicesAttr().Set(Vt.IntArray(jointBody0Indices))
            jointInstancer.GetPhysicsBody1IndicesAttr().Set(Vt.IntArray(jointBody1Indices))
            jointInstancer.GetPhysicsLocalPos0sAttr().Set(Vt.Vec3fArray(jointLocalPos0))
            jointInstancer.GetPhysicsLocalPos1sAttr().Set(Vt.Vec3fArray(jointLocalPos1))
            jointInstancer.GetPhysicsLocalRot0sAttr().Set(Vt.QuathArray(jointLocalRot0))
            jointInstancer.GetPhysicsLocalRot1sAttr().Set(Vt.QuathArray(jointLocalRot1))

            

            # attachScopePath = scopePath.AppendChild("HookAttachment")
            # fixJointPath = attachScopePath.AppendChild("fixJoint")
            # self._createFixJoint(fixJointPath)  # locks all DoFs

            # fixJointPrim = self._stage.GetPrimAtPath(fixJointPath)
            # fixJoint = UsdPhysics.Joint(fixJointPrim)
            # # Body0 = the Hook (no indices needed, because it's a single prim)
            # hookPath = self.hookPath
            # fixJoint.GetBody0Rel().SetTargets([hookPath])
            # # Body1 = the rope's first link (point instancer), so we set index [0]
            # fixJoint.GetBody1Rel().SetTargets([instancerPath])
            # fixJointPrim.CreateAttribute("physics:body1:indices", Sdf.ValueTypeNames.IntArray).Set([0])

            # # local anchor on the hook side (body0)
            # fixJointPrim.CreateAttribute("physics:localPos0", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(0.0, 0.0, 0.0))
            # # local anchor on the rope's link side (body1)
            # fixJointPrim.CreateAttribute("physics:localPos1", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(-jointX, 0.0, 0.0))

            # fixJointPrim.CreateAttribute("physics:localRot0", Sdf.ValueTypeNames.Quath).Set(Gf.Quatf(1.0))
            # fixJointPrim.CreateAttribute("physics:localRot1", Sdf.ValueTypeNames.Quath).Set(Gf.Quatf(1.0))

            # Last link – Payload
            # fixjoint_path = Sdf.Path(f"/World/cable_{ropeInd}_joint_attach")
            # fixjoint = UsdPhysics.FixedJoint.Define(self._stage, fixjoint_path)
            # fixjoint.CreateBody0Rel().SetTargets([instancerPath])
            # fixjoint.CreateBody1Rel().SetTargets([hookPath])

            # fixjoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
            # fixjoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
            # fixjoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            # fixjoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    
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
                UsdGeom.Scope.Define(self._stage, finalPathRoot)

        # Finally, append the primPath to get the "leaf" path (e.g., tableTopActor)
        finalPrimPath = finalPathRoot.AppendChild(primPath)
        if not self._stage.GetPrimAtPath(finalPrimPath):
            UsdGeom.Scope.Define(self._stage, finalPrimPath)

        return finalPrimPath

    def create_box(self, rootPath, primPath, dimensions, position, color, positionMod = None):
        boxActorPath = self.get_path(rootPath, primPath)
        newPosition = Gf.Vec3f(0.0)
        orientation = Gf.Quatf(1.0)

        # deep copy to avoid modifying reference
        for i in range(3):
            newPosition[i] = position[i]
            if positionMod:
                newPosition[i] *= positionMod[i]

        cubeGeom = UsdGeom.Cube.Define(self._stage, boxActorPath)
        cubePrim = self._stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(1.0)
        half_extent = 0.5
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(self.orient_pos(newPosition))
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(self.orient_dim(dimensions))
        cubeGeom.CreateDisplayColorAttr().Set([color])    
            
    def create_table(self):
        # always create table, do not skip if instance exists because it can vary between rooms
        tableDim = Gf.Vec3f(self._tableSurfaceDim[0], self._tableSurfaceDim[1], self._tableHeight)
        tableThickness = 1.0
        legThickness = 1.0
        legOffset = 0.0
        legTotalOffset = (legOffset + legThickness*0.5)

        self._tableTopPath = self.create_box(
            "roomScene/colliders/table","tableTopActor",
            Gf.Vec3f(tableDim[0], tableDim[1], tableThickness),
            Gf.Vec3f(0.0, 0.0, tableDim[2] - tableThickness*0.5),
            self._tableColor
        )

        offsetArr = [Gf.Vec2f(-1.0,-1.0), Gf.Vec2f(-1.0,1.0), Gf.Vec2f(1.0,-1.0), Gf.Vec2f(1.0,1.0)]

        i = 0
        for offsetValue in offsetArr:
            self.create_box(
                "roomScene/colliders/table","tableLeg" + str(i) + "Actor",
                Gf.Vec3f(legThickness, legThickness, tableDim[2] - tableThickness),
                Gf.Vec3f(offsetValue[0]*(tableDim[0]*0.5 - legTotalOffset), offsetValue[1]*(tableDim[1]*0.5 - legTotalOffset), (tableDim[2] - tableThickness)*0.5 ),
                self._tableColor
            )
            i += 1

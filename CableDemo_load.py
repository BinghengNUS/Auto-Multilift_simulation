import carb
from pxr import UsdLux, UsdGeom, Sdf, Gf, UsdPhysics, UsdShade, PhysxSchema, Vt
import omni.physxdemos as demo
from omni.physx.scripts import physicsUtils
import omni.physx.bindings._physx as physx_bindings

class RigidBodyRopesDemo(demo.Base):
    title = "Cable test"
    category = demo.Categories.JOINTS
    short_description = "Self made cable by Gao"
    description = "Cable with elasticity"

    kit_settings = {
        physx_bindings.SETTING_MOUSE_PICKING_FORCE: 10.0,
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
    }
    
    params = {
        "num_ropes": demo.IntParam(3, 1, 100, 1),
        "rope_length": demo.IntParam(300, 100, 1000, 10)
    }

    def create(self, stage, num_ropes, rope_length):
        self._stage = stage
        self._defaultPrimPath, scene = demo.setup_physics_scene(self, stage, primPathAsString=False)
        room = demo.get_demo_room(self, stage, zoom=0.3, hasTable=False)

        # Rope configuration:
        self.capsuleDensity = 0.0005
        self._payloadSize = 5
        self._payloadColor = [0,0.5,1]
        self._linkHalfLength = 2
        self._linkRadius = 1      # radius = 1
        # Capsule total length = height(2) + 2*radius(1) = 2 + 2 = 4 units
        self._ropeLength = rope_length
        self._numRopes = num_ropes
        self._ropeSpacing = 15.0
        self._ropeColor = demo.get_primary_color()
        self._coneAngleLimit = 110
        self._rope_damping = 0.0   # Increased damping
        self._rope_stiffness = 0.0 # Increased stiffness
        self._slideLimit = 10
        self._slide_damping = 10
        self._slide_stiffness = 100000

        # Collider capsule configuration:
        self._capsuleZ = 50.0
        self._capsuleHeight = 400.0
        self._capsuleRadius = 20.0
        self._capsuleRestOffset = -2.0
        self._capsuleColor = demo.get_static_color()

        # Physics material:
        self._contactOffset = 2.0
        self._physicsMaterialPath = self._defaultPrimPath.AppendChild("PhysicsMaterial")
        UsdShade.Material.Define(self._stage, self._physicsMaterialPath)
        material = UsdPhysics.MaterialAPI.Apply(self._stage.GetPrimAtPath(self._physicsMaterialPath))
        material.CreateStaticFrictionAttr().Set(0.5)
        material.CreateDynamicFrictionAttr().Set(0.5)
        material.CreateRestitutionAttr().Set(0)

        self._createRopes()
        self._createColliderCapsule()

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

    def _createPayload(self, path: Sdf.Path):
        payloadGeom = UsdGeom.Cube.Define(self._stage, path)
        payloadGeom.CreateSizeAttr(self._payloadSize)
        payloadGeom.CreateDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*self._payloadColor)]))

        UsdPhysics.CollisionAPI.Apply(payloadGeom.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(payloadGeom.GetPrim())
        massAPI = UsdPhysics.MassAPI.Apply(payloadGeom.GetPrim())
        massAPI.CreateDensityAttr().Set(1)  # Set density
        physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(payloadGeom.GetPrim())
        physxCollisionAPI.CreateRestOffsetAttr().Set(0.1)
        physxCollisionAPI.CreateContactOffsetAttr().Set(0.2)
        physicsUtils.add_physics_material_to_prim(self._stage, payloadGeom.GetPrim(), self._physicsMaterialPath)

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
        
        
        

    def _createColliderCapsule(self):
        capsulePath = self._defaultPrimPath.AppendChild("CapsuleCollider")
        capsuleGeom = UsdGeom.Capsule.Define(self._stage, capsulePath)
        capsuleGeom.CreateHeightAttr(self._capsuleHeight)
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

    def _createRopes(self):
        linkLength = 4.0
        numLinks = int(self._ropeLength / linkLength)

        # Center the rope along X so that the rope is under the red capsule if capsule at X=0
        xStart = -numLinks * linkLength * 0.5  
        yStart = -(self._numRopes // 2) * self._ropeSpacing

        payloadHalf = self._payloadSize * 0.5
        jointX = 2.0  # half of link length

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
            meshIndices.append(1)  # payload
            positions.append(Gf.Vec3f(payloadCenterX, y, z))
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

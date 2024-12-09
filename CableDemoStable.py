import carb
from pxr import UsdLux, UsdGeom, Sdf, Gf, UsdPhysics, UsdShade, PhysxSchema
import omni.physxdemos as demo
from omni.physx.scripts import physicsUtils
import omni.physx.bindings._physx as physx_bindings


class RigidBodyRopesDemo(demo.Base):
    title = "Cable Stable"
    category = demo.Categories.JOINTS
    short_description = "Self made cable by Gao, stable"
    description = "Cable with elasticity"

    kit_settings = {
        physx_bindings.SETTING_MOUSE_PICKING_FORCE: 10.0,
        "persistent/app/viewport/displayOptions": demo.get_viewport_minimal_display_options_int(),
    }
    
    params = { "num_ropes": demo.IntParam(3, 1, 100, 1),
        "rope_length": demo.IntParam(300, 100, 1000, 10)}

    def create(self, stage, num_ropes, rope_length):
        self._stage = stage
        self._defaultPrimPath, scene = demo.setup_physics_scene(self, stage, primPathAsString = False)
        room = demo.get_demo_room(self, stage, zoom = 0.3, hasTable = False)

        # configure ropes:
        self._linkHalfLength = 2
        self._linkRadius = 0.5 * self._linkHalfLength
        self._ropeLength = rope_length
        self._numRopes = num_ropes
        self._ropeSpacing = 15.0
        self._ropeColor = demo.get_primary_color()
        self._coneAngleLimit = 110
        self._rope_damping = 0.0
        self._rope_stiffness = 0.0
        self._slideLimit = 10.0
        self._slide_stiffness = 1
        self._slide_damping = 0.1

        # configure collider capsule:
        self._capsuleZ = 50.0
        self._capsuleHeight = 400.0
        self._capsuleRadius = 20.0
        self._capsuleRestOffset = -2.0
        self._capsuleColor = demo.get_static_color()

        # physics options:
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
        capsuleGeom.CreateHeightAttr(self._linkHalfLength)
        capsuleGeom.CreateRadiusAttr(self._linkRadius)
        capsuleGeom.CreateAxisAttr("X")
        capsuleGeom.CreateDisplayColorAttr().Set([self._ropeColor])

        UsdPhysics.CollisionAPI.Apply(capsuleGeom.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(capsuleGeom.GetPrim())
        massAPI = UsdPhysics.MassAPI.Apply(capsuleGeom.GetPrim())
        massAPI.CreateDensityAttr().Set(0.005)
        physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(capsuleGeom.GetPrim())
        physxCollisionAPI.CreateRestOffsetAttr().Set(0.0)
        physxCollisionAPI.CreateContactOffsetAttr().Set(self._contactOffset)
        physicsUtils.add_physics_material_to_prim(self._stage, capsuleGeom.GetPrim(), self._physicsMaterialPath)

    def _createJoint(self, jointPath):        
        joint = UsdPhysics.Joint.Define(self._stage, jointPath)

        # locked DOF (lock - low is greater than high)
        d6Prim = joint.GetPrim()
        limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "transX")
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)
        limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "transY")
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)
        limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "transZ")
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)
        limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, "rotX")
        limitAPI.CreateLowAttr(1.0)
        limitAPI.CreateHighAttr(-1.0)

        # Moving DOF:
        dofs = ["rotY", "rotZ"]
        for d in dofs:
            limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, d)
            limitAPI.CreateLowAttr(-self._coneAngleLimit)
            limitAPI.CreateHighAttr(self._coneAngleLimit)

            # joint drives for rope dynamics:
            driveAPI = UsdPhysics.DriveAPI.Apply(d6Prim, d)
            driveAPI.CreateTypeAttr("force")
            driveAPI.CreateDampingAttr(self._rope_damping)
            driveAPI.CreateStiffnessAttr(self._rope_stiffness)
            
    def _createPrismaticJoint(self, jointPath):
        # 定义关节
        joint = UsdPhysics.PrismaticJoint.Define(self._stage, jointPath)
        # 获取关节的原语
        prim = joint.GetPrim()
        # 设置滑动方向的限制（例如，transX 表示沿 X 轴滑动）
        limitAPI = UsdPhysics.LimitAPI.Apply(prim, "transX")
        limitAPI.CreateLowAttr(-self._slideLimit)  # 例如：-10.0
        limitAPI.CreateHighAttr(self._slideLimit)  # 例如：10.0
        # 锁定其他方向的运动和旋转
        lockedDOFs = ["transY", "transZ", "rotX", "rotY", "rotZ"]
        for dof in lockedDOFs:
            limitAPI = UsdPhysics.LimitAPI.Apply(prim, dof)
            limitAPI.CreateLowAttr(1.0)  
            limitAPI.CreateHighAttr(-1.0)
        driveAPI = UsdPhysics.DriveAPI.Apply(prim, "transX")
        driveAPI.CreateTypeAttr("force")
        driveAPI.CreateDampingAttr(self._slide_damping)  # damping
        driveAPI.CreateStiffnessAttr(self._slide_stiffness)  # stiffness


    def _createColliderCapsule(self):
        capsulePath = self._defaultPrimPath.AppendChild("CapsuleCollider")
        capsuleGeom = UsdGeom.Capsule.Define(self._stage, capsulePath)
        capsuleGeom.CreateHeightAttr(self._capsuleHeight)
        capsuleGeom.CreateRadiusAttr(self._capsuleRadius)
        capsuleGeom.CreateAxisAttr("Y")
        capsuleGeom.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, self._capsuleZ))
        capsuleGeom.CreateDisplayColorAttr().Set([self._capsuleColor])

        # make the capsule high-quality render
        capsulePrim = capsuleGeom.GetPrim()
        capsulePrim.CreateAttribute("refinementEnableOverride", Sdf.ValueTypeNames.Bool, True).Set(True)
        capsulePrim.CreateAttribute("refinementLevel", Sdf.ValueTypeNames.Int, True).Set(2)

        UsdPhysics.CollisionAPI.Apply(capsulePrim)
        physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(capsulePrim)
        physxCollisionAPI.CreateRestOffsetAttr().Set(self._capsuleRestOffset)
        physxCollisionAPI.CreateContactOffsetAttr().Set(self._contactOffset)
        physicsUtils.add_physics_material_to_prim(self._stage, capsulePrim, self._physicsMaterialPath)

    def _createRopes(self):
        linkLength = 2.0 * self._linkHalfLength - self._linkRadius
        numLinks = int(self._ropeLength / linkLength)
        xStart = -numLinks * linkLength * 0.5
        yStart = -(self._numRopes // 2) * self._ropeSpacing

        for ropeInd in range(self._numRopes):
            scopePath = self._defaultPrimPath.AppendChild(f"Rope{ropeInd}")
            UsdGeom.Scope.Define(self._stage, scopePath)
            
            # Capsule instancer
            instancerPath = scopePath.AppendChild("rigidBodyInstancer")
            rboInstancer = UsdGeom.PointInstancer.Define(self._stage, instancerPath)
            
            capsulePath = instancerPath.AppendChild("capsule")
            self._createCapsule(capsulePath)
            
            meshIndices = []
            positions = []
            orientations = []
            
            y = yStart + ropeInd * self._ropeSpacing
            z = self._capsuleZ + self._capsuleRadius + self._linkRadius * 1.4            

            for linkInd in range(numLinks):
                meshIndices.append(0)
                x = xStart + linkInd * linkLength
                positions.append(Gf.Vec3f(x, y, z))
                orientations.append(Gf.Quath(1.0))

            meshList = rboInstancer.GetPrototypesRel()
            # Add mesh reference to point instancer
            meshList.AddTarget(capsulePath)

            rboInstancer.GetProtoIndicesAttr().Set(meshIndices)
            rboInstancer.GetPositionsAttr().Set(positions)
            rboInstancer.GetOrientationsAttr().Set(orientations)
            
            # Joint and Prismatic Instancer
            jointInstancerPath = scopePath.AppendChild("jointInstancer")
            jointInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, jointInstancerPath)
            
            jointPath = jointInstancerPath.AppendChild("joint")
            self._createJoint(jointPath)

            prismaticInstancerPath = scopePath.AppendChild("prismaticInstancer")
            prismaticInstancer = PhysxSchema.PhysxPhysicsJointInstancer.Define(self._stage, prismaticInstancerPath)
            
            prismaticPath = prismaticInstancerPath.AppendChild("prismatic")
            self._createPrismaticJoint(prismaticPath)
            
            meshIndices = []
            body0s = []
            body0indices = []
            localPos0 = []
            localRot0 = []
            body1s = []
            body1indices = []
            localPos1 = []
            localRot1 = []      
            body0s.append(instancerPath)
            body1s.append(instancerPath)

            jointX = self._linkHalfLength - 0.5 * self._linkRadius
            for linkInd in range(numLinks - 1):
                meshIndices.append(0)
                
                if linkInd % 2 == 0:
                    # Joint connection
                    body0indices.append(linkInd)
                    body1indices.append(linkInd + 1)
                    localPos0.append(Gf.Vec3f(jointX, 0, 0)) 
                    localPos1.append(Gf.Vec3f(-jointX, 0, 0)) 
                    localRot0.append(Gf.Quath(1.0))
                    localRot1.append(Gf.Quath(1.0))
                else:
                    # Prismatic connection
                    body0indices.append(linkInd)
                    body1indices.append(linkInd + 1)
                    localPos0.append(Gf.Vec3f(jointX, 0, 0))
                    localPos1.append(Gf.Vec3f(-jointX, 0, 0))
                    localRot0.append(Gf.Quath(1.0))
                    localRot1.append(Gf.Quath(1.0))

            # Add Joint prototypes
            meshList = jointInstancer.GetPhysicsPrototypesRel()
            meshList.AddTarget(jointPath)
            
            jointInstancer.GetPhysicsProtoIndicesAttr().Set(meshIndices)
            jointInstancer.GetPhysicsBody0sRel().SetTargets(body0s)
            jointInstancer.GetPhysicsBody0IndicesAttr().Set(body0indices)
            jointInstancer.GetPhysicsLocalPos0sAttr().Set(localPos0)
            jointInstancer.GetPhysicsLocalRot0sAttr().Set(localRot0)
            jointInstancer.GetPhysicsBody1sRel().SetTargets(body1s)
            jointInstancer.GetPhysicsBody1IndicesAttr().Set(body1indices)
            jointInstancer.GetPhysicsLocalPos1sAttr().Set(localPos1)
            jointInstancer.GetPhysicsLocalRot1sAttr().Set(localRot1)

            # Add Prismatic prototypes
            meshList = prismaticInstancer.GetPhysicsPrototypesRel()
            meshList.AddTarget(prismaticPath)
            
            prismaticInstancer.GetPhysicsProtoIndicesAttr().Set(meshIndices)
            prismaticInstancer.GetPhysicsBody0sRel().SetTargets(body0s)
            prismaticInstancer.GetPhysicsBody0IndicesAttr().Set(body0indices)
            prismaticInstancer.GetPhysicsLocalPos0sAttr().Set(localPos0)
            prismaticInstancer.GetPhysicsLocalRot0sAttr().Set(localRot0)
            prismaticInstancer.GetPhysicsBody1sRel().SetTargets(body1s)
            prismaticInstancer.GetPhysicsBody1IndicesAttr().Set(body1indices)
            prismaticInstancer.GetPhysicsLocalPos1sAttr().Set(localPos1)
            prismaticInstancer.GetPhysicsLocalRot1sAttr().Set(localRot1)




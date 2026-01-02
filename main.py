from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
from pxr import UsdGeom, UsdLux, Gf, UsdPhysics, PhysxSchema

from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.core.objects import GroundPlane
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.types import ArticulationAction

# -------------------------------------------------
# World and Robot
# -------------------------------------------------
world = World(stage_units_in_meters=1.0)
GroundPlane("/World/Ground")

franka = Franka(prim_path="/World/Franka", name="franka")
world.scene.add(franka)

ee = XFormPrim("/World/Franka/panda_hand")
stage = world.scene.stage

# -------------------------------------------------
# Table
# -------------------------------------------------
table = UsdGeom.Cube.Define(stage, "/World/Table")
table.CreateSizeAttr(1.0)
table.AddTranslateOp().Set(Gf.Vec3f(0.67, 0.0, 0.30))
table.AddScaleOp().Set(Gf.Vec3f(0.4, 0.4, 0.6))
table.CreateDisplayColorAttr([(0.7, 0.7, 0.7)])
UsdPhysics.CollisionAPI.Apply(table.GetPrim())

# -------------------------------------------------
# Cube (Pick Object)
# -------------------------------------------------
cube_pos = np.array([0.67, 0.0, 0.61])

cube = UsdGeom.Cube.Define(stage, "/World/Cube")
cube.CreateSizeAttr(1.0)
cube.AddTranslateOp().Set(Gf.Vec3f(*cube_pos))
cube.AddScaleOp().Set(Gf.Vec3f(0.05, 0.05, 0.05))
cube.CreateDisplayColorAttr([(1.0, 0.0, 0.0)])

cube_prim = stage.GetPrimAtPath("/World/Cube")

# Collision
UsdPhysics.CollisionAPI.Apply(cube_prim)

# Rigid body
UsdPhysics.RigidBodyAPI.Apply(cube_prim)

# Mass
mass_api = UsdPhysics.MassAPI.Apply(cube_prim)
mass_api.CreateMassAttr(0.05)

# PhysX stability settings
physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(cube_prim)
physx_rb.CreateSolverPositionIterationCountAttr(16)
physx_rb.CreateSolverVelocityIterationCountAttr(16)

# -------------------------------------------------
# Target
# -------------------------------------------------
target = UsdGeom.Cube.Define(stage, "/World/Target")
target.CreateSizeAttr(1.0)
target.AddTranslateOp().Set(Gf.Vec3f(0.5, 0.3, 0.61))
target.AddScaleOp().Set(Gf.Vec3f(0.05, 0.05, 0.01))
target.CreateDisplayColorAttr([(0.0, 1.0, 0.0)])

# -------------------------------------------------
# Light
# -------------------------------------------------
light = UsdLux.DistantLight.Define(stage, "/World/Light")
light.CreateIntensityAttr(3000)
light.CreateAngleAttr(0.5)

# -------------------------------------------------
# Init
# -------------------------------------------------
world.reset()
franka.initialize()

controller = franka.get_articulation_controller()

# -------------------------------------------------
# Joint Configurations
# -------------------------------------------------
home = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.04, 0.04])

pick = np.array([
    0.0, 0.50, 0.0, -0.90,
    0.0, 1.35, 0.785,
    0.04, 0.04
])

# -------------------------------------------------
# Home
# -------------------------------------------------
for _ in range(100):
    controller.apply_action(ArticulationAction(joint_positions=home))
    world.step(render=True)

# -------------------------------------------------
# Approach
# -------------------------------------------------
for _ in range(120):
    controller.apply_action(ArticulationAction(joint_positions=pick))
    world.step(render=True)

# -------------------------------------------------
# Grasp (close gripper)
# -------------------------------------------------
grasp = pick.copy()
grasp[-2:] = [0.015, 0.015]

for _ in range(80):
    controller.apply_action(ArticulationAction(joint_positions=grasp))
    world.step(render=True)

# -------------------------------------------------
# Attach cube using FixedJoint
# -------------------------------------------------
grasp_joint = UsdPhysics.FixedJoint.Define(
    stage,
    "/World/GraspJoint"
)
grasp_joint.CreateBody0Rel().SetTargets(
    ["/World/Franka/panda_hand"]
)
grasp_joint.CreateBody1Rel().SetTargets(
    ["/World/Cube"]
)

# -------------------------------------------------
# Lift
# -------------------------------------------------
lift = grasp.copy()
lift[1] -= 0.5

for _ in range(120):
    controller.apply_action(ArticulationAction(joint_positions=lift))
    world.step(render=True)

# -------------------------------------------------
# Move
# -------------------------------------------------
move = np.array([
    0.5, 0.2, 0.2, -1.2,
    0.0, 1.4, 0.785,
    0.015, 0.015
])

for _ in range(150):
    controller.apply_action(ArticulationAction(joint_positions=move))
    world.step(render=True)

# -------------------------------------------------
# Descend
# -------------------------------------------------
place = move.copy()
place[1] += 0.3

for _ in range(100):
    controller.apply_action(ArticulationAction(joint_positions=place))
    world.step(render=True)

# -------------------------------------------------
# Release
# -------------------------------------------------
release = place.copy()
release[-2:] = [0.04, 0.04]

for _ in range(80):
    controller.apply_action(ArticulationAction(joint_positions=release))
    world.step(render=True)

# Remove joint to detach cube
stage.RemovePrim("/World/GraspJoint")

# -------------------------------------------------
# Return Home
# -------------------------------------------------
for _ in range(120):
    controller.apply_action(ArticulationAction(joint_positions=home))
    world.step(render=True)

simulation_app.close()

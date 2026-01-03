# sim/world.py

import numpy as np
from pxr import UsdGeom, UsdLux, Gf, UsdPhysics, PhysxSchema

from omni.isaac.core import World
from omni.isaac.core.objects import GroundPlane


class SimulationWorld:
    def __init__(self):
        # Isaac World 생성
        self.world = World(stage_units_in_meters=1.0)
        self.stage = self.world.scene.stage

        # 기본 환경
        self._add_ground()
        self._add_light()

        # 작업 환경
        self._add_table()
        self._add_cube()
        self._add_target()

    # -------------------------------------------------
    # Public API
    # -------------------------------------------------
    def reset(self):
        self.world.reset()

    def step(self, render=True):
        self.world.step(render=render)

    def get_world(self):
        return self.world

    # -------------------------------------------------
    # Private: Environment
    # -------------------------------------------------
    def _add_ground(self):
        GroundPlane("/World/Ground")

    def _add_light(self):
        light = UsdLux.DistantLight.Define(self.stage, "/World/Light")
        light.CreateIntensityAttr(3000)
        light.CreateAngleAttr(0.5)

    def _add_table(self):
        table = UsdGeom.Cube.Define(self.stage, "/World/Table")
        table.CreateSizeAttr(1.0)
        table.AddTranslateOp().Set(Gf.Vec3f(0.67, 0.0, 0.30))
        table.AddScaleOp().Set(Gf.Vec3f(0.4, 0.4, 0.6))
        table.CreateDisplayColorAttr([(0.7, 0.7, 0.7)])

        UsdPhysics.CollisionAPI.Apply(table.GetPrim())

    def _add_cube(self):
        cube_pos = np.array([0.67, 0.0, 0.61])

        cube = UsdGeom.Cube.Define(self.stage, "/World/Cube")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3f(*cube_pos))
        cube.AddScaleOp().Set(Gf.Vec3f(0.05, 0.05, 0.05))
        cube.CreateDisplayColorAttr([(1.0, 0.0, 0.0)])

        cube_prim = cube.GetPrim()

        # Physics
        UsdPhysics.CollisionAPI.Apply(cube_prim)
        UsdPhysics.RigidBodyAPI.Apply(cube_prim)

        mass_api = UsdPhysics.MassAPI.Apply(cube_prim)
        mass_api.CreateMassAttr(0.05)

        physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(cube_prim)
        physx_rb.CreateSolverPositionIterationCountAttr(16)
        physx_rb.CreateSolverVelocityIterationCountAttr(16)

    def _add_target(self):
        target = UsdGeom.Cube.Define(self.stage, "/World/Target")
        target.CreateSizeAttr(1.0)
        target.AddTranslateOp().Set(Gf.Vec3f(0.5, 0.3, 0.61))
        target.AddScaleOp().Set(Gf.Vec3f(0.05, 0.05, 0.01))
        target.CreateDisplayColorAttr([(0.0, 1.0, 0.0)])

# sim/world.py

import numpy as np
from pxr import UsdGeom, UsdLux, Gf, UsdPhysics, PhysxSchema, UsdShade, Sdf

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
        
        # Material 적용 (회색 테이블)
        self._apply_color_material(table.GetPrim(), (0.7, 0.7, 0.7), "/World/Looks/TableMat")
        
        UsdPhysics.CollisionAPI.Apply(table.GetPrim())

    def _add_cube(self):
        cube_pos = np.array([0.67, 0.0, 0.61])

        cube = UsdGeom.Cube.Define(self.stage, "/World/Cube")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3f(*cube_pos))
        cube.AddScaleOp().Set(Gf.Vec3f(0.05, 0.05, 0.05))
        
        cube_prim = cube.GetPrim()

        # Material 적용 (빨간색 큐브) - DisplayColor 대신
        self._apply_color_material(cube_prim, (1.0, 0.0, 0.0), "/World/Looks/RedMat")

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
        
        # Material 적용 (초록색 타겟)
        self._apply_color_material(target.GetPrim(), (0.0, 1.0, 0.0), "/World/Looks/GreenMat")

    # -------------------------------------------------
    # Helper: Material 생성
    # -------------------------------------------------
    def _apply_color_material(self, prim, color_rgb, material_path):
        """
        간단한 색상 Material 생성 및 적용
        
        Args:
            prim: UsdPrim 객체
            color_rgb: (r, g, b) tuple (0.0~1.0)
            material_path: Material 경로 (예: "/World/Looks/RedMat")
        """
        # Material 생성
        material = UsdShade.Material.Define(self.stage, material_path)
        shader = UsdShade.Shader.Define(self.stage, f"{material_path}/Shader")
        
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color_rgb)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        
        # Shader 출력 연결
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        
        # Prim에 Material 바인딩
        UsdShade.MaterialBindingAPI(prim).Bind(material)
        
        print(f"Applied material {material_path} with color {color_rgb}")
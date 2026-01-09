# sim/world.py
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid, FixedCuboid


class SimulationWorld:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)

    def setup_scene(self):
        """장면 설정 - 받침대 위에 큐브"""
        
        # 조명 추가
        self._add_lighting()
        
        # 바닥 (밝은 회색)
        from omni.isaac.core.objects import GroundPlane
        ground = GroundPlane(
            prim_path="/World/Ground",
            size=10.0,
            color=np.array([0.8, 0.8, 0.8])
        )
        self.world.scene.add(ground)
        
        # 받침대 (고정된 큐브 - 테이블 역할)
        pedestal = FixedCuboid(
            prim_path="/World/Pedestal",
            name="pedestal",
            position=np.array([0.5, 0.0, 0.30]),  # 높이 30cm
            scale=np.array([0.2, 0.2, 0.6]),      # 20cm x 20cm, 높이 60cm
            color=np.array([0.6, 0.4, 0.2]),      # 갈색
        )
        self.world.scene.add(pedestal)
        
        # 큐브 (받침대 위에 배치)
        # 받침대 높이(0.6m) + 받침대 절반(0.3m) + 큐브 절반(0.025m)
        cube_height = 0.60 + 0.025
        
        cube = DynamicCuboid(
            prim_path="/World/Cube",
            name="cube",
            position=np.array([0.5, 0.0, cube_height]),
            scale=np.array([0.05, 0.05, 0.05]),
            color=np.array([1.0, 1.0, 1.0]),
            mass=0.1,
        )
        self.world.scene.add(cube)
        
        print("[World] Scene created with pedestal")
        print(f"  Pedestal position: {pedestal.get_world_pose()[0]}")
        print(f"  Cube initial position: {cube.get_world_pose()[0]}")

    def _add_lighting(self):
        """조명 추가"""
        from pxr import UsdLux, Gf
        from omni.isaac.core.utils.stage import get_current_stage
        
        stage = get_current_stage()
        
        # Distant Light
        distant_light = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
        distant_light.CreateIntensityAttr(3000)
        distant_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 0.95))
        
        # Dome Light
        dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
        dome_light.CreateIntensityAttr(1000)
        dome_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))

    def get_world(self):
        return self.world

    def reset(self):
        self.world.reset()
        self.setup_scene()

    def step(self, render=True):
        self.world.step(render=render)
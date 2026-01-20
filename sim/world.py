# sim/world.py
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid, FixedCuboid, GroundPlane
from omni.isaac.franka import Franka
from pxr import UsdLux, Gf
from omni.isaac.core.utils.stage import get_current_stage


class SimulationWorld:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)

    def setup_scene(self):
        """장면 설정 - 받침대 위에 큐브"""
        
        # 조명 추가
        self._add_lighting()
        
        # 바닥 (밝은 회색)
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


# ============================================================
# 메인 파일용 간단한 setup 함수
# ============================================================

def setup_world():
    """
    메인 파일에서 사용할 간단한 월드 설정 함수
    
    Returns:
        world: World 객체
        franka: Franka 로봇
        camera: Camera 객체
    """
    print("[Setup] Creating world...")
    world = World()
    
    # 조명
    _add_lights()
    
    # 바닥
    world.scene.add(
        GroundPlane(
            "/World/Ground",
            z_position=0,
            color=np.array([0.1, 0.2, 0.4])
        )
    )
    
    # 로봇
    franka = world.scene.add(
        Franka(prim_path="/World/Franka", name="franka")
    )
    
    # 테이블
    table = world.scene.add(
        FixedCuboid(
            prim_path="/World/Table",
            name="table",
            position=np.array([0.5, 0.0, 0.25]),
            scale=np.array([0.6, 0.6, 0.5]),
            color=np.array([0.6, 0.4, 0.2])
        )
    )
    
    # 큐브들 (3개)
    cube_positions = [
        [0.5, 0.0, 0.55],
        [0.4, -0.15, 0.55],
        [0.4, 0.15, 0.55],
    ]
    
    cube_colors = [
        [1.0, 0.0, 0.0],  # Red
        [0.0, 0.0, 1.0],  # Blue
        [1.0, 1.0, 0.0],  # Yellow
    ]
    
    for i, (pos, color) in enumerate(zip(cube_positions, cube_colors)):
        world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/Cube_{i}",
                name=f"cube_{i}",
                position=pos,
                scale=[0.05, 0.05, 0.05],
                mass=0.05,
                color=np.array(color)
            )
        )
    
    print(f"[Setup] Added {len(cube_positions)} cubes")
    
    # 카메라
    from sim.camera import setup_camera
    camera = setup_camera(world)
    
    # 월드 리셋
    print("[Setup] Resetting world...")
    world.reset()
    
    # 안정화
    print("[Setup] Stabilizing...")
    for _ in range(60):
        world.step(render=True)
    
    print("[Setup] ✓ World setup complete!\n")
    
    return world, franka, camera


def _add_lights():
    """조명 설정"""
    stage = get_current_stage()
    
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(1000)
    dome.CreateColorAttr((0.8, 0.9, 1.0))
    
    sun = UsdLux.DistantLight.Define(stage, "/World/DirectionalLight")
    sun.CreateIntensityAttr(5000)
    sun.CreateColorAttr((1.0, 0.95, 0.85))
    
    from pxr import UsdGeom
    xform = UsdGeom.Xformable(sun)
    xform.AddRotateXYZOp().Set((-45, 45, 0))
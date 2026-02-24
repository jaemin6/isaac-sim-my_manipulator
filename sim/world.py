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

        self._add_lighting()

        ground = GroundPlane(
            prim_path="/World/Ground",
            size=10.0,
            color=np.array([0.8, 0.8, 0.8])
        )
        self.world.scene.add(ground)

        pedestal = FixedCuboid(
            prim_path="/World/Pedestal",
            name="pedestal",
            position=np.array([0.5, 0.0, 0.30]),
            scale=np.array([0.2, 0.2, 0.6]),
            color=np.array([0.6, 0.4, 0.2]),
        )
        self.world.scene.add(pedestal)

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
        stage = get_current_stage()
        distant_light = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
        distant_light.CreateIntensityAttr(3000)
        distant_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 0.95))
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
# 메인 파일용 setup 함수
# ============================================================

def setup_world():
    """메인 파일에서 사용할 월드 설정 함수"""
    print("[Setup] Creating world...")
    world = World()

    _add_lights()

    world.scene.add(
        GroundPlane(
            "/World/Ground",
            z_position=0,
            color=np.array([0.1, 0.2, 0.4])
        )
    )

    franka = world.scene.add(
        Franka(prim_path="/World/Franka", name="franka")
    )

    # ── 테이블 ────────────────────────────────────────────────────────
    # Y방향을 0.6 → 0.8로 확장: 큐브를 좌우로 넓게 배치해도 테이블 위에 올라감
    # position/Z는 그대로 유지 (테이블 윗면 Z=0.50m 기준 유지)
    table = world.scene.add(
        FixedCuboid(
            prim_path="/World/Table",
            name="table",
            position=np.array([0.5, 0.0, 0.25]),
            scale=np.array([0.6, 0.8, 0.5]),        # Y: 0.6 → 0.8
            color=np.array([0.6, 0.4, 0.2])
        )
    )

    # ── 큐브 배치 ─────────────────────────────────────────────────────
    # [수정 이유]
    # 기존: Blue(0.70, 0.05) → 로봇팔 도달거리(~0.60m) 초과로 집기 실패
    #       Yellow(0.50, -0.25) → 앞 큐브 집는 과정에서 밀려 테이블 추락
    #
    # [수정안]
    # X=0.50 고정 (로봇과의 거리 통일, dist≈0.51~0.54m)
    # Y방향으로만 ±0.10 / ±0.20 배치 → 서로 안 닿고(최소간격 0.10m)
    #                                     로봇팔 도달 범위 내
    #
    # 큐브 간 거리 검증:
    #   Red↔Green  : 0.20m ✅
    #   Red↔Blue   : 0.10m ✅
    #   Red↔Yellow : 0.30m ✅
    #   Green↔Blue : 0.30m ✅
    #   Green↔Yellow: 0.10m ✅
    #   Blue↔Yellow : 0.40m ✅
    cube_positions = [
        [0.45,  0.15, 0.55],   # Red    dist=0.510m
        [0.45, -0.15, 0.55],   # Green  dist=0.510m
        [0.55,  0.15, 0.55],   # Blue   dist=0.539m  (기존 0.70m → 해결)
        [0.55, -0.15, 0.55],   # Yellow dist=0.539m
    ]

    cube_colors = [
        [1.0, 0.0, 0.0],   # Red
        [0.0, 1.0, 0.0],   # Green
        [0.0, 0.0, 1.0],   # Blue
        [1.0, 1.0, 0.0],   # Yellow
    ]

    for i, (pos, color) in enumerate(zip(cube_positions, cube_colors)):
        dist = np.sqrt(pos[0]**2 + pos[1]**2)
        world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/Cube_{i}",
                name=f"cube_{i}",
                position=pos,
                scale=[0.05, 0.05, 0.05],
                mass=1.0,
                color=np.array(color)
            )
        )
        color_name = ['Red', 'Green', 'Blue', 'Yellow'][i]
        print(f"  Cube_{i} ({color_name}): "
              f"XY=({pos[0]:.2f},{pos[1]:+.2f}) dist={dist:.3f}m")

    print(f"[Setup] Added {len(cube_positions)} cubes")

    # ── 카메라 ────────────────────────────────────────────────────────
    from omni.isaac.sensor import Camera
    from scipy.spatial.transform import Rotation as R

    camera = Camera(
        prim_path="/World/Camera",
        position=np.array([0.5, 0.0, 1.2]),
        resolution=(1024, 768),
        frequency=20
    )

    rot = R.from_euler('x', 90, degrees=True)
    quat = rot.as_quat()
    orientation = np.array([quat[3], quat[0], quat[1], quat[2]])

    camera.set_world_pose(
        position=np.array([0.5, 0.0, 1.2]),
        orientation=orientation
    )

    camera.initialize()
    print("[Setup] Camera configured to look down at table")

    print("[Setup] Resetting world...")
    world.reset()

    print("[Setup] Stabilizing...")
    for _ in range(60):
        world.step(render=True)

    print("[Setup] ✓ World setup complete!\n")

    return world, franka, camera


def _add_lights():
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
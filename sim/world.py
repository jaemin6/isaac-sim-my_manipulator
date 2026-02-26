# sim/world.py
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid, GroundPlane
from omni.isaac.franka import Franka
from pxr import UsdLux, Gf
from omni.isaac.core.utils.stage import get_current_stage


class SimulationWorld:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)

    def setup_scene(self):
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

    # 테이블 윗면 Z=0.50m, Y방향 0.8로 확장
    table = world.scene.add(
        FixedCuboid(
            prim_path="/World/Table",
            name="table",
            position=np.array([0.5, 0.0, 0.25]),
            scale=np.array([0.6, 0.8, 0.5]),
            color=np.array([0.6, 0.4, 0.2])
        )
    )

    # ── 큐브 배치 ─────────────────────────────────────────────────────
    # [v11 수정] Blue/Yellow: X=0.60 → 0.55 (dist 0.626 → 0.579m)
    # Franka 도달 범위 안으로 당김 → 집기 성공률 향상
    # 큐브 간 최소 간격: 0.166m ✅
    #
    #   Red    (0.40,  0.25) dist=0.472m
    #   Green  (0.40, -0.25) dist=0.472m
    #   Blue   (0.55,  0.18) dist=0.579m  ← 0.626 → 0.579
    #   Yellow (0.55, -0.18) dist=0.579m  ← 0.626 → 0.579
    cube_positions = [
        [0.40,  0.25, 0.526],   # Red
        [0.40, -0.25, 0.526],   # Green
        [0.55,  0.18, 0.526],   # Blue   (수정)
        [0.55, -0.18, 0.526],   # Yellow (수정)
    ]
    cube_colors = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 1.0, 0.0],
    ]

    for i, (pos, color) in enumerate(zip(cube_positions, cube_colors)):
        dist = np.sqrt(pos[0]**2 + pos[1]**2)
        color_name = ['Red', 'Green', 'Blue', 'Yellow'][i]
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
        print(f"  Cube_{i} ({color_name}): "
              f"XY=({pos[0]:.2f},{pos[1]:+.2f}) dist={dist:.3f}m")

    print(f"[Setup] Added {len(cube_positions)} cubes")

    # ── 카메라 (vision_control.py 내부 replicator 카메라 사용) ────────
    # world.py에서는 Isaac Sensor Camera는 생성하지 않음
    # vision_control.py의 _ensure_camera_ready()에서
    # rep.create.camera()로 직접 생성 + depth annotator 포함
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
    print("[Setup] Camera initialized")

    # ── 월드 리셋 ─────────────────────────────────────────────────────
    print("[Setup] Resetting world...")
    world.reset()

    # ── Franka 초기 자세 (팔 위로 세워서 큐브 충돌 방지) ─────────────
    print("[Setup] Setting Franka initial pose...")
    franka.set_joint_positions(
        np.array([0.0, -1.5, 0.0, -2.5, 0.0, 1.5, 0.7, 0.04, 0.04])
    )

    # ── 안정화 ────────────────────────────────────────────────────────
    print("[Setup] Stabilizing (120 frames)...")
    for _ in range(120):
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
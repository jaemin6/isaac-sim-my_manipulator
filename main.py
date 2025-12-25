from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

# ---------------------------
# Imports
# ---------------------------
from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.core.objects import GroundPlane
from omni.isaac.core.prims import XFormPrim
from pxr import UsdLux
import math

# ---------------------------
# World 생성
# ---------------------------
world = World(stage_units_in_meters=1.0)

# 바닥
ground = GroundPlane("/World/Ground")

# Franka 로봇
franka = Franka(
    prim_path="/World/Franka",
    name="franka"
)
world.scene.add(franka)

# End Effector (panda_hand)
ee = XFormPrim("/World/Franka/panda_hand")

# 조명
stage = world.scene.stage
light = UsdLux.DistantLight.Define(stage, "/World/Light")
light.CreateIntensityAttr(3000)
light.CreateAngleAttr(0.5)

# ---------------------------
# 초기화
# ---------------------------
world.reset()
print("Simulation start")

# ---------------------------
# Target EE 위치 (고정)
# ---------------------------
target_ee_pos = [0.45, -0.2, 0.6]

# ---------------------------
# Control Loop
# ---------------------------
t = 0.0
frame = 0

while simulation_app.is_running():
    t += 0.02
    frame += 1

    # 관절 애니메이션 (움직임 확인용)
    joints = [
        0.3 * math.sin(t),
        -0.5,
        0.3 * math.sin(t),
        -2.0,
        0.3 * math.cos(t),
        2.0,
        0.8,
        0.04,
        0.04
    ]
    franka.set_joint_positions(joints)

    # EE 월드 포즈
    ee_pos, ee_quat = ee.get_world_pose()

    # EE ↔ Target 거리 계산
    dx = target_ee_pos[0] - ee_pos[0]
    dy = target_ee_pos[1] - ee_pos[1]
    dz = target_ee_pos[2] - ee_pos[2]
    distance = math.sqrt(dx*dx + dy*dy + dz*dz)

    # 0.5초마다 출력 (30프레임 기준)
    if frame % 30 == 0:
        print(f"EE pos: {ee_pos}")
        print(f"Distance to target: {distance:.4f}")
        print("-" * 40)

    world.step(render=True)

simulation_app.close()

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

# =========================
# Imports
# =========================
from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.core.objects import GroundPlane, DynamicCuboid
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.types import ArticulationAction
from pxr import UsdLux
import numpy as np

# =========================
# World 생성
# =========================
world = World(stage_units_in_meters=1.0)

ground = GroundPlane("/World/Ground")

# Franka 로봇
franka = Franka(
    prim_path="/World/Franka",
    name="franka"
)
world.scene.add(franka)

ee = XFormPrim("/World/Franka/panda_hand")

# 집을 물체 (빨간 큐브) - 더 높게, 더 가까이
cube = DynamicCuboid(
    prim_path="/World/Cube",
    name="cube",
    position=np.array([0.4, 0.0, 0.3]),  # 높이 올림
    size=0.05,
    color=np.array([1.0, 0.0, 0.0])
)
world.scene.add(cube)

# 목표 위치 표시용
target_marker = DynamicCuboid(
    prim_path="/World/Target",
    name="target",
    position=np.array([0.3, 0.3, 0.3]),
    size=0.03,
    color=np.array([0.0, 1.0, 0.0])
)
world.scene.add(target_marker)
target_marker.set_collision_enabled(False)

# =========================
# 조명
# =========================
stage = world.scene.stage
light = UsdLux.DistantLight.Define(stage, "/World/Light")
light.CreateIntensityAttr(3000)
light.CreateAngleAttr(0.5)

# =========================
# 초기화
# =========================
world.reset()
franka.initialize()

# 큐브 물리 속성 조정
cube_prim = cube.prim
from pxr import UsdPhysics, PhysxSchema
mass_api = UsdPhysics.MassAPI.Apply(cube_prim)
mass_api.CreateMassAttr(0.1)  # 가벼운 물체

print("=== Pick and Place Simulation Started ===")
print(f"Cube position: {cube.get_world_pose()[0]}")
print(f"Target position: {target_marker.get_world_pose()[0]}")

# =========================
# 보정된 Joint 포즈들
# =========================
articulation_controller = franka.get_articulation_controller()

# 1. 홈 포지션
home_position = np.array([
    0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785,  # arm
    0.04, 0.04  # gripper open
])

# 2. 큐브 바로 위 (하강 준비)
above_cube = np.array([
    0.0, -0.3, 0.0, -1.8, 0.0, 1.5, 0.785,
    0.04, 0.04
])

# 3. 큐브 바로 위에서 아래로 (더 내려감)
at_cube = np.array([
    0.0, -0.2, 0.0, -1.5, 0.0, 1.3, 0.785,
    0.04, 0.04
])

# 4. 그리퍼 닫기 (같은 위치, 그리퍼만)
grasp_cube = np.array([
    0.0, -0.2, 0.0, -1.5, 0.0, 1.3, 0.785,
    0.0, 0.0  # gripper closed
])

# 5. 들어올리기
lift_cube = np.array([
    0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.785,
    0.0, 0.0
])

# 6. 목표로 이동
move_to_target = np.array([
    0.5, -0.4, 0.2, -1.8, 0.0, 1.4, 0.785,
    0.0, 0.0
])

# 7. 목표에서 하강
at_target = np.array([
    0.5, -0.2, 0.2, -1.5, 0.0, 1.3, 0.785,
    0.0, 0.0
])

# 8. 그리퍼 열기
release_cube = np.array([
    0.5, -0.2, 0.2, -1.5, 0.0, 1.3, 0.785,
    0.04, 0.04  # gripper open
])

# 9. 후퇴
retreat = np.array([
    0.5, -0.5, 0.2, -2.0, 0.0, 1.5, 0.785,
    0.04, 0.04
])

# =========================
# Phase 정의
# =========================
phases_list = [
    ("home", home_position, 100, " Moving to HOME"),
    ("above_cube", above_cube, 120, " Moving ABOVE cube"),
    ("at_cube", at_cube, 80, "⬇️  Descending to cube"),
    ("grasp", grasp_cube, 60, " GRASPING cube"),
    ("lift", lift_cube, 80, "⬆️  LIFTING cube"),
    ("move", move_to_target, 120, "➡️  Moving to TARGET"),
    ("at_target", at_target, 80, "⬇️  Descending to target"),
    ("release", release_cube, 60, " RELEASING cube"),
    ("retreat", retreat, 80, "🔙 Retreating"),
    ("done", retreat, 60, " DONE!")
]

# =========================
# Control Loop
# =========================
frame = 0
phase_idx = 0
phase_name, target_pos, duration, message = phases_list[phase_idx]

print(f"\n{message}")

while simulation_app.is_running():
    frame += 1
    
    # 현재 타겟으로 이동
    action = ArticulationAction(joint_positions=target_pos)
    articulation_controller.apply_action(action)
    
    # Phase 전환
    if frame >= duration and phase_idx < len(phases_list) - 1:
        phase_idx += 1
        frame = 0
        phase_name, target_pos, duration, message = phases_list[phase_idx]
        print(f"\n{message}")
    
    # 상태 출력
    if frame % 40 == 0:
        ee_pos, _ = ee.get_world_pose()
        cube_pos, _ = cube.get_world_pose()
        gripper_state = "OPEN" if target_pos[-1] > 0.02 else "CLOSED"
        
        print(f"[{phase_name.upper():12s}] EE:[{ee_pos[0]:.2f}, {ee_pos[1]:.2f}, {ee_pos[2]:.2f}] "
              f"Cube:[{cube_pos[0]:.2f}, {cube_pos[1]:.2f}, {cube_pos[2]:.2f}] "
              f"Gripper:{gripper_state}")
    
    world.step(render=True)

simulation_app.close()
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

# 집을 물체 (빨간 큐브)
cube = DynamicCuboid(
    prim_path="/World/Cube",
    name="cube",
    position=np.array([0.5, 0.0, 0.05]),  # 로봇 앞
    size=0.05,  # 5cm 크기
    color=np.array([1.0, 0.0, 0.0])  # 빨간색
)
world.scene.add(cube)

# 목표 위치 표시용 (초록 큐브 - 고정)
target_marker = DynamicCuboid(
    prim_path="/World/Target",
    name="target",
    position=np.array([0.3, 0.3, 0.05]),  # 목표 위치
    size=0.03,
    color=np.array([0.0, 1.0, 0.0])  # 초록색
)
world.scene.add(target_marker)
# 목표 마커를 고정 (물리 비활성화)
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
print("=== Pick and Place Simulation Started ===")
print(f"Cube position: {cube.get_world_pose()[0]}")
print(f"Target position: {target_marker.get_world_pose()[0]}")

# =========================
# Joint 설정
# =========================
articulation_controller = franka.get_articulation_controller()

# 미리 정의된 포즈들
home_position = np.array([0.0, -1.0, 0.0, -2.5, 0.0, 1.5, 0.785, 0.04, 0.04])

# 큐브 위로 이동 (pick 준비)
pre_pick_position = np.array([
    0.0,      # joint1
    -0.4,     # joint2
    0.0,      # joint3
    -2.0,     # joint4
    0.0,      # joint5
    1.6,      # joint6
    0.785,    # joint7
    0.04,     # gripper open
    0.04
])

# 큐브 집기 (그리퍼만 닫힘)
pick_position = np.array([
    0.0,
    -0.4,
    0.0,
    -2.0,
    0.0,
    1.6,
    0.785,
    0.0,      # gripper close
    0.0
])

# 들어올리기
lift_position = np.array([
    0.0,
    -0.8,     # 위로 올림
    0.0,
    -2.2,
    0.0,
    1.8,
    0.785,
    0.0,      # gripper closed
    0.0
])

# 목표 위치로 이동
place_position = np.array([
    0.5,      # 회전해서 목표로
    -0.6,
    0.3,
    -2.0,
    0.0,
    1.6,
    0.785,
    0.0,      # gripper closed
    0.0
])

# 놓기 (그리퍼 열기)
release_position = np.array([
    0.5,
    -0.6,
    0.3,
    -2.0,
    0.0,
    1.6,
    0.785,
    0.04,     # gripper open
    0.04
])

# =========================
# Control Loop
# =========================
frame = 0
phase = "home"
phases = {
    "home": (home_position, 120, "pre_pick"),
    "pre_pick": (pre_pick_position, 120, "pick"),
    "pick": (pick_position, 60, "lift"),
    "lift": (lift_position, 100, "place"),
    "place": (place_position, 120, "release"),
    "release": (release_position, 60, "done"),
    "done": (release_position, 60, "done")
}

print("\n Phase: Moving to HOME position...")

while simulation_app.is_running():
    frame += 1
    
    if phase in phases:
        target_pos, duration, next_phase = phases[phase]
        
        # 타겟 포지션으로 이동
        action = ArticulationAction(joint_positions=target_pos)
        articulation_controller.apply_action(action)
        
        # Phase 전환
        if frame >= duration:
            if next_phase != phase:
                frame = 0
                phase = next_phase
                
                # Phase별 메시지
                if phase == "pre_pick":
                    print("\n Phase: Moving above CUBE...")
                elif phase == "pick":
                    print("\n Phase: CLOSING gripper...")
                elif phase == "lift":
                    print("\n⬆  Phase: LIFTING cube...")
                elif phase == "place":
                    print("\n  Phase: Moving to TARGET...")
                elif phase == "release":
                    print("\n Phase: RELEASING cube...")
                elif phase == "done":
                    print("\n Phase: DONE!")
    
    # 상태 출력 (매 60 프레임)
    if frame % 60 == 0 and phase != "done":
        ee_pos, _ = ee.get_world_pose()
        cube_pos, _ = cube.get_world_pose()
        gripper_state = "OPEN" if target_pos[-1] > 0.02 else "CLOSED"
        
        print(f"\n[Frame {frame}] Phase: {phase.upper()}")
        print(f"  EE position: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]")
        print(f"  Cube position: [{cube_pos[0]:.3f}, {cube_pos[1]:.3f}, {cube_pos[2]:.3f}]")
        print(f"  Gripper: {gripper_state}")
        print("-" * 50)

    world.step(render=True)

simulation_app.close()
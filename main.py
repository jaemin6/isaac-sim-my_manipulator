from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

# =========================
# Imports
# =========================
from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.core.objects import GroundPlane
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.types import ArticulationAction
from pxr import UsdLux
import numpy as np

# =========================
# World 생성
# =========================
world = World(stage_units_in_meters=1.0)

ground = GroundPlane("/World/Ground")

franka = Franka(
    prim_path="/World/Franka",
    name="franka"
)
world.scene.add(franka)

ee = XFormPrim("/World/Franka/panda_hand")

# =========================
# 조명
# =========================
stage = world.scene.stage
light = UsdLux.DistantLight.Define(stage, "/World/Light")
light.CreateIntensityAttr(3000)
light.CreateAngleAttr(0.5)

# =========================
# 초기화 (⚠️ 필수)
# =========================
world.reset()
franka.initialize()
print("Simulation start")

# =========================
# Joint 정보 확인
# =========================
num_dof = franka.num_dof
print(f"Number of DOF: {num_dof}")
print(f"Joint names: {franka.dof_names}")

# =========================
# 컨트롤러 가져오기
# =========================
articulation_controller = franka.get_articulation_controller()

# =========================
# 간단한 joint position 타겟 (9개: 7 arm + 2 gripper)
# =========================
target_joint_positions = np.array([
    0.0,      # panda_joint1
    -0.785,   # panda_joint2
    0.0,      # panda_joint3
    -2.356,   # panda_joint4
    0.0,      # panda_joint5
    1.571,    # panda_joint6
    0.785,    # panda_joint7
    0.04,     # panda_finger_joint1 (gripper)
    0.04      # panda_finger_joint2 (gripper)
])

target_joint_positions_close = np.array([
    0.0,      # panda_joint1
    -0.785,   # panda_joint2
    0.0,      # panda_joint3
    -2.356,   # panda_joint4
    0.0,      # panda_joint5
    1.571,    # panda_joint6
    0.785,    # panda_joint7
    0.0,      # panda_finger_joint1 (gripper closed)
    0.0       # panda_finger_joint2 (gripper closed)
])

# =========================
# Control Loop
# =========================
frame = 0
phase = "moving"  # moving -> wait -> closing

while simulation_app.is_running():
    frame += 1

    # Phase 1: 타겟 포지션으로 이동 (그리퍼 열림)
    if phase == "moving":
        action = ArticulationAction(
            joint_positions=target_joint_positions
        )
        articulation_controller.apply_action(action)
        
        if frame == 150:
            print("Reached target with gripper open")
            phase = "wait"
            frame = 0

    # Phase 2: 대기
    elif phase == "wait":
        if frame == 60:
            print("Closing gripper...")
            phase = "closing"
            frame = 0

    # Phase 3: 그리퍼 닫기
    elif phase == "closing":
        action = ArticulationAction(
            joint_positions=target_joint_positions_close
        )
        articulation_controller.apply_action(action)
        
        if frame == 60:
            print("Gripper closed, done!")
            phase = "done"

    # 상태 출력
    if frame % 60 == 0:
        ee_pos, ee_rot = ee.get_world_pose()
        current_joints = franka.get_joint_positions()
        print(f"\nFrame: {frame}, Phase: {phase}")
        print(f"EE pos: {ee_pos}")
        print(f"Gripper joints: {current_joints[-2:]}")
        print("-" * 40)

    world.step(render=True)

simulation_app.close()
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.core.objects import GroundPlane
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.types import ArticulationAction
from pxr import UsdLux, UsdGeom, Gf, UsdPhysics
import numpy as np

# =========================
# World
# =========================
world = World(stage_units_in_meters=1.0)
ground = GroundPlane("/World/Ground")

franka = Franka(prim_path="/World/Franka", name="franka")
world.scene.add(franka)

ee = XFormPrim("/World/Franka/panda_hand")
stage = world.scene.stage

# =========================
# 큰 테이블 (바닥에서 50cm 높이)
# =========================
table_geom = UsdGeom.Cube.Define(stage, "/World/Table")
table_geom.CreateSizeAttr(1.0)
table_geom.AddTranslateOp().Set(Gf.Vec3f(0.5, 0.0, 0.25))  # z=0.25 = 중심
table_geom.AddScaleOp().Set(Gf.Vec3f(0.8, 0.8, 0.5))  # 더 크게
table_geom.CreateDisplayColorAttr([(0.7, 0.7, 0.7)])
UsdPhysics.CollisionAPI.Apply(table_geom.GetPrim())

# =========================
# 고정된 큐브 (물리 없음, 시각화만)
# =========================
cube_pos = np.array([0.4, 0.0, 0.525])  # 테이블 위 (0.5 + 0.025)

cube_geom = UsdGeom.Cube.Define(stage, "/World/Cube")
cube_geom.CreateSizeAttr(1.0)
cube_geom.AddTranslateOp().Set(Gf.Vec3f(cube_pos[0], cube_pos[1], cube_pos[2]))
cube_geom.AddScaleOp().Set(Gf.Vec3f(0.05, 0.05, 0.05))
cube_geom.CreateDisplayColorAttr([(1.0, 0.0, 0.0)])
# 물리 없음 - 완전 고정!

# 목표 마커
target_geom = UsdGeom.Cube.Define(stage, "/World/Target")
target_geom.CreateSizeAttr(1.0)
target_geom.AddTranslateOp().Set(Gf.Vec3f(0.3, 0.25, 0.525))
target_geom.AddScaleOp().Set(Gf.Vec3f(0.05, 0.05, 0.01))
target_geom.CreateDisplayColorAttr([(0.0, 1.0, 0.0)])

# 조명
light = UsdLux.DistantLight.Define(stage, "/World/Light")
light.CreateIntensityAttr(3000)
light.CreateAngleAttr(0.5)

# 초기화
world.reset()
franka.initialize()

print("=== Pick and Place - Fixed Cube Edition ===")
print(f"✓ Cube FIXED at: {cube_pos}")
print(f"✓ Table top at z=0.5m")

articulation_controller = franka.get_articulation_controller()

# =========================
# Joint 조합 테스트 (큐브 위치 [0.4, 0.0, 0.525] 기준)
# =========================
test_configs = [
    # 큐브 X=0.4, Y=0.0에 도달하려면 joint1≈0, joint2를 조정
    ("A", [0.0, 0.5, 0.0, -0.8, 0.0, 1.3, 0.785]),
    ("B", [0.0, 0.4, 0.0, -0.9, 0.0, 1.3, 0.785]),
    ("C", [0.0, 0.45, 0.0, -0.85, 0.0, 1.3, 0.785]),
    ("D", [0.0, 0.55, 0.0, -0.75, 0.0, 1.3, 0.785]),
    ("E", [0.0, 0.35, 0.0, -0.95, 0.0, 1.3, 0.785]),
    ("F", [0.0, 0.48, 0.0, -0.82, 0.0, 1.3, 0.785]),
    ("G", [0.0, 0.42, 0.0, -0.88, 0.0, 1.3, 0.785]),
]

print("\n" + "="*70)
print("Testing configurations to reach cube...")
print("="*70)

best_config = None
best_dist = float('inf')

for name, joints in test_configs:
    full_joints = np.array(joints + [0.04, 0.04])
    
    # 이동
    for _ in range(100):
        action = ArticulationAction(joint_positions=full_joints)
        articulation_controller.apply_action(action)
        world.step(render=True)
    
    # 확인
    ee_pos, _ = ee.get_world_pose()
    dist = np.linalg.norm(ee_pos - cube_pos)
    
    print(f"{name}: EE=[{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}] "
          f"Dist={dist:.4f}m")
    
    if dist < best_dist:
        best_dist = dist
        best_config = (name, full_joints)

print("="*70)
print(f" BEST: {best_config[0]} with {best_dist:.4f}m distance")
print("="*70)

# =========================
# 최적 조합으로 시퀀스 실행
# =========================
if best_dist < 0.15:  # 15cm 이내면 시도
    print("\n Distance acceptable! Attempting grasp sequence...")
    
    best_joints = best_config[1]
    
    # 1. 접근
    for _ in range(80):
        action = ArticulationAction(joint_positions=best_joints)
        articulation_controller.apply_action(action)
        world.step(render=True)
    
    ee_pos, _ = ee.get_world_pose()
    print(f" Approach: EE at [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]")
    
    # 2. 그리퍼 닫기
    grasp_joints = best_joints.copy()
    grasp_joints[-2:] = [0.015, 0.015]
    
    for _ in range(60):
        action = ArticulationAction(joint_positions=grasp_joints)
        articulation_controller.apply_action(action)
        world.step(render=True)
    
    print(" Gripper closed")
    
    # 3. 들어올리기
    lift_joints = grasp_joints.copy()
    lift_joints[1] -= 0.4
    
    for _ in range(100):
        action = ArticulationAction(joint_positions=lift_joints)
        articulation_controller.apply_action(action)
        world.step(render=True)
    
    ee_pos, _ = ee.get_world_pose()
    print(f" Lifted: EE at [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]")
    
    # 4. 목표로 이동
    move_joints = np.array([0.4, 0.0, 0.2, -1.5, 0.0, 1.5, 0.785, 0.015, 0.015])
    for _ in range(150):
        action = ArticulationAction(joint_positions=move_joints)
        articulation_controller.apply_action(action)
        world.step(render=True)
    
    print("  Moved to target area")
    
    # 5. 놓기
    release_joints = move_joints.copy()
    release_joints[-2:] = [0.04, 0.04]
    
    for _ in range(60):
        action = ArticulationAction(joint_positions=release_joints)
        articulation_controller.apply_action(action)
        world.step(render=True)
    
    print("✋ Released")
    print("\n Pick and Place sequence complete!")

else:
    print(f"\n Best distance {best_dist:.3f}m is too far. Need better joint angles.")
    print("Hint: Adjust test_configs to get closer to cube")

# 홈으로
home = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.04, 0.04])
for _ in range(100):
    action = ArticulationAction(joint_positions=home)
    articulation_controller.apply_action(action)
    world.step(render=True)

print("\n Done!")
simulation_app.close()
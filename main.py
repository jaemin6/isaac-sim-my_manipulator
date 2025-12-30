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

# Franka base 위치 확인용
franka_base = XFormPrim("/World/Franka")
ee = XFormPrim("/World/Franka/panda_hand")
stage = world.scene.stage

# 테이블 - 로봇 베이스 기준으로 앞에
table_geom = UsdGeom.Cube.Define(stage, "/World/Table")
table_geom.CreateSizeAttr(1.0)
table_geom.AddTranslateOp().Set(Gf.Vec3f(0.5, 0.0, 0.25))
table_geom.AddScaleOp().Set(Gf.Vec3f(0.6, 0.6, 0.5))
table_geom.CreateDisplayColorAttr([(0.7, 0.7, 0.7)])
UsdPhysics.CollisionAPI.Apply(table_geom.GetPrim())

# 큐브 - 로봇 앞 50cm, 테이블 위
cube_pos = np.array([0.5, 0.0, 0.525])

cube_geom = UsdGeom.Cube.Define(stage, "/World/Cube")
cube_geom.CreateSizeAttr(1.0)
cube_geom.AddTranslateOp().Set(Gf.Vec3f(cube_pos[0], cube_pos[1], cube_pos[2]))
cube_geom.AddScaleOp().Set(Gf.Vec3f(0.05, 0.05, 0.05))
cube_geom.CreateDisplayColorAttr([(1.0, 0.0, 0.0)])

# 목표
target_geom = UsdGeom.Cube.Define(stage, "/World/Target")
target_geom.CreateSizeAttr(1.0)
target_geom.AddTranslateOp().Set(Gf.Vec3f(0.4, 0.3, 0.525))
target_geom.AddScaleOp().Set(Gf.Vec3f(0.05, 0.05, 0.01))
target_geom.CreateDisplayColorAttr([(0.0, 1.0, 0.0)])

# 조명
light = UsdLux.DistantLight.Define(stage, "/World/Light")
light.CreateIntensityAttr(3000)
light.CreateAngleAttr(0.5)

# 초기화
world.reset()
franka.initialize()

# 로봇 베이스 위치 확인
base_pos, _ = franka_base.get_world_pose()

print("=== Coordinate System Check ===")
print(f"🤖 Franka base at: [{base_pos[0]:.3f}, {base_pos[1]:.3f}, {base_pos[2]:.3f}]")
print(f"🎯 Cube at: [{cube_pos[0]:.3f}, {cube_pos[1]:.3f}, {cube_pos[2]:.3f}]")

articulation_controller = franka.get_articulation_controller()

# =========================
# 광범위 탐색 - joint2, joint4 조합
# =========================
print("\n" + "="*75)
print("Grid search: joint2 vs joint4")
print("="*75)

best_config = None
best_dist = float('inf')
results = []

# joint2: 0.5~1.0 (팔 구부림), joint4: -0.8~-1.4 (팔꿈치)
for j2 in np.arange(0.5, 1.05, 0.1):
    for j4 in np.arange(-0.8, -1.45, -0.1):
        joints = [0.0, j2, 0.0, j4, 0.0, 1.35, 0.785]
        full_joints = np.array(joints + [0.04, 0.04])
        
        # 이동
        for _ in range(80):
            action = ArticulationAction(joint_positions=full_joints)
            articulation_controller.apply_action(action)
            world.step(render=True)
        
        # 측정
        ee_pos, _ = ee.get_world_pose()
        dist = np.linalg.norm(ee_pos - cube_pos)
        
        results.append((j2, j4, ee_pos, dist))
        
        if dist < best_dist:
            best_dist = dist
            best_config = (j2, j4, full_joints, ee_pos)

# 결과 정리
print("\nTop 10 results:")
results_sorted = sorted(results, key=lambda x: x[3])

for i, (j2, j4, ee_pos, dist) in enumerate(results_sorted[:10], 1):
    dx = ee_pos[0] - cube_pos[0]
    dy = ee_pos[1] - cube_pos[1]
    dz = ee_pos[2] - cube_pos[2]
    
    marker = "🎯" if dist < 0.05 else "✓" if dist < 0.08 else ""
    
    print(f"{i:2d}. j2={j2:.2f} j4={j4:.2f} → "
          f"EE=[{ee_pos[0]:.3f},{ee_pos[1]:.3f},{ee_pos[2]:.3f}] "
          f"Err=[{dx:+.2f},{dy:+.2f},{dz:+.2f}] "
          f"D={dist:.4f} {marker}")

print("="*75)
print(f"🏆 BEST: j2={best_config[0]:.2f}, j4={best_config[1]:.2f} → {best_dist*100:.1f}cm")
print(f"   EE: [{best_config[3][0]:.3f}, {best_config[3][1]:.3f}, {best_config[3][2]:.3f}]")
print(f"   Cube: [{cube_pos[0]:.3f}, {cube_pos[1]:.3f}, {cube_pos[2]:.3f}]")
print("="*75)

# =========================
# 최적 설정으로 Pick & Place
# =========================
if best_dist < 0.08:
    print(f"\n🎉 Distance {best_dist*100:.1f}cm - LET'S GO!")
    
    best_joints = best_config[2]
    
    # 접근
    for _ in range(100):
        action = ArticulationAction(joint_positions=best_joints)
        articulation_controller.apply_action(action)
        world.step(render=True)
    print("📍 Approached")
    
    # 그리퍼 닫기
    grasp_joints = best_joints.copy()
    grasp_joints[-2:] = [0.018, 0.018]
    
    for _ in range(80):
        action = ArticulationAction(joint_positions=grasp_joints)
        articulation_controller.apply_action(action)
        world.step(render=True)
    print("✊ GRASPED")
    
    # 들어올리기
    lift_joints = grasp_joints.copy()
    lift_joints[1] -= 0.4
    
    for _ in range(120):
        action = ArticulationAction(joint_positions=lift_joints)
        articulation_controller.apply_action(action)
        world.step(render=True)
    
    ee_final, _ = ee.get_world_pose()
    print(f"⬆️  LIFTED to z={ee_final[2]:.3f}m")
    
    # 이동
    move_joints = np.array([0.5, 0.3, 0.2, -1.3, 0.0, 1.4, 0.785, 0.018, 0.018])
    for _ in range(150):
        action = ArticulationAction(joint_positions=move_joints)
        articulation_controller.apply_action(action)
        world.step(render=True)
    print("➡️  MOVED")
    
    # 하강
    place_joints = move_joints.copy()
    place_joints[1] += 0.35
    for _ in range(100):
        action = ArticulationAction(joint_positions=place_joints)
        articulation_controller.apply_action(action)
        world.step(render=True)
    print("⬇️  PLACED")
    
    # 놓기
    release_joints = place_joints.copy()
    release_joints[-2:] = [0.04, 0.04]
    for _ in range(80):
        action = ArticulationAction(joint_positions=release_joints)
        articulation_controller.apply_action(action)
        world.step(render=True)
    print("✋ RELEASED")
    
    print("\n🎊🎊🎊 PICK AND PLACE COMPLETE! 🎊🎊🎊")

else:
    print(f"\n⚠️  Best: {best_dist*100:.1f}cm - need IK solver for better precision")

# 홈
home = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.04, 0.04])
for _ in range(100):
    action = ArticulationAction(joint_positions=home)
    articulation_controller.apply_action(action)
    world.step(render=True)

print("\n✅ Done!")
simulation_app.close()
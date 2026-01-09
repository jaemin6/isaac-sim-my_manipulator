# main.py
"""
가장 단순한 방식으로 Franka 로봇 제어
Isaac Sim 공식 예제 패턴 사용
"""

import sys
import os
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from isaacsim.simulation_app import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, VisualSphere
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.types import ArticulationAction
from pxr import UsdPhysics, Gf
import omni.kit.app


# -------------------------------------------------
# Utils
# -------------------------------------------------
def get_cube_position():
    from pxr import UsdGeom
    stage = get_current_stage()
    cube = stage.GetPrimAtPath("/World/Cube")
    if not cube:
        return None

    xform = UsdGeom.Xformable(cube)
    pos = xform.ComputeLocalToWorldTransform(0).ExtractTranslation()
    return np.array([pos[0], pos[1], pos[2]])


def disable_physics(prim_path):
    stage = get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        return

    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        prim.RemoveAPI(UsdPhysics.CollisionAPI)


def attach_cube_to_ee():
    stage = get_current_stage()
    cube = stage.GetPrimAtPath("/World/Cube")
    ee = stage.GetPrimAtPath("/World/Franka/panda_hand")
    
    if not cube or not ee:
        print("[ERROR] Cannot find cube or end effector")
        return False

    joint = UsdPhysics.FixedJoint.Define(
        stage, "/World/FixedJoint_Grasp"
    )
    joint.CreateBody0Rel().SetTargets([cube.GetPath()])
    joint.CreateBody1Rel().SetTargets([ee.GetPath()])

    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

    print("[Joint] Cube attached to EE")
    return True


# -------------------------------------------------
# Main
# -------------------------------------------------
def setup_scene_lighting():
    """조명과 바닥 추가"""
    from pxr import UsdLux, UsdGeom, Gf
    
    stage = get_current_stage()
    
    # 1. 간단한 바닥 추가 (Isaac Sim 내장 GroundPlane 사용)
    from omni.isaac.core.utils.stage import add_reference_to_stage
    add_reference_to_stage(
        usd_path="/Isaac/Environments/Grid/default_environment.usd",
        prim_path="/World/Environment"
    )
    
    # 2. Dome Light 추가 (환경 조명)
    dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome_light.CreateIntensityAttr(2000.0)
    
    # 3. Distant Light 추가 (태양광 같은 효과)
    distant_light = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    distant_light.CreateIntensityAttr(5000.0)
    xform = UsdGeom.Xformable(distant_light)
    xform.ClearXformOpOrder()
    xform.AddRotateXYZOp().Set(Gf.Vec3f(-45, 45, 0))
    
    # 4. 스포트라이트 추가 (로봇 위에서 비추기)
    spot_light = UsdLux.SphereLight.Define(stage, "/World/SpotLight")
    spot_light.CreateIntensityAttr(50000.0)
    spot_light.CreateRadiusAttr(0.1)
    xform = UsdGeom.Xformable(spot_light)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 2.0))
    
    print("[Scene] Lighting and ground added")


def main():
    # 1. World 생성
    print("[Main] Creating world...")
    world = World(stage_units_in_meters=1.0)
    
    # 2. 조명과 바닥 추가
    setup_scene_lighting()
    
    # 3. 로봇 추가
    print("[Main] Adding Franka robot...")
    from omni.isaac.franka import Franka
    
    franka = world.scene.add(
        Franka(
            prim_path="/World/Franka",
            name="franka_robot"
        )
    )
    
    # 3. 테이블 추가 (큐브를 올려놓을 곳)
    print("[Main] Adding table...")
    from omni.isaac.core.objects import FixedCuboid
    
    table = world.scene.add(
        FixedCuboid(
            prim_path="/World/Table",
            name="table",
            position=np.array([0.5, 0.0, 0.2]),  # 테이블 위치
            scale=np.array([0.6, 0.6, 0.4]),  # scale 사용
            color=np.array([0.7, 0.5, 0.3])  # 나무색
        )
    )
    
    # 4. 큐브 추가 (테이블 위에)
    cube_pos = get_cube_position()
    if cube_pos is None:
        print("[Main] Adding cube on table...")
        cube = world.scene.add(
            DynamicCuboid(
                prim_path="/World/Cube",
                name="target_cube",
                position=np.array([0.5, 0.0, 0.45]),  # 테이블 위
                scale=np.array([0.05, 0.05, 0.05]),  # scale 사용
                color=np.array([1.0, 0.2, 0.2]),  # 빨간색
                mass=0.05  # 가벼운 큐브
            )
        )
    
    # 5. World reset
    print("[Main] Resetting world...")
    world.reset()
    
    # 6. 초기 안정화
    print("[Main] Stabilizing...")
    for _ in range(100):
        world.step(render=True)
    
    print("[Main] Setup complete!")

    # -----------------------------
    # Debug markers
    # -----------------------------
    ee_marker = VisualSphere(
        prim_path="/World/Debug/EE_Marker",
        name="ee_marker",
        radius=0.015,
        color=np.array([1, 0, 0])
    )
    world.scene.add(ee_marker)
    disable_physics("/World/Debug/EE_Marker")

    target_marker = VisualSphere(
        prim_path="/World/Debug/Target_Marker",
        name="target_marker",
        radius=0.025,
        color=np.array([0, 1, 0])
    )
    world.scene.add(target_marker)
    disable_physics("/World/Debug/Target_Marker")

    # 큐브 위치 확인
    cube_pos = get_cube_position()
    if cube_pos is None:
        print("[ERROR] Cube not found")
        return
    
    target_marker.set_world_pose(position=cube_pos)
    
    ee_pos, _ = franka.end_effector.get_world_pose()
    ee_marker.set_world_pose(position=ee_pos)

    print(f"[Init] EE: {ee_pos}")
    print(f"[Init] Cube: {cube_pos}")

    app = omni.kit.app.get_app()

    # -----------------------------
    # 수동 관절 제어 (큐브 위치 기반)
    # -----------------------------
    
    print(f"\n[Planning] Cube is at: {cube_pos}")
    
    # 큐브 X 위치에 따라 관절1 (base rotation) 조정
    # 로봇 기준으로 X축 앞쪽이면 양수, 뒤쪽이면 음수
    cube_x = cube_pos[0]
    cube_y = cube_pos[1]
    
    # 큐브 방향 각도 계산
    import math
    angle_to_cube = math.atan2(cube_y, cube_x)
    
    print(f"[Planning] Angle to cube: {math.degrees(angle_to_cube):.1f}°")
    
    # -------------------------------------------------
    # 1. Pre-grasp 위치로 이동
    # -------------------------------------------------
    print("\n[Step 1] Moving to pre-grasp position")
    
    # 큐브를 향하도록 관절 각도 설정
    pre_grasp_joints = np.array([
        angle_to_cube,  # panda_joint1: 큐브 방향으로 회전
        -0.2,           # panda_joint2: 어깨 (더 적게 구부림 = 앞으로 뻗기)
        0.0,            # panda_joint3
        -1.8,           # panda_joint4: 팔꿈치 (더 펴기 = 앞으로 뻗기)
        0.0,            # panda_joint5
        2.0,            # panda_joint6: 손목 (더 올리기)
        0.785,          # panda_joint7
        0.04,           # gripper open
        0.04
    ])
    
    initial_joints = franka.get_joint_positions()
    steps = 250
    
    for i in range(steps):
        alpha = i / steps
        target_joints = initial_joints + alpha * (pre_grasp_joints - initial_joints)
        franka.set_joint_positions(target_joints)
        
        world.step(render=True)
        app.update()
        
        ee_pos, _ = franka.end_effector.get_world_pose()
        ee_marker.set_world_pose(position=ee_pos)
        
        if i % 40 == 0:
            dist_to_cube = np.linalg.norm(ee_pos[:2] - cube_pos[:2])
            print(f"  step {i:3d} | EE: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}] | XY dist: {dist_to_cube:.3f}m")
    
    ee_pos, _ = franka.end_effector.get_world_pose()
    print(f"  ✓ Pre-grasp reached")
    print(f"[Current] EE: {ee_pos}")
    print(f"[Target] Cube: {cube_pos}")
    
    # -------------------------------------------------
    # 2. Grasp 위치로 하강
    # -------------------------------------------------
    print("\n[Step 2] Descending to grasp position")
    
    # 더 아래로 내리기 위해 joint4를 더 구부림
    grasp_joints = pre_grasp_joints.copy()
    grasp_joints[1] -= 0.1   # shoulder 약간만 조정
    grasp_joints[3] -= 0.2   # elbow 약간만 구부림 (너무 많이 구부리면 뒤로 감)
    
    current_joints = franka.get_joint_positions()
    steps = 200
    
    for i in range(steps):
        alpha = i / steps
        target_joints = current_joints + alpha * (grasp_joints - current_joints)
        franka.set_joint_positions(target_joints)
        
        world.step(render=True)
        app.update()
        
        ee_pos, _ = franka.end_effector.get_world_pose()
        ee_marker.set_world_pose(position=ee_pos)
        
        if i % 40 == 0:
            xy_dist = np.linalg.norm(ee_pos[:2] - cube_pos[:2])
            z_diff = ee_pos[2] - cube_pos[2]
            print(f"  step {i:3d} | XY dist: {xy_dist:.3f}m | Z diff: {z_diff:.3f}m")
    
    ee_pos, _ = franka.end_effector.get_world_pose()
    print(f"  ✓ Grasp position reached")
    print(f"  Final EE: {ee_pos}")

    # -------------------------------------------------
    # 3. Close gripper
    # -------------------------------------------------
    print("\n[Step 3] Closing gripper")
    
    for i in range(100):
        franka.gripper.close()
        world.step(render=True)
        app.update()
        
        if i % 20 == 0:
            gripper_pos = franka.gripper.get_joint_positions()
            print(f"  Gripper positions: {gripper_pos}")

    # 그립 확인 (XY 평면에서의 거리 중요)
    ee_pos, _ = franka.end_effector.get_world_pose()
    xy_dist = np.linalg.norm(ee_pos[:2] - cube_pos[:2])
    z_dist = abs(ee_pos[2] - cube_pos[2])
    
    print(f"[Grasp Check]")
    print(f"  XY distance: {xy_dist:.4f} m")
    print(f"  Z distance: {z_dist:.4f} m")

    if xy_dist < 0.08 and z_dist < 0.15:  # XY 8cm, Z 15cm 이내
        if attach_cube_to_ee():
            print("[Grasp] ✓ SUCCESS - Cube attached")
        else:
            print("[Grasp] ✗ FAILED - Could not attach")
    else:
        print(f"[Grasp] ✗ FAILED - Too far from cube")
        print(f"[Hint] 관절 각도 조정 필요:")
        print(f"  - XY가 멀면: joint1 (base rotation) 조정")
        print(f"  - Z가 높으면: joint2, joint4 (shoulder, elbow) 더 구부리기")

    # 추가 대기
    for _ in range(60):
        world.step(render=True)

    # -------------------------------------------------
    # 4. Lift
    # -------------------------------------------------
    print("\n[Step 4] Lifting cube")
    
    lift_joints = grasp_joints.copy()
    lift_joints[1] += 0.4   # shoulder 올리기
    lift_joints[3] += 0.6   # elbow 펴기
    
    current_joints = franka.get_joint_positions()
    steps = 250
    
    for i in range(steps):
        alpha = i / steps
        target_joints = current_joints + alpha * (lift_joints - current_joints)
        franka.set_joint_positions(target_joints)
        
        world.step(render=True)
        app.update()
        
        if i % 50 == 0:
            ee_pos, _ = franka.end_effector.get_world_pose()
            print(f"  Lift step {i:3d} | EE height: {ee_pos[2]:.3f} m")

    print("\n✓ GRASP SEQUENCE COMPLETED")
    print("\n[Tip] 큐브를 잡지 못했다면:")
    print("  1. XY 거리가 멀면 → joint1 값 조정")
    print("  2. Z가 안 맞으면 → joint2, joint4 값 조정")
    print("  3. 코드에서 pre_grasp_joints 배열을 수정하세요")

    # 결과 유지
    while simulation_app.is_running():
        world.step(render=True)
        app.update()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[Main] Interrupted by user")
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
    finally:
        simulation_app.close()
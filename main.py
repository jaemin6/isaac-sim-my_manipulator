# main.py
"""
카메라 없이 큐브의 실제 위치로 직접 grasp 시도
(camera calibration 우회)
"""

import sys
import os
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from isaacsim.simulation_app import SimulationApp
simulation_app = SimulationApp({"headless": False})

from sim.world import SimulationWorld
from sim.robot import FrankaRobot
from omni.isaac.core.objects import VisualSphere
from omni.isaac.core.utils.stage import get_current_stage
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

    joint = UsdPhysics.FixedJoint.Define(
        stage, "/World/FixedJoint_Grasp"
    )
    joint.CreateBody0Rel().SetTargets([cube.GetPath()])
    joint.CreateBody1Rel().SetTargets([ee.GetPath()])

    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

    print("[Joint] Cube attached to EE")


# -------------------------------------------------
# Main
# -------------------------------------------------
def main():
    sim_world = SimulationWorld()
    world = sim_world.get_world()
    sim_world.reset()

    for _ in range(30):
        sim_world.step(render=True)

    robot = FrankaRobot(world)
    robot.initialize()

    for _ in range(30):
        sim_world.step(render=True)

    # -----------------------------
    # Debug markers
    # -----------------------------
    ee_marker = VisualSphere(
        "/World/Debug/EE_Marker", "ee_marker",
        radius=0.015, color=np.array([1, 0, 0])
    )
    world.scene.add(ee_marker)
    disable_physics("/World/Debug/EE_Marker")

    target_marker = VisualSphere(
        "/World/Debug/Target_Marker", "target_marker",
        radius=0.025, color=np.array([0, 1, 0])
    )
    world.scene.add(target_marker)
    disable_physics("/World/Debug/Target_Marker")

    ee_pos, _ = robot.get_ee_pose()
    ee_marker.set_world_pose(position=ee_pos)

    cube_pos = get_cube_position()
    if cube_pos is None:
        print("[ERROR] Cube not found")
        return

    target_marker.set_world_pose(position=cube_pos)

    print("[Init] EE:", ee_pos)
    print("[Cube]", cube_pos)

    app = omni.kit.app.get_app()

    # -----------------------------
    # FIXED orientation (중요!!)
    # -----------------------------
    GRIPPER_DOWN = np.array([0.707, 0.707, 0.0, 0.0])

    # -------------------------------------------------
    # 1. Pre-grasp
    # -------------------------------------------------
    pre_grasp = cube_pos.copy()
    pre_grasp[2] += 0.25

    print("\n[Step 1] Pre-grasp")
    for step in range(300):
        robot.move_ee_to_pose(pre_grasp, GRIPPER_DOWN)
        sim_world.step(render=True)
        app.update()

        ee_pos, _ = robot.get_ee_pose()
        ee_marker.set_world_pose(position=ee_pos)

        dist = np.linalg.norm(ee_pos - pre_grasp)
        if step % 20 == 0:
            print(f"  step {step} | dist: {dist:.3f} m")

        if dist < 0.03:
            print(f"  ✓ Pre-grasp reached at step {step}")
            break

    # -------------------------------------------------
    # 2. Grasp descend
    # -------------------------------------------------
    grasp_pos = cube_pos.copy()
    grasp_pos[2] -= 0.015  # 큐브 윗면보다 살짝 아래

    print("\n[Step 2] Grasp descend")
    for step in range(300):
        robot.move_ee_to_pose(grasp_pos, GRIPPER_DOWN)
        sim_world.step(render=True)
        app.update()

        ee_pos, _ = robot.get_ee_pose()
        ee_marker.set_world_pose(position=ee_pos)

        dist = np.linalg.norm(ee_pos - grasp_pos)
        if step % 20 == 0:
            print(f"  step {step} | dist: {dist:.3f} m")

        if dist < 0.02:
            print(f"  ✓ Grasp pose reached at step {step}")
            break

    # -------------------------------------------------
    # 3. Close gripper + grasp check
    # -------------------------------------------------
    print("\n[Step 3] Close gripper")
    robot.close_gripper(width=0.015)

    ee_pos, _ = robot.get_ee_pose()

    FINGER_OFFSET_Z = 0.07
    finger_pos = ee_pos.copy()
    finger_pos[2] -= FINGER_OFFSET_Z

    dist = np.linalg.norm(finger_pos - cube_pos)
    print(f"[Grasp Check] Finger-Cube dist: {dist:.4f} m")

    if dist < 0.03:
        attach_cube_to_ee()
        print("[Grasp] SUCCESS")
    else:
        print("[Grasp] FAILED")

    for _ in range(60):
        sim_world.step(render=True)

    # -------------------------------------------------
    # 4. Lift
    # -------------------------------------------------
    lift_pos = grasp_pos.copy()
    lift_pos[2] += 0.35

    print("\n[Step 4] Lift")
    for step in range(240):
        robot.move_ee_to_pose(lift_pos, GRIPPER_DOWN)
        sim_world.step(render=True)
        app.update()

    print("\n GRASP SEQUENCE COMPLETED")

    while simulation_app.is_running():
        sim_world.step(render=True)
        app.update()


if __name__ == "__main__":
    main()

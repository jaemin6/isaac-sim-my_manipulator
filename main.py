# main.py
"""
가장 단순한 방식으로 Franka 로봇 제어
Isaac Sim 공식 예제 패턴 사용
"""

import sys
import os
import math
import numpy as np

# -------------------------------------------------
# Path setup
# -------------------------------------------------
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

# -------------------------------------------------
# Isaac Sim App
# -------------------------------------------------
from isaacsim.simulation_app import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid, VisualSphere
from omni.isaac.core.utils.stage import get_current_stage
from pxr import UsdPhysics, Gf
import omni.kit.app

# -------------------------------------------------
# Joint tuning parameters (기준값)
# -------------------------------------------------

PRE_GRASP_JOINTS = np.array([
    0.0,    # joint1 (base) → 런타임에 cube angle로 덮어씀
    -0.2,   # joint2 (shoulder)
    0.0,    # joint3
    -1.8,   # joint4 (elbow)
    0.0,    # joint5
    2.0,    # joint6 (wrist)
    0.785,  # joint7
    0.04,   # gripper left
    0.04    # gripper right
])

GRASP_JOINTS_DELTA = {
    "shoulder": -0.1,   # joint2
    "elbow": -0.2      # joint4
}

LIFT_JOINTS_DELTA = {
    "shoulder": +0.4,  # joint2
    "elbow": +0.6      # joint4
}

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


def setup_scene_lighting():
    from pxr import UsdLux, UsdGeom, Gf
    from omni.isaac.core.utils.stage import add_reference_to_stage

    stage = get_current_stage()

    add_reference_to_stage(
        usd_path="/Isaac/Environments/Grid/default_environment.usd",
        prim_path="/World/Environment"
    )

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(2000.0)

    sun = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    sun.CreateIntensityAttr(5000.0)
    xform = UsdGeom.Xformable(sun)
    xform.AddRotateXYZOp().Set(Gf.Vec3f(-45, 45, 0))


# -------------------------------------------------
# Main
# -------------------------------------------------
def main():
    print("[Main] Creating world...")
    world = World(stage_units_in_meters=1.0)

    setup_scene_lighting()

    print("[Main] Adding Franka robot...")
    from omni.isaac.franka import Franka
    franka = world.scene.add(
        Franka(
            prim_path="/World/Franka",
            name="franka_robot"
        )
    )

    print("[Main] Adding table...")
    world.scene.add(
        FixedCuboid(
            prim_path="/World/Table",
            name="table",
            position=np.array([0.5, 0.0, 0.2]),
            scale=np.array([0.6, 0.6, 0.4]),
            color=np.array([0.7, 0.5, 0.3])
        )
    )

    if get_cube_position() is None:
        print("[Main] Adding cube...")
        world.scene.add(
            DynamicCuboid(
                prim_path="/World/Cube",
                name="target_cube",
                position=np.array([0.5, 0.0, 0.45]),
                scale=np.array([0.05, 0.05, 0.05]),
                color=np.array([1.0, 0.2, 0.2]),
                mass=0.05
            )
        )

    print("[Main] Resetting world...")
    world.reset()

    print("[Main] Stabilizing...")
    for _ in range(100):
        world.step(render=True)

    print("[Main] Setup complete!")

    # Debug markers
    ee_marker = VisualSphere(
        prim_path="/World/Debug/EE",
        name="ee_marker",
        radius=0.015,
        color=np.array([1, 0, 0])
    )
    world.scene.add(ee_marker)
    disable_physics("/World/Debug/EE")

    target_marker = VisualSphere(
        prim_path="/World/Debug/Target",
        name="target_marker",
        radius=0.025,
        color=np.array([0, 1, 0])
    )
    world.scene.add(target_marker)
    disable_physics("/World/Debug/Target")

    cube_pos = get_cube_position()
    target_marker.set_world_pose(position=cube_pos)

    ee_pos, _ = franka.end_effector.get_world_pose()
    ee_marker.set_world_pose(position=ee_pos)

    print(f"[Init] EE: {ee_pos}")
    print(f"[Init] Cube: {cube_pos}")

    app = omni.kit.app.get_app()

    # -------------------------------------------------
    # Planning
    # -------------------------------------------------
    angle_to_cube = math.atan2(cube_pos[1], cube_pos[0])
    print(f"[Planning] Angle to cube: {math.degrees(angle_to_cube):.1f}°")

    # -------------------------------------------------
    # Step 1: Pre-grasp
    # -------------------------------------------------
    print("\n[Step 1] Moving to pre-grasp position")

    pre_grasp_joints = PRE_GRASP_JOINTS.copy()
    pre_grasp_joints[0] = angle_to_cube

    start_joints = franka.get_joint_positions()
    for i in range(250):
        alpha = i / 250
        joints = start_joints + alpha * (pre_grasp_joints - start_joints)
        franka.set_joint_positions(joints)
        world.step(render=True)
        app.update()

    # -------------------------------------------------
    # Step 2: Descend
    # -------------------------------------------------
    print("\n[Step 2] Descending")

    grasp_joints = pre_grasp_joints.copy()
    grasp_joints[1] += GRASP_JOINTS_DELTA["shoulder"]
    grasp_joints[3] += GRASP_JOINTS_DELTA["elbow"]

    start_joints = franka.get_joint_positions()
    for i in range(200):
        alpha = i / 200
        joints = start_joints + alpha * (grasp_joints - start_joints)
        franka.set_joint_positions(joints)
        world.step(render=True)
        app.update()

    # -------------------------------------------------
    # Step 3: Close gripper & attach
    # -------------------------------------------------
    print("\n[Step 3] Closing gripper")
    for _ in range(100):
        franka.gripper.close()
        world.step(render=True)

    attach_cube_to_ee()

    # -------------------------------------------------
    # Step 4: Lift
    # -------------------------------------------------
    print("\n[Step 4] Lifting")

    lift_joints = grasp_joints.copy()
    lift_joints[1] += LIFT_JOINTS_DELTA["shoulder"]
    lift_joints[3] += LIFT_JOINTS_DELTA["elbow"]

    start_joints = franka.get_joint_positions()
    for i in range(250):
        alpha = i / 250
        joints = start_joints + alpha * (lift_joints - start_joints)
        franka.set_joint_positions(joints)
        world.step(render=True)
        app.update()

    print("\n✓ GRASP SEQUENCE COMPLETED")

    while simulation_app.is_running():
        world.step(render=True)
        app.update()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
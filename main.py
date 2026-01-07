# main.py
import sys
import os
import cv2
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from isaacsim.simulation_app import SimulationApp

# 반드시 가장 먼저
simulation_app = SimulationApp({"headless": False})

from sim.world import SimulationWorld
from sim.robot import FrankaRobot
from sim.camera import SimulationCamera
from vision.detector import SimpleObjectDetector
import omni.kit.app


def main():
    # =====================
    # 상태 플래그
    # =====================
    has_grasped = False

    # =====================
    # World
    # =====================
    sim_world = SimulationWorld()
    world = sim_world.get_world()
    sim_world.reset()

    # 초기 안정화
    for _ in range(10):
        sim_world.step(render=True)

    # =====================
    # Robot
    # =====================
    robot = FrankaRobot(world)
    robot.initialize()

    # 로봇 상태 안정화
    for _ in range(5):
        sim_world.step(render=True)

    # EE Marker 추가
    from omni.isaac.core.objects import VisualSphere

    ee_marker = VisualSphere(
        prim_path="/World/Debug/EE_Marker",
        name="ee_marker",
        radius=0.015,
        color=np.array([1.0, 0.0, 0.0])
    )
    world.scene.add(ee_marker)

    ee_pos, ee_ori = robot.get_ee_pose()
    # EE marker 초기 위치 동기화
    ee_marker.set_world_pose(position=ee_pos)

    print("[Init EE Pose]")
    print("  Position:", ee_pos)
    print("  Orientation:", ee_ori)

    print("[Init] Loading robot...")
    for _ in range(60):
        sim_world.step(render=True)

    # =====================
    # Camera
    # =====================
    print("[Init] Initializing camera...")
    camera = SimulationCamera(
        prim_path="/World/FrankaCamera",
        position=(0.0, 0.0, 2.5),
        look_at=(0.67, 0.0, 0.61),
    )

    app = omni.kit.app.get_app()

    print("[Init] Camera warm-up...")
    for _ in range(60):
        sim_world.step(render=True)
        app.update()

    camera.get_camera_info()

    # =====================
    # Detector
    # =====================
    detector = SimpleObjectDetector(
        hsv_lower=(0, 0, 100),
        hsv_upper=(180, 95, 255),
        min_area=50,
    )

    print("\n[Main] Start main loop")
    frame_count = 0

    # =====================
    # Main Loop
    # =====================
    while simulation_app.is_running():

        sim_world.step(render=True)
        app.update()

        # EE Visualization
        ee_pos, _ = robot.get_ee_pose()
        ee_marker.set_world_pose(position=ee_pos)

        rgb, depth = camera.get_rgb_depth()
        if rgb is None or depth is None:
            continue

        # RGBA → RGB
        if rgb.shape[2] == 4:
            vis = rgb[:, :, :3].copy()
        else:
            vis = rgb.copy()

        # =====================
        # Object Detection
        # =====================
        result = detector.detect(vis)

        if (
            isinstance(result, tuple)
            and len(result) == 3
            and not has_grasped
        ):
            _, _, mask = result
            ys, xs = np.where(mask > 0)

            if len(xs) == 0:
                continue

            cx = int(xs.mean())
            cy = int(ys.mean())

            cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)

            z = depth[cy, cx]
            if z <= 0.1:
                continue

            # =====================
            # Pixel → World
            # =====================
            world_point = camera.pixel_depth_to_world(cx, cy, z)

            print("\n" + "=" * 50)
            print("[Vision] Object detected")
            print(f"  Pixel : ({cx}, {cy})")
            print(f"  Depth : {z:.3f} m")
            print(
                f"  World : "
                f"x={world_point[0]:.3f}, "
                f"y={world_point[1]:.3f}, "
                f"z={world_point[2]:.3f}"
            )
            print("=" * 50)

            # =====================
            # Grasp Sequence (IK)
            # =====================

            # 1. Pre-grasp
            pre_grasp = world_point.copy()
            pre_grasp[2] += 0.15

            print("[Robot] Step 1: Pre-grasp")
            robot.move_ee_to_position(pre_grasp)
            for _ in range(30):
                sim_world.step(render=True)
                app.update()

            # 2. Grasp 접근
            grasp_pos = world_point.copy()
            grasp_pos[2] += 0.06

            print("[Robot] Step 2: Grasp approach")
            robot.move_ee_to_position(grasp_pos)
            for _ in range(30):
                sim_world.step(render=True)
                app.update()

            # 3. Gripper close
            print("[Robot] Step 3: Close gripper")
            robot.close_gripper()
            for _ in range(20):
                sim_world.step(render=True)
                app.update()

            # 4. Lift
            lift_pos = grasp_pos.copy()
            lift_pos[2] += 0.20

            print("[Robot] Step 4: Lift")
            robot.move_ee_to_position(lift_pos)
            for _ in range(40):
                sim_world.step(render=True)
                app.update()

            has_grasped = True
            print("\n[SUCCESS] Grasp sequence completed!\n")

        # =====================
        # Debug Save
        # =====================
        if frame_count % 60 == 0:
            cv2.imwrite(
                f"/tmp/debug_rgb_{frame_count}.png",
                cv2.cvtColor(vis, cv2.COLOR_RGB2BGR),
            )

        frame_count += 1


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        simulation_app.close()

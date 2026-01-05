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
    # World
    # =====================
    sim_world = SimulationWorld()
    world = sim_world.get_world()
    sim_world.reset()

    sim_world.step(render=True)

    # =====================
    # Robot (아직 제어 안 함)
    # =====================
    robot = FrankaRobot(world)
    robot.initialize()

    print("Loading robot...")
    for _ in range(50):
        sim_world.step(render=True)

    # =====================
    # Camera
    # =====================
    print("Initializing camera...")
    camera = SimulationCamera(
        prim_path="/World/FrankaCamera",
        position=(0.0, 0.0, 2.5),
        look_at=(0.0, 0.0, 0.5),
    )

    print("Camera warming up...")
    app = omni.kit.app.get_app()
    for i in range(60):
        sim_world.step(render=True)
        app.update()
        if i % 10 == 0:
            print(f"  Warmup {i}/60")

    detector = SimpleObjectDetector()
    camera.get_camera_info()

    print("\nStart main loop")
    frame_count = 0

    # =====================
    # Main loop
    # =====================
    while simulation_app.is_running():

        sim_world.step(render=True)
        app.update()

        rgb, depth = camera.get_rgb_depth()

        if rgb is None or not isinstance(rgb, np.ndarray):
            frame_count += 1
            continue

        # RGBA → RGB
        if rgb.shape[2] == 4:
            vis = rgb[:, :, :3].copy()
        else:
            vis = rgb.copy()

        # =====================
        # Detector 테스트 (핵심)
        # =====================
        result = detector.detect(vis)

        if isinstance(result, tuple) and len(result) == 3:
            _, _, mask = result

            print(f"[Detector] mask shape={mask.shape}, dtype={mask.dtype}")

            # 마스크 시각화
            mask_vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            cv2.imwrite(f"/tmp/debug_mask_{frame_count}.png", mask_vis)

            # 마스크 중심 계산
            ys, xs = np.where(mask > 0)

            if len(xs) > 0:
                cx = int(xs.mean())
                cy = int(ys.mean())

                print(f"[Detector] mask center pixel: ({cx}, {cy})")

                # 시각화
                cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)

                # depth 확인
                if depth is not None:
                    z = depth[cy, cx]
                    print(f"{Detector} depth at center: {z:.3f} m")

            else:
                print("[Detector] mask is empty")

        # if result is not None:
        #     print(f"[Detector output] {result} | type={type(result)}")

        #     # bbox일 가능성 (x1, y1, x2, y2)
        #     if isinstance(result, (list, tuple)) and len(result) == 4:
        #         x1, y1, x2, y2 = map(int, result)

        #         cv2.rectangle(
        #             vis,
        #             (x1, y1),
        #             (x2, y2),
        #             (0, 255, 0),
        #             2
        #         )

        #         cx = int((x1 + x2) / 2)
        #         cy = int((y1 + y2) / 2)

        #         cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)

        #         print(f"  bbox center pixel: ({cx}, {cy})")

        #         if depth is not None:
        #             z = depth[cy, cx]
        #             print(f"  depth at center: {z:.3f} m")

        # =====================
        # 디버그 저장
        # =====================
        if frame_count % 60 == 0:
            rgb_path = f"/tmp/debug_rgb_{frame_count}.png"
            cv2.imwrite(rgb_path, cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
            print(f"[Saved] {rgb_path}")

            if depth is not None:
                depth_vis = camera.visualize_depth(depth, 0.5, 3.0)
                if depth_vis is not None:
                    depth_path = f"/tmp/debug_depth_{frame_count}.png"
                    cv2.imwrite(depth_path, depth_vis)
                    print(f"[Saved] {depth_path}")

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

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
    has_grasped = False
    # =====================
    # World
    # =====================
    sim_world = SimulationWorld()
    world = sim_world.get_world()
    sim_world.reset()

    sim_world.step(render=True)

    # =====================
    # Robot
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
        look_at=(0.67, 0.0, 0.61),
    )

    print("Camera warming up...")
    app = omni.kit.app.get_app()
    for i in range(60):
        sim_world.step(render=True)
        app.update()
        if i % 10 == 0:
            print(f"  Warmup {i}/60")

    detector = SimpleObjectDetector(
        hsv_lower=(0, 0, 100),
        hsv_upper=(180, 95, 255),
        min_area=50,
    )
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

        # 디버깅: 첫 프레임에만 HSV 분석 (생략 - 필요하면 유지)

        # =====================
        # Detector
        # =====================
        result = detector.detect(vis)

        if isinstance(result, tuple) and len(result) == 3:
            center, bbox, mask = result

            # 마스크 중심 계산
            ys, xs = np.where(mask > 0)

            if len(xs) > 0:
                cx = int(xs.mean())
                cy = int(ys.mean())

                # 시각화
                cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)

                # =====================
                # 좌표 변환 및 로봇 제어
                # =====================
                if depth is not None:
                    z = depth[cy, cx]
                    
                    if z > 0.1 and not has_grasped:
                        world_point = camera.pixel_depth_to_world(cx, cy, z)
                        
                        print(f"\n[Vision] Detected object at:")
                        print(f"  Pixel: ({cx}, {cy})")
                        print(f"  Depth: {z:.3f} m")
                        print(f"  World: ({world_point[0]:.3f}, {world_point[1]:.3f}, {world_point[2]:.3f})")
                        
                        # ==========================================
                        # 로봇 제어 시작
                        # ==========================================
                        
                        # 1. Pre-grasp 위치 (물체 위 15cm)
                        pre_grasp_pos = world_point.copy()
                        pre_grasp_pos[2] += 0.15
                        
                        print(f"\n[Robot] Step 1: Moving to pre-grasp position")
                        print(f"  Target: {pre_grasp_pos}")
                        
                        # TODO: robot.move_to(pre_grasp_pos) 구현 필요
                        # 임시: 위치만 출력
                        
                        # 2. Grasp 위치로 하강
                        grasp_pos = world_point.copy()
                        grasp_pos[2] += 0.02  # 2cm 위 (큐브 크기 고려)
                        
                        print(f"[Robot] Step 2: Moving to grasp position")
                        print(f"  Target: {grasp_pos}")
                        
                        # TODO: robot.move_to(grasp_pos)
                        
                        # 3. 그리퍼 닫기
                        print(f"[Robot] Step 3: Closing gripper")
                        # TODO: robot.close_gripper()
                        
                        # 4. 들어올리기
                        lift_pos = grasp_pos.copy()
                        lift_pos[2] += 0.20
                        
                        print(f"[Robot] Step 4: Lifting object")
                        print(f"  Target: {lift_pos}")
                        
                        # TODO: robot.move_to(lift_pos)
                        
                        has_grasped = True
                        print("\n[Success] Grasp sequence completed!\n")

        # =====================
        # 디버그 저장
        # =====================
        if frame_count % 60 == 0:
            rgb_path = f"/tmp/debug_rgb_{frame_count}.png"
            cv2.imwrite(rgb_path, cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))

            if depth is not None:
                depth_vis = camera.visualize_depth(depth, 0.5, 3.0)
                if depth_vis is not None:
                    depth_path = f"/tmp/debug_depth_{frame_count}.png"
                    cv2.imwrite(depth_path, depth_vis)

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
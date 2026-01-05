# main.py
import sys
import os
import cv2
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from isaacsim.simulation_app import SimulationApp

# 무조건 제일 먼저
simulation_app = SimulationApp({"headless": False})

from sim.world import SimulationWorld
from sim.robot import FrankaRobot
from sim.camera import SimulationCamera
from vision.detector import SimpleObjectDetector
import omni.kit.app


def main():
    # =====================
    # World 먼저
    # =====================
    sim_world = SimulationWorld()
    world = sim_world.get_world()
    sim_world.reset()

    # 최소 1프레임 렌더
    sim_world.step(render=True)

    # =====================
    # Robot
    # =====================
    robot = FrankaRobot(world)
    robot.initialize()

    # 로봇 로딩 대기 (증가)
    print("Loading robot...")
    for _ in range(50):  # 30 -> 50으로 증가
        sim_world.step(render=True)

    # =====================
    # Camera (로봇 로딩 후 초기화)
    # =====================
    print("Initializing camera...")
    camera = SimulationCamera(
        prim_path="/World/FrankaCamera",
        position=(0.0, 0.0, 2.5),  # 위에서 아래를 보도록
        look_at=(0.0, 0.0, 0.5),  # 테이블 위의 물체를 바라봄
    )

    # 카메라 초기화 후 충분한 워밍업
    print("Camera warming up...")
    app = omni.kit.app.get_app()
    
    for i in range(60):  # 20 -> 60으로 증가
        sim_world.step(render=True)
        app.update()  # 렌더링 동기화
        if i % 10 == 0:
            print(f"  Warmup {i}/60")

    detector = SimpleObjectDetector()

    # 카메라 정보 출력
    camera.get_camera_info()
    
    # 첫 번째 이미지 확인
    print("\nTesting first image capture...")
    for attempt in range(5):
        print(f"  Attempt {attempt + 1}/5...")
        
        # 렌더링 동기화
        for _ in range(5):
            sim_world.step(render=True)
            app.update()
        
        # RGB + Depth 동시 가져오기
        test_rgb, test_depth = camera.get_rgb_depth()
        
        if test_rgb is not None:
            print(f"    ✓ RGB Shape: {test_rgb.shape}")
            print(f"    ✓ RGB Stats: min={test_rgb.min()}, max={test_rgb.max()}, mean={test_rgb.mean():.2f}")
            
            if test_depth is not None:
                print(f"    ✓ Depth Shape: {test_depth.shape}")
                print(f"    ✓ Depth Stats: min={test_depth.min():.3f}m, max={test_depth.max():.3f}m, mean={test_depth.mean():.3f}m")
                
                # Depth 시각화
                depth_vis = camera.visualize_depth(test_depth)
                if depth_vis is not None:
                    cv2.imwrite(f"/tmp/test_depth_{attempt}.png", depth_vis)
                    print(f"    ✓ Saved depth visualization")
            
            # 테스트 저장
            if test_rgb.shape[2] == 4:
                test_save = test_rgb[:, :, :3]
            else:
                test_save = test_rgb
            cv2.imwrite(f"/tmp/test_capture_{attempt}.png", cv2.cvtColor(test_save, cv2.COLOR_RGB2BGR))
            print(f"    ✓ Saved test image")
            break
        else:
            print(f"    ✗ Failed")

    print("\nStart main loop")
    frame_count = 0

    # =====================
    # Main loop
    # =====================
    while simulation_app.is_running():

        # 시뮬레이션 스텝
        sim_world.step(render=True)
        
        # 렌더링 완료 대기 (CRITICAL!)
        app.update()

        # RGB + Depth 동시 가져오기 (효율적!)
        rgb, depth = camera.get_rgb_depth()

        # 안전 가드 - RGB
        if rgb is None:
            if frame_count % 60 == 0:
                print(f"Frame {frame_count}: RGB is None")
            frame_count += 1
            continue

        if not isinstance(rgb, np.ndarray):
            if frame_count % 60 == 0:
                print(f"Frame {frame_count}: RGB is not ndarray")
            frame_count += 1
            continue

        if rgb.ndim != 3:
            if frame_count % 60 == 0:
                print(f"Frame {frame_count}: RGB dim is {rgb.ndim}, expected 3")
            frame_count += 1
            continue

        if rgb.shape[2] != 3 and rgb.shape[2] != 4:
            if frame_count % 60 == 0:
                print(f"Frame {frame_count}: RGB channels is {rgb.shape[2]}")
            frame_count += 1
            continue

        # RGB 복사 (RGBA인 경우 RGB로 변환)
        if rgb.shape[2] == 4:
            vis = rgb[:, :, :3].copy()
        else:
            vis = rgb.copy()

        # 이미지 저장 및 디버깅
        if frame_count % 60 == 0:
            # RGB 저장
            save_path = f"/tmp/isaac_cam_{frame_count}.png"
            cv2.imwrite(save_path, cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
            print(f"[Saved RGB] {save_path}")
            print(f"  Shape: {vis.shape}, dtype: {vis.dtype}")
            print(f"  Stats: min={vis.min()}, max={vis.max()}, mean={vis.mean():.2f}")
            
            # 중앙 픽셀 값 확인
            h, w = vis.shape[:2]
            center_pixel = vis[h//2, w//2]
            print(f"  Center pixel RGB: {center_pixel}")
            
            # Depth 저장 및 분석
            if depth is not None:
                # Depth 시각화
                depth_colored = camera.visualize_depth(depth, min_depth=0.5, max_depth=3.0)
                if depth_colored is not None:
                    depth_path = f"/tmp/isaac_depth_{frame_count}.png"
                    cv2.imwrite(depth_path, depth_colored)
                    print(f"[Saved Depth] {depth_path}")
                
                # Depth 통계
                print(f"  Depth Stats: min={depth.min():.3f}m, max={depth.max():.3f}m, mean={depth.mean():.3f}m")
                center_depth = depth[h//2, w//2]
                print(f"  Center depth: {center_depth:.3f}m")

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
        # close는 딱 한 번, 맨 마지막
        simulation_app.close()
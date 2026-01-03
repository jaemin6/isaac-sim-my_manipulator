# main.py
# 프로젝트 루트 디렉토리를 sys.path에 명시적으로 추가
import sys
import os

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from isaacsim.simulation_app import SimulationApp

# 1. SimulationApp 먼저 생성
simulation_app = SimulationApp({"headless": False})

# 2. 그 다음 Isaac 관련 모듈 import
from sim.world import SimulationWorld
from sim.robot import FrankaRobot
from sim.camera import SimulationCamera
from vision.detector import SimpleObjectDetector

def main():
    # World 생성
    sim_world = SimulationWorld()
    world = sim_world.get_world()

    # World reset (physics 준비)
    sim_world.reset()

    # 로봇 생성 & initialize
    robot = FrankaRobot(world)
    robot.initialize()

    # 카메라 생성
    camera = SimulationCamera()

    # Detector 생성
    detector = SimpleObjectDetector()

    # 카메라가 준비될 때까지 몇 step 실행
    print("Initializing camera...")
    for _ in range(5):
        sim_world.step(render=True)
    
    print("Starting main loop...")
    frame_count = 0
    
    # 메인 루프
    while simulation_app.is_running():
        sim_world.step(render=True)
        
        # RGB 이미지 가져오기
        rgb = camera.get_rgb()
        
        if rgb is not None:
            # 객체 감지
            detections = detector.detect(rgb)
            
            # 감지 결과 출력 (필요시)
            if frame_count % 30 == 0:  # 30 프레임마다 출력
                print(f"Frame {frame_count}: Detected {len(detections) if detections else 0} objects")
            
            # TODO: 감지된 객체에 따라 로봇 제어
            # if detections:
            #     target = detections[0]  # 첫 번째 객체
            #     robot.move_to_target(target)
        else:
            if frame_count < 10:  # 처음 10 프레임만 경고
                print(f"Frame {frame_count}: Camera data not ready")
        
        frame_count += 1

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        simulation_app.close()
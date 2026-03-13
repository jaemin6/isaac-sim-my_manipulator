"""
main.py
- 경로 강제 주입 로직 강화
- ROS2 Bridge 통합 버전
"""

import sys
import os

# [수정] Isaac Sim 4.0+ 경로 강제 주입 (가장 먼저 실행되어야 함)
ISAAC_ROOT = "/home/jemini/isaac-sim"
BRIDGE_PATH = os.path.join(ISAAC_ROOT, "exts/isaacsim.ros2.bridge")
# humble 폴더 안의 모든 패키지를 인식시키기 위해 상위 humble 폴더 추가
HUMBLE_PATH = os.path.join(BRIDGE_PATH, "humble")

# 파이썬이 모듈을 찾을 때 최우선으로 여기를 보게 함
if BRIDGE_PATH not in sys.path:
    sys.path.insert(0, BRIDGE_PATH)
if HUMBLE_PATH not in sys.path:
    sys.path.insert(0, HUMBLE_PATH)

# [추가] 중요: isaacsim.ros2 라는 가상 네임스페이스를 위해 폴더 구조 인식 강제화
try:
    import isaacsim.ros2.bridge as bridge_mod
    print("[ROS2] 시스템 경로 연결 성공!")
except ImportError:
    # 만약 안된다면 한 단계 더 깊게 직접 연결
    sys.path.insert(0, os.path.join(BRIDGE_PATH, "pip_prebundle"))

from isaacsim.simulation_app import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.utils.extensions import enable_extension

# Isaac Sim ros2 브릿지 강제 연결
print("[System] ROS2 Bridge 강제 활성화 시도 중...")
enable_extension("isaacsim.ros2.bridge")

# SimulationApp 실행 후에 다른 모듈들을 불러와야 안전합니다.
import omni.kit.app
import omni.timeline
from sim.world import setup_world
from controllers.unified_controller import UnifiedController

def main():
    print("\n" + "="*60)
    print("로봇 학습 시스템 시작 (ROS2 Bridge 연동 모드)")
    print("="*60)

    # world 생성 시 ROS2 Bridge가 정상적으로 초기화되는지 확인
    try:
        world, franka, ros2_bridge = setup_world()
    except Exception as e:
        print(f"[Setup Error] World 설정 중 오류 발생: {e}")
        return

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    world.reset()

    # 시뮬레이션 초기 안정화
    for _ in range(60):
        world.step(render=True)

    controller = UnifiedController(franka, world, ros2_bridge)
    app = omni.kit.app.get_app()

    print("\n[Ready] Press 1/2/3/4 to select phase")
    print("        Press SPACE to start grasp\n")

    while simulation_app.is_running():
        controller.update()
        world.step(render=True)
        app.update()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[Error] {e}")
        import traceback
        traceback.print_exc()
    finally:
        simulation_app.close()
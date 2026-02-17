"""
main.py
- Phase 포함
- 키보드로 모드 전환 (1/2/3/4 키)
"""

from isaacsim.simulation_app import SimulationApp
simulation_app = SimulationApp({"headless": False})

# Python 경로에 현재 디렉토리 추가
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import omni.kit.app
from sim.world import setup_world
from controllers.unified_controller import UnifiedController


def main():
    print("\n" + "="*60)
    print("로봇 학습 시스템 시작")
    print("="*60)
    
    # 월드 생성 (로봇, 큐브, 카메라 포함)
    world, franka, camera = setup_world()

    # reset 전 플레이
    import omni.timeline
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    world.reset()

    for _ in range(50):
        world.step(render = True)

    # 통합 컨트롤러 (모든 Phase 통합)
    controller = UnifiedController(franka, world, camera)
    
    # 메인 루프
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
"""
maim.py
- Phase 포함
- 키보드로 모드 전환 (1/2/3/4 키)
"""

from isaacsim.simulation_app import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.kit.app
from sim.world import setup_world
from controllers.unified_controller import UnifiedController


def main():
    print("\n" + "="*60)
    print("로봇 학습 시스템 시작")
    print("="*60)
    
    # 월드 생성
    world, franka, camera = setup_world()
    
    # 통합 컨트롤러
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
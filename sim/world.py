from omni.isaac.core import World
from omni.isaac.core.objects import GroundPlane

class SimulationWorld:
    def __init__(self):
        # 1. Isaac World 초기화 (기본 단위: 미터)
        self.world = World(stage_units_in_meters=1.0)
        self.stage = self.world.scene.stage

        # 2. 필수 기본 환경 구성
        self._setup_scene()

    def _setup_scene(self):
        # 바닥 생성
        GroundPlane("/World/Ground")
        # 여기에 로봇이나 추가 오브젝트 로드 로직을 추가

    # -------------------------------------------------
    # 핵심 제어 API
    # -------------------------------------------------
    def reset(self):
        """시뮬레이션 초기화"""
        self.world.reset()

    def step(self, render=True):
        """물리 타임스텝 진행"""
        self.world.step(render=render)

    def get_world(self):
        """World 객체 접근"""
        return self.world

if __name__ == "__main__":
    sim_world = SimulationWorld()
    sim_world.reset()
    
    while True:
        sim_world.step()
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.franka import Franka

# 월드 생성
world = World(stage_units_in_meters=1.0)

# Franka 로봇 추가
franka = Franka(
    prim_path="/World/Franka",
    name="franka"
)

# 초기화
world.reset()

print("Franka loaded")

# 시뮬레이션 루프
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()

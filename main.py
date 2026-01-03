from isaacsim.simulation_app import SimulationApp
# 1. SimulationApp 먼저 생성
simulation_app = SimulationApp({"headless": False})
# 2. 그 다음 Isaac 관련 모듈 import
from sim.world import SimulationWorld
from sim.robot import FrankaRobot
from sim.camera import SimulationCamera
from vision.perception import PerceptionSystem

# World 생성
sim_world = SimulationWorld()
world = sim_world.get_world()

# World reset (physics 준비)
sim_world.reset()

# Physics를 최소 1 step 돌림
sim_world.step(render=True)

# 로봇 생성 & initialize
robot = FrankaRobot(world)
robot.initialize()

camera = SimulationCamera()

perception = PerceptionSystem(camera)

while simulation_app.is_running():
    sim_world.step(render=True)
    perception.step()

simulation_app.close()


from isaacsim.simulation_app import SimulationApp
# 1. SimulationApp 먼저 생성
simulation_app = SimulationApp({"headless": False})
# 2. 그 다음 Isaac 관련 모듈 import
from sim.world import SimulationWorld
from sim.robot import FrankaRobot

sim_world = SimulationWorld()
world = sim_world.get_world()

robot = FrankaRobot(world)
robot.initialize()

sim_world.reset()

while simulation_app.is_running():
    sim_world.step(render=True)

simulation_app.close()


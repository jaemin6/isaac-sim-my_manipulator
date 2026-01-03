from omni.isaac.kit import SimulationApp
from sim.world import SimulationWorld
from sim.robot import FrankaRobot

simulation_app = SimulationApp({"headless": False})

sim_world = SimulationWorld()
world = sim_world.get_world()

robot = FrankaRobot(world)
robot.initialize()

sim_world.reset()

while simulation_app.is_running():
    sim_world.step(render=True)

simulation_app.close()


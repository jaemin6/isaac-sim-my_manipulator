from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.franka import Franka

world = World(stage_units_in_meters=1.0)

franka = Franka(
    prim_path="/World/Franka",
    name="franka"
)

world.reset()

print("Simulation Start")

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()

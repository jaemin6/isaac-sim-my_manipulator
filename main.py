from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.franka import Franka
from omni.isaac.core.objects import GroundPlane
from pxr import UsdLux

world = World(stage_units_in_meters=1.0)

# 바닥
ground = GroundPlane("/World/Ground")

# 로봇
franka = Franka(
    prim_path="/World/Franka",
    name="franka"
)

# 조명
stage = world.scene.stage
light = UsdLux.DistantLight.Define(stage, "/World/Light")
light.CreateIntensityAttr(3000)
light.CreateAngleAttr(0.5)

world.reset()

# 테스트로 자세 변경
franka.set_joint_positions([0, -0.5, 0, -2.0, 0, 2.0, 0.8])

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()

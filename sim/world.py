# sim/world.py
world = World(stage_units_in_meters=1.0)
GroundPlane("/World/Ground")

# Table
UsdGeom.Cube.Define(stage, "/World/Table")
UsdLux.DistantLight.Define(stage, "/World/Light")

# Cube / Target
UsdGeom.Cube.Define(stage, "/World/Cube")
UsdGeom.Cube.Define(stage, "/World/Target")

# Physics (Collision, RigidBody, Mass)
UsdPhysics.*
PhysxSchema.*

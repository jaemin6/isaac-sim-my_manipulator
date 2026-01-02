# sim/robot.py
import numpy as np
from omni.isaac.franka import Franka
from omni.isaac.core.utils.types import ArticulationAction

class FrankaRobot:
    def __init__(self, world, prim_path="/World/Franka"):
        self.franka = Franka(prim_path=prim_path, name="franka")
        world.scene.add(self.franka)

    def initialize(self):
        self.franka.initialize()
        self.controller = self.franka.get_articulation_controller()

    def get_ee_pose(self):
        return self.franka.get_end_effector_pose()

    def apply_joint_positions(self, joints):
        action = ArticulationAction(joint_positions=joints)
        self.controller.apply_action(action)

    def open_gripper(self):
        joints = self.franka.get_joint_positions()
        joints[-2:] = [0.04, 0.04]
        self.apply_joint_positions(joints)

    def close_gripper(self):
        joints = self.franka.get_joint_positions()
        joints[-2:] = [0.015, 0.015]
        self.apply_joint_positions(joints)

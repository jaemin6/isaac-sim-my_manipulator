# sim/robot.py
import numpy as np
from omni.isaac.franka import Franka
from omni.isaac.core.utils.types import ArticulationAction


class FrankaRobot:
    def __init__(self, world, prim_path="/World/Franka"):
        self.world = world

        # Franka 생성
        self.franka = Franka(
            prim_path=prim_path,
            name="franka"
        )
        self.world.scene.add(self.franka)

        self.controller = None

    def initialize(self):
        self.franka.initialize()
        self.controller = self.franka.get_articulation_controller()
        print("[Robot] Franka initialized")

    def get_ee_pose(self):
        position, orientation = self.franka.end_effector.get_world_pose()
        return position, orientation

    def move_ee_to_position(self, target_position):
        """
        ⚠ IK Solver 없이
        간단한 position 기반 EE 이동 (디버깅용)
        """
        current_pos, _ = self.get_ee_pose()
        delta = np.array(target_position) - np.array(current_pos)

        joints = self.franka.get_joint_positions()

        # 매우 단순한 매핑 (디버깅 목적)
        joints[0] += delta[0] * 0.5
        joints[1] += delta[1] * 0.5
        joints[2] += delta[2] * 0.3

        action = ArticulationAction(joint_positions=joints)
        self.controller.apply_action(action)

    def close_gripper(self):
        joints = self.franka.get_joint_positions()
        joints[-2:] = [0.015, 0.015]
        self.controller.apply_action(
            ArticulationAction(joint_positions=joints)
        )

    def open_gripper(self):
        joints = self.franka.get_joint_positions()
        joints[-2:] = [0.04, 0.04]
        self.controller.apply_action(
            ArticulationAction(joint_positions=joints)
        )

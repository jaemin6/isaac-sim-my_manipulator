# sim/robot.py
import numpy as np
from omni.isaac.franka import Franka
from omni.isaac.core.utils.types import ArticulationAction


class FrankaRobot:
    def __init__(self, world):
        self.world = world

        # Franka 로드 (USD 기반)
        self.franka = Franka(
            prim_path="/World/Franka",
            name="franka"
        )
        self.world.scene.add(self.franka)

        self.controller = None

    def initialize(self):
        """
        Franka 초기화 + articulation controller 연결
        """
        self.franka.initialize()
        self.controller = self.franka.get_articulation_controller()
        print("[Robot] Franka initialized")

    # -------------------------------------------------
    # 상태 조회
    # -------------------------------------------------
    def get_joint_positions(self):
        return self.franka.get_joint_positions()

    def get_ee_pose(self):
        pos, ori = self.franka.end_effector.get_world_pose()
        return np.array(pos), np.array(ori)

    # -------------------------------------------------
    # 기본 동작 (IK X, 관절 제어만)
    # -------------------------------------------------
    def move_to_home(self):
        """
        Franka 기본 안전 자세
        """
        home_joints = np.array([
            0.0,        # joint1
            -0.785,     # joint2
            0.0,        # joint3
            -2.356,     # joint4
            0.0,        # joint5
            1.571,     # joint6
            0.785       # joint7
        ])

        action = ArticulationAction(joint_positions=home_joints)
        self.controller.apply_action(action)

    def move_joint_delta(self, delta):
        """
        현재 관절 값에서 delta 만큼 이동
        """
        current = self.get_joint_positions()
        target = current + delta
        action = ArticulationAction(joint_positions=target)
        self.controller.apply_action(action)

    # -------------------------------------------------
    # 그리퍼
    # -------------------------------------------------
    def open_gripper(self):
        self.franka.gripper.apply_action(
            ArticulationAction(joint_positions=[0.04, 0.04])
        )
        print("[Robot] Gripper opened")

    def close_gripper(self, width=0.015):
        self.franka.gripper.apply_action(
            ArticulationAction(joint_positions=[width, width])
        )
        print("[Robot] Gripper closed")

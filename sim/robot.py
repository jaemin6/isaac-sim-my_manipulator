from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
import numpy as np


class FrankaRobot:
    def __init__(self, world, prim_path="/World/Franka", name="franka"):
        self.world = world
        self.prim_path = prim_path
        self.name = name
        self.robot = None
        
    def add_to_scene(self):
        """World의 scene에 로봇 추가 (reset 전에 호출)"""
        from omni.isaac.franka import Franka
        
        # Franka 로봇을 scene에 추가
        self.robot = self.world.scene.add(
            Franka(
                prim_path=self.prim_path,
                name=self.name,
                end_effector_prim_name="panda_hand"
            )
        )
        print(f"[Robot] Added Franka to scene at {self.prim_path}")
        return self.robot

    def get_ee_pose(self):
        """End effector의 현재 위치와 orientation 반환"""
        ee_pos, ee_rot = self.robot.end_effector.get_world_pose()
        return ee_pos, ee_rot

    def move_to_joint_positions(self, positions):
        """관절 위치로 직접 이동"""
        self.robot.set_joint_positions(positions)

    def apply_action(self, action):
        """ArticulationAction 적용"""
        self.robot.apply_action(action)

    def get_joint_positions(self):
        """현재 관절 위치 반환"""
        return self.robot.get_joint_positions()

    def close_gripper(self):
        """그리퍼 닫기"""
        self.robot.gripper.close()

    def open_gripper(self):
        """그리퍼 열기"""
        self.robot.gripper.open()
    
    def get_gripper_position(self):
        """현재 그리퍼 위치 반환"""
        return self.robot.gripper.get_joint_positions()
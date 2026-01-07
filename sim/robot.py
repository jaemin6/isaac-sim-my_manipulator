# sim/robot.py 
import numpy as np
from omni.isaac.franka import Franka
from omni.isaac.core.utils.types import ArticulationAction


class FrankaRobot:
    def __init__(self, world, prim_path="/World/Franka"):
        self.franka = Franka(prim_path=prim_path, name="franka")
        world.scene.add(self.franka)
        self.world = world

    def initialize(self):
        self.franka.initialize()
        self.controller = self.franka.get_articulation_controller()
        print("[Robot] Initialized (simple control mode)")

    def get_ee_pose(self):
        """엔드 이펙터의 현재 pose 반환"""
        position, orientation = self.franka.end_effector.get_world_pose()
        return position, orientation

    def move_to(self, target_position, duration=3.0):
        """
        목표 위치로 이동 (시간 기반)
        
        Args:
            target_position: np.array([x, y, z])
            duration: 이동 시간 (초)
        """
        print(f"[Robot] Moving to: {target_position}")
        
        current_pos, _ = self.get_ee_pose()
        steps = int(duration * 60)  # 60 FPS 가정
        
        for i in range(steps):
            alpha = (i + 1) / steps
            
            # 선형 보간
            interpolated_pos = (1 - alpha) * np.array(current_pos) + alpha * np.array(target_position)
            
            # 간단한 위치 기반 제어
            delta = interpolated_pos - np.array(current_pos)
            
            current_joints = self.franka.get_joint_positions()
            current_joints[0] += delta[0] * 0.5
            current_joints[1] += delta[1] * 0.5
            current_joints[2] += delta[2] * 0.3
            
            self.apply_joint_positions(current_joints)
            self.world.step(render=True)
            
            # 현재 위치 업데이트
            current_pos, _ = self.get_ee_pose()
            
            if i % 30 == 0:
                distance = np.linalg.norm(np.array(current_pos) - np.array(target_position))
                print(f"  Progress: {i}/{steps}, distance: {distance:.3f}m")
        
        final_pos, _ = self.get_ee_pose()
        distance = np.linalg.norm(np.array(final_pos) - np.array(target_position))
        print(f"[Robot] Move completed (final distance: {distance:.3f}m)")

    def apply_joint_positions(self, joints):
        action = ArticulationAction(joint_positions=joints)
        self.controller.apply_action(action)

    def open_gripper(self):
        joints = self.franka.get_joint_positions()
        joints[-2:] = [0.04, 0.04]
        self.apply_joint_positions(joints)
        
        # 그리퍼 동작 시간
        for _ in range(30):
            self.world.step(render=True)
        
        print("[Robot] Gripper opened")

    def close_gripper(self):
        joints = self.franka.get_joint_positions()
        joints[-2:] = [0.015, 0.015]
        self.apply_joint_positions(joints)
        
        # 그리퍼 동작 시간
        for _ in range(30):
            self.world.step(render=True)
        
        print("[Robot] Gripper closed")
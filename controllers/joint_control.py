# controllers/joint_control.py
"""
Phase 1: Joint Control
- 기존 방식의 Joint angle 기반 제어
- 키보드로 수동 제어 가능
- Auto-grasp/place 지원
"""

import sys
import os
import numpy as np
from omni.isaac.core.utils.types import ArticulationAction

# 절대 경로로 utils 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
utils_dir = os.path.join(parent_dir, 'utils')

sys.path.insert(0, parent_dir)
sys.path.insert(0, utils_dir)

# utils. 접두사 없이 직접 import
from cube_utils import (
    get_cube_position,
    find_nearest_cube,
    attach_cube_to_ee,
    detach_cube
)


class JointController:
    """Phase 1: Joint Control"""
    
    def __init__(self, franka, world):
        self.franka = franka
        self.world = world
        self.controller = franka.get_articulation_controller()
        
        # Joint 상태
        self.target_joints = franka.get_joint_positions()
        
        # 상태
        self.gripper_closed = False
        self.cube_attached = False
        self.current_cube_index = None
        
        print("[Phase 1] Joint Control initialized")
    
    def manual_control(self, key):
        """
        수동 Joint 제어
        
        Args:
            key (str): 'w', 's', 'a', 'd', 'q', 'e', 'r', 'f', 'g'
        """
        step = 0.1
        
        if key == 'w':
            self.target_joints[1] -= step
            print(f"[Joint] Shoulder Up: {self.target_joints[1]:.2f}")
        elif key == 's':
            self.target_joints[1] += step
            print(f"[Joint] Shoulder Down: {self.target_joints[1]:.2f}")
        elif key == 'a':
            self.target_joints[0] -= step
            print(f"[Joint] Base Left: {self.target_joints[0]:.2f}")
        elif key == 'd':
            self.target_joints[0] += step
            print(f"[Joint] Base Right: {self.target_joints[0]:.2f}")
        elif key == 'q':
            self.target_joints[3] -= step
            print(f"[Joint] Elbow: {self.target_joints[3]:.2f}")
        elif key == 'e':
            self.target_joints[3] += step
            print(f"[Joint] Elbow: {self.target_joints[3]:.2f}")
        elif key == 'r':
            self.target_joints[5] -= step
            print(f"[Joint] Wrist: {self.target_joints[5]:.2f}")
        elif key == 'f':
            self.target_joints[5] += step
            print(f"[Joint] Wrist: {self.target_joints[5]:.2f}")
        elif key == 'g':
            self.toggle_gripper()
    
    def toggle_gripper(self):
        """그리퍼 열고 닫기"""
        if self.gripper_closed:
            self.franka.gripper.open()
            self.gripper_closed = False
            print("[Gripper] OPEN")
        else:
            self.franka.gripper.close()
            self.gripper_closed = True
            print("[Gripper] CLOSED")
    
    def auto_grasp(self, cube_index=None):
        """
        자동 grasp (Joint angle 기반)
        
        Args:
            cube_index (int): 큐브 인덱스. None이면 가장 가까운 큐브
        
        Returns:
            bool: 성공 여부
        """
        print("\n[Phase 1] Auto-Grasp starting...")
        
        # 1. 큐브 선택
        if cube_index is None:
            ee_pos, _ = self.franka.end_effector.get_world_pose()
            nearest = find_nearest_cube(ee_pos)
            
            if nearest is None:
                print("[Error] No cube detected!")
                return False
            
            cube_index = nearest['index']
            cube_pos = nearest['position']
            print(f"  Selected: Cube_{cube_index}")
        else:
            cube_pos = get_cube_position(cube_index)
            if cube_pos is None:
                print(f"[Error] Cube_{cube_index} not found!")
                return False
        
        self.current_cube_index = cube_index
        print(f"  Position: ({cube_pos[0]:.2f}, {cube_pos[1]:.2f}, {cube_pos[2]:.2f})")
        
        # 2. 그리퍼 열기
        print("[Phase 1] Opening gripper...")
        for _ in range(30):
            self.franka.gripper.open()
            self.world.step(render=True)
        self.gripper_closed = False
        
        # 3. 각도 계산
        angle = np.arctan2(cube_pos[1], cube_pos[0])
        print(f"[Phase 1] Rotating to angle: {np.degrees(angle):.1f} deg")
        
        # 4. Pre-grasp pose
        pre_grasp = np.array([
            angle,
            -0.5,
            0.0,
            -2.0,
            0.0,
            1.8,
            0.8,
            0.04,
            0.04
        ])
        
        print("[Phase 1] Moving to pre-grasp...")
        for _ in range(200):
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=pre_grasp)
                )
            except:
                pass
            self.world.step(render=True)
        
        # 5. Approach
        grasp_pose = pre_grasp.copy()
        grasp_pose[3] -= 0.5
        
        print("[Phase 1] Approaching...")
        for _ in range(100):
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=grasp_pose)
                )
            except:
                pass
            self.world.step(render=True)
        
        # 6. Close gripper
        print("[Phase 1] Closing gripper...")
        for _ in range(60):
            self.franka.gripper.close()
            self.world.step(render=True)
        self.gripper_closed = True
        
        for _ in range(20):
            self.world.step(render=True)
        
        # 7. Attach
        attach_cube_to_ee(cube_index)
        self.cube_attached = True
        
        # 8. Lift
        lift_pose = grasp_pose.copy()
        lift_pose[1] += 0.3
        lift_pose[3] += 0.6
        
        print("[Phase 1] Lifting...")
        for _ in range(150):
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=lift_pose)
                )
            except:
                pass
            self.world.step(render=True)
        
        self.target_joints = lift_pose
        
        print("[Phase 1] ✓ Grasp complete!\n")
        return True
    
    def place(self, target_position):
        """
        큐브 놓기
        
        Args:
            target_position (np.array): 목표 위치 [x, y, z]
        
        Returns:
            bool: 성공 여부
        """
        if not self.cube_attached or self.current_cube_index is None:
            print("[Error] No cube attached!")
            return False
        
        print("\n[Phase 1] Place starting...")
        print(f"  Target: ({target_position[0]:.2f}, {target_position[1]:.2f}, {target_position[2]:.2f})")
        
        # 1. 각도 계산
        angle = np.arctan2(target_position[1], target_position[0])
        
        # 2. Hover pose
        hover_pose = np.array([
            angle,
            -0.5,
            0.0,
            -2.0,
            0.0,
            1.8,
            0.8,
            0.01,
            0.01
        ])
        
        print("[Phase 1] Moving to hover...")
        for _ in range(200):
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=hover_pose)
                )
            except:
                pass
            self.world.step(render=True)
        
        # 3. Lower
        place_pose = hover_pose.copy()
        place_pose[3] -= 0.3
        
        print("[Phase 1] Lowering...")
        for _ in range(100):
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=place_pose)
                )
            except:
                pass
            self.world.step(render=True)
        
        for _ in range(30):
            self.world.step(render=True)
        
        # 4. Detach
        print("[Phase 1] Releasing cube...")
        detach_cube(self.current_cube_index)
        self.cube_attached = False
        self.current_cube_index = None
        
        # 5. Open gripper
        for _ in range(40):
            self.franka.gripper.open()
            self.world.step(render=True)
        self.gripper_closed = False
        
        # 6. Retreat (높이 올라가기!)
        retreat_pose = place_pose.copy()
        retreat_pose[1] += 0.5  # shoulder 더 높이!
        retreat_pose[3] += 0.8  # elbow 더 높이!
        
        print("[Phase 1] Retreating...")
        for _ in range(120):
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=retreat_pose)
                )
            except:
                pass
            self.world.step(render=True)
        
        self.target_joints = retreat_pose
        
        print("[Phase 1] ✓ Place complete!\n")
        return True
    
    def update(self):
        """매 프레임 업데이트"""
        try:
            self.controller.apply_action(
                ArticulationAction(joint_positions=self.target_joints)
            )
        except:
            pass
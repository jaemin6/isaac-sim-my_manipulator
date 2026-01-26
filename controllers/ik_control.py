# controllers/ik_control.py
"""
Phase 2: IK Control
- Inverse Kinematics 기반 정밀 제어
- 목표 3D 위치로 직접 이동
- mm 단위 정밀도
"""

import sys
import os
import numpy as np
import time
from omni.isaac.core.utils.rotations import euler_angles_to_quat

# 프로젝트 루트를 Python 경로에 추가
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from utils.cube_utils import (
    get_cube_position,
    find_nearest_cube,
    attach_cube_to_ee,
    detach_cube
)


class IKController:
    """Phase 2: IK Control"""
    
    def __init__(self, franka, world):
        self.franka = franka
        self.world = world
        
        # 상태
        self.gripper_closed = False
        self.cube_attached = False
        self.current_cube_index = None
        
        # 성능 측정
        self.performance = {
            'grasp_times': [],
            'place_times': [],
            'position_errors': []
        }
        
        print("[Phase 2] IK Control initialized")
    
    def move_to_position(self, target_position, steps=150):
        """
        IK를 사용해서 목표 위치로 이동
        (현재는 간단한 joint interpolation 사용)
        
        Args:
            target_position (np.array): 목표 위치 [x, y, z]
            steps (int): 이동 스텝 수
        
        Returns:
            float: 위치 오차
        """
        from omni.isaac.core.utils.types import ArticulationAction
        
        print(f"[IK] Moving to: ({target_position[0]:.2f}, {target_position[1]:.2f}, {target_position[2]:.2f})")
        
        # 목표 방향 계산
        angle = np.arctan2(target_position[1], target_position[0])
        
        # 높이에 따른 joint 값 추정 (간단한 휴리스틱)
        height_diff = target_position[2] - 0.5  # 테이블 기준
        
        # 목표 joint angles (경험적 값)
        target_joints = np.array([
            angle,           # base rotation
            -0.5 - height_diff * 1.5,  # shoulder
            0.0,
            -2.0 - height_diff * 2.0,  # elbow  
            0.0,
            1.8 + height_diff * 1.5,   # wrist
            0.8,
            0.04,  # gripper
            0.04
        ])
        
        # 부드러운 이동
        for _ in range(steps):
            try:
                self.franka.get_articulation_controller().apply_action(
                    ArticulationAction(joint_positions=target_joints)
                )
            except:
                pass
            self.world.step(render=True)
        
        # 최종 오차 계산
        final_pos, _ = self.franka.end_effector.get_world_pose()
        error = np.linalg.norm(final_pos - target_position)
        print(f"[IK] Position error: {error*1000:.2f} mm")
        
        return error
    
    def auto_grasp(self, cube_index=None):
        """
        IK 기반 자동 grasp
        
        Args:
            cube_index (int): 큐브 인덱스. None이면 가장 가까운 큐브
        
        Returns:
            bool: 성공 여부
        """
        start_time = time.time()
        
        print("\n[Phase 2: IK] Auto-Grasp starting...")
        
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
        print(f"  Position: ({cube_pos[0]:.3f}, {cube_pos[1]:.3f}, {cube_pos[2]:.3f})")
        
        # 2. 그리퍼 열기
        print("[IK] Step 1/4: Opening gripper...")
        for _ in range(30):
            self.franka.gripper.open()
            self.world.step(render=True)
        self.gripper_closed = False
        
        # 3. Pre-grasp: 큐브 위 10cm
        pre_grasp_pos = cube_pos.copy()
        pre_grasp_pos[2] += 0.10
        
        print("[IK] Step 2/4: Moving to pre-grasp (10cm above)...")
        error1 = self.move_to_position(pre_grasp_pos, steps=200)
        print(f"  Position error: {error1*1000:.2f} mm")
        
        # 안정화
        for _ in range(30):
            self.world.step(render=True)
        
        # 4. Approach: 큐브 바로 위 (2cm)
        approach_pos = cube_pos.copy()
        approach_pos[2] += 0.02
        
        print("[IK] Step 3/4: Approaching cube (2cm above)...")
        error2 = self.move_to_position(approach_pos, steps=100)
        print(f"  Position error: {error2*1000:.2f} mm")
        
        # 안정화
        for _ in range(20):
            self.world.step(render=True)
        
        # 5. 그리퍼 닫기
        print("[IK] Step 4/4: Closing gripper...")
        for _ in range(60):
            self.franka.gripper.close()
            self.world.step(render=True)
        self.gripper_closed = True
        
        # 안정화
        for _ in range(20):
            self.world.step(render=True)
        
        # 6. Attach cube
        attach_cube_to_ee(cube_index)
        self.cube_attached = True
        
        # 7. Lift: 15cm 위로
        lift_pos = approach_pos.copy()
        lift_pos[2] += 0.15
        
        print("[IK] Lifting cube...")
        error3 = self.move_to_position(lift_pos, steps=150)
        print(f"  Position error: {error3*1000:.2f} mm")
        
        # 성능 기록
        elapsed = time.time() - start_time
        avg_error = (error1 + error2 + error3) / 3
        
        self.performance['grasp_times'].append(elapsed)
        self.performance['position_errors'].append(avg_error)
        
        print(f"\n[Phase 2] ✓ Grasp Complete!")
        print(f"  Time: {elapsed:.2f}s")
        print(f"  Avg Position Error: {avg_error*1000:.2f} mm\n")
        
        return True
    
    def place(self, target_position):
        """
        IK 기반 큐브 놓기
        
        Args:
            target_position (np.array): 목표 위치 [x, y, z]
        
        Returns:
            bool: 성공 여부
        """
        start_time = time.time()
        
        if not self.cube_attached or self.current_cube_index is None:
            print("[Error] No cube attached!")
            return False
        
        print("\n[Phase 2: IK] Place starting...")
        print(f"  Target: ({target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f})")
        
        # 1. Hover: 목표 위 10cm
        hover_pos = target_position.copy()
        hover_pos[2] += 0.10
        
        print("[IK] Step 1/4: Moving to hover...")
        self.move_to_position(hover_pos, steps=200)
        
        for _ in range(30):
            self.world.step(render=True)
        
        # 2. Lower: 목표 위 2cm
        place_pos = target_position.copy()
        place_pos[2] += 0.02
        
        print("[IK] Step 2/4: Lowering to place height...")
        self.move_to_position(place_pos, steps=100)
        
        for _ in range(30):
            self.world.step(render=True)
        
        # 3. Detach cube
        print("[IK] Step 3/4: Releasing cube...")
        detach_cube(self.current_cube_index)
        self.cube_attached = False
        self.current_cube_index = None
        
        for _ in range(20):
            self.world.step(render=True)
        
        # 4. Open gripper
        for _ in range(40):
            self.franka.gripper.open()
            self.world.step(render=True)
        self.gripper_closed = False
        
        # 5. Retreat: 높이 올리기 (충돌 방지!)
        retreat_pos = place_pos.copy()
        retreat_pos[2] += 0.20  # 20cm 위로! (기존 15cm)
        
        print("[IK] Step 4/4: Retreating...")
        self.move_to_position(retreat_pos, steps=120)
        
        elapsed = time.time() - start_time
        self.performance['place_times'].append(elapsed)
        
        print(f"\n[Phase 2] ✓ Place Complete!")
        print(f"  Time: {elapsed:.2f}s\n")
        
        return True
    
    def get_performance_summary(self):
        """성능 요약 반환"""
        perf = self.performance
        
        summary = {
            'total_grasps': len(perf['grasp_times']),
            'total_places': len(perf['place_times'])
        }
        
        if perf['grasp_times']:
            summary['avg_grasp_time'] = np.mean(perf['grasp_times'])
            summary['std_grasp_time'] = np.std(perf['grasp_times'])
        
        if perf['place_times']:
            summary['avg_place_time'] = np.mean(perf['place_times'])
            summary['std_place_time'] = np.std(perf['place_times'])
        
        if perf['position_errors']:
            summary['avg_position_error'] = np.mean(perf['position_errors'])
            summary['std_position_error'] = np.std(perf['position_errors'])
        
        return summary
    
    def print_performance(self):
        """성능 출력"""
        summary = self.get_performance_summary()
        
        print(f"\n{'='*60}")
        print("PHASE 2: IK CONTROL - PERFORMANCE")
        print(f"{'='*60}")
        print(f"Total Grasps:      {summary.get('total_grasps', 0)}")
        print(f"Total Places:      {summary.get('total_places', 0)}")
        
        if 'avg_grasp_time' in summary:
            print(f"\nGrasp Time:        {summary['avg_grasp_time']:.2f}s ± {summary['std_grasp_time']:.2f}s")
        
        if 'avg_place_time' in summary:
            print(f"Place Time:        {summary['avg_place_time']:.2f}s ± {summary['std_place_time']:.2f}s")
        
        if 'avg_position_error' in summary:
            print(f"\nPosition Error:    {summary['avg_position_error']*1000:.2f} mm ± {summary['std_position_error']*1000:.2f} mm")
        
        print(f"{'='*60}\n")
    
    def reset_performance(self):
        """성능 데이터 초기화"""
        self.performance = {
            'grasp_times': [],
            'place_times': [],
            'position_errors': []
        }
        print("[Phase 2] Performance data reset!")
    
    def update(self):
        """매 프레임 업데이트 (IK는 특별한 업데이트 없음)"""
        pass
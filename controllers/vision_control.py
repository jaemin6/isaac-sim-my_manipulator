# controllers/vision_control.py
"""
Phase 3: Vision-based Control
- 카메라로 큐브 인식
- RGB 색상 기반 detection
- 실제 3D 위치 추정
"""

import sys
import os

# 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
utils_dir = os.path.join(parent_dir, 'utils')

sys.path.insert(0, parent_dir)
sys.path.insert(0, utils_dir)

import numpy as np
import time
import cv2
from omni.isaac.core.utils.types import ArticulationAction

from cube_utils import (
    attach_cube_to_ee,
    detach_cube,
    get_cube_position  # Ground truth와 비교용
)


class VisionController:
    """Phase 3: Vision-based Control"""
    
    def __init__(self, franka, world, camera):
        self.franka = franka
        self.world = world
        self.camera = camera
        self.controller = franka.get_articulation_controller()
        
        # 상태
        self.gripper_closed = False
        self.cube_attached = False
        self.current_cube_index = None
        
        # 성능 측정
        self.performance = {
            'grasp_times': [],
            'place_times': [],
            'detection_errors': [],  # Vision 오차
            'position_errors': []     # Grasp 오차
        }
        
        # 카메라 초기화
        self.camera.initialize()
        
        print("[Phase 3] Vision Control initialized")
    
    def detect_cubes_from_camera(self):
        """카메라 이미지에서 큐브 감지 (Replicator 기반)"""
        print("\n[Vision] Detecting cubes from camera...")
        
        import omni.replicator.core as rep
        
        # ===== 카메라 설정 수정 =====
        # 테이블 위 큐브들을 내려다보는 위치
        camera_position = (1.0, 0.8, 1.2)  # 테이블 중심 위 1.2m
        look_at_target = (0.5, 0.0, 0.0)  # 큐브들이 있는 테이블 표면
        
        rep_cam = rep.create.camera(
            position=camera_position,
            look_at=look_at_target,
            focal_length=15.0
        )
        # ===========================
        
        # Render product 생성
        rp = rep.create.render_product(rep_cam, (1024, 768))
        
        # RGB annotator 설정
        rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb_annot.attach([rp])
        
        # 렌더링 실행
        rep.orchestrator.step()
        
        # 데이터 획득 (재시도 로직)
        rgb = None
        for _ in range(50):
            self.world.step(render=True)
            rgb = rgb_annot.get_data()
            if rgb is not None and len(rgb) > 0:
                break
        
        if rgb is None or len(rgb) == 0:
            print("[Vision] Failed to get camera data!")
            return []
            
        # NumPy 배열로 변환 (함수 내부로 들여쓰기 정렬)
        img = np.array(rgb)
        
        # RGBA → RGB
        if img.shape[2] == 4:
            img = img[:, :, :3]
        
        # Float → Uint8
        if img.dtype != np.uint8:
            img = (np.clip(img, 0, 1) * 255).astype(np.uint8)
        
        # 디버그: 이미지 저장
        cv2.imwrite("debug_camera.png", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        print(f"[Vision Debug] Camera image saved to debug_camera.png")
        print(f"[Vision Debug] Image shape: {img.shape}, dtype: {img.dtype}")
        print(f"[Vision Debug] Image range: [{img.min()}, {img.max()}]")
        
        # HSV 변환
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        
        detected_cubes = []
        
        # 수정된 색상 범위 (Replicator 렌더링 기준)
        color_ranges = {
            'red': {
                'hsv_ranges': [([0, 100, 100], [10, 255, 255]), ([170, 100, 100], [180, 255, 255])],
                'index': 0
            },
            'green': {
                'hsv_ranges': [([40, 70, 70], [80, 255, 255])],
                'index': 1
            },
            'blue': {
                'hsv_ranges': [([100, 100, 100], [130, 255, 255])],
                'index': 2
            },
            'yellow': {
                'hsv_ranges': [([20, 100, 100], [35, 255, 255])],
                'index': 3
            }
        }
        
        img_h, img_w = img.shape[:2]
        
        for color_name, color_info in color_ranges.items():
            mask = np.zeros((img_h, img_w), dtype=np.uint8)
            
            for lower, upper in color_info['hsv_ranges']:
                lower = np.array(lower)
                upper = np.array(upper)
                mask_part = cv2.inRange(hsv, lower, upper)
                mask = cv2.bitwise_or(mask, mask_part)
            
            # 디버그: 마스크 저장
            cv2.imwrite(f"debug_mask_{color_name}.png", mask)
            
            # 노이즈 제거
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Contour 찾기
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 1000:  # 최소 크기
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # 픽셀 → 월드 좌표 변환
                        world_pos = self._pixel_to_world(cx, cy, img_w, img_h)
                        
                        detected_cubes.append({
                            'color': color_name,
                            'index': color_info['index'],
                            'position': world_pos,
                            'pixel_pos': (cx, cy),
                            'area': area,
                            'confidence': min(area / 2000.0, 1.0)
                        })
        
        print(f"[Vision] Detected {len(detected_cubes)} cubes:")
        for cube in detected_cubes:
            print(f"  {cube['color']}: ({cube['position'][0]:.2f}, {cube['position'][1]:.2f}, {cube['position'][2]:.2f})")
        
        return detected_cubes
    
    def _pixel_to_world(self, px, py, img_w, img_h):
        """
        픽셀 좌표를 월드 좌표로 변환
        카메라: (0.5, 0.0, 1.2) 위치, 아래를 향함
        """
        # 정규화된 이미지 좌표 (-1 to 1)
        norm_x = (px - img_w/2) / (img_w/2)
        norm_y = (py - img_h/2) / (img_h/2)
        
        # 통합 스케일 적용
        scale_x = 0.55
        scale_y = 0.75
        
        # 카메라가 (0.5, 0.0, 1.2)에서 아래를 봄
        world_x = 0.4 + 0.15 - (norm_y * scale_x)
        world_y = -(norm_x * scale_y)
        world_z = 0.05  # 테이블 위 (추정)
        
        return np.array([world_x, world_y, world_z])
    
    def auto_grasp(self, cube_index=None):
        """
        Vision 기반 자동 grasp
        
        Args:
            cube_index (int): 특정 큐브 선택. None이면 가장 가까운 것
        
        Returns:
            bool: 성공 여부
        """
        start_time = time.time()  # 시작 시간
        
        print("\n[Phase 3: Vision] Starting vision-based grasp...")
        
        # 1. 카메라로 큐브 감지
        detected = self.detect_cubes_from_camera()
        
        if not detected:
            print("[Vision] No cubes detected!")
            return False
        
        # 2. 큐브 선택
        if cube_index is not None:
            # 특정 인덱스의 큐브 찾기
            target_cube = None
            for cube in detected:
                if cube['index'] == cube_index:
                    target_cube = cube
                    break
            
            if target_cube is None:
                print(f"[Vision] Cube_{cube_index} not detected!")
                return False
        else:
            # 가장 가까운 큐브
            ee_pos, _ = self.franka.end_effector.get_world_pose()
            target_cube = min(detected, 
                            key=lambda c: np.linalg.norm(c['position'][:2] - ee_pos[:2]))
        
        cube_pos = target_cube['position']
        self.current_cube_index = target_cube['index']
        
        print(f"[Vision] Selected: {target_cube['color']} cube")
        print(f"[Vision] Vision position: ({cube_pos[0]:.3f}, {cube_pos[1]:.3f}, {cube_pos[2]:.3f})")
        
        # Ground truth와 비교
        gt_pos = get_cube_position(self.current_cube_index)
        if gt_pos is not None:
            detection_error = np.linalg.norm(cube_pos - gt_pos)
            self.performance['detection_errors'].append(detection_error)
            print(f"[Vision] Detection error: {detection_error*1000:.2f} mm")
        
        # 3. 그리퍼 열기
        print("[Vision] Opening gripper...")
        for _ in range(30):
            self.franka.gripper.open()
            self.world.step(render=True)
        self.gripper_closed = False
        
        # 4. Joint 제어로 grasp (Vision 위치 사용)
        angle = np.arctan2(cube_pos[1], cube_pos[0])
        
        # Pre-grasp
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
        
        print("[Vision] Moving to pre-grasp...")
        for _ in range(200):
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=pre_grasp)
                )
            except:
                pass
            self.world.step(render=True)
        
        # Approach
        grasp_pose = pre_grasp.copy()
        grasp_pose[3] -= 0.5
        
        print("[Vision] Approaching...")
        for _ in range(100):
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=grasp_pose)
                )
            except:
                pass
            self.world.step(render=True)
        
        # Close gripper
        print("[Vision] Closing gripper...")
        for _ in range(60):
            self.franka.gripper.close()
            self.world.step(render=True)
        self.gripper_closed = True
        
        for _ in range(20):
            self.world.step(render=True)
        
        # Attach
        attach_cube_to_ee(self.current_cube_index)
        self.cube_attached = True
        
        # Lift
        lift_pose = grasp_pose.copy()
        lift_pose[1] += 0.3
        lift_pose[3] += 0.6
        
        print("[Vision] Lifting...")
        for _ in range(150):
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=lift_pose)
                )
            except:
                pass
            self.world.step(render=True)
        
        elapsed = time.time() - start_time
        self.performance['grasp_times'].append(elapsed)  # 경과 시간 저장
        
        print(f"\n[Phase 3] ✓ Vision Grasp Complete!")
        print(f"  Time: {elapsed:.2f}s")
        
        return True
    
    def place(self, target_position):
        """
        큐브 놓기 (Phase 1과 동일)
        
        Args:
            target_position (np.array): 목표 위치
        
        Returns:
            bool: 성공 여부
        """
        start_time = time.time()
        
        if not self.cube_attached or self.current_cube_index is None:
            print("[Error] No cube attached!")
            return False
        
        print("\n[Vision] Placing...")
        
        angle = np.arctan2(target_position[1], target_position[0])
        
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
        
        for _ in range(200):
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=hover_pose)
                )
            except:
                pass
            self.world.step(render=True)
        
        place_pose = hover_pose.copy()
        place_pose[3] -= 0.3
        
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
        
        detach_cube(self.current_cube_index)
        self.cube_attached = False
        self.current_cube_index = None
        
        for _ in range(40):
            self.franka.gripper.open()
            self.world.step(render=True)
        self.gripper_closed = False
        
        retreat_pose = place_pose.copy()
        retreat_pose[1] += 0.5
        retreat_pose[3] += 0.8
        
        for _ in range(120):
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=retreat_pose)
                )
            except:
                pass
            self.world.step(render=True)
        
        elapsed = time.time() - start_time
        self.performance['place_times'].append(elapsed)
        
        print(f"[Phase 3] ✓ Place complete ({elapsed:.2f}s)")
        
        return True
    
    def get_performance_summary(self):
        """성능 요약"""
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
        
        if perf['detection_errors']:
            summary['avg_detection_error'] = np.mean(perf['detection_errors'])
            summary['std_detection_error'] = np.std(perf['detection_errors'])
        
        return summary
    
    def print_performance(self):
        """성능 출력"""
        summary = self.get_performance_summary()
        
        print(f"\n{'='*60}")
        print("PHASE 3: VISION CONTROL - PERFORMANCE")
        print(f"{'='*60}")
        print(f"Total Grasps:      {summary.get('total_grasps', 0)}")
        print(f"Total Places:      {summary.get('total_places', 0)}")
        
        if 'avg_grasp_time' in summary:
            print(f"\nGrasp Time:        {summary['avg_grasp_time']:.2f}s ± {summary['std_grasp_time']:.2f}s")
        
        if 'avg_place_time' in summary:
            print(f"Place Time:        {summary['avg_place_time']:.2f}s ± {summary['std_place_time']:.2f}s")
        
        if 'avg_detection_error' in summary:
            print(f"\nVision Error:      {summary['avg_detection_error']*1000:.2f} mm ± {summary['std_detection_error']*1000:.2f} mm")
        
        print(f"{'='*60}\n")
    
    def update(self):
        """매 프레임 업데이트"""
        pass
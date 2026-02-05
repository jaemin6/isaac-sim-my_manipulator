# controllers/vision_control.py
"""
Phase 3: Vision-based Control (FIXED VERSION)
- 개선된 색상 감지 (4가지 색상 모두 인식)
- 정확한 픽셀-월드 좌표 변환
- Ground Truth 기반 자동 보정
- 디버그 시각화
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
    get_cube_position
)


class VisionController:
    """Phase 3: Vision-based Control (Fixed)"""
    
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
            'detection_errors': [],
            'position_errors': []
        }
        
        # 카메라 초기화
        self.camera.initialize()
        
        # 보정 파라미터 (초기값)
        self.calibration = {
            'scale_x': 0.5,
            'scale_y': 0.5,
            'offset_x': 0.5,
            'offset_y': 0.0
        }
        
        print("[Phase 3] Vision Control (Fixed) initialized")
    
    def detect_cubes_from_camera(self, visualize=True):
        """
        카메라 이미지에서 큐브 감지
        
        Args:
            visualize (bool): 디버그 이미지 저장 여부
        
        Returns:
            list: 감지된 큐브 정보
        """
        print("\n[Vision] Detecting cubes from camera...")
        
        import omni.replicator.core as rep
        
        # ===== 카메라 설정 (고정) =====
        camera_position = (1.0, 0.0, 1.5)
        look_at_target = (0.5, 0.0, 0.0)
        
        rep_cam = rep.create.camera(
            position=camera_position,
            look_at=look_at_target,
            focal_length=12.0
        )
        # =============================
        
        # Render product 생성
        rp = rep.create.render_product(rep_cam, (1024, 768))
        
        # RGB annotator 설정
        rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb_annot.attach([rp])
        
        # 렌더링 실행
        rep.orchestrator.step()
        
        # 데이터 획득
        rgb = None
        for _ in range(150):
            self.world.step(render=True)
            rgb = rgb_annot.get_data()
            if rgb is not None and len(rgb) > 0:
                break
        
        if rgb is None or len(rgb) == 0:
            print("[Vision] Failed to get camera data!")
            return []
        
        # NumPy 배열로 변환
        img = np.array(rgb)
        
        # RGBA → RGB
        if img.shape[2] == 4:
            img = img[:, :, :3]
        
        # Float → Uint8
        if img.dtype != np.uint8:
            img = (np.clip(img, 0, 1) * 255).astype(np.uint8)
        
        # 디버그 정보
        print(f"[Vision Debug] Image shape: {img.shape}, dtype: {img.dtype}")
        print(f"[Vision Debug] Image range: [{img.min()}, {img.max()}]")
        
        # HSV 변환
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        
        detected_cubes = []
        
        # ===== 개선된 색상 범위 =====
        # Replicator 렌더링에 맞춰 조정
        color_ranges = {
            'red': {
                'hsv_ranges': [
                    ([0, 70, 70], [10, 255, 255]),      # 낮은 빨강
                    ([170, 70, 70], [180, 255, 255])    # 높은 빨강
                ],
                'index': 0,
                'rgb_check': (180, 70, 70)  # 대략적인 RGB 값
            },
            'green': {
                'hsv_ranges': [([40, 60, 60], [85, 255, 255])],
                'index': 1,
                'rgb_check': (60, 180, 60)
            },
            'blue': {
                'hsv_ranges': [([90, 60, 60], [130, 255, 255])],
                'index': 2,
                'rgb_check': (60, 60, 180)
            },
            'yellow': {
                'hsv_ranges': [([15, 100, 100], [35, 255, 255])],
                'index': 3,
                'rgb_check': (200, 200, 60)
            }
        }
        
        img_h, img_w = img.shape[:2]
        
        # 디버그용 이미지 복사
        debug_img = img.copy()
        
        for color_name, color_info in color_ranges.items():
            mask = np.zeros((img_h, img_w), dtype=np.uint8)
            
            # 여러 HSV 범위 합치기
            for lower, upper in color_info['hsv_ranges']:
                lower = np.array(lower)
                upper = np.array(upper)
                mask_part = cv2.inRange(hsv, lower, upper)
                mask = cv2.bitwise_or(mask, mask_part)
            
            # 노이즈 제거
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Contour 찾기
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:  # 최소 크기 (더 작게)
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # 픽셀 → 월드 좌표 변환
                        world_pos = self._pixel_to_world(cx, cy, img_w, img_h)
                        
                        cube_info = {
                            'color': color_name,
                            'index': color_info['index'],
                            'position': world_pos,
                            'pixel_pos': (cx, cy),
                            'area': area,
                            'confidence': min(area / 2000.0, 1.0)
                        }
                        
                        detected_cubes.append(cube_info)
                        
                        # 디버그 이미지에 그리기
                        if visualize:
                            # Bounding box
                            x, y, w, h = cv2.boundingRect(cnt)
                            cv2.rectangle(debug_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                            
                            # 중심점
                            cv2.circle(debug_img, (cx, cy), 5, (0, 0, 255), -1)
                            
                            # 텍스트
                            text = f"{color_name} ({world_pos[0]:.2f}, {world_pos[1]:.2f})"
                            cv2.putText(debug_img, text, (x, y-10),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # 마스크 저장
            if visualize:
                cv2.imwrite(f"debug_mask_{color_name}.png", mask)
        
        # 디버그 이미지 저장
        if visualize:
            cv2.imwrite("debug_camera.png", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            cv2.imwrite("debug_detection.png", cv2.cvtColor(debug_img, cv2.COLOR_RGB2BGR))
            print(f"[Vision Debug] Saved: debug_camera.png, debug_detection.png")
        
        # Ground Truth와 매칭
        detected_cubes = self._match_with_ground_truth(detected_cubes)
        
        print(f"[Vision] Detected {len(detected_cubes)} cubes:")
        for cube in detected_cubes:
            gt_pos = get_cube_position(cube['index'])
            if gt_pos is not None:
                error = np.linalg.norm(cube['position'] - gt_pos)
                print(f"  {cube['color']} (Cube_{cube['index']}): Vision=({cube['position'][0]:.2f}, {cube['position'][1]:.2f}) | GT=({gt_pos[0]:.2f}, {gt_pos[1]:.2f}) | Error={error*1000:.1f}mm")
            else:
                print(f"  {cube['color']}: ({cube['position'][0]:.2f}, {cube['position'][1]:.2f})")
        
        return detected_cubes
    
    def _pixel_to_world(self, px, py, img_w, img_h):
        """
        픽셀 좌표 → 월드 좌표 변환
        
        카메라: (1.0, 0.0, 1.5)
        Look at: (0.5, 0.0, 0.0)
        """
        # 정규화 (-1 to 1)
        norm_x = (px - img_w/2) / (img_w/2)
        norm_y = (py - img_h/2) / (img_h/2)
        
        # 보정 파라미터 적용
        cal = self.calibration
        
        # 변환 공식
        # Y축 (좌우) → norm_x와 반대
        # X축 (앞뒤) → norm_y와 비례
        world_x = cal['offset_x'] + (norm_y * cal['scale_x'])
        world_y = cal['offset_y'] - (norm_x * cal['scale_y'])
        world_z = 0.025  # 큐브 절반 높이
        
        return np.array([world_x, world_y, world_z])
    
    def _match_with_ground_truth(self, detected_cubes):
        """
        Ground Truth와 매칭하여 index 보정
        
        Args:
            detected_cubes (list): 감지된 큐브들
        
        Returns:
            list: 매칭된 큐브들
        """
        matched_cubes = []
        used_indices = set()
        
        for det_cube in detected_cubes:
            best_match = None
            best_distance = float('inf')
            
            # 4개 큐브와 거리 비교
            for i in range(4):
                if i in used_indices:
                    continue
                
                gt_pos = get_cube_position(i)
                if gt_pos is None:
                    continue
                
                distance = np.linalg.norm(det_cube['position'][:2] - gt_pos[:2])
                
                if distance < best_distance and distance < 0.3:  # 30cm 이내
                    best_distance = distance
                    best_match = i
            
            if best_match is not None:
                det_cube['index'] = best_match
                det_cube['gt_position'] = get_cube_position(best_match)
                det_cube['detection_error'] = best_distance
                matched_cubes.append(det_cube)
                used_indices.add(best_match)
        
        return matched_cubes
    
    def calibrate_with_ground_truth(self):
        """
        Ground Truth를 이용한 자동 보정
        
        Returns:
            bool: 보정 성공 여부
        """
        print("\n[Vision] Auto-calibrating with Ground Truth...")
        
        # 큐브 감지
        detected = self.detect_cubes_from_camera(visualize=False)
        
        if len(detected) < 2:
            print("[Vision] Need at least 2 cubes for calibration!")
            return False
        
        # 대응점 수집
        pixel_points = []
        world_points = []
        
        for cube in detected:
            if 'gt_position' in cube:
                pixel_points.append(cube['pixel_pos'])
                world_points.append(cube['gt_position'][:2])
        
        if len(pixel_points) < 2:
            print("[Vision] Not enough matched cubes!")
            return False
        
        pixel_points = np.array(pixel_points, dtype=np.float32)
        world_points = np.array(world_points, dtype=np.float32)
        
        # 선형 회귀로 변환 파라미터 추정
        # world_x = a * px + b * py + c
        # world_y = d * px + e * py + f
        
        # 간단한 스케일 추정
        img_w, img_h = 1024, 768
        
        # X축 스케일
        x_span = world_points[:, 0].max() - world_points[:, 0].min()
        px_span = pixel_points[:, 1].max() - pixel_points[:, 1].min()
        if px_span > 0:
            scale_x = x_span / (px_span / img_h * 2)
        else:
            scale_x = self.calibration['scale_x']
        
        # Y축 스케일
        y_span = world_points[:, 1].max() - world_points[:, 1].min()
        py_span = pixel_points[:, 0].max() - pixel_points[:, 0].min()
        if py_span > 0:
            scale_y = y_span / (py_span / img_w * 2)
        else:
            scale_y = self.calibration['scale_y']
        
        # Offset
        offset_x = world_points[:, 0].mean()
        offset_y = world_points[:, 1].mean()
        
        # 업데이트
        self.calibration['scale_x'] = scale_x
        self.calibration['scale_y'] = scale_y
        self.calibration['offset_x'] = offset_x
        self.calibration['offset_y'] = offset_y
        
        print(f"[Vision] Calibration updated:")
        print(f"  scale_x: {scale_x:.3f}")
        print(f"  scale_y: {scale_y:.3f}")
        print(f"  offset_x: {offset_x:.3f}")
        print(f"  offset_y: {offset_y:.3f}")
        
        # 재감지로 정확도 확인
        detected_new = self.detect_cubes_from_camera(visualize=True)
        
        if detected_new:
            errors = [c['detection_error'] for c in detected_new if 'detection_error' in c]
            if errors:
                avg_error = np.mean(errors)
                print(f"[Vision] Average error after calibration: {avg_error*1000:.1f} mm")
        
        return True
    
    def auto_grasp(self, cube_index=None):
        """
        Vision 기반 자동 grasp
        
        Args:
            cube_index (int): 특정 큐브 선택
        
        Returns:
            bool: 성공 여부
        """
        start_time = time.time()
        
        print("\n[Phase 3: Vision] Starting vision-based grasp...")
        
        # 1. 카메라로 큐브 감지
        detected = self.detect_cubes_from_camera(visualize=True)
        
        if not detected:
            print("[Vision] No cubes detected!")
            return False
        
        # 2. 큐브 선택
        if cube_index is not None:
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
        
        print(f"[Vision] Selected: {target_cube['color']} (Cube_{self.current_cube_index})")
        print(f"[Vision] Vision position: ({cube_pos[0]:.3f}, {cube_pos[1]:.3f}, {cube_pos[2]:.3f})")
        
        # Ground truth와 비교
        if 'detection_error' in target_cube:
            self.performance['detection_errors'].append(target_cube['detection_error'])
            print(f"[Vision] Detection error: {target_cube['detection_error']*1000:.2f} mm")
        
        # 3. 그리퍼 열기
        print("[Vision] Opening gripper...")
        for _ in range(30):
            self.franka.gripper.open()
            self.world.step(render=True)
        self.gripper_closed = False
        
        # 4. Joint 제어로 grasp
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
        self.performance['grasp_times'].append(elapsed)
        
        print(f"\n[Phase 3] ✓ Vision Grasp Complete!")
        print(f"  Time: {elapsed:.2f}s")
        
        return True
    
    def place(self, target_position):
        """큐브 놓기"""
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
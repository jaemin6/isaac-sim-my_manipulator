# controllers/vision_control.py
"""
Phase 3: Vision-based Control (HOMOGRAPHY VERSION)
- 카메라 위치 고정 (1.0, 0.0, 1.5)
- Homography 기반 정확한 좌표 변환
- 4개 큐브로 초기 캘리브레이션
"""

import sys
import os

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
    """Phase 3: Vision-based Control (Homography)"""
    
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
        
        # Homography 행렬 (초기화 후 설정)
        self.homography_matrix = None
        self.calibrated = False
        
        print("[Phase 3] Vision Control (Homography) initialized")
        print("[Vision] ⚠️  Run calibration first: controller.calibrate_homography()")
    
    def detect_cubes_from_camera(self, visualize=True):
        """카메라 이미지에서 큐브 감지"""
        
        import omni.replicator.core as rep
        
        # 카메라 설정 (고정)
        camera_position = (1.0, 0.0, 1.5)
        look_at_target = (0.5, 0.0, 0.0)
        
        rep_cam = rep.create.camera(
            position=camera_position,
            look_at=look_at_target,
            focal_length=12.0
        )
        
        # Render product
        rp = rep.create.render_product(rep_cam, (1024, 768))
        
        # RGB annotator
        rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb_annot.attach([rp])
        
        # 렌더링
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
        
        # 이미지 변환
        img = np.array(rgb)
        if img.shape[2] == 4:
            img = img[:, :, :3]
        if img.dtype != np.uint8:
            img = (np.clip(img, 0, 1) * 255).astype(np.uint8)
        
        # 밝기 정규화 (핵심!)
        img_normalized = self._normalize_brightness(img)
        
        print(f"[Vision Debug] Image shape: {img.shape}")
        print(f"[Vision Debug] Original range: [{img.min()}, {img.max()}]")
        print(f"[Vision Debug] Normalized range: [{img_normalized.min()}, {img_normalized.max()}]")
        
        # HSV 변환
        hsv = cv2.cvtColor(img_normalized, cv2.COLOR_RGB2HSV)
        
        detected_cubes = []
        debug_img = img.copy()
        
        # 개선된 색상 범위 (밝기 정규화 후)
        color_ranges = {
            'red': {
                'hsv_ranges': [
                    ([0, 100, 100], [10, 255, 255]),
                    ([170, 100, 100], [180, 255, 255])
                ],
                'index': 0
            },
            'green': {
                'hsv_ranges': [([50, 100, 100], [80, 255, 255])],
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
            
            # 노이즈 제거
            kernel = np.ones((7, 7), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Contour
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 300:  # 최소 크기
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Homography로 변환
                        if self.homography_matrix is not None:
                            world_pos = self._pixel_to_world_homography(cx, cy)
                        else:
                            world_pos = np.array([0.5, 0.0, 0.025])
                        
                        cube_info = {
                            'color': color_name,
                            'index': color_info['index'],
                            'position': world_pos,
                            'pixel_pos': (cx, cy),
                            'area': area
                        }
                        
                        detected_cubes.append(cube_info)
                        
                        # 시각화
                        if visualize:
                            x, y, w, h = cv2.boundingRect(cnt)
                            cv2.rectangle(debug_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                            cv2.circle(debug_img, (cx, cy), 5, (0, 0, 255), -1)
                            text = f"{color_name}"
                            cv2.putText(debug_img, text, (x, y-10),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            if visualize:
                cv2.imwrite(f"debug_mask_{color_name}.png", mask)
        
        # Ground Truth 매칭
        detected_cubes = self._match_with_ground_truth(detected_cubes)
        
        if visualize:
            cv2.imwrite("debug_camera.png", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            cv2.imwrite("debug_normalized.png", cv2.cvtColor(img_normalized, cv2.COLOR_RGB2BGR))
            cv2.imwrite("debug_detection.png", cv2.cvtColor(debug_img, cv2.COLOR_RGB2BGR))
        
        # 결과 출력
        print(f"[Vision] Detected {len(detected_cubes)} cubes:")
        for cube in detected_cubes:
            gt_pos = get_cube_position(cube['index'])
            if gt_pos is not None:
                error = np.linalg.norm(cube['position'][:2] - gt_pos[:2])
                print(f"  {cube['color']} (Cube_{cube['index']}): "
                      f"Pixel=({cube['pixel_pos'][0]}, {cube['pixel_pos'][1]}) | "
                      f"Vision=({cube['position'][0]:.2f}, {cube['position'][1]:.2f}) | "
                      f"GT=({gt_pos[0]:.2f}, {gt_pos[1]:.2f}) | "
                      f"Error={error*1000:.1f}mm")
        
        return detected_cubes
    
    def _normalize_brightness(self, img):
        """
        밝기 정규화 (CLAHE)
        → 조명 변화에 강인하게
        """
        # LAB 색공간 변환
        lab = cv2.cvtColor(img, cv2.COLOR_RGB2LAB)
        l, a, b = cv2.split(lab)
        
        # L 채널만 CLAHE 적용
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l_clahe = clahe.apply(l)
        
        # 재결합
        lab_clahe = cv2.merge([l_clahe, a, b])
        img_normalized = cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2RGB)
        
        return img_normalized
    
    def _pixel_to_world_homography(self, px, py):
        """Homography로 픽셀 → 월드 변환"""
        if self.homography_matrix is None:
            return np.array([0.5, 0.0, 0.025])
        
        # 동차 좌표
        pixel_point = np.array([[px, py]], dtype=np.float32)
        
        # 변환
        world_point = cv2.perspectiveTransform(
            pixel_point.reshape(1, 1, 2),
            self.homography_matrix
        )
        
        world_x = world_point[0, 0, 0]
        world_y = world_point[0, 0, 1]
        world_z = 0.025
        
        return np.array([world_x, world_y, world_z])
    
    def calibrate_homography(self):
        """
        4개 큐브로 Homography 보정
        
        Returns:
            bool: 성공 여부
        """
        print("\n" + "="*60)
        print("HOMOGRAPHY CALIBRATION")
        print("="*60)
        
        # 큐브 감지 (Homography 없이)
        self.homography_matrix = None
        detected = self.detect_cubes_from_camera(visualize=False)
        
        if len(detected) < 4:
            print(f"[Vision] Need 4 cubes for calibration! (found {len(detected)})")
            return False
        
        # 픽셀 좌표와 월드 좌표 수집
        pixel_points = []
        world_points = []
        
        for i in range(4):
            gt_pos = get_cube_position(i)
            if gt_pos is None:
                continue
            
            # 해당 큐브 찾기
            found = False
            for cube in detected:
                dist = np.linalg.norm(
                    np.array(cube['pixel_pos']) - np.array(detected[i]['pixel_pos'])
                )
                if dist < 50:  # 같은 큐브로 간주
                    pixel_points.append(cube['pixel_pos'])
                    world_points.append(gt_pos[:2])
                    found = True
                    break
            
            if not found:
                # 임시: detected 순서대로 매칭
                if i < len(detected):
                    pixel_points.append(detected[i]['pixel_pos'])
                    world_points.append(gt_pos[:2])
        
        if len(pixel_points) < 4:
            print(f"[Vision] Could not match 4 cubes! (matched {len(pixel_points)})")
            return False
        
        # NumPy 배열로 변환
        src_points = np.array(pixel_points, dtype=np.float32)
        dst_points = np.array(world_points, dtype=np.float32)
        
        print(f"\n[Calibration] Matched points:")
        for i, (px, py) in enumerate(pixel_points):
            wx, wy = world_points[i]
            print(f"  Cube {i}: Pixel({px:.0f}, {py:.0f}) → World({wx:.3f}, {wy:.3f})")
        
        # Homography 계산
        self.homography_matrix, status = cv2.findHomography(src_points, dst_points)
        
        if self.homography_matrix is None:
            print("[Vision] Homography calculation failed!")
            return False
        
        self.calibrated = True
        
        print(f"\n[Calibration] Homography matrix:")
        print(self.homography_matrix)
        
        # 검증
        print(f"\n[Calibration] Verification:")
        total_error = 0
        for i, (px, py) in enumerate(pixel_points):
            transformed = self._pixel_to_world_homography(px, py)
            gt_x, gt_y = world_points[i]
            error = np.linalg.norm(transformed[:2] - np.array([gt_x, gt_y]))
            total_error += error
            print(f"  Point {i}: Error = {error*1000:.1f} mm")
        
        avg_error = total_error / len(pixel_points)
        print(f"\n[Calibration] Average error: {avg_error*1000:.1f} mm")
        
        if avg_error < 0.05:  # 50mm 이하
            print("[Calibration] ✅ SUCCESS!")
        else:
            print("[Calibration] ⚠️  High error, may need adjustment")
        
        print("="*60 + "\n")
        
        # 재감지로 확인
        detected_new = self.detect_cubes_from_camera(visualize=True)
        
        return True
    
    def _match_with_ground_truth(self, detected_cubes):
        """Ground Truth 매칭"""
        matched_cubes = []
        used_indices = set()
        
        for det_cube in detected_cubes:
            best_match = None
            best_distance = float('inf')
            
            for i in range(4):
                if i in used_indices:
                    continue
                
                gt_pos = get_cube_position(i)
                if gt_pos is None:
                    continue
                
                distance = np.linalg.norm(det_cube['position'][:2] - gt_pos[:2])
                
                if distance < best_distance and distance < 0.4:
                    best_distance = distance
                    best_match = i
            
            if best_match is not None:
                det_cube['index'] = best_match
                det_cube['gt_position'] = get_cube_position(best_match)
                det_cube['detection_error'] = best_distance
                matched_cubes.append(det_cube)
                used_indices.add(best_match)
        
        return matched_cubes
    
    def auto_grasp(self, cube_index=None):
        """Vision 기반 자동 grasp"""
        
        if not self.calibrated:
            print("[Vision] ⚠️  Camera not calibrated! Run calibrate_homography() first")
            return False
        
        start_time = time.time()
        
        print("\n[Phase 3: Vision] Starting vision-based grasp...")
        
        # 큐브 감지
        detected = self.detect_cubes_from_camera(visualize=True)
        
        if not detected:
            print("[Vision] No cubes detected!")
            return False
        
        # 큐브 선택
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
            ee_pos, _ = self.franka.end_effector.get_world_pose()
            target_cube = min(detected, 
                            key=lambda c: np.linalg.norm(c['position'][:2] - ee_pos[:2]))
        
        cube_pos = target_cube['position']
        self.current_cube_index = target_cube['index']
        
        print(f"[Vision] Selected: {target_cube['color']} (Cube_{self.current_cube_index})")
        
        if 'detection_error' in target_cube:
            self.performance['detection_errors'].append(target_cube['detection_error'])
            print(f"[Vision] Detection error: {target_cube['detection_error']*1000:.2f} mm")
        
        # Grasp 실행
        print("[Vision] Opening gripper...")
        for _ in range(30):
            self.franka.gripper.open()
            self.world.step(render=True)
        self.gripper_closed = False
        
        angle = np.arctan2(cube_pos[1], cube_pos[0])
        
        pre_grasp = np.array([
            angle, -0.5, 0.0, -2.0, 0.0, 1.8, 0.8, 0.04, 0.04
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
        
        print("[Vision] Closing gripper...")
        for _ in range(60):
            self.franka.gripper.close()
            self.world.step(render=True)
        self.gripper_closed = True
        
        for _ in range(20):
            self.world.step(render=True)
        
        attach_cube_to_ee(self.current_cube_index)
        self.cube_attached = True
        
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
        
        print(f"\n[Phase 3] ✓ Vision Grasp Complete! (Time: {elapsed:.2f}s)")
        
        return True
    
    def place(self, target_position):
        """큐브 놓기"""
        start_time = time.time()
        
        if not self.cube_attached:
            print("[Error] No cube attached!")
            return False
        
        print("\n[Vision] Placing...")
        
        angle = np.arctan2(target_position[1], target_position[0])
        
        hover_pose = np.array([
            angle, -0.5, 0.0, -2.0, 0.0, 1.8, 0.8, 0.01, 0.01
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
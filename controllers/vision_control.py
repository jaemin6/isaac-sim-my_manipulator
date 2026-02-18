# controllers/vision_control.py
"""
Phase 3: Vision-based Control (HOMOGRAPHY VERSION)

[수정사항 v5 - Timeline 멈춤 버그 수정]
- rep.orchestrator.step() 제거
     → 이게 Isaac Sim Timeline을 내부적으로 Stop시키는 원인이었음
     → world.step(render=True) 몇 번으로 대체
- 카메라 이미지 취득 후 Timeline 상태 복구 보험 추가
- 상태머신 유지 (UI 렌더링 정상 작동)
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
import omni.kit.app
import omni.timeline
import omni.replicator.core as rep
from omni.isaac.core.utils.types import ArticulationAction

from cube_utils import (
    attach_cube_to_ee,
    detach_cube,
    get_cube_position
)

# 상태 정의
STATE_IDLE          = "idle"
STATE_OPEN_GRIPPER  = "open_gripper"
STATE_PRE_GRASP     = "pre_grasp"
STATE_APPROACH      = "approach"
STATE_CLOSE_GRIPPER = "close_gripper"
STATE_ATTACH        = "attach"
STATE_LIFT          = "lift"
STATE_PLACE_HOVER   = "place_hover"
STATE_PLACE_DOWN    = "place_down"
STATE_DETACH        = "detach"
STATE_OPEN_AFTER    = "open_after"
STATE_RETREAT       = "retreat"


class VisionController:
    """Phase 3: Vision-based Control (상태머신)"""

    def __init__(self, franka, world, camera):
        self.franka = franka
        self.world = world
        self.camera = camera
        self.controller = franka.get_articulation_controller()
        self.app = omni.kit.app.get_app()
        self.timeline = omni.timeline.get_timeline_interface()

        # 상태머신
        self.state = STATE_IDLE
        self.step_count = 0
        self.start_time = None

        # 그랩 관련
        self.current_cube_index = None
        self.cube_attached = False
        self.gripper_closed = False
        self._pre_grasp_pose = None
        self._grasp_pose = None
        self._lift_pose = None
        self._hover_pose = None
        self._place_pose = None
        self._retreat_pose = None

        # 성능 측정
        self.performance = {
            'grasp_times': [],
            'place_times': [],
            'detection_errors': [],
            'position_errors': []
        }

        # 카메라
        self.camera.initialize()
        self.homography_matrix = None
        self.calibrated = False
        self.rgb_annot = None
        self._rep_camera_ready = False

        print("[Phase 3] Vision Control (State Machine) initialized")

    # ------------------------------------------------------------------ #
    #  Replicator 카메라 (Lazy init)
    # ------------------------------------------------------------------ #

    def _ensure_camera_ready(self):
        """SimulationApp이 완전히 뜬 후 첫 사용 시점에 딱 한 번 생성"""
        if self._rep_camera_ready:
            return

        print("[Vision] Setting up replicator camera (once)...")

        rep_cam = rep.create.camera(
            position=(1.0, 0.0, 1.5),
            look_at=(0.5, 0.0, 0.0),
            focal_length=12.0
        )
        rp = rep.create.render_product(rep_cam, (1024, 768))
        self.rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self.rgb_annot.attach([rp])

        self._rep_camera_ready = True
        print("[Vision] Replicator camera ready.")

    def _get_camera_image(self):
        """
        ✅ rep.orchestrator.step() 없이 카메라 이미지 취득
        rep.orchestrator.step()은 Timeline을 내부적으로 Stop시키는 버그가 있음
        → world.step(render=True) 몇 번으로 대체
        → 이미지 취득 후 Timeline이 멈췄으면 자동 복구
        """
        self._ensure_camera_ready()

        # world.step으로 렌더링 갱신 (rep.orchestrator.step 대신)
        for _ in range(5):
            self.world.step(render=True)
            self.app.update()

        rgb = self.rgb_annot.get_data()

        # ✅ Timeline 복구 보험: 이미지 취득 과정에서 멈췄으면 재시작
        if not self.timeline.is_playing():
            print("[Vision] ⚠️  Timeline stopped during capture, restarting...")
            self.timeline.play()
            for _ in range(10):
                self.world.step(render=True)
                self.app.update()

        if rgb is None or len(rgb) == 0:
            print("[Vision] Failed to get camera data!")
            return None

        img = np.array(rgb)
        if img.shape[2] == 4:
            img = img[:, :, :3]
        if img.dtype != np.uint8:
            img = (np.clip(img, 0, 1) * 255).astype(np.uint8)

        return img

    # ------------------------------------------------------------------ #
    #  비전 처리
    # ------------------------------------------------------------------ #

    def _normalize_brightness(self, img):
        lab = cv2.cvtColor(img, cv2.COLOR_RGB2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        return cv2.cvtColor(cv2.merge([clahe.apply(l), a, b]), cv2.COLOR_LAB2RGB)

    def _pixel_to_world_homography(self, px, py):
        if self.homography_matrix is None:
            return np.array([0.5, 0.0, 0.025])
        wp = cv2.perspectiveTransform(
            np.array([[px, py]], dtype=np.float32).reshape(1, 1, 2),
            self.homography_matrix)
        return np.array([wp[0, 0, 0], wp[0, 0, 1], 0.025])

    def _match_with_ground_truth(self, detected_cubes):
        matched, used = [], set()
        for det in detected_cubes:
            best_i, best_d = None, float('inf')
            for i in range(4):
                if i in used:
                    continue
                gt = get_cube_position(i)
                if gt is None:
                    continue
                d = np.linalg.norm(det['position'][:2] - gt[:2])
                if d < best_d and d < 0.4:
                    best_d, best_i = d, i
            if best_i is not None:
                det['index'] = best_i
                det['gt_position'] = get_cube_position(best_i)
                det['detection_error'] = best_d
                matched.append(det)
                used.add(best_i)
        return matched

    def detect_cubes_from_camera(self, visualize=True):
        """카메라 이미지에서 큐브 감지"""
        img = self._get_camera_image()
        if img is None:
            return []

        img_normalized = self._normalize_brightness(img)

        print(f"[Vision Debug] Image shape: {img.shape}")
        print(f"[Vision Debug] Original range: [{img.min()}, {img.max()}]")
        print(f"[Vision Debug] Normalized range: [{img_normalized.min()}, {img_normalized.max()}]")

        hsv = cv2.cvtColor(img_normalized, cv2.COLOR_RGB2HSV)
        detected_cubes = []
        debug_img = img.copy()

        color_ranges = {
            'red':    {'hsv_ranges': [([0,80,80],[10,255,255]),([170,80,80],[180,255,255])], 'index': 0},
            'green':  {'hsv_ranges': [([35,60,60],[85,255,255])],                           'index': 1},
            'blue':   {'hsv_ranges': [([95,80,80],[130,255,255])],                          'index': 2},
            'yellow': {'hsv_ranges': [([18,80,80],[35,255,255])],                           'index': 3},
        }

        img_h, img_w = img.shape[:2]

        for color_name, color_info in color_ranges.items():
            mask = np.zeros((img_h, img_w), dtype=np.uint8)
            for lower, upper in color_info['hsv_ranges']:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, np.array(lower), np.array(upper)))

            kernel = np.ones((7, 7), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            for cnt in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
                area = cv2.contourArea(cnt)
                if area > 200:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        world_pos = (self._pixel_to_world_homography(cx, cy)
                                     if self.homography_matrix is not None
                                     else np.array([0.5, 0.0, 0.025]))
                        detected_cubes.append({
                            'color': color_name, 'index': color_info['index'],
                            'position': world_pos, 'pixel_pos': (cx, cy), 'area': area
                        })
                        if visualize:
                            x, y, w, h = cv2.boundingRect(cnt)
                            cv2.rectangle(debug_img, (x,y), (x+w,y+h), (0,255,0), 2)
                            cv2.circle(debug_img, (cx,cy), 5, (0,0,255), -1)
                            cv2.putText(debug_img, color_name, (x,y-10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            if visualize:
                cv2.imwrite(f"debug_mask_{color_name}.png", mask)

        detected_cubes = self._match_with_ground_truth(detected_cubes)

        if visualize:
            cv2.imwrite("debug_camera.png", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            cv2.imwrite("debug_detection.png", cv2.cvtColor(debug_img, cv2.COLOR_RGB2BGR))

        print(f"[Vision] Detected {len(detected_cubes)} cubes:")
        for cube in detected_cubes:
            gt_pos = get_cube_position(cube['index'])
            if gt_pos is not None:
                error = np.linalg.norm(cube['position'][:2] - gt_pos[:2])
                print(f"  {cube['color']} (Cube_{cube['index']}): "
                      f"Pixel=({cube['pixel_pos'][0]}, {cube['pixel_pos'][1]}) | "
                      f"Vision=({cube['position'][0]:.2f}, {cube['position'][1]:.2f}) | "
                      f"GT=({gt_pos[0]:.2f}, {gt_pos[1]:.2f}) | Error={error*1000:.1f}mm")

        return detected_cubes

    # ------------------------------------------------------------------ #
    #  캘리브레이션
    # ------------------------------------------------------------------ #

    def calibrate_homography(self):
        print("\n" + "="*60)
        print("HOMOGRAPHY CALIBRATION")
        print("="*60)

        self.homography_matrix = None
        detected = self.detect_cubes_from_camera(visualize=False)

        if len(detected) < 4:
            print(f"[Vision] Need 4 cubes! (found {len(detected)})")
            return False

        pixel_points, world_points = [], []
        for i in range(4):
            gt_pos = get_cube_position(i)
            if gt_pos is None:
                continue
            found = False
            for cube in detected:
                if np.linalg.norm(np.array(cube['pixel_pos']) - np.array(detected[i]['pixel_pos'])) < 50:
                    pixel_points.append(cube['pixel_pos'])
                    world_points.append(gt_pos[:2])
                    found = True
                    break
            if not found and i < len(detected):
                pixel_points.append(detected[i]['pixel_pos'])
                world_points.append(gt_pos[:2])

        if len(pixel_points) < 4:
            print(f"[Vision] Could not match 4 cubes! (matched {len(pixel_points)})")
            return False

        self.homography_matrix, _ = cv2.findHomography(
            np.array(pixel_points, dtype=np.float32),
            np.array(world_points,  dtype=np.float32)
        )

        if self.homography_matrix is None:
            print("[Vision] Homography failed!")
            return False

        self.calibrated = True

        print(f"\n[Calibration] Matched points:")
        for i, (px, py) in enumerate(pixel_points):
            print(f"  Cube {i}: Pixel({px:.0f}, {py:.0f}) → World({world_points[i][0]:.3f}, {world_points[i][1]:.3f})")

        print(f"\n[Calibration] Homography matrix:\n{self.homography_matrix}")

        total_error = sum(
            np.linalg.norm(self._pixel_to_world_homography(*pp)[:2] - np.array(wp))
            for pp, wp in zip(pixel_points, world_points)
        )
        avg_error = total_error / len(pixel_points)
        print(f"\n[Calibration] Average error: {avg_error*1000:.1f} mm")
        print("[Calibration] ✅ SUCCESS!" if avg_error < 0.05 else "[Calibration] ⚠️  High error")
        print("="*60 + "\n")

        self.detect_cubes_from_camera(visualize=True)
        return True

    # ------------------------------------------------------------------ #
    #  상태머신 진입점
    # ------------------------------------------------------------------ #

    def auto_grasp(self, cube_index=None):
        """상태머신 시작 - 실제 동작은 update()에서 매 프레임 진행"""
        if not self.calibrated:
            print("[Vision] ⚠️  Not calibrated!")
            return False

        if self.state != STATE_IDLE:
            print(f"[Vision] Already running: {self.state}")
            return False

        print("\n[Phase 3: Vision] Starting vision-based grasp (state machine)...")

        detected = self.detect_cubes_from_camera(visualize=True)
        if not detected:
            print("[Vision] No cubes detected!")
            return False

        if cube_index is not None:
            target = next((c for c in detected if c['index'] == cube_index), None)
            if target is None:
                print(f"[Vision] Cube_{cube_index} not found!")
                return False
        else:
            ee_pos, _ = self.franka.end_effector.get_world_pose()
            target = min(detected, key=lambda c: np.linalg.norm(c['position'][:2] - ee_pos[:2]))

        cube_pos = target['position']
        self.current_cube_index = target['index']

        print(f"[Vision] Selected: {target['color']} (Cube_{self.current_cube_index})")
        if 'detection_error' in target:
            self.performance['detection_errors'].append(target['detection_error'])
            print(f"[Vision] Detection error: {target['detection_error']*1000:.2f} mm")

        angle = np.arctan2(cube_pos[1], cube_pos[0])
        self._pre_grasp_pose = np.array([angle, -0.5, 0.0, -2.0, 0.0, 1.8, 0.8, 0.04, 0.04])
        self._grasp_pose = self._pre_grasp_pose.copy(); self._grasp_pose[3] -= 0.5
        self._lift_pose = self._grasp_pose.copy()
        self._lift_pose[1] += 0.3; self._lift_pose[3] += 0.6

        self.start_time = time.time()
        self.step_count = 0
        self.state = STATE_OPEN_GRIPPER

        return True

    def place(self, target_position):
        """place 상태머신 시작"""
        if not self.cube_attached:
            print("[Error] No cube attached!")
            return False

        if self.state != STATE_IDLE:
            print(f"[Vision] Busy: {self.state}")
            return False

        print("\n[Vision] Starting place (state machine)...")

        angle = np.arctan2(target_position[1], target_position[0])
        self._hover_pose = np.array([angle, -0.5, 0.0, -2.0, 0.0, 1.8, 0.8, 0.01, 0.01])
        self._place_pose = self._hover_pose.copy(); self._place_pose[3] -= 0.3
        self._retreat_pose = self._place_pose.copy()
        self._retreat_pose[1] += 0.5; self._retreat_pose[3] += 0.8

        self.start_time = time.time()
        self.step_count = 0
        self.state = STATE_PLACE_HOVER

        return True

    def is_busy(self):
        return self.state != STATE_IDLE

    # ------------------------------------------------------------------ #
    #  매 프레임 업데이트 (상태머신)
    # ------------------------------------------------------------------ #

    def update(self):
        """메인 루프에서 매 프레임 호출 - 블로킹 없이 상태 한 단계씩 진행"""

        if self.state == STATE_IDLE:
            return

        elif self.state == STATE_OPEN_GRIPPER:
            self.franka.gripper.open()
            self.step_count += 1
            if self.step_count >= 30:
                print("[Vision] Opening gripper...")
                self.gripper_closed = False
                self.step_count = 0
                self.state = STATE_PRE_GRASP

        elif self.state == STATE_PRE_GRASP:
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=self._pre_grasp_pose))
            except:
                pass
            self.step_count += 1
            if self.step_count >= 200:
                print("[Vision] Moving to pre-grasp... done")
                self.step_count = 0
                self.state = STATE_APPROACH

        elif self.state == STATE_APPROACH:
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=self._grasp_pose))
            except:
                pass
            self.step_count += 1
            if self.step_count >= 100:
                print("[Vision] Approaching... done")
                self.step_count = 0
                self.state = STATE_CLOSE_GRIPPER

        elif self.state == STATE_CLOSE_GRIPPER:
            self.franka.gripper.close()
            self.step_count += 1
            if self.step_count >= 60:
                print("[Vision] Closing gripper... done")
                self.gripper_closed = True
                self.step_count = 0
                self.state = STATE_ATTACH

        elif self.state == STATE_ATTACH:
            self.step_count += 1
            if self.step_count >= 20:
                attach_cube_to_ee(self.current_cube_index)
                self.cube_attached = True
                self.step_count = 0
                self.state = STATE_LIFT

        elif self.state == STATE_LIFT:
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=self._lift_pose))
            except:
                pass
            self.step_count += 1
            if self.step_count >= 150:
                print("[Vision] Lifting... done")
                elapsed = time.time() - self.start_time
                self.performance['grasp_times'].append(elapsed)
                print(f"\n[Phase 3] ✓ Vision Grasp Complete! (Time: {elapsed:.2f}s)")
                self.step_count = 0
                self.state = STATE_IDLE

        elif self.state == STATE_PLACE_HOVER:
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=self._hover_pose))
            except:
                pass
            self.step_count += 1
            if self.step_count >= 200:
                self.step_count = 0
                self.state = STATE_PLACE_DOWN

        elif self.state == STATE_PLACE_DOWN:
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=self._place_pose))
            except:
                pass
            self.step_count += 1
            if self.step_count >= 100:
                self.step_count = 0
                self.state = STATE_DETACH

        elif self.state == STATE_DETACH:
            self.step_count += 1
            if self.step_count >= 30:
                detach_cube(self.current_cube_index)
                self.cube_attached = False
                self.current_cube_index = None
                self.step_count = 0
                self.state = STATE_OPEN_AFTER

        elif self.state == STATE_OPEN_AFTER:
            self.franka.gripper.open()
            self.step_count += 1
            if self.step_count >= 40:
                self.gripper_closed = False
                self.step_count = 0
                self.state = STATE_RETREAT

        elif self.state == STATE_RETREAT:
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=self._retreat_pose))
            except:
                pass
            self.step_count += 1
            if self.step_count >= 120:
                elapsed = time.time() - self.start_time
                self.performance['place_times'].append(elapsed)
                print(f"[Phase 3] ✓ Place complete ({elapsed:.2f}s)")
                self.step_count = 0
                self.state = STATE_IDLE

    # ------------------------------------------------------------------ #
    #  성능
    # ------------------------------------------------------------------ #

    def get_performance_summary(self):
        perf = self.performance
        summary = {'total_grasps': len(perf['grasp_times']), 'total_places': len(perf['place_times'])}
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
        summary = self.get_performance_summary()
        print(f"\n{'='*60}")
        print("PHASE 3: VISION CONTROL - PERFORMANCE")
        print(f"{'='*60}")
        print(f"Total Grasps:  {summary.get('total_grasps', 0)}")
        print(f"Total Places:  {summary.get('total_places', 0)}")
        if 'avg_grasp_time' in summary:
            print(f"Grasp Time:    {summary['avg_grasp_time']:.2f}s ± {summary['std_grasp_time']:.2f}s")
        if 'avg_place_time' in summary:
            print(f"Place Time:    {summary['avg_place_time']:.2f}s ± {summary['std_place_time']:.2f}s")
        if 'avg_detection_error' in summary:
            print(f"Vision Error:  {summary['avg_detection_error']*1000:.2f} mm ± {summary['std_detection_error']*1000:.2f} mm")
        print(f"{'='*60}\n")
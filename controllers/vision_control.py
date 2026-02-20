# controllers/vision_control.py
"""
Phase 3: Vision-based Control (STABLE STACKING VERSION)
- 수직 스태킹을 위한 다중 관절 보정 적용
- 충돌 방지를 위한 High-Transit 경로 추가
- 물리 안정성을 위한 소프트 랜딩 적용
- [수정] calibrate_homography(): homography 없이 픽셀 먼저 수집 후 계산
"""

import sys
import os
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
STATE_TRANSIT       = "transit"        # 공중 안전 이동 상태
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

CUBE_HEIGHT = 0.05

# 홈 포지션 (카메라 시야 확보용)
HOME_POSE = np.array([0.0, -1.2, 0.0, -1.2, 0.0, 1.6, 0.7, 0.04, 0.04])

# 공중 대기용 안전 포즈 (이동 시 충돌 방지)
TRANSIT_POSE = np.array([0.0, -0.6, 0.0, -1.5, 0.0, 1.8, 0.8, 0.04, 0.04])

# 캘리브레이션용 색상 순서 및 범위 (색상 = 큐브 인덱스 고정)
COLOR_ORDER = ['red', 'green', 'blue', 'yellow']
COLOR_RANGES = {
    'red':    [([0,100,100],[10,255,255]), ([170,100,100],[180,255,255])],
    'green':  [([35,80,80],[85,255,255])],
    'blue':   [([100,100,100],[130,255,255])],
    'yellow': [([20,100,100],[35,255,255])],
}


class VisionController:
    def __init__(self, franka, world, camera):
        self.franka = franka
        self.world = world
        self.camera = camera
        self.controller = franka.get_articulation_controller()
        self.app = omni.kit.app.get_app()
        self.timeline = omni.timeline.get_timeline_interface()

        self.state = STATE_IDLE
        self.step_count = 0
        self.start_time = None
        self.current_cube_index = None
        self.cube_attached = False

        self.performance = {
            'grasp_times': [],
            'place_times': [],
            'detection_errors': [],
            'position_errors': []
        }

        self.camera.initialize()
        self.homography_matrix = None
        self.calibrated = False
        self.rgb_annot = None
        self._rep_camera_ready = False

    # ------------------------------------------------------------------ #
    #  카메라
    # ------------------------------------------------------------------ #

    def _ensure_camera_ready(self):
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
        self._ensure_camera_ready()
        for _ in range(5):
            self.world.step(render=True)
            self.app.update()
        rgb = self.rgb_annot.get_data()

        # Timeline 복구
        if not self.timeline.is_playing():
            print("[Vision] ⚠️  Timeline stopped, restarting...")
            self.timeline.play()
            for _ in range(10):
                self.world.step(render=True)
                self.app.update()

        if rgb is None or len(rgb) == 0:
            return None
        img = np.array(rgb)
        if img.shape[2] == 4:
            img = img[:, :, :3]
        if img.dtype != np.uint8:
            img = (np.clip(img, 0, 1) * 255).astype(np.uint8)
        return img

    # ------------------------------------------------------------------ #
    #  보조
    # ------------------------------------------------------------------ #

    def _interpolate_joint_trajectory(self, start_pose, end_pose, steps):
        trajectory = []
        for i in range(steps):
            t = i / max(steps - 1, 1)
            alpha = 3 * t**2 - 2 * t**3  # smoothstep
            trajectory.append(start_pose + alpha * (end_pose - start_pose))
        return trajectory

    def _get_current_joint_positions(self):
        js = self.franka.get_joint_positions()
        gs = self.franka.gripper.get_joint_positions()
        res = np.zeros(9)
        res[:7] = js[:7]
        res[7:] = gs
        return res

    def _pixel_to_world_homography(self, px, py):
        if self.homography_matrix is None:
            return np.array([0.5, 0.0, 0.025])
        wp = cv2.perspectiveTransform(
            np.array([[px, py]], dtype=np.float32).reshape(1, 1, 2),
            self.homography_matrix
        )
        return np.array([wp[0, 0, 0], wp[0, 0, 1], 0.025])

    def _get_color_mask(self, hsv, color_name):
        h, w = hsv.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        for lower, upper in COLOR_RANGES[color_name]:
            mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask

    # ------------------------------------------------------------------ #
    #  캘리브레이션 (수정: homography 없이 픽셀 먼저 수집)
    # ------------------------------------------------------------------ #

    def calibrate_homography(self):
        print("\n" + "="*60)
        print("HOMOGRAPHY CALIBRATION")
        print("="*60)

        img = self._get_camera_image()
        if img is None:
            print("[Vision] Failed to get camera image!")
            return False

        print(f"[Vision Debug] Image shape: {img.shape}")
        print(f"[Vision Debug] Range: [{img.min()}, {img.max()}]")

        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        # ── 1단계: homography 없이 픽셀 좌표만 수집 ──────────────────
        # (이 시점엔 homography_matrix=None이므로 위치 추정 불가 → 색상으로만 식별)
        pixel_pts = []
        world_pts = []

        for color_name in COLOR_ORDER:
            cube_idx = COLOR_ORDER.index(color_name)
            gt = get_cube_position(cube_idx)
            if gt is None:
                print(f"  [{color_name}] GT position not found, skip")
                continue

            mask = self._get_color_mask(hsv, color_name)
            cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
            if not cnts:
                print(f"  [{color_name}] No contour found")
                continue

            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) < 300:
                print(f"  [{color_name}] Contour too small ({cv2.contourArea(c):.0f})")
                continue

            M = cv2.moments(c)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            pixel_pts.append([cx, cy])
            world_pts.append(gt[:2])
            print(f"  {color_name} (Cube_{cube_idx}): "
                  f"Pixel=({cx}, {cy}) → World=({gt[0]:.3f}, {gt[1]:.3f})")

        if len(pixel_pts) < 4:
            print(f"[Vision] Need 4 cubes, found {len(pixel_pts)}")
            return False

        # ── 2단계: 픽셀 → 월드 homography 계산 ───────────────────────
        self.homography_matrix, _ = cv2.findHomography(
            np.array(pixel_pts, dtype=np.float32),
            np.array(world_pts,  dtype=np.float32)
        )

        if self.homography_matrix is None:
            print("[Vision] Homography computation failed!")
            return False

        # 검증
        total_err = 0.0
        for (px, py), wp in zip(pixel_pts, world_pts):
            pred = self._pixel_to_world_homography(px, py)[:2]
            total_err += np.linalg.norm(pred - np.array(wp))
        avg_err = total_err / len(pixel_pts)

        print(f"\n[Calibration] Homography matrix:\n{self.homography_matrix}")
        print(f"[Calibration] Average error: {avg_err*1000:.1f} mm")

        self.calibrated = True
        print("[Calibration] ✅ SUCCESS!" if avg_err < 0.05 else "[Calibration] ⚠️  High error")
        print("="*60 + "\n")
        return True

    # ------------------------------------------------------------------ #
    #  큐브 감지
    # ------------------------------------------------------------------ #

    def detect_cubes_from_camera(self, visualize=True, exclude_cubes=None):
        img = self._get_camera_image()
        if img is None:
            return []

        print(f"[Vision Debug] Image shape: {img.shape}")
        print(f"[Vision Debug] Original range: [{img.min()}, {img.max()}]")

        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        detected = []

        for color_name in COLOR_ORDER:
            mask = self._get_color_mask(hsv, color_name)
            cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
            if not cnts:
                continue
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) < 150:  # 쌓인 큐브는 작게 보일 수 있으므로 임계값 낮춤
                continue
            M = cv2.moments(c)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            color_idx = COLOR_ORDER.index(color_name)
            detected.append({
                'color': color_name,
                'index': color_idx,
                'position': self._pixel_to_world_homography(cx, cy),
                'pixel_pos': (cx, cy),
                'area': cv2.contourArea(c)
            })

        # GT 매칭 (exclude_cubes 제외)
        matched = []
        used_gt = set()
        for d in detected:
            best_i, best_d = None, 0.4
            for i in range(4):
                if exclude_cubes and i in exclude_cubes:
                    continue
                if i in used_gt:
                    continue
                gt = get_cube_position(i)
                if gt is None:
                    continue
                dist = np.linalg.norm(d['position'][:2] - gt[:2])
                if dist < best_d:
                    best_d, best_i = dist, i
            if best_i is not None:
                d['index'] = best_i
                d['detection_error'] = best_d
                matched.append(d)
                used_gt.add(best_i)

        print(f"[Vision] Detected {len(matched)} cubes:")
        for cube in matched:
            gt = get_cube_position(cube['index'])
            err = np.linalg.norm(cube['position'][:2] - gt[:2]) if gt is not None else 0
            print(f"  {cube['color']} (Cube_{cube['index']}): "
                  f"Pixel=({cube['pixel_pos'][0]}, {cube['pixel_pos'][1]}) | "
                  f"Vision=({cube['position'][0]:.2f}, {cube['position'][1]:.2f}) | "
                  f"GT=({gt[0]:.2f}, {gt[1]:.2f}) | Error={err*1000:.1f}mm")

        return matched

    # ------------------------------------------------------------------ #
    #  Grasp / Place 시작
    # ------------------------------------------------------------------ #

    def auto_grasp(self, cube_index=None, exclude_cubes=None):
        if not self.calibrated:
            print("[Vision] ⚠️  Not calibrated!")
            return False
        if self.state != STATE_IDLE:
            print(f"[Vision] Busy: {self.state}")
            return False

        print("\n[Phase 3: Vision] Starting vision-based grasp (state machine)...")
        det = self.detect_cubes_from_camera(exclude_cubes=exclude_cubes)
        if not det:
            print("[Vision] No cubes detected!")
            return False

        if cube_index is not None:
            target = next((c for c in det if c['index'] == cube_index), None)
            if target is None:
                print(f"[Vision] Cube_{cube_index} not found!")
                return False
        else:
            target = det[0]

        self.current_cube_index = target['index']
        pos = target['position']
        angle = np.arctan2(pos[1], pos[0])

        if 'detection_error' in target:
            self.performance['detection_errors'].append(target['detection_error'])
            print(f"[Vision] Selected: {target['color']} (Cube_{self.current_cube_index})")
            print(f"[Vision] Detection error: {target['detection_error']*1000:.2f} mm")

        # 경로 설정
        self._transit_pose = TRANSIT_POSE.copy()
        self._transit_pose[0] = angle
        self._pre_grasp_pose = np.array([angle, -0.45, 0.0, -2.1, 0.0, 1.8, 0.8, 0.04, 0.04])
        self._grasp_pose = self._pre_grasp_pose.copy()
        self._grasp_pose[3] -= 0.55
        self._lift_pose = self._grasp_pose.copy()
        self._lift_pose[1] -= 0.4
        self._lift_pose[3] += 0.8

        self.start_time = time.time()
        self.step_count = 0
        self.state = STATE_OPEN_GRIPPER
        return True

    def place(self, target_pos, stack_index=0):
        if not self.cube_attached:
            print("[Error] No cube attached!")
            return False
        if self.state != STATE_IDLE:
            print(f"[Vision] Busy: {self.state}")
            return False

        print("\n[Vision] Starting place (state machine)...")
        angle = np.arctan2(target_pos[1], target_pos[0])

        # ✅ 수직 스태킹 핵심 보정
        # 높이가 올라갈수록 어깨(j1)는 세우고(-), 팔꿈치(j3)는 펴야(+) 수직 유지
        j1_corr = stack_index * 0.045
        j3_corr = stack_index * 0.16

        print(f"[Vision] Stack index: {stack_index}, j1_corr: {j1_corr:.3f}, j3_corr: {j3_corr:.3f}")

        self._transit_pose = TRANSIT_POSE.copy()
        self._transit_pose[0] = angle
        self._hover_pose = np.array([angle, -0.5 + j1_corr, 0.0, -2.2 + j3_corr, 0.0, 1.8, 0.8, 0.01, 0.01])
        self._place_pose = self._hover_pose.copy()
        self._place_pose[3] -= 0.15  # 살짝 더 내려가서 안착

        self.start_time = time.time()
        self.step_count = 0
        self.state = STATE_TRANSIT
        return True

    def is_busy(self):
        return self.state != STATE_IDLE

    # ------------------------------------------------------------------ #
    #  상태머신 업데이트
    # ------------------------------------------------------------------ #

    def update(self):
        if self.state == STATE_IDLE:
            return

        curr_joints = self._get_current_joint_positions()

        if self.state == STATE_OPEN_GRIPPER:
            self.franka.gripper.open()
            self.step_count += 1
            if self.step_count >= 30:
                self.step_count = 0
                self.state = STATE_TRANSIT

        elif self.state == STATE_TRANSIT:
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(curr_joints, self._transit_pose, 120)
            if self.step_count < 120:
                self.controller.apply_action(ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 120:
                self.step_count = 0
                # cube_attached 여부로 grasp/place 분기
                self.state = STATE_PRE_GRASP if not self.cube_attached else STATE_PLACE_HOVER

        elif self.state == STATE_PRE_GRASP:
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(curr_joints, self._pre_grasp_pose, 100)
            if self.step_count < 100:
                self.controller.apply_action(ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 100:
                print("[Vision] Moving to pre-grasp... done")
                self.step_count = 0
                self.state = STATE_APPROACH

        elif self.state == STATE_APPROACH:
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(curr_joints, self._grasp_pose, 80)
            if self.step_count < 80:
                self.controller.apply_action(ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 80:
                print("[Vision] Approaching... done")
                self.step_count = 0
                self.state = STATE_CLOSE_GRIPPER

        elif self.state == STATE_CLOSE_GRIPPER:
            self.franka.gripper.close()
            self.step_count += 1
            if self.step_count >= 60:
                print("[Vision] Closing gripper... done")
                self.step_count = 0
                self.state = STATE_ATTACH

        elif self.state == STATE_ATTACH:
            self.step_count += 1
            if self.step_count >= 10:
                attach_cube_to_ee(self.current_cube_index)
                self.cube_attached = True
                print(f"[Vision] Attached Cube_{self.current_cube_index}")
                self.step_count = 0
                self.state = STATE_LIFT

        elif self.state == STATE_LIFT:
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(curr_joints, self._lift_pose, 120)
            if self.step_count < 120:
                self.controller.apply_action(ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 120:
                elapsed = time.time() - self.start_time
                self.performance['grasp_times'].append(elapsed)
                print(f"[Vision] Lifting... done")
                print(f"\n[Phase 3] ✓ Vision Grasp Complete! (Time: {elapsed:.2f}s)")
                self.step_count = 0
                self.state = STATE_IDLE

        elif self.state == STATE_PLACE_HOVER:
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(curr_joints, self._hover_pose, 150)
            if self.step_count < 150:
                self.controller.apply_action(ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 150:
                self.step_count = 0
                self.state = STATE_PLACE_DOWN

        elif self.state == STATE_PLACE_DOWN:
            # ✅ 소프트 랜딩 (200 스텝)
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(curr_joints, self._place_pose, 200)
            if self.step_count < 200:
                self.controller.apply_action(ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 200:
                self.step_count = 0
                self.state = STATE_DETACH

        elif self.state == STATE_DETACH:
            self.step_count += 1
            if self.step_count >= 50:  # 물리 안정화 대기
                detach_cube(self.current_cube_index)
                self.cube_attached = False
                self.current_cube_index = None
                self.step_count = 0
                self.state = STATE_OPEN_AFTER

        elif self.state == STATE_OPEN_AFTER:
            self.franka.gripper.open()
            self.step_count += 1
            if self.step_count >= 50:
                self.step_count = 0
                self.state = STATE_RETREAT

        elif self.state == STATE_RETREAT:
            # ✅ HOME_POSE로 복귀 → 카메라 시야 확보
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(curr_joints, HOME_POSE, 150)
            if self.step_count < 150:
                self.controller.apply_action(ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 150:
                elapsed = time.time() - self.start_time
                self.performance['place_times'].append(elapsed)
                print(f"[Phase 3] ✓ Place complete (home restored) ({elapsed:.2f}s)")
                self.step_count = 0
                self.state = STATE_IDLE

    # ------------------------------------------------------------------ #
    #  성능
    # ------------------------------------------------------------------ #

    def get_performance_summary(self):
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
# controllers/vision_control.py
"""
Phase 3: Vision-based Control v11

[v11 핵심 수정]
  1. Depth annotator를 기존 replicator 카메라에 추가
     - 기존: RGB annotator만 → 색상 감지만 가능
     - v11: 같은 render product에 depth annotator 추가
            → 색상 감지(4개 다 잡힘) + depth로 3D 위치 추정 동시 가능

  2. Depth 좌표 변환 직접 구현
     - SimulationCamera.pixel_depth_to_world() 좌표계 불일치 문제 해결
     - 카메라 파라미터(position, look_at, fov)를 직접 사용해 변환

  3. 위치 추정 우선순위
     1순위: Depth → 오차 < 100mm 이면 사용
     2순위: Homography (4개 감지 시)
     3순위: GT (RigidPrim 실시간)

[v10 유지]
  - calibrated 강제 통과 (GT fallback 보장)
  - j1 동적 계산, GRASP_DELTA_J3=-0.55
  - STACK_HOVER_POSES, PLACE_SETTLE, DETACH_WAIT
"""

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
    get_cube_position,
)

# ------------------------------------------------------------------ #
#  상태 정의
# ------------------------------------------------------------------ #
STATE_IDLE           = "idle"
STATE_OPEN_GRIPPER   = "open_gripper"
STATE_TRANSIT        = "transit"
STATE_PRE_GRASP      = "pre_grasp"
STATE_APPROACH       = "approach"
STATE_CLOSE_GRIPPER  = "close_gripper"
STATE_ATTACH         = "attach"
STATE_LIFT           = "lift"
STATE_PLACE_HOVER    = "place_hover"
STATE_PLACE_DOWN     = "place_down"
STATE_PLACE_SETTLE   = "place_settle"
STATE_DETACH         = "detach"
STATE_OPEN_AFTER     = "open_after"
STATE_RETREAT        = "retreat"

# ------------------------------------------------------------------ #
#  환경 수치
# ------------------------------------------------------------------ #
TABLE_TOP_Z  = 0.50
CUBE_SCALE   = 0.05
CUBE_HALF    = 0.025
CUBE_REST_Z  = TABLE_TOP_Z + CUBE_HALF   # 0.525m

# Depth 신뢰 임계값
DEPTH_TRUST_MM = 100.0   # mm

# ------------------------------------------------------------------ #
#  카메라 파라미터 (replicator 카메라 기준)
# ------------------------------------------------------------------ #
CAM_POSITION  = np.array([1.0, 0.0, 1.5])
CAM_LOOK_AT   = np.array([0.5, 0.0, 0.0])
CAM_RES_W     = 1024
CAM_RES_H     = 768
CAM_FOCAL_LEN = 12.0   # rep.create.camera focal_length

# ------------------------------------------------------------------ #
#  공통 포즈
# ------------------------------------------------------------------ #
HOME_POSE    = np.array([0.0, -1.2, 0.0, -1.2, 0.0, 1.6, 0.7, 0.04, 0.04])
TRANSIT_POSE = np.array([0.0, -0.6, 0.0, -1.5, 0.0, 1.8, 0.8, 0.04, 0.04])

# ------------------------------------------------------------------ #
#  색상 범위 (HSV)
# ------------------------------------------------------------------ #
COLOR_ORDER = ['red', 'green', 'blue', 'yellow']
COLOR_RANGES = {
    'red':    [([0,100,100],[10,255,255]), ([170,100,100],[180,255,255])],
    'green':  [([35,80,80],[85,255,255])],
    'blue':   [([100,100,100],[130,255,255])],
    'yellow': [([20,100,100],[35,255,255])],
}

# ------------------------------------------------------------------ #
#  층별 Hover 포즈
# ------------------------------------------------------------------ #
STACK_HOVER_POSES = {
    0: np.array([0.0, -0.55, 0.0, -2.22, 0.0, 1.90, 0.8, 0.01, 0.01]),
    1: np.array([0.0, -0.50, 0.0, -2.07, 0.0, 1.95, 0.8, 0.01, 0.01]),
    2: np.array([0.0, -0.45, 0.0, -1.92, 0.0, 2.05, 0.8, 0.01, 0.01]),
    3: np.array([0.0, -0.40, 0.0, -1.77, 0.0, 2.15, 0.8, 0.01, 0.01]),
}
STACK_PLACE_DELTA_J3 = -0.08

# ------------------------------------------------------------------ #
#  Attach/Detach 파라미터
# ------------------------------------------------------------------ #
VELOCITY_THRESHOLD       = 0.005
ATTACH_MIN_WAIT_FRAMES   = 60
ATTACH_TIMEOUT_FRAMES    = 100
ATTACH_STABLE_COUNT_NEED = 5
DETACH_WAIT_FRAMES       = 50
PLACE_SETTLE_FRAMES      = 40

# ------------------------------------------------------------------ #
#  Grasp 파라미터
# ------------------------------------------------------------------ #
GRASP_PRE_J3   = -1.9
GRASP_DELTA_J3 = -0.55

J1_BASE     = -0.45
J1_DIST_REF = 0.50
J1_GAIN     = 0.60


def _calc_j1(dist_xy):
    j1 = J1_BASE + (dist_xy - J1_DIST_REF) * J1_GAIN
    return float(np.clip(j1, -0.8, -0.2))


class VisionController:
    def __init__(self, franka, world, camera):
        self.franka     = franka
        self.world      = world
        self.camera     = camera   # 기존 Isaac Sensor Camera (호환용)
        self.controller = franka.get_articulation_controller()
        self.app        = omni.kit.app.get_app()
        self.timeline   = omni.timeline.get_timeline_interface()

        self.state              = STATE_IDLE
        self.step_count         = 0
        self.start_time         = None
        self.current_cube_index = None
        self.cube_attached      = False
        self._stable_count      = 0

        self.performance = {
            'grasp_times':      [],
            'place_times':      [],
            'detection_errors': [],
            'depth_used':       0,
            'gt_fallback':      0,
        }

        self.homography_matrix  = None
        self.calibrated         = False

        # replicator 카메라 (RGB + Depth 동시)
        self._rep_ready    = False
        self.rgb_annot     = None
        self.depth_annot   = None

        print(f"[VisionController v11]")
        print(f"  큐브 안착 Z   = {CUBE_REST_Z:.3f}m")
        print(f"  Depth 신뢰    = {DEPTH_TRUST_MM:.0f}mm 이내")

    # ------------------------------------------------------------------ #
    #  [v11] replicator 카메라 초기화 - RGB + Depth 동시
    # ------------------------------------------------------------------ #

    def _ensure_camera_ready(self):
        if self._rep_ready:
            return
        print("[Vision] Setting up replicator camera (RGB + Depth)...")

        rep_cam = rep.create.camera(
            position=CAM_POSITION.tolist(),
            look_at=CAM_LOOK_AT.tolist(),
            focal_length=CAM_FOCAL_LEN,
        )
        rp = rep.create.render_product(rep_cam, (CAM_RES_W, CAM_RES_H))

        # RGB annotator
        self.rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self.rgb_annot.attach([rp])

        # [v11] Depth annotator - 같은 render product에 추가
        self.depth_annot = rep.AnnotatorRegistry.get_annotator(
            "distance_to_camera"
        )
        self.depth_annot.attach([rp])

        self._rep_ready = True
        print("[Vision] Replicator camera ready (RGB + Depth).")

    # ------------------------------------------------------------------ #
    #  프레임 취득
    # ------------------------------------------------------------------ #

    def _get_frames(self):
        """RGB + Depth 동시 취득"""
        self._ensure_camera_ready()

        for _ in range(5):
            self.world.step(render=True)
            self.app.update()

        if not self.timeline.is_playing():
            print("[Vision] ⚠️  Timeline stopped, restarting...")
            self.timeline.play()
            for _ in range(10):
                self.world.step(render=True)
                self.app.update()

        # RGB
        rgb_data = self.rgb_annot.get_data()
        rgb = None
        if rgb_data is not None and len(rgb_data) > 0:
            rgb = np.array(rgb_data)
            if rgb.ndim == 3 and rgb.shape[2] == 4:
                rgb = rgb[:, :, :3]
            if rgb.dtype != np.uint8:
                rgb = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)

        # Depth
        depth_data = self.depth_annot.get_data()
        depth = None
        if depth_data is not None and len(depth_data) > 0:
            depth = np.array(depth_data, dtype=np.float32)

        return rgb, depth

    def _get_color_mask(self, hsv, color_name):
        h, w = hsv.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        for lower, upper in COLOR_RANGES[color_name]:
            mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
        kernel = np.ones((5, 5), np.uint8)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # ------------------------------------------------------------------ #
    #  [v11] Depth 기반 3D 위치 변환
    #  카메라 position/look_at으로 직접 회전행렬 계산
    # ------------------------------------------------------------------ #

    def _depth_pixel_to_world(self, u, v, depth_map):
        """
        replicator 카메라 파라미터 기반 픽셀→world 변환.
        CAM_POSITION, CAM_LOOK_AT, CAM_FOCAL_LEN 사용.
        """
        if depth_map is None:
            return None
        try:
            h, w = depth_map.shape[:2]
            u_c = int(np.clip(u, 0, w - 1))
            v_c = int(np.clip(v, 0, h - 1))

            # 주변 3x3 median으로 노이즈 제거
            u0 = max(0, u_c-1); u1 = min(w, u_c+2)
            v0 = max(0, v_c-1); v1 = min(h, v_c+2)
            d = float(np.median(depth_map[v0:v1, u0:u1]))

            if not (0.05 < d < 5.0):
                return None

            # 카메라 내부 파라미터 (pinhole 근사)
            # focal_length(mm) → pixel: Isaac Sim은 sensor_size 36mm 기준
            sensor_w = 36.0
            fx = (CAM_FOCAL_LEN / sensor_w) * CAM_RES_W
            fy = (CAM_FOCAL_LEN / sensor_w) * CAM_RES_H
            cx = CAM_RES_W / 2.0
            cy = CAM_RES_H / 2.0

            # 카메라 좌표계 (OpenCV: X오른쪽, Y아래, Z앞)
            x_cam = (u_c - cx) * d / fx
            y_cam = (v_c - cy) * d / fy
            z_cam = d

            # 카메라→월드 회전행렬
            forward = CAM_LOOK_AT - CAM_POSITION
            forward = forward / np.linalg.norm(forward)

            world_up = np.array([0.0, 0.0, 1.0])
            right = np.cross(world_up, forward)
            if np.linalg.norm(right) < 1e-6:
                right = np.array([1.0, 0.0, 0.0])
            else:
                right = right / np.linalg.norm(right)

            up = np.cross(forward, right)
            up = up / np.linalg.norm(up)

            # 카메라 좌표 → 월드 좌표
            # Isaac Sim replicator: X=right, Y=up(반전), Z=forward
            point_cam = np.array([x_cam, -y_cam, z_cam])
            R_mat = np.column_stack([right, up, forward])
            point_world = CAM_POSITION + R_mat @ point_cam

            # Z는 큐브 안착 높이로 고정
            point_world[2] = CUBE_REST_Z
            return point_world

        except Exception as e:
            print(f"[Vision] Depth 변환 오류: {e}")
            return None

    def _pixel_to_world_homography(self, u, v):
        if self.homography_matrix is None:
            return None
        try:
            wp = cv2.perspectiveTransform(
                np.array([[u, v]], dtype=np.float32).reshape(1, 1, 2),
                self.homography_matrix
            )
            return np.array([wp[0,0,0], wp[0,0,1], CUBE_REST_Z])
        except Exception:
            return None

    def _best_position(self, u, v, depth_map, cube_index):
        """
        우선순위: Depth → Homography → GT
        """
        gt = get_cube_position(cube_index)

        # 1순위: Depth
        pos_d = self._depth_pixel_to_world(u, v, depth_map)
        if pos_d is not None and gt is not None:
            err_mm = np.linalg.norm(pos_d[:2] - gt[:2]) * 1000
            if err_mm < DEPTH_TRUST_MM:
                self.performance['depth_used'] += 1
                return pos_d, 'depth', err_mm / 1000
            else:
                print(f"[Vision] Depth 오차 {err_mm:.0f}mm > "
                      f"{DEPTH_TRUST_MM:.0f}mm → Homography 시도")

        # 2순위: Homography
        pos_h = self._pixel_to_world_homography(u, v)
        if pos_h is not None and gt is not None:
            err_mm = np.linalg.norm(pos_h[:2] - gt[:2]) * 1000
            if err_mm < DEPTH_TRUST_MM:
                return pos_h, 'homography', err_mm / 1000

        # 3순위: GT
        if gt is not None:
            self.performance['gt_fallback'] += 1
            return gt, 'gt', 0.0

        return None, None, 0.0

    # ------------------------------------------------------------------ #
    #  캘리브레이션
    # ------------------------------------------------------------------ #

    def calibrate_homography(self):
        print("\n" + "="*60)
        print("HOMOGRAPHY CALIBRATION")
        print("="*60)

        rgb, depth = self._get_frames()
        if rgb is None:
            print("[Vision] 카메라 실패 → GT 모드로 진행")
            self.calibrated = True
            return True

        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        pixel_pts, world_pts = [], []

        for color_name in COLOR_ORDER:
            cube_idx = COLOR_ORDER.index(color_name)
            gt = get_cube_position(cube_idx)
            if gt is None:
                continue

            mask = self._get_color_mask(hsv, color_name)
            cnts = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
            if not cnts or cv2.contourArea(max(cnts, key=cv2.contourArea)) < 300:
                print(f"  {color_name} (Cube_{cube_idx}): 미감지")
                continue

            c  = max(cnts, key=cv2.contourArea)
            M  = cv2.moments(c)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            pos_d = self._depth_pixel_to_world(cx, cy, depth)
            err_d = (np.linalg.norm(pos_d[:2] - gt[:2]) * 1000
                     if pos_d is not None else -1)

            pixel_pts.append([cx, cy])
            world_pts.append(gt[:2])

            depth_str = (f"Depth=({pos_d[0]:.3f},{pos_d[1]:.3f}) "
                         f"Err={err_d:.0f}mm"
                         if pos_d is not None else "Depth=N/A")
            print(f"  {color_name} (Cube_{cube_idx}): "
                  f"Pixel=({cx},{cy}) | GT=({gt[0]:.3f},{gt[1]:.3f}) | "
                  f"{depth_str}")

        if len(pixel_pts) >= 4:
            self.homography_matrix, _ = cv2.findHomography(
                np.array(pixel_pts, dtype=np.float32),
                np.array(world_pts,  dtype=np.float32)
            )
            status = "완료" if self.homography_matrix is not None else "실패"
            print(f"[Calibration] Homography {status}")
        else:
            print(f"[Calibration] 감지 {len(pixel_pts)}개 "
                  f"(4개 필요, Homography 생략)")

        self.calibrated = True
        print("[Calibration] ✅ calibrated=True")
        print("="*60 + "\n")
        return True

    # ------------------------------------------------------------------ #
    #  큐브 감지
    # ------------------------------------------------------------------ #

    def detect_cubes_from_camera(self, exclude_cubes=None):
        rgb, depth = self._get_frames()
        if rgb is None:
            return self._gt_fallback(exclude_cubes)

        hsv      = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        detected = []

        for i in range(4):
            if exclude_cubes and i in exclude_cubes:
                continue

            color_name = COLOR_ORDER[i]
            mask       = self._get_color_mask(hsv, color_name)
            cnts       = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

            if not cnts or cv2.contourArea(max(cnts, key=cv2.contourArea)) < 100:
                gt = get_cube_position(i)
                if gt is not None:
                    detected.append({
                        'color': color_name, 'index': i,
                        'position': gt, 'pixel_pos': None,
                        'method': 'gt_no_detect', 'detection_error': 0.0,
                    })
                    self.performance['gt_fallback'] += 1
                continue

            c  = max(cnts, key=cv2.contourArea)
            M  = cv2.moments(c)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            pos, method, err = self._best_position(cx, cy, depth, i)
            if pos is not None:
                detected.append({
                    'color': color_name, 'index': i,
                    'position': pos, 'pixel_pos': (cx, cy),
                    'method': method, 'detection_error': err,
                })

        print(f"[Vision] Detected {len(detected)} cubes:")
        for cube in detected:
            p   = cube['position']
            pix = cube['pixel_pos']
            m   = cube['method']
            err = cube['detection_error']
            pix_str = f"({pix[0]},{pix[1]})" if pix else "N/A"
            print(f"  {cube['color']} (Cube_{cube['index']}): "
                  f"Pixel={pix_str} | "
                  f"Pos=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) | "
                  f"[{m}] Err={err*1000:.1f}mm")

        return detected

    def _gt_fallback(self, exclude_cubes=None):
        fallback = []
        for i in range(4):
            if exclude_cubes and i in exclude_cubes:
                continue
            pos = get_cube_position(i)
            if pos is not None:
                fallback.append({
                    'color': COLOR_ORDER[i], 'index': i,
                    'position': pos, 'pixel_pos': None,
                    'method': 'gt_fallback', 'detection_error': 0.0,
                })
        print(f"[Vision] GT Fallback: {len(fallback)} cubes")
        return fallback

    # ------------------------------------------------------------------ #
    #  보조
    # ------------------------------------------------------------------ #

    def _interpolate_joint_trajectory(self, start_pose, end_pose, steps):
        trajectory = []
        for i in range(steps):
            t     = i / max(steps - 1, 1)
            alpha = 3 * t**2 - 2 * t**3
            trajectory.append(start_pose + alpha * (end_pose - start_pose))
        return trajectory

    def _get_current_joint_positions(self):
        js  = self.franka.get_joint_positions()
        gs  = self.franka.gripper.get_joint_positions()
        res = np.zeros(9)
        res[:7] = js[:7]; res[7:] = gs
        return res

    def _get_current_joint_velocities(self):
        jv  = self.franka.get_joint_velocities()
        res = np.zeros(9)
        res[:7] = jv[:7]
        return res

    # ------------------------------------------------------------------ #
    #  Grasp 시작
    # ------------------------------------------------------------------ #

    def auto_grasp(self, cube_index=None, exclude_cubes=None):
        if not self.calibrated:
            print("[Vision] 캘리브레이션 미완료 → GT 모드 강제 진행")
            self.calibrated = True
        if self.state != STATE_IDLE:
            print(f"[Vision] Busy: {self.state}")
            return False

        print("\n[Phase 3: Vision] Starting grasp...")
        det = self.detect_cubes_from_camera(exclude_cubes=exclude_cubes)
        if not det:
            print("[Vision] No cubes detected!")
            return False

        if cube_index is not None:
            target = next((c for c in det if c['index'] == cube_index), None)
            if target is None:
                pos = get_cube_position(cube_index)
                if pos is None:
                    return False
                target = {
                    'color': COLOR_ORDER[cube_index], 'index': cube_index,
                    'position': pos, 'method': 'gt_fallback',
                    'detection_error': 0.0
                }
        else:
            target = det[0]

        self.current_cube_index = target['index']
        pos     = target['position']
        angle   = np.arctan2(pos[1], pos[0])
        dist_xy = np.sqrt(pos[0]**2 + pos[1]**2)
        j1_grasp = _calc_j1(dist_xy)

        self.performance['detection_errors'].append(
            target.get('detection_error', 0.0))

        print(f"[Vision] Target: {target['color']} "
              f"(Cube_{self.current_cube_index}) [{target['method']}]")
        print(f"[Vision] Pos: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        print(f"[Vision] dist_xy={dist_xy:.3f}m → j1={j1_grasp:.3f}")

        self._transit_pose = TRANSIT_POSE.copy()
        self._transit_pose[0] = angle

        self._pre_grasp_pose = np.array(
            [angle, j1_grasp, 0.0, GRASP_PRE_J3, 0.0, 1.8, 0.8, 0.04, 0.04])

        self._grasp_pose = self._pre_grasp_pose.copy()
        self._grasp_pose[3] += GRASP_DELTA_J3

        self._lift_pose = self._grasp_pose.copy()
        self._lift_pose[1] -= 0.4
        self._lift_pose[3] += 0.8

        print(f"[Vision] j3: pre={self._pre_grasp_pose[3]:.3f}, "
              f"grasp={self._grasp_pose[3]:.3f}")

        self.start_time    = time.time()
        self.step_count    = 0
        self._stable_count = 0
        self.state = STATE_OPEN_GRIPPER
        return True

    # ------------------------------------------------------------------ #
    #  Place 시작
    # ------------------------------------------------------------------ #

    def place(self, target_pos, stack_index=0):
        if not self.cube_attached:
            print("[Error] No cube attached!")
            return False
        if self.state != STATE_IDLE:
            print(f"[Vision] Busy: {self.state}")
            return False

        print("\n[Vision] Starting place...")
        angle = np.arctan2(target_pos[1], target_pos[0])

        layer      = min(stack_index, max(STACK_HOVER_POSES.keys()))
        hover_pose = STACK_HOVER_POSES[layer].copy()
        hover_pose[0] = angle

        place_pose = hover_pose.copy()
        place_pose[3] += STACK_PLACE_DELTA_J3

        target_z = TABLE_TOP_Z + CUBE_SCALE * (stack_index + 0.5)
        print(f"[Vision] Stack={stack_index} | Layer={layer} | angle={angle:.3f}")
        print(f"[Vision] 목표 Z={target_z:.4f}m | "
              f"Hover j3={hover_pose[3]:.3f}, Place j3={place_pose[3]:.3f}")

        self._transit_pose = TRANSIT_POSE.copy()
        self._transit_pose[0] = angle
        self._hover_pose   = hover_pose
        self._place_pose   = place_pose

        self.start_time    = time.time()
        self.step_count    = 0
        self._stable_count = 0
        self.state = STATE_TRANSIT
        return True

    def is_busy(self):
        return self.state != STATE_IDLE

    # ------------------------------------------------------------------ #
    #  상태머신
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
                self._traj = self._interpolate_joint_trajectory(
                    curr_joints, self._transit_pose, 120)
            if self.step_count < 120:
                self.controller.apply_action(
                    ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 120:
                self.step_count = 0
                self.state = (STATE_PRE_GRASP
                              if not self.cube_attached else STATE_PLACE_HOVER)

        elif self.state == STATE_PRE_GRASP:
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(
                    curr_joints, self._pre_grasp_pose, 100)
            if self.step_count < 100:
                self.controller.apply_action(
                    ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 100:
                print("[Vision] Pre-grasp... done")
                self.step_count = 0
                self.state = STATE_APPROACH

        elif self.state == STATE_APPROACH:
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(
                    curr_joints, self._grasp_pose, 80)
            if self.step_count < 80:
                self.controller.apply_action(
                    ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 80:
                print("[Vision] Approach... done")
                self.step_count = 0
                self.state = STATE_CLOSE_GRIPPER

        elif self.state == STATE_CLOSE_GRIPPER:
            self.franka.gripper.close()
            self.step_count += 1
            if self.step_count >= 120:
                print("[Vision] Close gripper... done")
                self.step_count    = 0
                self._stable_count = 0
                self.state = STATE_ATTACH

        elif self.state == STATE_ATTACH:
            self.franka.gripper.close()
            self.step_count += 1
            if self.step_count < ATTACH_MIN_WAIT_FRAMES:
                return

            vel     = self._get_current_joint_velocities()
            max_vel = np.max(np.abs(vel[:7]))

            if max_vel < VELOCITY_THRESHOLD:
                self._stable_count += 1
            else:
                self._stable_count = 0

            is_stable  = self._stable_count >= ATTACH_STABLE_COUNT_NEED
            is_timeout = self.step_count >= ATTACH_TIMEOUT_FRAMES

            if is_stable or is_timeout:
                if is_timeout and not is_stable:
                    print(f"[Vision] ⚠️  Attach timeout "
                          f"(vel={max_vel:.4f}, stable={self._stable_count}f)")
                else:
                    print(f"[Vision] ✅ Stable "
                          f"(vel={max_vel:.4f}, {self._stable_count}f)")
                attach_cube_to_ee(self.current_cube_index)
                self.cube_attached = True
                print(f"[Vision] Attached Cube_{self.current_cube_index}")
                self.step_count    = 0
                self._stable_count = 0
                self.state = STATE_LIFT

        elif self.state == STATE_LIFT:
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(
                    curr_joints, self._lift_pose, 120)
            if self.step_count < 120:
                self.controller.apply_action(
                    ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 120:
                elapsed = time.time() - self.start_time
                self.performance['grasp_times'].append(elapsed)
                print(f"[Phase 3] ✓ Grasp Complete! ({elapsed:.2f}s)")
                self.step_count = 0
                self.state = STATE_IDLE

        elif self.state == STATE_PLACE_HOVER:
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(
                    curr_joints, self._hover_pose, 150)
            if self.step_count < 150:
                self.controller.apply_action(
                    ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 150:
                self.step_count = 0
                self.state = STATE_PLACE_DOWN

        elif self.state == STATE_PLACE_DOWN:
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(
                    curr_joints, self._place_pose, 200)
            if self.step_count < 200:
                self.controller.apply_action(
                    ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 200:
                print("[Vision] Place down... done")
                self.step_count = 0
                self.state = STATE_PLACE_SETTLE

        elif self.state == STATE_PLACE_SETTLE:
            self.step_count += 1
            if self.step_count >= PLACE_SETTLE_FRAMES:
                print("[Vision] Settle done → detach")
                self.step_count = 0
                self.state = STATE_DETACH

        elif self.state == STATE_DETACH:
            self.step_count += 1
            if self.step_count >= DETACH_WAIT_FRAMES:
                detach_cube(self.current_cube_index)
                self.cube_attached      = False
                self.current_cube_index = None
                self.step_count         = 0
                self.state = STATE_OPEN_AFTER

        elif self.state == STATE_OPEN_AFTER:
            self.franka.gripper.open()
            self.step_count += 1
            if self.step_count >= 50:
                self.step_count = 0
                self.state = STATE_RETREAT

        elif self.state == STATE_RETREAT:
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(
                    curr_joints, HOME_POSE, 150)
            if self.step_count < 150:
                self.controller.apply_action(
                    ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 150:
                elapsed = time.time() - self.start_time
                self.performance['place_times'].append(elapsed)
                print(f"[Phase 3] ✓ Place complete ({elapsed:.2f}s)")
                self.step_count = 0
                self.state = STATE_IDLE

    # ------------------------------------------------------------------ #
    #  성능
    # ------------------------------------------------------------------ #

    def get_performance_summary(self):
        perf    = self.performance
        summary = {
            'total_grasps': len(perf['grasp_times']),
            'total_places': len(perf['place_times']),
            'depth_used':   perf['depth_used'],
            'gt_fallback':  perf['gt_fallback'],
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
        print(f"Total Grasps : {summary.get('total_grasps', 0)}")
        print(f"Total Places : {summary.get('total_places', 0)}")
        print(f"Depth 사용   : {summary.get('depth_used', 0)}회")
        print(f"GT Fallback  : {summary.get('gt_fallback', 0)}회")
        if 'avg_grasp_time' in summary:
            print(f"Grasp Time   : {summary['avg_grasp_time']:.2f}s "
                  f"± {summary['std_grasp_time']:.2f}s")
        if 'avg_place_time' in summary:
            print(f"Place Time   : {summary['avg_place_time']:.2f}s "
                  f"± {summary['std_place_time']:.2f}s")
        if 'avg_detection_error' in summary:
            print(f"Vision Error : "
                  f"{summary['avg_detection_error']*1000:.2f} mm "
                  f"± {summary['std_detection_error']*1000:.2f} mm")
        print(f"{'='*60}\n")
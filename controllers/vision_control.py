# controllers/vision_control.py
"""
Phase 3: Vision-based Control v12

[v12 변경사항]
  1. ROS2 퍼블리시 통합
     - 큐브 감지 결과 → /isaac/cube_detections (PoseArray)
     - 스태킹 상태   → /isaac/stack_status (String)
     - EE 포즈       → /isaac/ee_pose (PoseStamped)

  2. ROS2 서브스크라이브
     - /stack_command → "start" / "stop" / "reset"
     - /target_pose   → 외부에서 place 위치 지정

  3. j5 원상복구 (0.0 고정)
     - j5 동적 계산이 오히려 큐브 날아가는 현상 유발

[v11 유지]
  - RGB+Depth replicator 카메라
  - sensor_w=20.955 (Depth 정확도)
  - Homography → GT fallback
  - Blue/Yellow dist=0.579m
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
DEPTH_TRUST_MM = 100.0

# ------------------------------------------------------------------ #
#  카메라 파라미터
# ------------------------------------------------------------------ #
CAM_POSITION  = np.array([1.0, 0.0, 1.5])
CAM_LOOK_AT   = np.array([0.5, 0.0, 0.0])
CAM_RES_W     = 1024
CAM_RES_H     = 768
CAM_FOCAL_LEN = 12.0

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
J1_BASE        = -0.45
J1_DIST_REF    = 0.50
J1_GAIN        = 0.60


def _calc_j1(dist_xy):
    return float(np.clip(
        J1_BASE + (dist_xy - J1_DIST_REF) * J1_GAIN, -0.8, -0.2))


class VisionController:
    def __init__(self, franka, world, ros2_bridge=None):
        """
        Args:
            franka     : Franka 로봇 인스턴스
            world      : Isaac Sim World
            ros2_bridge: setup_world()에서 반환된 ROS2 bridge dict
        """
        self.franka      = franka
        self.world       = world
        self.ros2_bridge = ros2_bridge or {'enabled': False}
        self.controller  = franka.get_articulation_controller()
        self.app         = omni.kit.app.get_app()
        self.timeline    = omni.timeline.get_timeline_interface()

        self.state              = STATE_IDLE
        self.step_count         = 0
        self.start_time         = None
        self.current_cube_index = None
        self.cube_attached      = False
        self._stable_count      = 0
        self._external_command  = None   # ROS2에서 받은 명령
        self._external_target   = None   # ROS2에서 받은 place 위치

        self.performance = {
            'grasp_times':      [],
            'place_times':      [],
            'detection_errors': [],
            'depth_used':       0,
            'gt_fallback':      0,
        }

        self.homography_matrix = None
        self.calibrated        = False
        self._rep_ready        = False
        self.rgb_annot         = None
        self.depth_annot       = None

        # ROS2 퍼블리셔/서브스크라이버
        self._ros2_pub  = {}
        self._ros2_sub  = {}
        self._ros2_node = None

        self._init_ros2_comms()

        print(f"[VisionController v12]")
        print(f"  큐브 안착 Z  = {CUBE_REST_Z:.3f}m")
        print(f"  Depth 신뢰   = {DEPTH_TRUST_MM:.0f}mm")
        ros2_str = "✅ 활성화" if self.ros2_bridge.get('enabled') else "⚠️  비활성화"
        print(f"  ROS2 Bridge  = {ros2_str}")

    # ------------------------------------------------------------------ #
    #  [v12] ROS2 통신 초기화
    # ------------------------------------------------------------------ #

    def _init_ros2_comms(self):
        """ROS2 퍼블리셔/서브스크라이버 초기화"""
        if not self.ros2_bridge.get('enabled'):
            print("[ROS2] Bridge 비활성화 → ROS2 통신 스킵")
            return

        try:
            import rclpy
            from rclpy.node import Node
            from geometry_msgs.msg import PoseArray, Pose, PoseStamped
            from std_msgs.msg import String

            # ROS2 노드 생성
            self._ros2_node = rclpy.create_node('isaac_vision_controller')

            # ── 퍼블리셔 ─────────────────────────────────────────────
            # 큐브 감지 결과
            self._ros2_pub['cube_detections'] = self._ros2_node.create_publisher(
                PoseArray, '/isaac/cube_detections', 10)

            # 스태킹 상태 문자열
            self._ros2_pub['stack_status'] = self._ros2_node.create_publisher(
                String, '/isaac/stack_status', 10)

            # EE 현재 포즈
            self._ros2_pub['ee_pose'] = self._ros2_node.create_publisher(
                PoseStamped, '/isaac/ee_pose', 10)

            # ── 서브스크라이버 ───────────────────────────────────────
            # 외부 스태킹 명령 ("start" / "stop" / "reset")
            self._ros2_sub['stack_command'] = self._ros2_node.create_subscription(
                String,
                '/stack_command',
                self._on_stack_command,
                10
            )

            # 외부 place 위치 지정
            self._ros2_sub['target_pose'] = self._ros2_node.create_subscription(
                Pose,
                '/target_pose',
                self._on_target_pose,
                10
            )

            print("[ROS2] ✅ Publishers/Subscribers initialized")
            print("  Pub: /isaac/cube_detections, /isaac/stack_status, /isaac/ee_pose")
            print("  Sub: /stack_command, /target_pose")

        except Exception as e:
            print(f"[ROS2] 통신 초기화 실패: {e}")
            self.ros2_bridge['enabled'] = False

    # ── ROS2 콜백 ─────────────────────────────────────────────────────

    def _on_stack_command(self, msg):
        """
        /stack_command 토픽 수신 콜백
        msg.data: "start" / "stop" / "reset"
        """
        cmd = msg.data.strip().lower()
        print(f"[ROS2] ← stack_command: '{cmd}'")
        self._external_command = cmd

    def _on_target_pose(self, msg):
        """
        /target_pose 토픽 수신 콜백
        외부에서 place 위치를 지정
        """
        pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        print(f"[ROS2] ← target_pose: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        self._external_target = pos

    # ── ROS2 퍼블리시 헬퍼 ────────────────────────────────────────────

    def _publish_cube_detections(self, detected):
        """감지된 큐브 위치를 PoseArray로 퍼블리시"""
        if not self.ros2_bridge.get('enabled'):
            return
        try:
            from geometry_msgs.msg import PoseArray, Pose
            from std_msgs.msg import Header
            import builtin_interfaces.msg

            msg = PoseArray()
            msg.header.frame_id = "world"

            for cube in detected:
                p = cube['position']
                pose = Pose()
                pose.position.x = float(p[0])
                pose.position.y = float(p[1])
                pose.position.z = float(p[2])
                pose.orientation.w = 1.0
                msg.poses.append(pose)

            self._ros2_pub['cube_detections'].publish(msg)
        except Exception as e:
            print(f"[ROS2] cube_detections 퍼블리시 실패: {e}")

    def _publish_stack_status(self, status_str):
        """스태킹 상태 퍼블리시"""
        if not self.ros2_bridge.get('enabled'):
            return
        try:
            from std_msgs.msg import String
            msg = String()
            msg.data = status_str
            self._ros2_pub['stack_status'].publish(msg)
            print(f"[ROS2] → stack_status: '{status_str}'")
        except Exception as e:
            print(f"[ROS2] stack_status 퍼블리시 실패: {e}")

    def _publish_ee_pose(self):
        """EE 현재 포즈 퍼블리시"""
        if not self.ros2_bridge.get('enabled'):
            return
        try:
            from geometry_msgs.msg import PoseStamped
            ee_pos, ee_ori = self.franka.get_ee_pose()
            msg = PoseStamped()
            msg.header.frame_id = "world"
            msg.pose.position.x = float(ee_pos[0])
            msg.pose.position.y = float(ee_pos[1])
            msg.pose.position.z = float(ee_pos[2])
            msg.pose.orientation.w = float(ee_ori[0])
            msg.pose.orientation.x = float(ee_ori[1])
            msg.pose.orientation.y = float(ee_ori[2])
            msg.pose.orientation.z = float(ee_ori[3])
            self._ros2_pub['ee_pose'].publish(msg)
        except Exception as e:
            pass  # EE pose는 매 프레임 퍼블리시, 에러 로그 생략

    def _spin_ros2(self):
        """ROS2 콜백 처리 (non-blocking)"""
        if not self.ros2_bridge.get('enabled') or self._ros2_node is None:
            return
        try:
            import rclpy
            rclpy.spin_once(self._ros2_node, timeout_sec=0)
        except Exception:
            pass

    # ------------------------------------------------------------------ #
    #  replicator 카메라 초기화
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

        self.rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self.rgb_annot.attach([rp])

        self.depth_annot = rep.AnnotatorRegistry.get_annotator(
            "distance_to_camera")
        self.depth_annot.attach([rp])

        self._rep_ready = True
        print("[Vision] Replicator camera ready (RGB + Depth).")

    # ------------------------------------------------------------------ #
    #  프레임 취득
    # ------------------------------------------------------------------ #

    def _get_frames(self):
        self._ensure_camera_ready()

        for _ in range(5):
            self.world.step(render=True)
            self.app.update()

        if not self.timeline.is_playing():
            self.timeline.play()
            for _ in range(10):
                self.world.step(render=True)
                self.app.update()

        rgb_data = self.rgb_annot.get_data()
        rgb = None
        if rgb_data is not None and len(rgb_data) > 0:
            rgb = np.array(rgb_data)
            if rgb.ndim == 3 and rgb.shape[2] == 4:
                rgb = rgb[:, :, :3]
            if rgb.dtype != np.uint8:
                rgb = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)

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
    #  Depth 기반 3D 위치 변환
    # ------------------------------------------------------------------ #

    def _depth_pixel_to_world(self, u, v, depth_map):
        if depth_map is None:
            return None
        try:
            h, w = depth_map.shape[:2]
            u_c = int(np.clip(u, 0, w - 1))
            v_c = int(np.clip(v, 0, h - 1))
            u0 = max(0, u_c-1); u1 = min(w, u_c+2)
            v0 = max(0, v_c-1); v1 = min(h, v_c+2)
            d = float(np.median(depth_map[v0:v1, u0:u1]))

            if not (0.05 < d < 5.0):
                return None

            sensor_w = 20.955
            fx = (CAM_FOCAL_LEN / sensor_w) * CAM_RES_W
            fy = (CAM_FOCAL_LEN / sensor_w) * CAM_RES_H
            cx = CAM_RES_W / 2.0
            cy = CAM_RES_H / 2.0

            x_cam = (u_c - cx) * d / fx
            y_cam = (v_c - cy) * d / fy
            z_cam = d

            forward = CAM_LOOK_AT - CAM_POSITION
            forward = forward / np.linalg.norm(forward)
            world_up = np.array([0.0, 0.0, 1.0])
            right = np.cross(world_up, forward)
            right = right / np.linalg.norm(right)
            up = np.cross(forward, right)
            up = up / np.linalg.norm(up)

            point_cam  = np.array([x_cam, -y_cam, z_cam])
            R_mat      = np.column_stack([right, up, forward])
            point_world = CAM_POSITION + R_mat @ point_cam
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
        gt = get_cube_position(cube_index)

        pos_d = self._depth_pixel_to_world(u, v, depth_map)
        if pos_d is not None and gt is not None:
            err_mm = np.linalg.norm(pos_d[:2] - gt[:2]) * 1000
            if err_mm < DEPTH_TRUST_MM:
                self.performance['depth_used'] += 1
                return pos_d, 'depth', err_mm / 1000
            else:
                print(f"[Vision] Depth 오차 {err_mm:.0f}mm > "
                      f"{DEPTH_TRUST_MM:.0f}mm → Homography 시도")

        pos_h = self._pixel_to_world_homography(u, v)
        if pos_h is not None and gt is not None:
            err_mm = np.linalg.norm(pos_h[:2] - gt[:2]) * 1000
            if err_mm < DEPTH_TRUST_MM:
                return pos_h, 'homography', err_mm / 1000

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
            print("[Vision] 카메라 실패 → GT 모드")
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
            if not cnts or cv2.contourArea(
                    max(cnts, key=cv2.contourArea)) < 300:
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
            print(f"[Calibration] 감지 {len(pixel_pts)}개 → Homography 생략")

        self.calibrated = True
        self._publish_stack_status("calibrated")
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

            if not cnts or cv2.contourArea(
                    max(cnts, key=cv2.contourArea)) < 100:
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
            p       = cube['position']
            pix     = cube['pixel_pos']
            m       = cube['method']
            err     = cube['detection_error']
            pix_str = f"({pix[0]},{pix[1]})" if pix else "N/A"
            print(f"  {cube['color']} (Cube_{cube['index']}): "
                  f"Pixel={pix_str} | "
                  f"Pos=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) | "
                  f"[{m}] Err={err*1000:.1f}mm")

        # ROS2 퍼블리시
        self._publish_cube_detections(detected)

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
    #  [v12] 외부 명령 처리
    # ------------------------------------------------------------------ #

    def process_external_commands(self):
        """
        ROS2에서 받은 명령 처리.
        main loop에서 매 프레임 호출.
        """
        self._spin_ros2()

        if self._external_command is None:
            return

        cmd = self._external_command
        self._external_command = None

        if cmd == "stop":
            print("[ROS2] 명령: STOP")
            self.state      = STATE_IDLE
            self.step_count = 0
            self._publish_stack_status("stopped")

        elif cmd == "reset":
            print("[ROS2] 명령: RESET")
            self.state              = STATE_IDLE
            self.step_count         = 0
            self.cube_attached      = False
            self.current_cube_index = None
            self._publish_stack_status("reset")

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

        # [v12] j5=0.0 고정 (v11로 복구)
        self._pre_grasp_pose = np.array(
            [angle, j1_grasp, 0.0, GRASP_PRE_J3, 0.0, 1.8, 0.8, 0.04, 0.04])

        self._grasp_pose = self._pre_grasp_pose.copy()
        self._grasp_pose[3] += GRASP_DELTA_J3

        self._lift_pose = self._grasp_pose.copy()
        self._lift_pose[1] -= 0.4
        self._lift_pose[3] += 0.8

        print(f"[Vision] j3: pre={self._pre_grasp_pose[3]:.3f}, "
              f"grasp={self._grasp_pose[3]:.3f}")

        self._publish_stack_status(
            f"grasping_cube_{self.current_cube_index}")
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

        # ROS2 외부 target_pose 우선 사용
        if self._external_target is not None:
            print(f"[ROS2] 외부 target_pose 사용: {self._external_target}")
            target_pos          = self._external_target
            self._external_target = None

        print("\n[Vision] Starting place...")
        angle = np.arctan2(target_pos[1], target_pos[0])

        layer      = min(stack_index, max(STACK_HOVER_POSES.keys()))
        hover_pose = STACK_HOVER_POSES[layer].copy()
        hover_pose[0] = angle

        place_pose = hover_pose.copy()
        place_pose[3] += STACK_PLACE_DELTA_J3

        target_z = TABLE_TOP_Z + CUBE_SCALE * (stack_index + 0.5)
        print(f"[Vision] Stack={stack_index} | Layer={layer} | "
              f"angle={angle:.3f}")
        print(f"[Vision] 목표 Z={target_z:.4f}m | "
              f"Hover j3={hover_pose[3]:.3f}, Place j3={place_pose[3]:.3f}")

        self._transit_pose = TRANSIT_POSE.copy()
        self._transit_pose[0] = angle
        self._hover_pose   = hover_pose
        self._place_pose   = place_pose

        self._publish_stack_status(f"placing_layer_{stack_index}")
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
            self._publish_ee_pose()
            return

        # ROS2 명령 처리 (매 프레임)
        self.process_external_commands()

        # EE 포즈 퍼블리시 (매 프레임)
        self._publish_ee_pose()

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
                              if not self.cube_attached
                              else STATE_PLACE_HOVER)

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
                self._publish_stack_status("idle")
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

    def shutdown(self):
        """종료 시 ROS2 노드 정리"""
        if self._ros2_node is not None:
            try:
                self._ros2_node.destroy_node()
                print("[ROS2] Node destroyed")
            except Exception:
                pass
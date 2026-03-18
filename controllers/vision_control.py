# controllers/vision_control.py
"""
Phase 3: Vision-based Control v13

[v13 변경사항]
  1. Depth 카메라 개선
     - CameraIntrinsics 클래스: fx/fy/cx/cy 명시적 관리
     - GT 기반 자동 intrinsics 캘리브레이션 (sensor_w 역산)
     - 5x5 패치 median depth 샘플링 (노이즈 감소)
     - 캘리브레이션 후 Depth 정확도 재확인 출력

  2. Point Cloud 퍼블리시 (PointCloud2 XYZRGB)
     - /isaac/pointcloud → RViz2 3D 시각화
     - 테이블 위 영역만 샘플링 (효율화)
     - 4픽셀 서브샘플링으로 10fps 유지

  3. 추가 ROS2 토픽
     - /isaac/camera_info  → CameraInfo (내부 파라미터)
     - /isaac/pointcloud   → PointCloud2 (XYZRGB)

  4. 키보드 명령 확장 (/stack_command)
     - start / stop / reset / status / calibrate
"""

import numpy as np
import time
import cv2
import struct
from ultralytics import YOLO
import omni.kit.app
import omni.timeline
import omni.replicator.core as rep
from omni.isaac.core.utils.types import ArticulationAction

from cube_utils import (
    attach_cube_to_ee,
    detach_cube,
    get_cube_position,
)

# ── 상태 ──────────────────────────────────────────────────────────────
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

# ── 환경 수치 ─────────────────────────────────────────────────────────
TABLE_TOP_Z    = 0.50
CUBE_SCALE     = 0.05
CUBE_HALF      = 0.025
CUBE_REST_Z    = TABLE_TOP_Z + CUBE_HALF   # 0.525m
DEPTH_TRUST_MM = 50.0

# ── 카메라 파라미터 ───────────────────────────────────────────────────
CAM_POSITION  = np.array([1.0, 0.0, 1.5])
CAM_LOOK_AT   = np.array([0.5, 0.0, 0.0])
CAM_RES_W     = 1024
CAM_RES_H     = 768
CAM_FOCAL_LEN = 12.0

# ── 공통 포즈 ─────────────────────────────────────────────────────────
HOME_POSE    = np.array([0.0, -1.2, 0.0, -1.2, 0.0, 1.6, 0.7, 0.04, 0.04])
TRANSIT_POSE = np.array([0.0, -0.6, 0.0, -1.5, 0.0, 1.8, 0.8, 0.04, 0.04])

# ── YOLO 모델
YOLO_MODEL_PATH = "/home/jemini/isaac-sim/standalone_examples/my_manipulator/yolo/runs/cubes_v1/weights/best.pt"

# ── 색상 범위 (HSV) ───────────────────────────────────────────────────
COLOR_ORDER = ['red', 'green', 'blue', 'yellow']
COLOR_RANGES = {
    'red':    [([0,100,100],[10,255,255]), ([170,100,100],[180,255,255])],
    'green':  [([35,80,80],[85,255,255])],
    'blue':   [([100,100,100],[130,255,255])],
    'yellow': [([20,100,100],[35,255,255])],
}

# ── 층별 Hover 포즈 ───────────────────────────────────────────────────
STACK_HOVER_POSES = {
    0: np.array([0.0, -0.55, 0.0, -2.22, 0.0, 1.90, 0.8, 0.01, 0.01]),
    1: np.array([0.0, -0.50, 0.0, -2.07, 0.0, 1.95, 0.8, 0.01, 0.01]),
    2: np.array([0.0, -0.45, 0.0, -1.92, 0.0, 2.05, 0.8, 0.01, 0.01]),
    3: np.array([0.0, -0.40, 0.0, -1.77, 0.0, 2.15, 0.8, 0.01, 0.01]),
}
STACK_PLACE_DELTA_J3 = -0.08

# ── Attach/Detach 파라미터 ────────────────────────────────────────────
VELOCITY_THRESHOLD       = 0.005
ATTACH_MIN_WAIT_FRAMES   = 60
ATTACH_TIMEOUT_FRAMES    = 100
ATTACH_STABLE_COUNT_NEED = 5
DETACH_WAIT_FRAMES       = 50
PLACE_SETTLE_FRAMES      = 40

# ── Grasp 파라미터 ────────────────────────────────────────────────────
GRASP_PRE_J3   = -1.9
GRASP_DELTA_J3 = -0.55
J1_BASE        = -0.45
J1_DIST_REF    = 0.50
J1_GAIN        = 0.60


def _calc_j1(dist_xy):
    return float(np.clip(
        J1_BASE + (dist_xy - J1_DIST_REF) * J1_GAIN, -0.8, -0.2))


def _build_cam_rotation(cam_pos, look_at):
    """
    카메라 회전 행렬 계산
    Isaac Sim 카메라: X=right, Y=up, Z=backward (OpenGL 방식)
    """
    forward  = look_at - cam_pos
    forward  = forward / np.linalg.norm(forward)
    world_up = np.array([0.0, 0.0, 1.0])
    right    = np.cross(forward, world_up)   # ← 순서 변경: forward × up
    if np.linalg.norm(right) < 1e-6:
        right = np.array([1.0, 0.0, 0.0])
    else:
        right = right / np.linalg.norm(right)
    up = np.cross(right, forward)            # ← right × forward
    up = up / np.linalg.norm(up)
    return np.column_stack([right, up, -forward])  # ← Z축 반전 (OpenGL)


# ====================================================================== #
#  CameraIntrinsics: fx/fy 명시적 관리 + GT 기반 자동 캘리브레이션
# ====================================================================== #

class CameraIntrinsics:
    def __init__(self, res_w, res_h, focal_len_mm, sensor_w_mm=20.955):
        self.res_w       = res_w
        self.res_h       = res_h
        self.focal_mm    = focal_len_mm
        self.sensor_w_mm = sensor_w_mm
        self._update_params()

    def _update_params(self):
        sensor_h_mm = self.sensor_w_mm * (self.res_h / self.res_w)
        self.fx = (self.focal_mm / self.sensor_w_mm) * self.res_w
        self.fy = (self.focal_mm / sensor_h_mm) * self.res_h
        self.cx = self.res_w / 2.0
        self.cy = self.res_h / 2.0

    def calibrate_from_gt(self, pixel_pts, world_pts, depth_map, cam_pos, cam_rot):
        """
        GT 월드 좌표 + 픽셀로 fx/fy 역산 캘리브레이션.
        월드 → 카메라 역변환 후 depth 비율로 sensor_w 추정.
        """
        if len(pixel_pts) < 2:
            return

        fx_list, fy_list = [], []
        R_inv = cam_rot.T

        for (u, v), wp in zip(pixel_pts, world_pts):
            u_c = int(np.clip(u, 2, self.res_w - 3))
            v_c = int(np.clip(v, 2, self.res_h - 3))
            patch = depth_map[v_c-2:v_c+3, u_c-2:u_c+3]
            d = float(np.median(patch))
            if not (0.05 < d < 5.0):
                continue

            # 월드 → 카메라 좌표 역변환
            pc = R_inv @ (wp[:3] - cam_pos)
            x_cam_gt = pc[0]
            y_cam_gt = -pc[1]

            if abs(u_c - self.cx) > 10:
                fx_est = x_cam_gt / ((u_c - self.cx) / d)
                if 100 < abs(fx_est) < 2000:
                    fx_list.append(fx_est)

            if abs(v_c - self.cy) > 10:
                fy_est = y_cam_gt / ((v_c - self.cy) / d)
                if 100 < abs(fy_est) < 2000:
                    fy_list.append(fy_est)

        if fx_list:
            fx_new = float(np.median(fx_list))
            sensor_w_new = (self.focal_mm * self.res_w) / fx_new
            print(f"[Intrinsics] fx: {self.fx:.1f} → {fx_new:.1f}"
                  f" | sensor_w: {self.sensor_w_mm:.3f} → {sensor_w_new:.3f}mm")
            self.sensor_w_mm = sensor_w_new
            self._update_params()

        if fy_list:
            fy_new = float(np.median(fy_list))
            print(f"[Intrinsics] fy: {self.fy:.1f} → {fy_new:.1f}")
            self.fy = fy_new

    def pixel_to_world(self, u, v, depth, cam_pos, cam_rot):
        """픽셀 + depth → 월드 좌표 (Isaac Sim OpenGL 카메라 좌표계)"""
        x_cam =  (u - self.cx) * depth / self.fx   # 오른쪽 +
        y_cam = -(v - self.cy) * depth / self.fy   # 위쪽 + (이미지는 아래로 증가)
        z_cam = -depth                              # forward가 -Z
        point_cam   = np.array([x_cam, y_cam, z_cam])
        point_world = cam_pos + cam_rot @ point_cam
        return point_world

    def to_camera_info_msg(self, frame_id="camera_frame"):
        """ROS2 CameraInfo 메시지 생성"""
        try:
            from sensor_msgs.msg import CameraInfo
            msg = CameraInfo()
            msg.header.frame_id  = frame_id
            msg.width  = self.res_w
            msg.height = self.res_h
            msg.distortion_model = "plumb_bob"
            msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            msg.k = [
                self.fx, 0.0,     self.cx,
                0.0,     self.fy, self.cy,
                0.0,     0.0,     1.0,
            ]
            msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            msg.p = [
                self.fx, 0.0,     self.cx, 0.0,
                0.0,     self.fy, self.cy, 0.0,
                0.0,     0.0,     1.0,     0.0,
            ]
            return msg
        except ImportError:
            return None


# ====================================================================== #
#  VisionController
# ====================================================================== #

class VisionController:
    def __init__(self, franka, world, ros2_bridge=None):
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
        self._external_command  = None
        self._external_target   = None

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
        self.yolo_model        = YOLO(YOLO_MODEL_PATH)
        self.rgb_annot         = None
        self.depth_annot       = None

        # [v13] Intrinsics + 카메라 회전
        self.cam_intrinsics = CameraIntrinsics(
            CAM_RES_W, CAM_RES_H, CAM_FOCAL_LEN, sensor_w_mm=20.955)
        self._cam_rot = _build_cam_rotation(CAM_POSITION, CAM_LOOK_AT)

        self._ros2_pub        = {}
        self._ros2_sub        = {}
        self._ros2_node       = None
        self._pc_frame_count  = 0
        self._pc_every        = 10  # 10프레임마다 pointcloud 1회

        self._init_ros2_comms()

        print("[VisionController v13]")
        print(f"  CUBE_REST_Z  = {CUBE_REST_Z:.3f}m")
        print(f"  DEPTH_TRUST  = {DEPTH_TRUST_MM:.0f}mm")
        print(f"  fx={self.cam_intrinsics.fx:.1f}, fy={self.cam_intrinsics.fy:.1f}")
        print(f"  ROS2 = {'✅' if self.ros2_bridge.get('enabled') else '⚠️'}")

    # ------------------------------------------------------------------ #
    #  ROS2 초기화
    # ------------------------------------------------------------------ #

    def _init_ros2_comms(self):
        if not self.ros2_bridge.get('enabled'):
            print("[ROS2] Bridge 비활성화 → ROS2 통신 스킵")
            return
        try:
            import rclpy
            from geometry_msgs.msg import PoseArray, Pose, PoseStamped
            from std_msgs.msg import String
            from sensor_msgs.msg import CameraInfo, PointCloud2

            self._ros2_node = rclpy.create_node('isaac_vision_controller')

            # 퍼블리셔
            self._ros2_pub['cube_detections'] = self._ros2_node.create_publisher(
                PoseArray, '/isaac/cube_detections', 10)
            self._ros2_pub['stack_status'] = self._ros2_node.create_publisher(
                String, '/isaac/stack_status', 10)
            self._ros2_pub['ee_pose'] = self._ros2_node.create_publisher(
                PoseStamped, '/isaac/ee_pose', 10)
            self._ros2_pub['camera_info'] = self._ros2_node.create_publisher(
                CameraInfo, '/isaac/camera_info', 10)
            self._ros2_pub['pointcloud'] = self._ros2_node.create_publisher(
                PointCloud2, '/isaac/pointcloud', 1)

            # 서브스크라이버
            self._ros2_sub['stack_command'] = self._ros2_node.create_subscription(
                String, '/stack_command', self._on_stack_command, 10)
            self._ros2_sub['target_pose'] = self._ros2_node.create_subscription(
                Pose, '/target_pose', self._on_target_pose, 10)

            print("[ROS2] ✅ Initialized")
            print("  Pub: /isaac/{cube_detections, stack_status, ee_pose, camera_info, pointcloud}")
            print("  Sub: /stack_command, /target_pose")

        except Exception as e:
            print(f"[ROS2] 초기화 실패: {e}")
            self.ros2_bridge['enabled'] = False

    def _on_stack_command(self, msg):
        cmd = msg.data.strip().lower()
        print(f"[ROS2] ← /stack_command: '{cmd}'")
        self._external_command = cmd

    def _on_target_pose(self, msg):
        pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        print(f"[ROS2] ← /target_pose: {pos.round(3)}")
        self._external_target = pos

    def _publish_cube_detections(self, detected):
        if not self.ros2_bridge.get('enabled'):
            return
        try:
            from geometry_msgs.msg import PoseArray, Pose
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
            print(f"[ROS2] cube_detections 오류: {e}")

    def _publish_stack_status(self, status_str):
        if not self.ros2_bridge.get('enabled'):
            return
        try:
            from std_msgs.msg import String
            msg = String()
            msg.data = status_str
            self._ros2_pub['stack_status'].publish(msg)
            print(f"[ROS2] → /stack_status: '{status_str}'")
        except Exception as e:
            print(f"[ROS2] stack_status 오류: {e}")

    def _publish_ee_pose(self):
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
        except Exception:
            pass

    def _publish_camera_info(self):
        if not self.ros2_bridge.get('enabled'):
            return
        try:
            msg = self.cam_intrinsics.to_camera_info_msg("camera_frame")
            if msg:
                self._ros2_pub['camera_info'].publish(msg)
        except Exception:
            pass

    def _publish_pointcloud(self, rgb, depth):
        """
        [v13] RGB + Depth → PointCloud2 (XYZRGB) 퍼블리시
        테이블 위 영역만 (z > 0.48m), 4픽셀 서브샘플링
        """
        if not self.ros2_bridge.get('enabled'):
            return
        if rgb is None or depth is None:
            return
        try:
            import sensor_msgs.msg as sensor_msgs

            h, w = depth.shape[:2]
            step = 4
            us = np.arange(0, w, step)
            vs = np.arange(0, h, step)
            uu, vv = np.meshgrid(us, vs)
            uu = uu.flatten().astype(np.int32)
            vv = vv.flatten().astype(np.int32)

            dd = depth[vv, uu]
            valid = (dd > 0.3) & (dd < 2.5)
            uu, vv, dd = uu[valid], vv[valid], dd[valid]

            if len(uu) == 0:
                return

            # 픽셀 → 월드
            x_cam = (uu - self.cam_intrinsics.cx) * dd / self.cam_intrinsics.fx
            y_cam = -((vv - self.cam_intrinsics.cy) * dd / self.cam_intrinsics.fy)
            z_cam = dd
            pts_cam   = np.stack([x_cam, y_cam, z_cam], axis=1)
            pts_world = (self._cam_rot @ pts_cam.T).T + CAM_POSITION

            # 테이블 위만
            mask      = pts_world[:, 2] > 0.48
            pts_world = pts_world[mask]
            uu_f      = uu[mask]
            vv_f      = vv[mask]

            if len(pts_world) == 0:
                return

            rgb_s = rgb[vv_f, uu_f]  # (N, 3)

            # PointCloud2 빌드 (XYZRGB)
            cloud = sensor_msgs.PointCloud2()
            cloud.header.frame_id = "world"
            cloud.height = 1
            cloud.width  = len(pts_world)
            cloud.fields = [
                sensor_msgs.PointField(name='x', offset=0,
                    datatype=sensor_msgs.PointField.FLOAT32, count=1),
                sensor_msgs.PointField(name='y', offset=4,
                    datatype=sensor_msgs.PointField.FLOAT32, count=1),
                sensor_msgs.PointField(name='z', offset=8,
                    datatype=sensor_msgs.PointField.FLOAT32, count=1),
                sensor_msgs.PointField(name='rgb', offset=12,
                    datatype=sensor_msgs.PointField.FLOAT32, count=1),
            ]
            cloud.is_bigendian = False
            cloud.point_step   = 16
            cloud.row_step     = 16 * len(pts_world)
            cloud.is_dense     = True

            buf = []
            for pt, c in zip(pts_world, rgb_s):
                r, g, b = int(c[0]), int(c[1]), int(c[2])
                rgb_packed = struct.unpack('f',
                    struct.pack('I', (r << 16) | (g << 8) | b))[0]
                buf.append(struct.pack('ffff',
                    float(pt[0]), float(pt[1]), float(pt[2]), rgb_packed))

            cloud.data = b''.join(buf)
            self._ros2_pub['pointcloud'].publish(cloud)

        except Exception as e:
            print(f"[ROS2] pointcloud 오류: {e}")

    def _spin_ros2(self):
        if not self.ros2_bridge.get('enabled') or self._ros2_node is None:
            return
        try:
            import rclpy
            rclpy.spin_once(self._ros2_node, timeout_sec=0)
        except Exception:
            pass

    # ------------------------------------------------------------------ #
    #  카메라 초기화 + 프레임 취득
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
        self.rgb_annot   = rep.AnnotatorRegistry.get_annotator("rgb")
        self.depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
        self.rgb_annot.attach([rp])
        self.depth_annot.attach([rp])
        self._rep_ready = True
        print("[Vision] Replicator camera ready.")

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

        # [v13] 주기적 PointCloud + CameraInfo 퍼블리시
        self._pc_frame_count += 1
        if self._pc_frame_count >= self._pc_every:
            self._pc_frame_count = 0
            self._publish_pointcloud(rgb, depth)
            self._publish_camera_info()

        return rgb, depth

    def _get_color_mask(self, hsv, color_name):
        h, w = hsv.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        for lower, upper in COLOR_RANGES[color_name]:
            mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
        kernel = np.ones((5, 5), np.uint8)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # ------------------------------------------------------------------ #
    #  [v13] 개선된 Depth 변환
    # ------------------------------------------------------------------ #

    def _depth_pixel_to_world(self, u, v, depth_map):
        if depth_map is None:
            return None
        try:
            h, w = depth_map.shape[:2]
            u_c = int(np.clip(u, 2, w - 3))
            v_c = int(np.clip(v, 2, h - 3))
            # 5x5 패치 median
            patch = depth_map[v_c-2:v_c+3, u_c-2:u_c+3]
            d = float(np.median(patch))
            if not (0.05 < d < 5.0):
                return None
            point_world = self.cam_intrinsics.pixel_to_world(
                u_c, v_c, d, CAM_POSITION, self._cam_rot)
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
                self.homography_matrix)
            return np.array([wp[0,0,0], wp[0,0,1], CUBE_REST_Z])
        except Exception:
            return None

    def _best_position(self, u, v, depth_map, cube_index):
        gt    = get_cube_position(cube_index)
        pos_d = self._depth_pixel_to_world(u, v, depth_map)

        if pos_d is not None and gt is not None:
            err_mm = np.linalg.norm(pos_d[:2] - gt[:2]) * 1000
            if err_mm < DEPTH_TRUST_MM:
                self.performance['depth_used'] += 1
                return pos_d, 'depth', err_mm / 1000
            print(f"[Vision] Depth 오차 {err_mm:.0f}mm > {DEPTH_TRUST_MM:.0f}mm → Homography")

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
    #  [v13] 캘리브레이션 (Homography + Depth Intrinsics)
    # ------------------------------------------------------------------ #

    def calibrate_homography(self):
        print("\n" + "="*60)
        print("HOMOGRAPHY + DEPTH INTRINSICS CALIBRATION (v13)")
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
            world_pts.append(gt)
            depth_str = (f"Depth=({pos_d[0]:.3f},{pos_d[1]:.3f}) Err={err_d:.0f}mm"
                         if pos_d is not None else "Depth=N/A")
            print(f"  {color_name} (Cube_{cube_idx}): "
                  f"Pixel=({cx},{cy}) | GT=({gt[0]:.3f},{gt[1]:.3f}) | {depth_str}")

        # [v13] Depth intrinsics 캘리브레이션
        if len(pixel_pts) >= 2 and depth is not None:
            print("\n[Intrinsics] Depth intrinsics 캘리브레이션...")
            self.cam_intrinsics.calibrate_from_gt(
                pixel_pts, world_pts, depth, CAM_POSITION, self._cam_rot)
            print(f"[Intrinsics] ✅ fx={self.cam_intrinsics.fx:.1f}, "
                  f"fy={self.cam_intrinsics.fy:.1f}, "
                  f"sensor_w={self.cam_intrinsics.sensor_w_mm:.3f}mm")

            # 캘리브레이션 후 정확도 재확인
            print("\n[Calibration] Depth 정확도 재확인:")
            for (px, py), gt in zip(pixel_pts, world_pts):
                pos_d = self._depth_pixel_to_world(px, py, depth)
                if pos_d is not None:
                    err_mm = np.linalg.norm(pos_d[:2] - gt[:2]) * 1000
                    print(f"  ({px},{py}) → ({pos_d[0]:.3f},{pos_d[1]:.3f}) "
                          f"GT=({gt[0]:.3f},{gt[1]:.3f}) Err={err_mm:.0f}mm")

        # Homography
        if len(pixel_pts) >= 4:
            pixel_arr = np.array([[p[0], p[1]] for p in pixel_pts], dtype=np.float32)
            world_arr = np.array([[w[0], w[1]] for w in world_pts], dtype=np.float32)
            self.homography_matrix, _ = cv2.findHomography(pixel_arr, world_arr)
            status = "완료" if self.homography_matrix is not None else "실패"
            print(f"\n[Calibration] Homography {status}")
        else:
            print(f"\n[Calibration] 감지 {len(pixel_pts)}개 → Homography 생략")

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

        results = self.yolo_model(rgb, conf=0.5, iou=0.3, verbose=False)

        # 클래스별 최고 confidence 박스만 선택
        best_per_class = {}
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls)
                conf   = float(box.conf)
                if cls_id >= len(COLOR_ORDER):
                    continue
                if cls_id not in best_per_class or conf > best_per_class[cls_id]['conf']:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    best_per_class[cls_id] = {
                        'conf': conf,
                        'cx': int((x1 + x2) / 2),
                        'cy': int((y1 + y2) / 2),
                    }

        detected = []
        for cls_id, info in best_per_class.items():
            if exclude_cubes and cls_id in exclude_cubes:
                continue
            color_name = COLOR_ORDER[cls_id]
            cx, cy = info['cx'], info['cy']

            pos, method, err = self._best_position(cx, cy, depth, cls_id)
            if pos is None:
                continue

            detected.append({
                'color':           color_name,
                'index':           cls_id,
                'position':        pos,
                'pixel_pos':       (cx, cy),
                'method':          f'yolo_{method}',
                'detection_error': err,
                'conf':            info['conf'],
            })

        # 감지 못한 큐브 GT fallback
        detected_indices = {d['index'] for d in detected}
        for i in range(4):
            if exclude_cubes and i in exclude_cubes:
                continue
            if i not in detected_indices:
                gt = get_cube_position(i)
                if gt is not None:
                    detected.append({
                        'color':           COLOR_ORDER[i],
                        'index':           i,
                        'position':        gt,
                        'pixel_pos':       None,
                        'method':          'gt_no_detect',
                        'detection_error': 0.0,
                        'conf':            0.0,
                    })
                    self.performance['gt_fallback'] += 1

        print(f"[Vision] YOLO detected {len(detected)} cubes:")
        for cube in detected:
            p   = cube['position']
            pix = cube['pixel_pos']
            print(f"  {cube['color']} (Cube_{cube['index']}): "
                f"Pixel={pix} | Pos=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) | "
                f"[{cube['method']}] conf={cube['conf']:.2f} "
                f"Err={cube['detection_error']*1000:.1f}mm")

        self._publish_cube_detections(detected)
        return detected

    def _gt_fallback(self, exclude_cubes=None):
        fallback = []
        for i in range(4):
            if exclude_cubes and i in exclude_cubes:
                continue
            pos = get_cube_position(i)
            if pos is not None:
                fallback.append({'color': COLOR_ORDER[i], 'index': i,
                                  'position': pos, 'pixel_pos': None,
                                  'method': 'gt_fallback', 'detection_error': 0.0})
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

    def process_external_commands(self):
        self._spin_ros2()
        if self._external_command is None:
            return
        cmd = self._external_command
        self._external_command = None

        if cmd == "stop":
            self.state = STATE_IDLE; self.step_count = 0
            self._publish_stack_status("stopped")
        elif cmd == "reset":
            self.state = STATE_IDLE; self.step_count = 0
            self.cube_attached = False; self.current_cube_index = None
            self._publish_stack_status("reset")
        elif cmd == "calibrate":
            if self.state == STATE_IDLE:
                self.calibrate_homography()
        elif cmd == "status":
            self._publish_stack_status(
                f"state={self.state}|attached={self.cube_attached}|cube={self.current_cube_index}")

    # ------------------------------------------------------------------ #
    #  Grasp / Place (v12와 동일 로직)
    # ------------------------------------------------------------------ #

    def auto_grasp(self, cube_index=None, exclude_cubes=None):
        if not self.calibrated:
            self.calibrated = True
        if self.state != STATE_IDLE:
            return False

        print("\n[Phase 3: Vision] Starting grasp...")
        det = self.detect_cubes_from_camera(exclude_cubes=exclude_cubes)
        if not det:
            return False

        if cube_index is not None:
            target = next((c for c in det if c['index'] == cube_index), None)
            if target is None:
                pos = get_cube_position(cube_index)
                if pos is None:
                    return False
                target = {'color': COLOR_ORDER[cube_index], 'index': cube_index,
                          'position': pos, 'method': 'gt_fallback', 'detection_error': 0.0}
        else:
            target = det[0]

        self.current_cube_index = target['index']
        pos     = target['position']
        angle   = np.arctan2(pos[1], pos[0])
        dist_xy = np.sqrt(pos[0]**2 + pos[1]**2)
        j1_grasp = _calc_j1(dist_xy)

        self.performance['detection_errors'].append(target.get('detection_error', 0.0))
        print(f"[Vision] Target: {target['color']} (Cube_{self.current_cube_index})")
        print(f"[Vision] Pos: {pos.round(3)} | dist_xy={dist_xy:.3f}m → j1={j1_grasp:.3f}")

        self._transit_pose        = TRANSIT_POSE.copy()
        self._transit_pose[0]     = angle
        self._pre_grasp_pose      = np.array([angle, j1_grasp, 0.0, GRASP_PRE_J3, 0.0, 1.8, 0.8, 0.04, 0.04])
        self._grasp_pose          = self._pre_grasp_pose.copy()
        self._grasp_pose[3]      += GRASP_DELTA_J3
        self._lift_pose           = self._grasp_pose.copy()
        self._lift_pose[1]       -= 0.4
        self._lift_pose[3]       += 0.8

        print(f"[Vision] j3: pre={self._pre_grasp_pose[3]:.3f}, grasp={self._grasp_pose[3]:.3f}")
        self._publish_stack_status(f"grasping_cube_{self.current_cube_index}")
        self.start_time = time.time()
        self.step_count = 0
        self._stable_count = 0
        self.state = STATE_OPEN_GRIPPER
        return True

    def place(self, target_pos, stack_index=0):
        if not self.cube_attached:
            return False
        if self.state != STATE_IDLE:
            return False
        if self._external_target is not None:
            target_pos = self._external_target
            self._external_target = None

        print("\n[Vision] Starting place...")
        angle = np.arctan2(target_pos[1], target_pos[0])
        layer = min(stack_index, max(STACK_HOVER_POSES.keys()))
        hover_pose       = STACK_HOVER_POSES[layer].copy()
        hover_pose[0]    = angle
        place_pose       = hover_pose.copy()
        place_pose[3]   += STACK_PLACE_DELTA_J3
        target_z = TABLE_TOP_Z + CUBE_SCALE * (stack_index + 0.5)
        print(f"[Vision] Stack={stack_index} | Layer={layer} | angle={angle:.3f}")
        print(f"[Vision] 목표 Z={target_z:.4f}m | Hover j3={hover_pose[3]:.3f}")

        self._transit_pose        = TRANSIT_POSE.copy()
        self._transit_pose[0]     = angle
        self._hover_pose          = hover_pose
        self._place_pose          = place_pose
        self._publish_stack_status(f"placing_layer_{stack_index}")
        self.start_time = time.time()
        self.step_count = 0
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

        self.process_external_commands()
        self._publish_ee_pose()
        curr_joints = self._get_current_joint_positions()

        if self.state == STATE_OPEN_GRIPPER:
            self.franka.gripper.open()
            self.step_count += 1
            if self.step_count >= 30:
                self.step_count = 0; self.state = STATE_TRANSIT

        elif self.state == STATE_TRANSIT:
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(curr_joints, self._transit_pose, 120)
            if self.step_count < 120:
                self.controller.apply_action(ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 120:
                self.step_count = 0
                self.state = STATE_PRE_GRASP if not self.cube_attached else STATE_PLACE_HOVER

        elif self.state == STATE_PRE_GRASP:
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(curr_joints, self._pre_grasp_pose, 100)
            if self.step_count < 100:
                self.controller.apply_action(ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 100:
                print("[Vision] Pre-grasp... done"); self.step_count = 0; self.state = STATE_APPROACH

        elif self.state == STATE_APPROACH:
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(curr_joints, self._grasp_pose, 80)
            if self.step_count < 80:
                self.controller.apply_action(ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 80:
                print("[Vision] Approach... done"); self.step_count = 0; self.state = STATE_CLOSE_GRIPPER

        elif self.state == STATE_CLOSE_GRIPPER:
            self.franka.gripper.close()
            self.step_count += 1
            if self.step_count >= 120:
                print("[Vision] Close gripper... done")
                self.step_count = 0; self._stable_count = 0; self.state = STATE_ATTACH

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
            if self._stable_count >= ATTACH_STABLE_COUNT_NEED or self.step_count >= ATTACH_TIMEOUT_FRAMES:
                if self.step_count >= ATTACH_TIMEOUT_FRAMES and self._stable_count < ATTACH_STABLE_COUNT_NEED:
                    print(f"[Vision] ⚠️  Attach timeout (vel={max_vel:.4f})")
                attach_cube_to_ee(self.current_cube_index)
                self.cube_attached = True
                print(f"[Vision] Attached Cube_{self.current_cube_index}")
                self.step_count = 0; self._stable_count = 0; self.state = STATE_LIFT

        elif self.state == STATE_LIFT:
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(curr_joints, self._lift_pose, 120)
            if self.step_count < 120:
                self.controller.apply_action(ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 120:
                elapsed = time.time() - self.start_time
                self.performance['grasp_times'].append(elapsed)
                print(f"[Phase 3] ✓ Grasp Complete! ({elapsed:.2f}s)")
                self.step_count = 0; self.state = STATE_IDLE

        elif self.state == STATE_PLACE_HOVER:
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(curr_joints, self._hover_pose, 150)
            if self.step_count < 150:
                self.controller.apply_action(ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 150:
                self.step_count = 0; self.state = STATE_PLACE_DOWN

        elif self.state == STATE_PLACE_DOWN:
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(curr_joints, self._place_pose, 200)
            if self.step_count < 200:
                self.controller.apply_action(ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 200:
                print("[Vision] Place down... done"); self.step_count = 0; self.state = STATE_PLACE_SETTLE

        elif self.state == STATE_PLACE_SETTLE:
            self.step_count += 1
            if self.step_count >= PLACE_SETTLE_FRAMES:
                print("[Vision] Settle done → detach"); self.step_count = 0; self.state = STATE_DETACH

        elif self.state == STATE_DETACH:
            self.step_count += 1
            if self.step_count >= DETACH_WAIT_FRAMES:
                detach_cube(self.current_cube_index)
                self.cube_attached = False; self.current_cube_index = None
                self.step_count = 0; self.state = STATE_OPEN_AFTER

        elif self.state == STATE_OPEN_AFTER:
            self.franka.gripper.open()
            self.step_count += 1
            if self.step_count >= 50:
                self.step_count = 0; self.state = STATE_RETREAT

        elif self.state == STATE_RETREAT:
            if self.step_count == 0:
                self._traj = self._interpolate_joint_trajectory(curr_joints, HOME_POSE, 150)
            if self.step_count < 150:
                self.controller.apply_action(ArticulationAction(self._traj[self.step_count]))
            self.step_count += 1
            if self.step_count >= 150:
                elapsed = time.time() - self.start_time
                self.performance['place_times'].append(elapsed)
                print(f"[Phase 3] ✓ Place complete ({elapsed:.2f}s)")
                self._publish_stack_status("idle")
                self.step_count = 0; self.state = STATE_IDLE

    # ------------------------------------------------------------------ #
    #  성능 / 종료
    # ------------------------------------------------------------------ #

    def get_performance_summary(self):
        perf = self.performance
        s = {'total_grasps': len(perf['grasp_times']),
             'total_places': len(perf['place_times']),
             'depth_used':   perf['depth_used'],
             'gt_fallback':  perf['gt_fallback']}
        if perf['grasp_times']:
            s['avg_grasp_time'] = np.mean(perf['grasp_times'])
            s['std_grasp_time'] = np.std(perf['grasp_times'])
        if perf['place_times']:
            s['avg_place_time'] = np.mean(perf['place_times'])
            s['std_place_time'] = np.std(perf['place_times'])
        if perf['detection_errors']:
            s['avg_detection_error'] = np.mean(perf['detection_errors'])
            s['std_detection_error'] = np.std(perf['detection_errors'])
        return s

    def print_performance(self):
        s = self.get_performance_summary()
        print(f"\n{'='*60}")
        print("PHASE 3: VISION CONTROL - PERFORMANCE")
        print(f"{'='*60}")
        print(f"Total Grasps : {s.get('total_grasps', 0)}")
        print(f"Total Places : {s.get('total_places', 0)}")
        print(f"Depth 사용   : {s.get('depth_used', 0)}회")
        print(f"GT Fallback  : {s.get('gt_fallback', 0)}회")
        if 'avg_grasp_time' in s:
            print(f"Grasp Time   : {s['avg_grasp_time']:.2f}s ± {s['std_grasp_time']:.2f}s")
        if 'avg_place_time' in s:
            print(f"Place Time   : {s['avg_place_time']:.2f}s ± {s['std_place_time']:.2f}s")
        if 'avg_detection_error' in s:
            print(f"Vision Error : {s['avg_detection_error']*1000:.2f}mm ± {s['std_detection_error']*1000:.2f}mm")
        print(f"Camera fx    : {self.cam_intrinsics.fx:.1f}")
        print(f"Camera fy    : {self.cam_intrinsics.fy:.1f}")
        print(f"sensor_w     : {self.cam_intrinsics.sensor_w_mm:.3f}mm")
        print(f"{'='*60}\n")

    def shutdown(self):
        if self._ros2_node is not None:
            try:
                self._ros2_node.destroy_node()
                print("[ROS2] Node destroyed")
            except Exception:
                pass
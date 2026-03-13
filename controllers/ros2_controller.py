# controllers/ros2_controller.py
"""
Isaac Sim ROS2 외부 제어 노드 v2

별도 터미널에서 실행:
  python3 ros2_controller.py
  또는
  ros2 run <package> ros2_controller

[v2 변경사항]
  1. 키보드 실시간 제어 (termios 기반, non-blocking)
     - 1~4: 명령 선택 메뉴
     - WASD: 타겟 위치 이동
     - Enter: 명령 실행
     - ESC / q: 종료

  2. 수신 토픽 추가
     - /isaac/camera_info  → CameraInfo 출력
     - /isaac/pointcloud   → PointCloud2 수신 확인

  3. RViz2 연동 퍼블리시
     - /visualization/cubes      → MarkerArray
     - /visualization/ee_trail   → LINE_STRIP Marker
     - /visualization/target_pos → 타겟 위치 Marker

  4. 상태 대시보드 (1Hz 터미널 출력)

────────────────────────────────────────────────────────────
RViz2 설정 (config/isaac_sim.rviz 파일 사용):
  rviz2 -d config/isaac_sim.rviz

키보드 명령:
  1  → 스태킹 start
  2  → 스태킹 stop
  3  → 리셋 reset
  4  → 캘리브레이션 calibrate
  5  → 상태 조회 status
  w  → 타겟 X +0.05m
  s  → 타겟 X -0.05m
  a  → 타겟 Y +0.05m
  d  → 타겟 Y -0.05m
  r  → 타겟 Z +0.025m
  f  → 타겟 Z -0.025m
  p  → 현재 타겟 위치로 place 명령
  i  → 상태 출력
  q  → 종료
────────────────────────────────────────────────────────────
"""

import sys
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState, CameraInfo, PointCloud2
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np


# ────────────────────────────────────────────────────────────────────── #
#  터미널 non-blocking 키 입력
# ────────────────────────────────────────────────────────────────────── #

def _setup_terminal():
    """raw 모드로 전환 (non-blocking 키 입력)"""
    try:
        import termios, tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setraw(fd)
        return old
    except Exception:
        return None


def _restore_terminal(old):
    try:
        import termios
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old)
    except Exception:
        pass


def _getch_nowait():
    """non-blocking 1문자 읽기. 없으면 None"""
    try:
        import select
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if r:
            ch = sys.stdin.read(1)
            return ch
    except Exception:
        pass
    return None


# ────────────────────────────────────────────────────────────────────── #
#  색상 상수
# ────────────────────────────────────────────────────────────────────── #
CUBE_COLORS = {
    0: (1.0, 0.0, 0.0, 1.0),   # Red
    1: (0.0, 1.0, 0.0, 1.0),   # Green
    2: (0.0, 0.0, 1.0, 1.0),   # Blue
    3: (1.0, 1.0, 0.0, 1.0),   # Yellow
}
CUBE_NAMES = ['Red', 'Green', 'Blue', 'Yellow']


# ====================================================================== #
#  IsaacSimController Node
# ====================================================================== #

class IsaacSimController(Node):

    def __init__(self):
        super().__init__('isaac_sim_controller')

        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # ── 서브스크라이버 ───────────────────────────────────────────
        self.create_subscription(Image,       '/isaac/rgb',             self._on_rgb,          qos_be)
        self.create_subscription(Image,       '/isaac/depth',           self._on_depth,        qos_be)
        self.create_subscription(JointState,  '/joint_states',          self._on_joint_states, qos_rel)
        self.create_subscription(PoseArray,   '/isaac/cube_detections', self._on_cube_det,     qos_rel)
        self.create_subscription(PoseStamped, '/isaac/ee_pose',         self._on_ee_pose,      qos_be)
        self.create_subscription(String,      '/isaac/stack_status',    self._on_stack_status, qos_rel)
        self.create_subscription(CameraInfo,  '/isaac/camera_info',     self._on_camera_info,  qos_rel)
        self.create_subscription(PointCloud2, '/isaac/pointcloud',      self._on_pointcloud,   qos_be)

        # ── 퍼블리셔 ─────────────────────────────────────────────────
        self.pub_cmd        = self.create_publisher(String,      '/stack_command',             qos_rel)
        self.pub_target     = self.create_publisher(Pose,        '/target_pose',               qos_rel)
        self.pub_markers    = self.create_publisher(MarkerArray, '/visualization/cubes',       qos_rel)
        self.pub_ee_trail   = self.create_publisher(Marker,      '/visualization/ee_trail',    qos_rel)
        self.pub_target_vis = self.create_publisher(Marker,      '/visualization/target_pos',  qos_rel)

        # ── 상태 ─────────────────────────────────────────────────────
        self.stack_status   = "unknown"
        self.cube_positions = []
        self.ee_trail       = []
        self.joint_states   = None
        self.camera_info    = None
        self.pc_count       = 0
        self.rgb_count      = 0
        self.depth_count    = 0

        # 타겟 위치 (조이스틱처럼 WASD로 조작)
        self.target_pos = np.array([0.50, 0.00, 0.525])

        # 1Hz 상태 대시보드 타이머
        self.create_timer(1.0, self._dashboard_timer)

        self._print_banner()

    # ── 수신 콜백 ──────────────────────────────────────────────────────

    def _on_rgb(self, msg):
        self.rgb_count += 1

    def _on_depth(self, msg):
        self.depth_count += 1

    def _on_joint_states(self, msg):
        self.joint_states = msg

    def _on_cube_det(self, msg):
        self.cube_positions = [
            (p.position.x, p.position.y, p.position.z)
            for p in msg.poses
        ]
        self._publish_cube_markers()

    def _on_ee_pose(self, msg):
        p = msg.pose.position
        self.ee_trail.append((p.x, p.y, p.z))
        if len(self.ee_trail) > 300:
            self.ee_trail = self.ee_trail[-300:]
        self._publish_ee_trail()

    def _on_stack_status(self, msg):
        self.stack_status = msg.data
        self.get_logger().info(f"[Status] {msg.data}")

    def _on_camera_info(self, msg):
        self.camera_info = msg

    def _on_pointcloud(self, msg):
        self.pc_count += 1

    # ── RViz2 마커 퍼블리시 ──────────────────────────────────────────

    def _publish_cube_markers(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, pos in enumerate(self.cube_positions):
            # 큐브 마커
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp    = now
            m.ns   = "cubes"; m.id = i
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position.x = pos[0]
            m.pose.position.y = pos[1]
            m.pose.position.z = pos[2]
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.05
            c = CUBE_COLORS.get(i, (1.0,1.0,1.0,1.0))
            m.color.r, m.color.g, m.color.b, m.color.a = c
            ma.markers.append(m)

            # 텍스트 라벨
            t = Marker()
            t.header.frame_id = "world"
            t.header.stamp    = now
            t.ns   = "cube_labels"; t.id = i + 100
            t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
            t.pose.position.x = pos[0]
            t.pose.position.y = pos[1]
            t.pose.position.z = pos[2] + 0.07
            t.pose.orientation.w = 1.0
            t.scale.z = 0.04
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = CUBE_NAMES[i] if i < len(CUBE_NAMES) else f"Cube_{i}"
            ma.markers.append(t)

        self.pub_markers.publish(ma)

    def _publish_ee_trail(self):
        if len(self.ee_trail) < 2:
            return
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns = "ee_trail"; m.id = 0
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.006
        m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; m.color.a = 0.9
        for p in self.ee_trail:
            pt = Point(); pt.x, pt.y, pt.z = p
            m.points.append(pt)
        self.pub_ee_trail.publish(m)

    def _publish_target_marker(self):
        """타겟 위치를 RViz2 구 마커로 표시"""
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns = "target"; m.id = 0
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x = float(self.target_pos[0])
        m.pose.position.y = float(self.target_pos[1])
        m.pose.position.z = float(self.target_pos[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.04
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.5; m.color.a = 0.9
        self.pub_target_vis.publish(m)

    # ── 명령 퍼블리시 ──────────────────────────────────────────────────

    def send_command(self, cmd: str):
        msg = String(); msg.data = cmd
        self.pub_cmd.publish(msg)
        self.get_logger().info(f"[CMD] → '{cmd}'")

    def send_target_pose(self, x=None, y=None, z=None):
        pos = self.target_pos.copy()
        if x is not None: pos[0] = x
        if y is not None: pos[1] = y
        if z is not None: pos[2] = z

        msg = Pose()
        msg.position.x = float(pos[0])
        msg.position.y = float(pos[1])
        msg.position.z = float(pos[2])
        msg.orientation.w = 1.0
        self.pub_target.publish(msg)
        self.get_logger().info(
            f"[Target] → ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        self._publish_target_marker()

    # ── 상태 대시보드 ──────────────────────────────────────────────────

    def _dashboard_timer(self):
        """1Hz 터미널 대시보드"""
        cam_str = ""
        if self.camera_info is not None:
            k = self.camera_info.k
            cam_str = f" | fx={k[0]:.0f} fy={k[4]:.0f}"

        print(f"\r[Dashboard] status={self.stack_status} | "
              f"cubes={len(self.cube_positions)} | "
              f"EE_trail={len(self.ee_trail)} | "
              f"PC={self.pc_count} RGB={self.rgb_count}{cam_str} | "
              f"Target=({self.target_pos[0]:.2f},{self.target_pos[1]:.2f},{self.target_pos[2]:.2f})",
              end='', flush=True)

    def print_full_status(self):
        print(f"\n{'='*60}")
        print(f"Stack Status : {self.stack_status}")
        print(f"Target Pos   : {self.target_pos.round(3)}")
        print(f"PointCloud   : {self.pc_count} 수신")
        if self.camera_info:
            k = self.camera_info.k
            print(f"Camera Info  : fx={k[0]:.1f}, fy={k[4]:.1f}, "
                  f"cx={k[2]:.1f}, cy={k[5]:.1f}")
        print(f"\nCubes ({len(self.cube_positions)}):")
        for i, pos in enumerate(self.cube_positions):
            name = CUBE_NAMES[i] if i < len(CUBE_NAMES) else f"Cube_{i}"
            print(f"  {name:6s}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        if self.joint_states:
            print(f"\nJoint States: {len(self.joint_states.position)} joints")
        print(f"{'='*60}\n")

    def _print_banner(self):
        print("\n" + "="*60)
        print("  Isaac Sim ROS2 Controller v2")
        print("="*60)
        print("  키보드 조작:")
        print("    1  → start stacking")
        print("    2  → stop")
        print("    3  → reset")
        print("    4  → calibrate (재캘리브레이션)")
        print("    5  → status 조회")
        print("    w/s → 타겟 X ±0.05m")
        print("    a/d → 타겟 Y ±0.05m")
        print("    r/f → 타겟 Z ±0.025m")
        print("    p   → place 명령 (현재 타겟 위치)")
        print("    i   → 상태 전체 출력")
        print("    q   → 종료")
        print("="*60)
        print("  RViz2 토픽:")
        print("    /isaac/rgb           → Image (Camera)")
        print("    /isaac/depth         → Image (Depth)")
        print("    /isaac/camera_info   → CameraInfo")
        print("    /isaac/pointcloud    → PointCloud2 (XYZRGB)")
        print("    /joint_states        → JointState (로봇 모델)")
        print("    /tf                  → TF Tree")
        print("    /visualization/cubes → MarkerArray (큐브 위치)")
        print("    /visualization/ee_trail → LINE_STRIP (EE 궤적)")
        print("    /visualization/target_pos → Sphere (타겟 위치)")
        print("="*60 + "\n")


# ====================================================================== #
#  키보드 루프
# ====================================================================== #

def keyboard_loop(node: IsaacSimController, stop_event: threading.Event):
    """non-blocking 키 입력 루프"""
    old_term = _setup_terminal()

    try:
        while not stop_event.is_set():
            ch = _getch_nowait()
            if ch is None:
                time.sleep(0.05)
                continue

            if ch == '1':
                node.send_command("start")
            elif ch == '2':
                node.send_command("stop")
            elif ch == '3':
                node.send_command("reset")
            elif ch == '4':
                node.send_command("calibrate")
            elif ch == '5':
                node.send_command("status")
            elif ch == 'w':
                node.target_pos[0] += 0.05
                print(f"\n[Target] X+ → {node.target_pos.round(3)}")
                node._publish_target_marker()
            elif ch == 's':
                node.target_pos[0] -= 0.05
                print(f"\n[Target] X- → {node.target_pos.round(3)}")
                node._publish_target_marker()
            elif ch == 'a':
                node.target_pos[1] += 0.05
                print(f"\n[Target] Y+ → {node.target_pos.round(3)}")
                node._publish_target_marker()
            elif ch == 'd':
                node.target_pos[1] -= 0.05
                print(f"\n[Target] Y- → {node.target_pos.round(3)}")
                node._publish_target_marker()
            elif ch == 'r':
                node.target_pos[2] += 0.025
                print(f"\n[Target] Z+ → {node.target_pos.round(3)}")
                node._publish_target_marker()
            elif ch == 'f':
                node.target_pos[2] -= 0.025
                print(f"\n[Target] Z- → {node.target_pos.round(3)}")
                node._publish_target_marker()
            elif ch == 'p':
                node.send_target_pose()
            elif ch == 'i':
                node.print_full_status()
            elif ch in ('q', '\x1b', '\x03'):  # q, ESC, Ctrl+C
                print("\n[Keyboard] 종료...")
                stop_event.set()
                rclpy.shutdown()
                break

    finally:
        _restore_terminal(old_term)


# ====================================================================== #
#  main
# ====================================================================== #

def main(args=None):
    rclpy.init(args=args)
    node = IsaacSimController()

    stop_event = threading.Event()

    # 키보드 스레드
    kb_thread = threading.Thread(
        target=keyboard_loop, args=(node, stop_event), daemon=True)
    kb_thread.start()

    try:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[Controller] 종료 중...")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("[Controller] 완료")


if __name__ == '__main__':
    main()
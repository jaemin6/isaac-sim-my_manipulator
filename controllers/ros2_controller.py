# controllers/ros2_controller.py
"""
Isaac Sim 외부 ROS2 제어 노드

Isaac Sim과 별도 터미널에서 실행:
  ros2 run <your_package> ros2_controller

기능:
  1. 시각화
     - /isaac/rgb          → RViz2 카메라 이미지
     - /isaac/depth        → RViz2 depth 이미지
     - /joint_states       → RViz2 로봇 모델
     - /isaac/cube_detections → 큐브 위치 마커
     - /isaac/ee_pose      → EE 궤적

  2. 외부 제어
     - /stack_command 퍼블리시 → Isaac Sim 명령
     - /target_pose 퍼블리시  → place 위치 지정

사용법:
  # 스태킹 시작
  ros2 topic pub /stack_command std_msgs/String "data: 'start'" --once

  # 스태킹 중지
  ros2 topic pub /stack_command std_msgs/String "data: 'stop'" --once

  # 특정 위치에 쌓기
  ros2 topic pub /target_pose geometry_msgs/Pose \\
    "position: {x: 0.5, y: 0.0, z: 0.525}" --once

  # RViz2 실행
  rviz2 -d config/isaac_sim.rviz
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

import numpy as np
import sys


# 큐브 색상 (RViz2 마커용)
CUBE_COLORS = {
    0: (1.0, 0.0, 0.0, 1.0),   # Red
    1: (0.0, 1.0, 0.0, 1.0),   # Green
    2: (0.0, 0.0, 1.0, 1.0),   # Blue
    3: (1.0, 1.0, 0.0, 1.0),   # Yellow
}
CUBE_NAMES = ['Red', 'Green', 'Blue', 'Yellow']


class IsaacSimController(Node):
    """
    Isaac Sim과 통신하는 ROS2 노드
    - Isaac Sim 데이터 수신 및 RViz2 시각화
    - 외부 명령 발행
    """

    def __init__(self):
        super().__init__('isaac_sim_controller')

        # QoS 프로파일
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── 서브스크라이버 (Isaac Sim → 이 노드) ────────────────────
        self.sub_rgb = self.create_subscription(
            Image, '/isaac/rgb',
            self._on_rgb, qos_best_effort)

        self.sub_depth = self.create_subscription(
            Image, '/isaac/depth',
            self._on_depth, qos_best_effort)

        self.sub_joint_states = self.create_subscription(
            JointState, '/joint_states',
            self._on_joint_states, qos_reliable)

        self.sub_cube_detections = self.create_subscription(
            PoseArray, '/isaac/cube_detections',
            self._on_cube_detections, qos_reliable)

        self.sub_ee_pose = self.create_subscription(
            PoseStamped, '/isaac/ee_pose',
            self._on_ee_pose, qos_best_effort)

        self.sub_stack_status = self.create_subscription(
            String, '/isaac/stack_status',
            self._on_stack_status, qos_reliable)

        # ── 퍼블리셔 (이 노드 → Isaac Sim) ─────────────────────────
        self.pub_stack_command = self.create_publisher(
            String, '/stack_command', qos_reliable)

        self.pub_target_pose = self.create_publisher(
            Pose, '/target_pose', qos_reliable)

        # ── RViz2 시각화 퍼블리셔 ───────────────────────────────────
        self.pub_cube_markers = self.create_publisher(
            MarkerArray, '/visualization/cubes', qos_reliable)

        self.pub_ee_trail = self.create_publisher(
            Marker, '/visualization/ee_trail', qos_reliable)

        # ── 상태 ────────────────────────────────────────────────────
        self.stack_status   = "unknown"
        self.cube_positions = []
        self.ee_trail       = []   # EE 궤적 포인트
        self.joint_states   = None

        # 10Hz 상태 출력 타이머
        self.create_timer(0.1, self._status_timer)

        self.get_logger().info("="*50)
        self.get_logger().info("Isaac Sim ROS2 Controller 시작")
        self.get_logger().info("="*50)
        self.get_logger().info("Sub: /isaac/rgb, /depth, /joint_states")
        self.get_logger().info("Sub: /isaac/cube_detections, /ee_pose")
        self.get_logger().info("Pub: /stack_command, /target_pose")
        self.get_logger().info("Pub: /visualization/cubes, /ee_trail")
        self.get_logger().info("")
        self.get_logger().info("명령어:")
        self.get_logger().info("  1: 스태킹 시작")
        self.get_logger().info("  2: 스태킹 중지")
        self.get_logger().info("  3: 리셋")
        self.get_logger().info("  q: 종료")

    # ------------------------------------------------------------------ #
    #  서브스크라이버 콜백
    # ------------------------------------------------------------------ #

    def _on_rgb(self, msg):
        """RGB 이미지 수신 (RViz2로 자동 전달됨)"""
        pass  # RViz2가 /isaac/rgb 직접 구독

    def _on_depth(self, msg):
        """Depth 이미지 수신"""
        pass  # RViz2가 /isaac/depth 직접 구독

    def _on_joint_states(self, msg):
        """Joint states 수신"""
        self.joint_states = msg

    def _on_cube_detections(self, msg):
        """
        큐브 감지 결과 수신 → RViz2 마커로 변환
        PoseArray: 각 Pose가 큐브 위치
        """
        self.cube_positions = [
            (p.position.x, p.position.y, p.position.z)
            for p in msg.poses
        ]

        # RViz2 마커 생성
        marker_array = MarkerArray()
        for i, pos in enumerate(self.cube_positions):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp    = self.get_clock().now().to_msg()
            marker.ns              = "cubes"
            marker.id              = i
            marker.type            = Marker.CUBE
            marker.action          = Marker.ADD

            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            color = CUBE_COLORS.get(i, (1.0, 1.0, 1.0, 1.0))
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]

            marker_array.markers.append(marker)

        self.pub_cube_markers.publish(marker_array)

    def _on_ee_pose(self, msg):
        """EE 포즈 수신 → 궤적 마커 업데이트"""
        pos = msg.pose.position
        self.ee_trail.append((pos.x, pos.y, pos.z))

        # 최근 200 포인트만 유지
        if len(self.ee_trail) > 200:
            self.ee_trail = self.ee_trail[-200:]

        # LINE_STRIP 마커로 퍼블리시
        if len(self.ee_trail) < 2:
            return

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp    = self.get_clock().now().to_msg()
        marker.ns              = "ee_trail"
        marker.id              = 0
        marker.type            = Marker.LINE_STRIP
        marker.action          = Marker.ADD
        marker.scale.x         = 0.005

        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.8

        from geometry_msgs.msg import Point
        for p in self.ee_trail:
            pt = Point()
            pt.x, pt.y, pt.z = p
            marker.points.append(pt)

        self.pub_ee_trail.publish(marker)

    def _on_stack_status(self, msg):
        """스태킹 상태 수신"""
        self.stack_status = msg.data
        self.get_logger().info(f"[Status] → {msg.data}")

    # ------------------------------------------------------------------ #
    #  상태 출력 타이머
    # ------------------------------------------------------------------ #

    def _status_timer(self):
        pass  # 필요 시 주기적 로그 추가

    # ------------------------------------------------------------------ #
    #  외부 제어 명령 퍼블리시
    # ------------------------------------------------------------------ #

    def send_command(self, cmd: str):
        """
        Isaac Sim에 명령 전송
        cmd: "start" / "stop" / "reset"
        """
        msg      = String()
        msg.data = cmd
        self.pub_stack_command.publish(msg)
        self.get_logger().info(f"[Command] → '{cmd}'")

    def send_target_pose(self, x: float, y: float, z: float):
        """
        Place 위치 지정
        Isaac Sim의 place()가 이 위치를 사용
        """
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.orientation.w = 1.0
        self.pub_target_pose.publish(msg)
        self.get_logger().info(
            f"[Target] → ({x:.3f}, {y:.3f}, {z:.3f})")

    def print_status(self):
        print(f"\n[Status] stack={self.stack_status} | "
              f"cubes={len(self.cube_positions)}")
        for i, pos in enumerate(self.cube_positions):
            name = CUBE_NAMES[i] if i < len(CUBE_NAMES) else f"Cube_{i}"
            print(f"  {name}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")


def interactive_loop(node: IsaacSimController):
    """
    키보드 입력으로 Isaac Sim 제어
    별도 스레드에서 실행
    """
    import threading

    def _loop():
        print("\n[Interactive] 준비 완료")
        print("  1: 스태킹 시작")
        print("  2: 스태킹 중지")
        print("  3: 리셋")
        print("  s: 상태 출력")
        print("  p <x> <y>: place 위치 지정 (예: p 0.5 0.0)")
        print("  q: 종료\n")

        while rclpy.ok():
            try:
                cmd = input("> ").strip()
            except EOFError:
                break

            if cmd == "1":
                node.send_command("start")
            elif cmd == "2":
                node.send_command("stop")
            elif cmd == "3":
                node.send_command("reset")
            elif cmd == "s":
                node.print_status()
            elif cmd.startswith("p "):
                parts = cmd.split()
                if len(parts) == 3:
                    try:
                        x = float(parts[1])
                        y = float(parts[2])
                        node.send_target_pose(x, y, 0.525)
                    except ValueError:
                        print("사용법: p <x> <y>")
                else:
                    print("사용법: p <x> <y>")
            elif cmd == "q":
                print("종료...")
                rclpy.shutdown()
                break
            else:
                print(f"알 수 없는 명령: '{cmd}'")

    t = threading.Thread(target=_loop, daemon=True)
    t.start()
    return t


def main(args=None):
    rclpy.init(args=args)
    node = IsaacSimController()

    # 인터랙티브 루프 시작
    interactive_loop(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS2 Controller 종료")


if __name__ == '__main__':
    main()
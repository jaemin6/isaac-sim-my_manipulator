# sim/world.py
"""
World Setup v12 - ROS2 Bridge 통합

[ROS2 연동 추가]
  - RGB/Depth 카메라 토픽 퍼블리시
  - Joint States 퍼블리시
  - TF Tree 퍼블리시
  - 외부 명령 수신 (stack_command)

토픽 목록:
  퍼블리시:
    /isaac/rgb          → sensor_msgs/Image
    /isaac/depth        → sensor_msgs/Image
    /isaac/camera_info  → sensor_msgs/CameraInfo
    /joint_states       → sensor_msgs/JointState
    /tf                 → tf2_msgs/TFMessage

  서브스크라이브:
    /stack_command      → std_msgs/String
    /target_pose        → geometry_msgs/Pose
"""

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid, GroundPlane
from omni.isaac.franka import Franka
from pxr import UsdLux, Gf
from omni.isaac.core.utils.stage import get_current_stage


def setup_world():
    print("[Setup] Creating world...")
    world = World()

    _add_lights()

    world.scene.add(
        GroundPlane(
            "/World/Ground",
            z_position=0,
            color=np.array([0.1, 0.2, 0.4])
        )
    )

    franka = world.scene.add(
        Franka(prim_path="/World/Franka", name="franka")
    )

    world.scene.add(
        FixedCuboid(
            prim_path="/World/Table",
            name="table",
            position=np.array([0.5, 0.0, 0.25]),
            scale=np.array([0.6, 0.8, 0.5]),
            color=np.array([0.6, 0.4, 0.2])
        )
    )

    # ── 큐브 배치 ─────────────────────────────────────────────────────
    # Blue/Yellow: X=0.55 (Franka 도달 범위 내)
    cube_positions = [
        [0.40,  0.25, 0.526],   # Red
        [0.40, -0.25, 0.526],   # Green
        [0.55,  0.18, 0.526],   # Blue
        [0.55, -0.18, 0.526],   # Yellow
    ]
    cube_colors = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 1.0, 0.0],
    ]
    color_names = ['Red', 'Green', 'Blue', 'Yellow']

    for i, (pos, color) in enumerate(zip(cube_positions, cube_colors)):
        dist = np.sqrt(pos[0]**2 + pos[1]**2)
        world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/Cube_{i}",
                name=f"cube_{i}",
                position=pos,
                scale=[0.05, 0.05, 0.05],
                mass=1.0,
                color=np.array(color)
            )
        )
        print(f"  Cube_{i} ({color_names[i]}): "
              f"XY=({pos[0]:.2f},{pos[1]:+.2f}) dist={dist:.3f}m")

    # ── ROS2 Bridge 초기화 ────────────────────────────────────────────
    ros2_bridge = _setup_ros2_bridge()

    # ── 월드 리셋 ─────────────────────────────────────────────────────
    print("[Setup] Resetting world...")
    world.reset()

    # ── Franka 초기 자세 (팔 위로 → 큐브 충돌 방지) ──────────────────
    franka.set_joint_positions(
        np.array([0.0, -1.5, 0.0, -2.5, 0.0, 1.5, 0.7, 0.04, 0.04])
    )

    # ── 안정화 ────────────────────────────────────────────────────────
    print("[Setup] Stabilizing (120 frames)...")
    for _ in range(120):
        world.step(render=True)

    print("[Setup] ✓ World setup complete!\n")
    return world, franka, ros2_bridge


def _setup_ros2_bridge():
    """
    Isaac Sim 4.5 ROS2 Bridge 초기화
    isaacsim.ros2.bridge 사용 (4.5부터 변경된 API)
    """
    print("[ROS2] Initializing ROS2 Bridge...")

    try:
        import rclpy
        from isaacsim.ros2.bridge import ROS2Camera, ROS2JointState, ROS2TFTree

        # rclpy 초기화
        if not rclpy.ok():
            rclpy.init()

        bridge = {
            'enabled': True,
            'camera': None,
            'joint_state': None,
            'tf': None,
        }

        # ── RGB + Depth 카메라 퍼블리시 ───────────────────────────────
        ros2_cam = ROS2Camera(
            prim_path="/World/Camera",
            rgb_topic="/isaac/rgb",
            depth_topic="/isaac/depth",
            camera_info_topic="/isaac/camera_info",
            frame_id="camera_frame",
            queue_size=1,
        )
        bridge['camera'] = ros2_cam
        print("[ROS2] Camera topics: /isaac/rgb, /isaac/depth, /isaac/camera_info")

        # ── Joint State 퍼블리시 ─────────────────────────────────────
        ros2_js = ROS2JointState(
            prim_path="/World/Franka",
            topic="/joint_states",
            frame_id="base_link",
            queue_size=10,
        )
        bridge['joint_state'] = ros2_js
        print("[ROS2] Joint state topic: /joint_states")

        # ── TF Tree 퍼블리시 ─────────────────────────────────────────
        ros2_tf = ROS2TFTree(
            prim_paths=[
                "/World/Franka",
                "/World/Camera",
            ],
            topic="/tf",
        )
        bridge['tf'] = ros2_tf
        print("[ROS2] TF topic: /tf")

        print("[ROS2] ✅ ROS2 Bridge initialized!")
        return bridge

    except ImportError as e:
        print(f"[ROS2] ⚠️  ROS2 Bridge import 실패: {e}")
        print("[ROS2] Extension 활성화 확인: Window → Extensions → isaacsim.ros2.bridge")
        return {'enabled': False}
    except Exception as e:
        print(f"[ROS2] ⚠️  ROS2 Bridge 초기화 실패: {e}")
        return {'enabled': False}


def _add_lights():
    stage = get_current_stage()
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(1000)
    dome.CreateColorAttr((0.8, 0.9, 1.0))
    sun = UsdLux.DistantLight.Define(stage, "/World/DirectionalLight")
    sun.CreateIntensityAttr(5000)
    sun.CreateColorAttr((1.0, 0.95, 0.85))
    from pxr import UsdGeom
    xform = UsdGeom.Xformable(sun)
    xform.AddRotateXYZOp().Set((-45, 45, 0))
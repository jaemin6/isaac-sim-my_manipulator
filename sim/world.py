# sim/world.py
"""
World Setup v13 - OmniGraph 기반 ROS2 Bridge

퍼블리시 토픽:
  /clock
  /joint_states
  /tf
  /isaac/rgb
  /isaac/depth
  /isaac/camera_info
"""

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid, GroundPlane
from omni.isaac.franka import Franka
from pxr import UsdLux, UsdGeom, Gf
from omni.isaac.core.utils.stage import get_current_stage
import omni.usd
import usdrt.Sdf


def setup_world():
    print("[Setup] Creating world...")
    world = World()

    _add_lights()

    world.scene.add(
        GroundPlane("/World/Ground", z_position=0, color=np.array([0.1, 0.2, 0.4]))
    )

    franka = world.scene.add(
        Franka(prim_path="/World/Franka", name="franka")
    )

    world.scene.add(
        FixedCuboid(
            prim_path="/World/Table", name="table",
            position=np.array([0.5, 0.0, 0.25]),
            scale=np.array([0.6, 0.8, 0.5]),
            color=np.array([0.6, 0.4, 0.2])
        )
    )

    cube_positions = [
        [0.40,  0.25, 0.526],
        [0.40, -0.25, 0.526],
        [0.55,  0.18, 0.526],
        [0.55, -0.18, 0.526],
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
                prim_path=f"/World/Cube_{i}", name=f"cube_{i}",
                position=pos, scale=[0.05, 0.05, 0.05],
                mass=1.0, color=np.array(color)
            )
        )
        print(f"  Cube_{i} ({color_names[i]}): "
              f"XY=({pos[0]:.2f},{pos[1]:+.2f}) dist={dist:.3f}m")

    # 카메라 프림 생성 (ROS2 그래프보다 먼저)
    _create_camera_prim()

    # 월드 리셋
    print("[Setup] Resetting world...")
    world.reset()

    franka.set_joint_positions(
        np.array([0.0, -1.5, 0.0, -2.5, 0.0, 1.5, 0.7, 0.04, 0.04])
    )

    # 안정화
    print("[Setup] Stabilizing (120 frames)...")
    for _ in range(120):
        world.step(render=True)

    # ROS2 Bridge (안정화 후 초기화)
    ros2_bridge = _setup_ros2_bridge()

    print("[Setup] ✓ World setup complete!\n")
    return world, franka, ros2_bridge


def _create_camera_prim():
    """
    탑다운 카메라 프림 생성
    Isaac Sim Z-up 좌표계 기준:
      - 위치: X=0.5(테이블 중앙), Y=0, Z=1.5(위)
      - 아래를 내려다보는 회전: X=-90도 (카메라 -Z 방향이 월드 -Z)
    """
    stage = get_current_stage()
    cam_path = "/World/TopCamera"

    cam_prim = UsdGeom.Camera(stage.DefinePrim(cam_path, "Camera"))
    xform_api = UsdGeom.XformCommonAPI(cam_prim)

    # 테이블 바로 위에서 수직으로 내려다보기
    # 위치: 테이블 중앙(0.5, 0, 0) 위 1.5m
    xform_api.SetTranslate(Gf.Vec3d(0.5, 0.0, 1.5))
    # Isaac Sim에서 카메라는 기본적으로 -Z를 바라봄
    # X축 -90도 = 아래를 바라봄
    xform_api.SetRotate((-90, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    cam_prim.GetHorizontalApertureAttr().Set(20.955)
    cam_prim.GetVerticalApertureAttr().Set(15.2955)
    cam_prim.GetProjectionAttr().Set("perspective")
    cam_prim.GetFocalLengthAttr().Set(12.0)
    cam_prim.GetFocusDistanceAttr().Set(400)

    print(f"[Setup] Camera prim: {cam_path} | pos=(0.5, 0, 1.5) | rot=(-90, 0, 0)")
    return cam_path


def _setup_ros2_bridge():
    """OmniGraph 기반 ROS2 Bridge 초기화"""
    print("[ROS2] Initializing ROS2 Bridge (OmniGraph)...")

    try:
        import rclpy
        if not rclpy.ok():
            rclpy.init()

        import omni.graph.core as og

        _create_clock_graph(og)
        _create_joint_state_tf_graph(og)
        _create_camera_graph(og)

        print("[ROS2] ✅ ROS2 Bridge initialized!")
        return {'enabled': True}

    except Exception as e:
        print(f"[ROS2] ⚠️  Bridge 초기화 실패: {e}")
        import traceback
        traceback.print_exc()
        return {'enabled': False}


def _create_clock_graph(og):
    try:
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/Graph/ROS_Clock", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime",    "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("PublishClock",   "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ("Context",        "isaacsim.ros2.bridge.ROS2Context"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick",        "PublishClock.inputs:execIn"),
                    ("Context.outputs:context",            "PublishClock.inputs:context"),
                    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ],
            },
        )
        print("[ROS2] ✓ /clock")
    except Exception as e:
        print(f"[ROS2] Clock 실패: {e}")


def _create_joint_state_tf_graph(og):
    try:
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/Graph/ROS_Robot", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime",    "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("PublishJoints",  "isaacsim.ros2.bridge.ROS2PublishJointState"),
                    ("PublishTF",      "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ("Context",        "isaacsim.ros2.bridge.ROS2Context"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick",        "PublishJoints.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick",        "PublishTF.inputs:execIn"),
                    ("Context.outputs:context",            "PublishJoints.inputs:context"),
                    ("Context.outputs:context",            "PublishTF.inputs:context"),
                    ("ReadSimTime.outputs:simulationTime", "PublishJoints.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ],
                keys.SET_VALUES: [
                    ("PublishJoints.inputs:topicName",  "/joint_states"),
                    ("PublishJoints.inputs:targetPrim", ["/World/Franka"]),
                    ("PublishTF.inputs:topicName",      "/tf"),
                    ("PublishTF.inputs:targetPrims",    ["/World/Franka"]),
                    # parentPrim 미설정 → Isaac Sim이 자동으로 "World" 프레임 사용
                ],
            },
        )
        print("[ROS2] ✓ /joint_states, /tf")
    except Exception as e:
        print(f"[ROS2] JointState/TF 실패: {e}")


def _create_camera_graph(og):
    """
    RGB + Depth + CameraInfo 퍼블리시
    뷰포트 0(Isaac Sim 기본 뷰)을 건드리지 않고
    별도 렌더 프로덕트를 새로 만들어서 사용
    """
    try:
        import omni.replicator.core as rep

        # 별도 렌더 프로덕트 생성 (뷰포트 0 건드리지 않음)
        render_product = rep.create.render_product(
            "/World/TopCamera",
            resolution=(1024, 768)
        )
        render_product_path = render_product.path

        keys = og.Controller.Keys
        (ros_camera_graph, _, _, _) = og.Controller.edit(
            {
                "graph_path": "/Graph/ROS_Camera",
                "evaluator_name": "push",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                keys.CREATE_NODES: [
                    ("OnTick",            "omni.graph.action.OnTick"),
                    ("cameraHelperRgb",   "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("cameraHelperInfo",  "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "cameraHelperRgb.inputs:execIn"),
                    ("OnTick.outputs:tick", "cameraHelperInfo.inputs:execIn"),
                    ("OnTick.outputs:tick", "cameraHelperDepth.inputs:execIn"),
                ],
                keys.SET_VALUES: [
                    ("cameraHelperRgb.inputs:frameId",              "camera_frame"),
                    ("cameraHelperRgb.inputs:topicName",            "isaac/rgb"),
                    ("cameraHelperRgb.inputs:type",                 "rgb"),
                    ("cameraHelperRgb.inputs:renderProductPath",    render_product_path),
                    ("cameraHelperInfo.inputs:frameId",             "camera_frame"),
                    ("cameraHelperInfo.inputs:topicName",           "isaac/camera_info"),
                    ("cameraHelperInfo.inputs:renderProductPath",   render_product_path),
                    ("cameraHelperDepth.inputs:frameId",            "camera_frame"),
                    ("cameraHelperDepth.inputs:topicName",          "isaac/depth"),
                    ("cameraHelperDepth.inputs:type",               "depth"),
                    ("cameraHelperDepth.inputs:renderProductPath",  render_product_path),
                ],
            },
        )

        og.Controller.evaluate_sync(ros_camera_graph)
        print("[ROS2] ✓ /isaac/rgb, /isaac/depth, /isaac/camera_info")
        print(f"[ROS2]   render_product: {render_product_path}")

    except Exception as e:
        print(f"[ROS2] Camera 실패: {e}")
        import traceback
        traceback.print_exc()


def _add_lights():
    stage = get_current_stage()
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(1000)
    dome.CreateColorAttr((0.8, 0.9, 1.0))
    sun = UsdLux.DistantLight.Define(stage, "/World/DirectionalLight")
    sun.CreateIntensityAttr(5000)
    sun.CreateColorAttr((1.0, 0.95, 0.85))
    xform = UsdGeom.Xformable(sun)
    xform.AddRotateXYZOp().Set((-45, 45, 0))
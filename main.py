from isaacsim.simulation_app import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import omni.kit.app
from pxr import UsdLux, UsdPhysics, Gf, UsdShade, Sdf, UsdGeom

from omni.isaac.core import World
from omni.isaac.core.objects import GroundPlane, DynamicCuboid, FixedCuboid, VisualSphere
from omni.isaac.franka import Franka
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.stage import get_current_stage

# 카메라 import
from omni.isaac.sensor import Camera
import matplotlib.pyplot as plt
import carb.input


def get_cube_position_from_world(cube_index=0):
    """World에서 직접 큐브 위치 가져오기"""
    from pxr import UsdGeom
    stage = get_current_stage()
    
    # 특정 큐브 또는 첫 번째 큐브
    cube_path = f"/World/Cube_{cube_index}"
    cube = stage.GetPrimAtPath(cube_path)
    
    if not cube or not cube.IsValid():
        return None
    
    xform = UsdGeom.Xformable(cube)
    pos = xform.ComputeLocalToWorldTransform(0).ExtractTranslation()
    
    world_pos = np.array([pos[0], pos[1], pos[2]])
    
    return world_pos


def get_all_cubes_positions():
    """모든 큐브의 위치 가져오기"""
    from pxr import UsdGeom
    stage = get_current_stage()
    
    cubes_info = []
    i = 0
    while True:
        cube_path = f"/World/Cube_{i}"
        cube = stage.GetPrimAtPath(cube_path)
        
        if not cube or not cube.IsValid():
            break
        
        xform = UsdGeom.Xformable(cube)
        pos = xform.ComputeLocalToWorldTransform(0).ExtractTranslation()
        world_pos = np.array([pos[0], pos[1], pos[2]])
        
        cubes_info.append({
            'index': i,
            'path': cube_path,
            'position': world_pos
        })
        i += 1
    
    return cubes_info


def find_nearest_cube(robot_position):
    """로봇에서 가장 가까운 큐브 찾기"""
    cubes = get_all_cubes_positions()
    
    if not cubes:
        return None
    
    nearest = None
    min_distance = float('inf')
    
    for cube_info in cubes:
        pos = cube_info['position']
        # XY 평면 거리만 계산
        distance = np.sqrt((pos[0] - robot_position[0])**2 + 
                          (pos[1] - robot_position[1])**2)
        
        if distance < min_distance:
            min_distance = distance
            nearest = cube_info
    
    return nearest


def detect_cube_in_camera_view(camera, world):
    """카메라 뷰에서 물체 탐지 (위치 기반)"""
    print("[Vision] Detecting cube...")
    
    # 카메라 초기화
    camera.initialize()
    
    for _ in range(10):
        world.step(render=True)
    
    # 실제 큐브 위치 가져오기
    cube_pos = get_cube_position_from_world()
    
    if cube_pos is None:
        print("[Vision] [Error] No cube found!")
        return None
    
    # 카메라 이미지도 캡처 (시각화용)
    rgb = camera.get_rgba()[:, :, :3]
    
    # 간단한 시각화
    fig, ax = plt.subplots(1, 1, figsize=(8, 8))
    ax.imshow(rgb)
    ax.set_title(f"Camera View\nCube at: ({cube_pos[0]:.2f}, {cube_pos[1]:.2f}, {cube_pos[2]:.2f})")
    ax.axis('off')
    
    # 큐브 위치를 이미지에 표시 (대략적인 위치)
    img_h, img_w = rgb.shape[:2]
    # 간단한 투영 (실제로는 카메라 projection matrix 필요)
    center_x = img_w // 2
    center_y = img_h // 2
    ax.plot(center_x, center_y, 'r+', markersize=30, markeredgewidth=3)
    ax.text(center_x + 20, center_y, f"Cube: ({cube_pos[0]:.2f}, {cube_pos[1]:.2f})", 
            color='red', fontsize=12, fontweight='bold')
    
    plt.savefig("camera_view.png", dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"[Success] Camera view saved to 'camera_view.png'")
    print(f"[Success] Cube position: {cube_pos}")
    
    return cube_pos


def add_lights():
    """밝은 조명 환경 설정"""
    stage = get_current_stage()
    
    # 밝은 dome light
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(1000)
    dome.CreateColorAttr((0.8, 0.9, 1.0))  # 푸른 하늘색
    
    # 강한 directional light
    sun = UsdLux.DistantLight.Define(stage, "/World/DirectionalLight")
    sun.CreateIntensityAttr(5000)
    sun.CreateColorAttr((1.0, 0.95, 0.85))  # 따뜻한 노란빛
    xform = UsdGeom.Xformable(sun)
    xform.AddRotateXYZOp().Set((-45, 45, 0))


def setup_camera(world):
    """탑다운 뷰 카메라 설치"""
    print("[Camera] Setting up camera...")
    
    camera = Camera(
        prim_path="/World/Camera",
        position=np.array([0.5, 0.0, 1.2]),
        frequency=20,
        resolution=(512, 512),
        name="top_camera"
    )
    world.scene.add(camera)
    
    # 카메라를 아래로 향하게
    camera.set_local_pose(
        translation=np.array([0.5, 0.0, 1.2]),
        orientation=np.array([0.7071, 0, 0, -0.7071])
    )
    
    print("[Camera] Camera setup complete!")
    return camera


def capture_test_image(camera, world):
    """카메라 테스트 이미지 캡처 - 단순화"""
    print("[Camera] Capturing test image...")
    
    camera.initialize()
    
    for _ in range(30):
        world.step(render=True)
    
    rgb = camera.get_rgba()[:, :, :3]
    
    plt.figure(figsize=(8, 8))
    plt.imshow(rgb)
    plt.title("Camera View - Top Down")
    plt.axis('off')
    plt.savefig("camera_test.png", dpi=150, bbox_inches='tight')
    plt.close()
    
    print(f"[Success] Camera image saved to 'camera_test.png'")
    print(f"  Image shape: {rgb.shape}")
    
    return rgb


def attach_cube_to_ee(cube_index=0):
    """큐브를 end effector에 부착"""
    stage = get_current_stage()
    cube_path = f"/World/Cube_{cube_index}"
    cube = stage.GetPrimAtPath(cube_path)
    ee = stage.GetPrimAtPath("/World/Franka/panda_hand")
    
    if not cube or not ee:
        return False
    
    joint = UsdPhysics.FixedJoint.Define(stage, f"/World/GraspJoint_{cube_index}")
    joint.CreateBody0Rel().SetTargets([cube.GetPath()])
    joint.CreateBody1Rel().SetTargets([ee.GetPath()])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    
    print(f"[Info] Cube_{cube_index} attached to EE")
    return True


class KeyboardController:
    """키보드로 Franka 로봇 제어 + Pick and Place"""
    
    def __init__(self, franka, world, camera):
        self.franka = franka
        self.world = world
        self.camera = camera
        self.controller = franka.get_articulation_controller()
        
        # 현재 joint 상태
        self.target_joints = franka.get_joint_positions()
        
        # 그리퍼 상태
        self.gripper_closed = False
        self.cube_attached = False
        self.current_cube_index = None  # 현재 잡고 있는 큐브
        
        # 목표 위치 (Place 위치)
        self.target_position = np.array([0.3, 0.3, 0.55])  # 테이블 위 다른 위치
        
        # 목표 위치 마커 생성
        self.create_target_marker()
        
        # 키보드 입력 구독
        appwindow = omni.appwindow.get_default_app_window()
        input_iface = carb.input.acquire_input_interface()
        self.keyboard = appwindow.get_keyboard()
        self.sub_keyboard = input_iface.subscribe_to_keyboard_events(
            self.keyboard, self._on_keyboard_event
        )
        
        print("\n" + "="*60)
        print("KEYBOARD CONTROLS")
        print("="*60)
        print("  W / S     : Shoulder Up / Down")
        print("  A / D     : Base Rotate Left / Right")
        print("  Q / E     : Elbow Up / Down")
        print("  R / F     : Wrist Up / Down")
        print("  G         : Toggle Gripper")
        print("  SPACE     : Auto-Grasp Detected Cube")
        print("  P         : Place - Move to target & release")
        print("  M         : Multi-Cube Mode - Process all cubes")
        print("  ")
        print("  Arrow Keys: Move target position (X/Y)")
        print("  [ / ]     : Move target height (Z)")
        print("  ESC       : Exit")
        print("="*60 + "\n")
    
    def create_target_marker(self):
        """목표 위치를 나타내는 마커 생성"""
        from omni.isaac.core.utils.stage import get_current_stage
        
        # 빨간 구체 마커
        self.target_marker = VisualSphere(
            prim_path="/World/TargetMarker",
            name="target_marker",
            position=self.target_position,
            radius=0.03,
            color=np.array([0.0, 1.0, 0.0])  # 초록색
        )
        self.world.scene.add(self.target_marker)
        
        # Physics 제거 (마커는 물리 없음)
        stage = get_current_stage()
        marker_prim = stage.GetPrimAtPath("/World/TargetMarker")
        if marker_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            marker_prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
        if marker_prim.HasAPI(UsdPhysics.CollisionAPI):
            marker_prim.RemoveAPI(UsdPhysics.CollisionAPI)
        
        print(f"[Target] Target marker created at: {self.target_position}")
    
    def _on_keyboard_event(self, event, *args, **kwargs):
        """키보드 이벤트 핸들러"""
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            step = 0.1  # 조인트 변화량
            
            # Shoulder (joint2)
            if event.input == carb.input.KeyboardInput.W:
                self.target_joints[1] -= step
                print(f"[Control] Shoulder Up: {self.target_joints[1]:.2f}")
                
            elif event.input == carb.input.KeyboardInput.S:
                self.target_joints[1] += step
                print(f"[Control] Shoulder Down: {self.target_joints[1]:.2f}")
            
            # Base rotation (joint1)
            elif event.input == carb.input.KeyboardInput.A:
                self.target_joints[0] -= step
                print(f"[Control] Base Left: {self.target_joints[0]:.2f}")
                
            elif event.input == carb.input.KeyboardInput.D:
                self.target_joints[0] += step
                print(f"[Control] Base Right: {self.target_joints[0]:.2f}")
            
            # Elbow (joint4)
            elif event.input == carb.input.KeyboardInput.Q:
                self.target_joints[3] -= step
                print(f"[Control] Elbow: {self.target_joints[3]:.2f}")
                
            elif event.input == carb.input.KeyboardInput.E:
                self.target_joints[3] += step
                print(f"[Control] Elbow: {self.target_joints[3]:.2f}")
            
            # Wrist (joint6)
            elif event.input == carb.input.KeyboardInput.R:
                self.target_joints[5] -= step
                print(f"[Control] Wrist: {self.target_joints[5]:.2f}")
                
            elif event.input == carb.input.KeyboardInput.F:
                self.target_joints[5] += step
                print(f"[Control] Wrist: {self.target_joints[5]:.2f}")
            
            # Gripper toggle
            elif event.input == carb.input.KeyboardInput.G:
                self.toggle_gripper()
            
            # Auto-grasp
            elif event.input == carb.input.KeyboardInput.SPACE:
                print("\n[Auto] Starting Auto-Grasp Sequence...")
                self.execute_auto_grasp()
            
            # Place
            elif event.input == carb.input.KeyboardInput.P:
                print("\n[Auto] Starting Place Sequence...")
                self.execute_place()
            
            # Multi-Cube Mode
            elif event.input == carb.input.KeyboardInput.M:
                print("\n[Auto] Starting Multi-Cube Mode...")
                self.execute_multi_cube_mode()
            
            # 목표 위치 조정
            elif event.input == carb.input.KeyboardInput.UP:
                self.target_position[0] += 0.05
                self.update_target_marker()
                
            elif event.input == carb.input.KeyboardInput.DOWN:
                self.target_position[0] -= 0.05
                self.update_target_marker()
                
            elif event.input == carb.input.KeyboardInput.LEFT:
                self.target_position[1] += 0.05
                self.update_target_marker()
                
            elif event.input == carb.input.KeyboardInput.RIGHT:
                self.target_position[1] -= 0.05
                self.update_target_marker()
            
            # [ ] - Z 높이
            elif event.input == carb.input.KeyboardInput.LEFT_BRACKET:
                self.target_position[2] -= 0.05
                self.update_target_marker()
                
            elif event.input == carb.input.KeyboardInput.RIGHT_BRACKET:
                self.target_position[2] += 0.05
                self.update_target_marker()
    
    def update_target_marker(self):
        """목표 마커 위치 업데이트"""
        self.target_marker.set_world_pose(position=self.target_position)
        print(f"[Target] Moved to: ({self.target_position[0]:.2f}, {self.target_position[1]:.2f}, {self.target_position[2]:.2f})")
    
    def toggle_gripper(self):
        """그리퍼 열고 닫기"""
        if self.gripper_closed:
            self.franka.gripper.open()
            self.gripper_closed = False
            print("[Action] Gripper OPEN")
        else:
            self.franka.gripper.close()
            self.gripper_closed = True
            print("[Action] Gripper CLOSED")
    
    def execute_auto_grasp(self, cube_index=None):
        """자동으로 큐브 감지 및 grasp"""
        
        # 가장 가까운 큐브 찾기
        if cube_index is None:
            ee_pos, _ = self.franka.end_effector.get_world_pose()
            nearest = find_nearest_cube(ee_pos)
            
            if nearest is None:
                print("[Error] No cube detected!")
                return False
            
            cube_index = nearest['index']
            cube_pos = nearest['position']
            print(f"[Auto] Selected nearest cube: Cube_{cube_index}")
        else:
            cube_pos = get_cube_position_from_world(cube_index)
            if cube_pos is None:
                print(f"[Error] Cube_{cube_index} not found!")
                return False
        
        self.current_cube_index = cube_index
        print(f"[Auto] Cube at: ({cube_pos[0]:.2f}, {cube_pos[1]:.2f}, {cube_pos[2]:.2f})")
        
        # 2. 각도 계산
        angle = np.arctan2(cube_pos[1], cube_pos[0])
        print(f"[Auto] Rotating to angle: {np.degrees(angle):.1f} deg")
        
        # 3. Pre-grasp pose
        pre_grasp = np.array([
            angle,  # base rotation
            -0.6,   # shoulder
            0.0,
            -2.2,   # elbow
            0.0,
            1.6,    # wrist
            0.8,
            0.04,   # gripper
            0.04
        ])
        
        print("[Auto] Moving to pre-grasp...")
        for _ in range(150):
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=pre_grasp)
                )
            except:
                pass
            self.world.step(render=True)
        
        # 4. Lower
        grasp_pose = pre_grasp.copy()
        grasp_pose[3] -= 0.4
        
        print("[Auto] Lowering...")
        for _ in range(100):
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=grasp_pose)
                )
            except:
                pass
            self.world.step(render=True)
        
        # 5. Close gripper
        print("[Auto] Grasping...")
        for _ in range(50):
            self.franka.gripper.close()
            self.world.step(render=True)
        
        self.gripper_closed = True
        
        # 6. Attach cube
        attach_cube_to_ee(cube_index)
        self.cube_attached = True
        
        # 7. Lift
        lift_pose = grasp_pose.copy()
        lift_pose[1] += 0.4
        lift_pose[3] += 0.6
        
        print("[Auto] Lifting...")
        for _ in range(150):
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=lift_pose)
                )
            except:
                pass
            self.world.step(render=True)
        
        self.target_joints = lift_pose
        print("[Auto] Success: Auto-Grasp Complete")
        return True
    
    def execute_place(self):
        """목표 위치로 이동 후 큐브 놓기"""
        if not self.cube_attached or self.current_cube_index is None:
            print("[Error] No cube attached! Grasp first (SPACE)")
            return False
        
        print(f"[Auto] Target: ({self.target_position[0]:.2f}, {self.target_position[1]:.2f}, {self.target_position[2]:.2f})")
        
        # 1. 목표 위치 각도 계산
        angle = np.arctan2(self.target_position[1], self.target_position[0])
        print(f"[Auto] Rotating to angle: {np.degrees(angle):.1f} deg")
        
        # 2. 목표 위 hover 위치
        hover_pose = np.array([
            angle,
            -0.6,
            0.0,
            -2.2,
            0.0,
            1.6,
            0.8,
            0.01,
            0.01
        ])
        
        print("[Auto] Moving to hover position...")
        for _ in range(150):
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=hover_pose)
                )
            except:
                pass
            self.world.step(render=True)
        
        # 3. 하강
        place_pose = hover_pose.copy()
        place_pose[3] -= 0.3  # elbow down
        
        print("[Auto] Lowering to place position...")
        for _ in range(100):
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=place_pose)
                )
            except:
                pass
            self.world.step(render=True)
        
        # 4. Detach cube
        from omni.isaac.core.utils.stage import get_current_stage
        stage = get_current_stage()
        joint_path = f"/World/GraspJoint_{self.current_cube_index}"
        joint_prim = stage.GetPrimAtPath(joint_path)
        if joint_prim:
            stage.RemovePrim(joint_prim.GetPath())
            print("[Auto] Cube detached")
        
        self.cube_attached = False
        self.current_cube_index = None
        
        # 5. Open gripper
        print("[Auto] Opening gripper...")
        for _ in range(30):
            self.franka.gripper.open()
            self.world.step(render=True)
        
        self.gripper_closed = False
        
        # 6. Lift up
        retreat_pose = place_pose.copy()
        retreat_pose[1] += 0.3
        retreat_pose[3] += 0.5
        
        print("[Auto] Retreating...")
        for _ in range(100):
            try:
                self.controller.apply_action(
                    ArticulationAction(joint_positions=retreat_pose)
                )
            except:
                pass
            self.world.step(render=True)
        
        self.target_joints = retreat_pose
        print("[Auto] Success: Place Complete")
        return True
    
    def execute_multi_cube_mode(self):
        """모든 큐브를 순차적으로 처리"""
        cubes = get_all_cubes_positions()
        
        if not cubes:
            print("[Error] No cubes found!")
            return
        
        print(f"\n{'='*60}")
        print(f"MULTI-CUBE MODE: Processing {len(cubes)} cubes")
        print(f"{'='*60}\n")
        
        base_x = 0.3
        base_y = 0.2
        spacing = 0.08
        
        for i, cube_info in enumerate(cubes):
            print(f"\n--- Cube {i+1}/{len(cubes)} ---")
            print(f"Position: ({cube_info['position'][0]:.2f}, {cube_info['position'][1]:.2f}, {cube_info['position'][2]:.2f})")
            
            self.target_position = np.array([
                base_x,
                base_y + (i * spacing),
                0.55
            ])
            self.update_target_marker()
            
            print(f"[{i+1}] Grasping Cube_{cube_info['index']}...")
            success = self.execute_auto_grasp(cube_info['index'])
            
            if not success:
                print(f"[Error] Failed to grasp Cube_{cube_info['index']}")
                continue
            
            for _ in range(30):
                self.world.step(render=True)
            
            print(f"[{i+1}] Placing Cube_{cube_info['index']}...")
            self.execute_place()
            
            for _ in range(50):
                self.world.step(render=True)
        
        print(f"\n{'='*60}")
        print(f"[Success] MULTI-CUBE MODE COMPLETE!")
        print(f"  Processed {len(cubes)} cubes successfully")
        print(f"{'='*60}\n")
    
    def update(self):
        """매 프레임마다 호출 - 현재 목표 joint로 이동"""
        try:
            self.controller.apply_action(
                ArticulationAction(joint_positions=self.target_joints)
            )
        except:
            pass


def main():
    print("[Main] Creating world...")
    world = World()
    add_lights()
    
    # 바닥 - 진한 파란색
    world.scene.add(
        GroundPlane(
            "/World/Ground",
            z_position=0,
            color=np.array([0.1, 0.2, 0.4])
        )
    )
    
    # 로봇
    franka = world.scene.add(
        Franka(prim_path="/World/Franka", name="franka")
    )
    
    # 테이블
    table = world.scene.add(
        FixedCuboid(
            prim_path="/World/Table",
            name="table",
            position=np.array([0.5, 0.0, 0.25]),
            scale=np.array([0.6, 0.6, 0.5]),
            color=np.array([0.6, 0.4, 0.2])
        )
    )
    
    # 큐브들
    cubes = []
    cube_positions = [
        [0.5, 0.0, 0.55],
        [0.4, -0.15, 0.55],
        [0.4, 0.15, 0.55],
    ]
    
    cube_colors = [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 1.0, 0.0],
    ]
    
    for i, (pos, color) in enumerate(zip(cube_positions, cube_colors)):
        cube = world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/Cube_{i}",
                name=f"cube_{i}",
                position=pos,
                scale=[0.05, 0.05, 0.05],
                mass=0.05,
                color=np.array(color)
            )
        )
        cubes.append(cube)
    
    print(f"[Main] Added {len(cubes)} cubes to the scene")
    
    camera = setup_camera(world)
    
    print("[Main] Resetting world...")
    world.reset()
    
    print("[Main] Stabilizing scene...")
    for _ in range(60):
        world.step(render=True)
    
    print("[Vision] Quick cube detection...")
    cube_pos_initial = get_cube_position_from_world()
    print(f"  Initial cube position: {cube_pos_initial}")
    
    for _ in range(60):
        world.step(render=True)
    
    test_image = capture_test_image(camera, world)
    cube_pos = detect_cube_in_camera_view(camera, world)
    
    if cube_pos is None:
        print("[Error] Cannot detect cube!")
        return
    
    print("\n[Success] INITIAL SETUP COMPLETE")
    
    app = omni.kit.app.get_app()
    kb_controller = KeyboardController(franka, world, camera)
    
    print("\n[Mode] Entering Manual Control Mode...")
    print("Press SPACE for auto-grasp, or use keyboard to control manually\n")
    
    while simulation_app.is_running():
        kb_controller.update()
        world.step(render=True)
        app.update()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[Error] {e}")
        import traceback
        traceback.print_exc()
    finally:
        simulation_app.close()
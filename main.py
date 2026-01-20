"""
통합 로봇 학습 시스템
Phase 1: Joint Control (기존 방식)
Phase 2: IK Control (정밀 제어)
Phase 3: Vision-based (카메라 인식)
Phase 4: RL (강화학습 - 준비중)
"""

from isaacsim.simulation_app import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import omni.kit.app
from pxr import UsdLux, UsdPhysics, Gf, UsdGeom
import time
import cv2

from omni.isaac.core import World
from omni.isaac.core.objects import GroundPlane, DynamicCuboid, FixedCuboid, VisualSphere
from omni.isaac.franka import Franka
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.rotations import euler_angles_to_quat

from omni.isaac.sensor import Camera
import matplotlib.pyplot as plt
import carb.input


# ============================================================
# 유틸리티 함수
# ============================================================

def get_cube_position_from_world(cube_index=0):
    """World에서 직접 큐브 위치 가져오기 (Ground Truth)"""
    stage = get_current_stage()
    cube_path = f"/World/Cube_{cube_index}"
    cube = stage.GetPrimAtPath(cube_path)
    
    if not cube or not cube.IsValid():
        return None
    
    xform = UsdGeom.Xformable(cube)
    pos = xform.ComputeLocalToWorldTransform(0).ExtractTranslation()
    return np.array([pos[0], pos[1], pos[2]])


def get_all_cubes_positions():
    """모든 큐브의 위치 가져오기"""
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
        distance = np.sqrt((pos[0] - robot_position[0])**2 + 
                          (pos[1] - robot_position[1])**2)
        
        if distance < min_distance:
            min_distance = distance
            nearest = cube_info
    
    return nearest


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
    
    return True


def detach_cube(cube_index):
    """큐브 분리"""
    stage = get_current_stage()
    joint_path = f"/World/GraspJoint_{cube_index}"
    joint_prim = stage.GetPrimAtPath(joint_path)
    if joint_prim:
        stage.RemovePrim(joint_prim.GetPath())
        return True
    return False


def add_lights():
    """조명 설정"""
    stage = get_current_stage()
    
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(1000)
    dome.CreateColorAttr((0.8, 0.9, 1.0))
    
    sun = UsdLux.DistantLight.Define(stage, "/World/DirectionalLight")
    sun.CreateIntensityAttr(5000)
    sun.CreateColorAttr((1.0, 0.95, 0.85))
    xform = UsdGeom.Xformable(sun)
    xform.AddRotateXYZOp().Set((-45, 45, 0))


def setup_camera(world):
    """카메라 설치"""
    camera = Camera(
        prim_path="/World/Camera",
        position=np.array([0.5, 0.0, 1.2]),
        frequency=20,
        resolution=(512, 512),
        name="top_camera"
    )
    world.scene.add(camera)
    
    camera.set_local_pose(
        translation=np.array([0.5, 0.0, 1.2]),
        orientation=np.array([0.7071, 0, 0, -0.7071])
    )
    
    return camera


# ============================================================
# Vision 시스템 (Phase 3)
# ============================================================

class VisionSystem:
    """Phase 3: 카메라 기반 물체 인식"""
    
    def __init__(self, camera, world):
        self.camera = camera
        self.world = world
        self.camera.initialize()
    
    def detect_cubes_from_camera(self):
        """카메라 이미지에서 큐브 감지"""
        for _ in range(10):
            self.world.step(render=True)
        
        rgb = self.camera.get_rgba()[:, :, :3]
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        
        detected_cubes = []
        
        color_ranges = {
            'red': ([0, 120, 70], [10, 255, 255]),
            'blue': ([100, 150, 0], [130, 255, 255]),
            'yellow': ([20, 100, 100], [30, 255, 255])
        }
        
        for color_name, (lower, upper) in color_ranges.items():
            lower = np.array(lower)
            upper = np.array(upper)
            
            mask = cv2.inRange(hsv, lower, upper)
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 100:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        img_h, img_w = rgb.shape[:2]
                        norm_x = (cx - img_w/2) / (img_w/2)
                        norm_y = (cy - img_h/2) / (img_h/2)
                        
                        fov_scale = 0.5
                        world_x = 0.5 - norm_y * fov_scale
                        world_y = -norm_x * fov_scale
                        world_z = 0.55
                        
                        detected_cubes.append({
                            'color': color_name,
                            'position': np.array([world_x, world_y, world_z]),
                            'pixel_pos': (cx, cy),
                            'area': area
                        })
        
        return detected_cubes
    
    def visualize_detection(self, detected_cubes):
        """감지 결과 시각화"""
        rgb = self.camera.get_rgba()[:, :, :3]
        vis_img = rgb.copy()
        
        for cube in detected_cubes:
            cx, cy = cube['pixel_pos']
            cv2.circle(vis_img, (cx, cy), 10, (0, 255, 0), -1)
            cv2.putText(vis_img, cube['color'], (cx+15, cy), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        plt.figure(figsize=(8, 8))
        plt.imshow(vis_img)
        plt.title("Vision Detection Results")
        plt.axis('off')
        plt.savefig("vision_detection.png", dpi=150, bbox_inches='tight')
        plt.close()
        
        print(f"[Vision] Detected {len(detected_cubes)} cubes")
        for cube in detected_cubes:
            print(f"  {cube['color']}: ({cube['position'][0]:.2f}, {cube['position'][1]:.2f})")


# ============================================================
# 통합 컨트롤러 (모든 Phase 포함)
# ============================================================

class UnifiedController:
    """Phase 1~4를 통합한 컨트롤러"""
    
    def __init__(self, franka, world, camera):
        self.franka = franka
        self.world = world
        self.camera = camera
        self.controller = franka.get_articulation_controller()
        
        # Vision 시스템
        self.vision = VisionSystem(camera, world)
        
        # 현재 모드
        self.current_mode = 1  # 1: Joint, 2: IK, 3: Vision, 4: RL
        
        # Joint 상태
        self.target_joints = franka.get_joint_positions()
        
        # 상태
        self.gripper_closed = False
        self.cube_attached = False
        self.current_cube_index = None
        self.target_position = np.array([0.3, 0.3, 0.55])
        
        # 성능 로그
        self.performance_logs = {
            'phase1_joint': {'attempts': 0, 'successes': 0, 'times': [], 'errors': []},
            'phase2_ik': {'attempts': 0, 'successes': 0, 'times': [], 'errors': []},
            'phase3_vision': {'attempts': 0, 'successes': 0, 'times': [], 'errors': []},
            'phase4_rl': {'attempts': 0, 'successes': 0, 'times': [], 'errors': []}
        }
        
        # 목표 마커
        self.create_target_marker()
        
        # 키보드
        appwindow = omni.appwindow.get_default_app_window()
        input_iface = carb.input.acquire_input_interface()
        self.keyboard = appwindow.get_keyboard()
        self.sub_keyboard = input_iface.subscribe_to_keyboard_events(
            self.keyboard, self._on_keyboard_event
        )
        
        self.print_controls()
    
    def print_controls(self):
        print("\n" + "="*60)
        print("통합 로봇 학습 시스템")
        print("="*60)
        print("MODE SELECTION:")
        print("  1         : Phase 1 - Joint Control (기존 방식)")
        print("  2         : Phase 2 - IK Control (정밀 제어)")
        print("  3         : Phase 3 - Vision-based (카메라 인식)")
        print("  4         : Phase 4 - RL (강화학습 - 준비중)")
        print("")
        print("ACTIONS:")
        print("  SPACE     : Auto-Grasp (현재 모드)")
        print("  P         : Place")
        print("  M         : Multi-Cube Mode")
        print("  V         : Vision Test")
        print("  L         : Show Performance Log")
        print("  C         : Compare All Phases")
        print("")
        print("MANUAL CONTROL (Phase 1 only):")
        print("  W/S       : Shoulder Up/Down")
        print("  A/D       : Base Rotate")
        print("  Q/E       : Elbow")
        print("  R/F       : Wrist")
        print("  G         : Toggle Gripper")
        print("")
        print("  Arrow Keys: Move target")
        print("  [ / ]     : Adjust height")
        print("="*60)
        print(f"Current Mode: Phase {self.current_mode}")
        print("="*60 + "\n")
    
    def create_target_marker(self):
        """목표 마커"""
        self.target_marker = VisualSphere(
            prim_path="/World/TargetMarker",
            name="target_marker",
            position=self.target_position,
            radius=0.03,
            color=np.array([0.0, 1.0, 0.0])
        )
        self.world.scene.add(self.target_marker)
        
        stage = get_current_stage()
        marker_prim = stage.GetPrimAtPath("/World/TargetMarker")
        if marker_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            marker_prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
        if marker_prim.HasAPI(UsdPhysics.CollisionAPI):
            marker_prim.RemoveAPI(UsdPhysics.CollisionAPI)
    
    def _on_keyboard_event(self, event, *args, **kwargs):
        """키보드 이벤트"""
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            
            # 모드 전환
            if event.input == carb.input.KeyboardInput.KEY_1:
                self.current_mode = 1
                print(f"\n[Mode] Phase 1: Joint Control")
            elif event.input == carb.input.KeyboardInput.KEY_2:
                self.current_mode = 2
                print(f"\n[Mode] Phase 2: IK Control")
            elif event.input == carb.input.KeyboardInput.KEY_3:
                self.current_mode = 3
                print(f"\n[Mode] Phase 3: Vision-based")
            elif event.input == carb.input.KeyboardInput.KEY_4:
                self.current_mode = 4
                print(f"\n[Mode] Phase 4: RL (준비중)")
            
            # 액션
            elif event.input == carb.input.KeyboardInput.SPACE:
                self.execute_grasp()
            elif event.input == carb.input.KeyboardInput.P:
                self.execute_place()
            elif event.input == carb.input.KeyboardInput.M:
                self.execute_multi_cube()
            elif event.input == carb.input.KeyboardInput.V:
                self.test_vision()
            elif event.input == carb.input.KeyboardInput.L:
                self.show_current_log()
            elif event.input == carb.input.KeyboardInput.C:
                self.compare_all_phases()
            
            # Phase 1 전용: 수동 Joint 제어
            elif self.current_mode == 1:
                step = 0.1
                if event.input == carb.input.KeyboardInput.W:
                    self.target_joints[1] -= step
                    print(f"[Joint] Shoulder Up: {self.target_joints[1]:.2f}")
                elif event.input == carb.input.KeyboardInput.S:
                    self.target_joints[1] += step
                    print(f"[Joint] Shoulder Down: {self.target_joints[1]:.2f}")
                elif event.input == carb.input.KeyboardInput.A:
                    self.target_joints[0] -= step
                    print(f"[Joint] Base Left: {self.target_joints[0]:.2f}")
                elif event.input == carb.input.KeyboardInput.D:
                    self.target_joints[0] += step
                    print(f"[Joint] Base Right: {self.target_joints[0]:.2f}")
                elif event.input == carb.input.KeyboardInput.Q:
                    self.target_joints[3] -= step
                    print(f"[Joint] Elbow: {self.target_joints[3]:.2f}")
                elif event.input == carb.input.KeyboardInput.E:
                    self.target_joints[3] += step
                    print(f"[Joint] Elbow: {self.target_joints[3]:.2f}")
                elif event.input == carb.input.KeyboardInput.R:
                    self.target_joints[5] -= step
                    print(f"[Joint] Wrist: {self.target_joints[5]:.2f}")
                elif event.input == carb.input.KeyboardInput.F:
                    self.target_joints[5] += step
                    print(f"[Joint] Wrist: {self.target_joints[5]:.2f}")
                elif event.input == carb.input.KeyboardInput.G:
                    self.toggle_gripper()
            
            # 목표 위치 조정
            if event.input == carb.input.KeyboardInput.UP:
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
            elif event.input == carb.input.KeyboardInput.LEFT_BRACKET:
                self.target_position[2] -= 0.05
                self.update_target_marker()
            elif event.input == carb.input.KeyboardInput.RIGHT_BRACKET:
                self.target_position[2] += 0.05
                self.update_target_marker()
    
    def update_target_marker(self):
        self.target_marker.set_world_pose(position=self.target_position)
        print(f"[Target] ({self.target_position[0]:.2f}, {self.target_position[1]:.2f}, {self.target_position[2]:.2f})")
    
    def toggle_gripper(self):
        """그리퍼 토글"""
        if self.gripper_closed:
            self.franka.gripper.open()
            self.gripper_closed = False
            print("[Gripper] OPEN")
        else:
            self.franka.gripper.close()
            self.gripper_closed = True
            print("[Gripper] CLOSED")
    
    # ========== 공통 인터페이스 ==========
    
    def execute_grasp(self):
        """현재 모드에 따라 grasp 실행"""
        if self.current_mode == 1:
            self.execute_joint_grasp()
        elif self.current_mode == 2:
            self.execute_ik_grasp()
        elif self.current_mode == 3:
            self.execute_vision_grasp()
        elif self.current_mode == 4:
            print("[RL] Phase 4 준비중...")
    
    def execute_place(self):
        """현재 모드에 따라 place 실행"""
        if self.current_mode == 1:
            self.execute_joint_place()
        elif self.current_mode == 2:
            self.execute_ik_place()
        elif self.current_mode == 3:
            self.execute_ik_place()  # Vision도 place는 IK 사용
        elif self.current_mode == 4:
            print("[RL] Phase 4 준비중...")
    
    def execute_multi_cube(self):
        """Multi-cube 모드"""
        if self.current_mode == 1:
            self.execute_multi_cube_joint()
        elif self.current_mode == 2:
            self.execute_multi_cube_ik()
        elif self.current_mode == 3:
            self.execute_multi_cube_vision()
        elif self.current_mode == 4:
            print("[RL] Phase 4 준비중...")
    
    # ========== Phase 1: Joint Control ==========
    
    def execute_joint_grasp(self, cube_index=None):
        """Phase 1: Joint 제어 기반 grasp"""
        start_time = time.time()
        log = self.performance_logs['phase1_joint']
        log['attempts'] += 1
        
        print("\n[Phase 1: Joint] Starting grasp...")
        
        if cube_index is None:
            ee_pos, _ = self.franka.end_effector.get_world_pose()
            nearest = find_nearest_cube(ee_pos)
            if not nearest:
                return False
            cube_index = nearest['index']
            cube_pos = nearest['position']
        else:
            cube_pos = get_cube_position_from_world(cube_index)
        
        self.current_cube_index = cube_index
        
        # Open gripper
        for _ in range(30):
            self.franka.gripper.open()
            self.world.step(render=True)
        self.gripper_closed = False
        
        # 각도 계산
        angle = np.arctan2(cube_pos[1], cube_pos[0])
        
        # Pre-grasp
        pre_grasp = np.array([angle, -0.5, 0.0, -2.0, 0.0, 1.8, 0.8, 0.04, 0.04])
        for _ in range(200):
            try:
                self.controller.apply_action(ArticulationAction(joint_positions=pre_grasp))
            except:
                pass
            self.world.step(render=True)
        
        # Approach
        grasp_pose = pre_grasp.copy()
        grasp_pose[3] -= 0.5
        for _ in range(100):
            try:
                self.controller.apply_action(ArticulationAction(joint_positions=grasp_pose))
            except:
                pass
            self.world.step(render=True)
        
        # Close gripper
        for _ in range(60):
            self.franka.gripper.close()
            self.world.step(render=True)
        self.gripper_closed = True
        
        for _ in range(20):
            self.world.step(render=True)
        
        attach_cube_to_ee(cube_index)
        self.cube_attached = True
        
        # Lift
        lift_pose = grasp_pose.copy()
        lift_pose[1] += 0.3
        lift_pose[3] += 0.6
        for _ in range(150):
            try:
                self.controller.apply_action(ArticulationAction(joint_positions=lift_pose))
            except:
                pass
            self.world.step(render=True)
        
        self.target_joints = lift_pose
        
        elapsed = time.time() - start_time
        log['successes'] += 1
        log['times'].append(elapsed)
        
        print(f"[Phase 1] ✓ Complete in {elapsed:.2f}s")
        return True
    
    def execute_joint_place(self):
        """Phase 1: Joint 제어 기반 place"""
        if not self.cube_attached:
            return False
        
        print("\n[Phase 1: Joint] Placing...")
        
        angle = np.arctan2(self.target_position[1], self.target_position[0])
        hover_pose = np.array([angle, -0.5, 0.0, -2.0, 0.0, 1.8, 0.8, 0.01, 0.01])
        
        for _ in range(200):
            try:
                self.controller.apply_action(ArticulationAction(joint_positions=hover_pose))
            except:
                pass
            self.world.step(render=True)
        
        place_pose = hover_pose.copy()
        place_pose[3] -= 0.4
        for _ in range(100):
            try:
                self.controller.apply_action(ArticulationAction(joint_positions=place_pose))
            except:
                pass
            self.world.step(render=True)
        
        for _ in range(30):
            self.world.step(render=True)
        
        detach_cube(self.current_cube_index)
        self.cube_attached = False
        self.current_cube_index = None
        
        for _ in range(40):
            self.franka.gripper.open()
            self.world.step(render=True)
        self.gripper_closed = False
        
        retreat_pose = place_pose.copy()
        retreat_pose[1] += 0.3
        retreat_pose[3] += 0.5
        for _ in range(120):
            try:
                self.controller.apply_action(ArticulationAction(joint_positions=retreat_pose))
            except:
                pass
            self.world.step(render=True)
        
        self.target_joints = retreat_pose
        print("[Phase 1] ✓ Place complete")
        return True
    
    def execute_multi_cube_joint(self):
        """Phase 1: Multi-cube"""
        cubes = get_all_cubes_positions()
        print(f"\n[Phase 1] Processing {len(cubes)} cubes...")
        
        for i, cube in enumerate(cubes):
            self.target_position = np.array([0.3, 0.2 + i*0.08, 0.55])
            self.update_target_marker()
            
            self.execute_joint_grasp(cube['index'])
            for _ in range(20):
                self.world.step(render=True)
            
            self.execute_joint_place()
            for _ in range(30):
                self.world.step(render=True)
    
    # ========== Phase 2: IK Control ==========
    
    def move_to_position_ik(self, target_position, steps=150):
        """IK로 목표 위치 이동"""
        current_pos, _ = self.franka.end_effector.get_world_pose()
        target_ori = euler_angles_to_quat(np.array([np.pi, 0, 0]))
        
        for i in range(steps):
            alpha = (i + 1) / steps
            interp_pos = current_pos + alpha * (target_position - current_pos)
            
            self.franka.end_effector.set_world_pose(
                position=interp_pos,
                orientation=target_ori
            )
            self.world.step(render=True)
        
        final_pos, _ = self.franka.end_effector.get_world_pose()
        error = np.linalg.norm(final_pos - target_position)
        return error
    
    def execute_ik_grasp(self, cube_index=None):
        """Phase 2: IK grasp"""
        start_time = time.time()
        log = self.performance_logs['phase2_ik']
        log['attempts'] += 1
        
        print("\n[Phase 2: IK] Starting grasp...")
        
        if cube_index is None:
            ee_pos, _ = self.franka.end_effector.get_world_pose()
            nearest = find_nearest_cube(ee_pos)
            if not nearest:
                return False
            cube_index = nearest['index']
            cube_pos = nearest['position']
        else:
            cube_pos = get_cube_position_from_world(cube_index)
        
        self.current_cube_index = cube_index
        print(f"  Cube_{cube_index} at ({cube_pos[0]:.2f}, {cube_pos[1]:.2f}, {cube_pos[2]:.2f})")
        
        # Open gripper
        for _ in range(30):
            self.franka.gripper.open()
            self.world.step(render=True)
        self.gripper_closed = False
        
        # Pre-grasp: 10cm 위
        pre_grasp = cube_pos.copy()
        pre_grasp[2] += 0.10
        error1 = self.move_to_position_ik(pre_grasp, 200)
        for _ in range(20):
            self.world.step(render=True)
        
        # Approach: 2cm 위
        approach = cube_pos.copy()
        
        approach[2] += 0.02
        error2 = self.move_to_position_ik(approach, 100)
        for _ in range(20):
            self.world.step(render=True)
        
        # Close gripper
        for _ in range(60):
            self.franka.gripper.close()
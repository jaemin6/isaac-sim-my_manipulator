# controllers/Unified Controller
"""
- 모든 Phase (1~4) 통합 관리
- 키보드로 모드 전환
- 각 Phase별 성능 비교
"""

import sys
import os
import numpy as np
import carb.input
from omni.isaac.core.objects import VisualSphere
from pxr import UsdPhysics
from omni.isaac.core.utils.stage import get_current_stage

# 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
utils_dir = os.path.join(parent_dir, 'utils')

sys.path.insert(0, parent_dir)
sys.path.insert(0, utils_dir)

from controllers.joint_control import JointController
from controllers.ik_control import IKController
from controllers.vision_control import VisionController
from cube_utils import get_all_cubes


class UnifiedController:
    """모든 Phase를 통합한 컨트롤러"""
    
    def __init__(self, franka, world, camera):
        self.franka = franka
        self.world = world
        self.camera = camera
        
        # Phase 컨트롤러들
        self.phase1 = JointController(franka, world)
        self.phase2 = IKController(franka, world)
        self.phase3 = VisionController(franka, world, camera)
        # Phase 4는 나중에 추가

        # Phase 3 보정 플래그 추가
        self.phase3.calibrated = False

        # 현재 모드
        self.current_phase = 1
        
        # 목표 위치
        self.target_position = np.array([0.3, 0.3, 0.55])
        
        # 목표 마커 생성
        self.create_target_marker()
        
        # 키보드 설정
        self.setup_keyboard()
        
        self.print_controls()
    
    def print_controls(self):
        """컨트롤 가이드 출력"""
        print("\n" + "="*60)
        print("통합 로봇 학습 시스템")
        print("="*60)
        print("MODE SELECTION:")
        print("  1         : Phase 1 - Joint Control (기존 방식)")
        print("  2         : Phase 2 - IK Control (정밀 제어)")
        print("  3         : Phase 3 - Vision (카메라 인식) [준비중]")
        print("  4         : Phase 4 - RL (강화학습) [준비중]")
        print("")
        print("ACTIONS:")
        print("  SPACE     : Auto-Grasp (현재 모드)")
        print("  P         : Place")
        print("  M         : Multi-Cube Mode (모든 큐브 처리)")
        print("  L         : Show Performance (Phase 2)")
        print("  C         : Compare All Phases")
        print("")
        print("MANUAL CONTROL (Phase 1 only):")
        print("  W/S       : Shoulder Up/Down")
        print("  A/D       : Base Rotate")
        print("  Q/E       : Elbow")
        print("  R/F       : Wrist")
        print("  G         : Toggle Gripper")
        print("")
        print("TARGET POSITION:")
        print("  Arrow Keys: Move target (X/Y)")
        print("  [ / ]     : Adjust height (Z)")
        print("="*60)
        print(f"Current Phase: {self.current_phase}")
        print("="*60 + "\n")
    
    def create_target_marker(self):
        """목표 위치 마커 생성"""
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
    
    def setup_keyboard(self):
        """키보드 입력 설정"""
        import omni.appwindow
        appwindow = omni.appwindow.get_default_app_window()
        input_iface = carb.input.acquire_input_interface()
        self.keyboard = appwindow.get_keyboard()
        self.sub_keyboard = input_iface.subscribe_to_keyboard_events(
            self.keyboard, self._on_keyboard_event
        )
    
    def _on_keyboard_event(self, event, *args, **kwargs):
        """키보드 이벤트 핸들러"""
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            
            # ===== 모드 전환 =====
            if event.input == carb.input.KeyboardInput.KEY_1:
                self.current_phase = 1
                print(f"\n[Mode] Phase 1: Joint Control")
                self.print_phase_info()
            
            elif event.input == carb.input.KeyboardInput.KEY_2:
                self.current_phase = 2
                print(f"\n[Mode] Phase 2: IK Control")
                self.print_phase_info()
            
            elif event.input == carb.input.KeyboardInput.KEY_3:
                self.current_phase = 3
                print(f"\n[Mode] Phase 3: Vision-based")
                self.print_phase_info()
            
            elif event.input == carb.input.KeyboardInput.KEY_4:
                self.current_phase = 4
                print(f"\n[Mode] Phase 4: RL [준비중]")
            
            # ===== 액션 =====
            elif event.input == carb.input.KeyboardInput.SPACE:
                self.execute_grasp()
            
            elif event.input == carb.input.KeyboardInput.P:
                self.execute_place()
            
            elif event.input == carb.input.KeyboardInput.M:
                self.execute_multi_cube()
            
            elif event.input == carb.input.KeyboardInput.L:
                self.show_performance()
            
            elif event.input == carb.input.KeyboardInput.C:
                self.compare_phases()
            
            # ===== Phase 1 전용: 수동 제어 =====
            elif self.current_phase == 1:
                if event.input == carb.input.KeyboardInput.W:
                    self.phase1.manual_control('w')
                elif event.input == carb.input.KeyboardInput.S:
                    self.phase1.manual_control('s')
                elif event.input == carb.input.KeyboardInput.A:
                    self.phase1.manual_control('a')
                elif event.input == carb.input.KeyboardInput.D:
                    self.phase1.manual_control('d')
                elif event.input == carb.input.KeyboardInput.Q:
                    self.phase1.manual_control('q')
                elif event.input == carb.input.KeyboardInput.E:
                    self.phase1.manual_control('e')
                elif event.input == carb.input.KeyboardInput.R:
                    self.phase1.manual_control('r')
                elif event.input == carb.input.KeyboardInput.F:
                    self.phase1.manual_control('f')
                elif event.input == carb.input.KeyboardInput.G:
                    self.phase1.manual_control('g')
            
            # ===== 목표 위치 조정 =====
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
        """목표 마커 위치 업데이트"""
        self.target_marker.set_world_pose(position=self.target_position)
        print(f"[Target] ({self.target_position[0]:.2f}, {self.target_position[1]:.2f}, {self.target_position[2]:.2f})")
    
    def print_phase_info(self):
        """현재 Phase 정보 출력"""
        if self.current_phase == 1:
            print("  - Joint angle 기반 제어")
            print("  - W/S/A/D/Q/E/R/F로 수동 제어 가능")
        elif self.current_phase == 2:
            print("  - Inverse Kinematics 기반")
            print("  - mm 단위 정밀 제어")
            print("  - 성능 측정 자동 기록")
        elif self.current_phase == 3:
            print("  - 카메라 비전 기반 인식")
            print("  - RGB 색상으로 큐브 감지")
            print("  - Detection 오차 측정")
    
    # ===== 공통 인터페이스 =====
    
    def execute_grasp(self):
        """현재 Phase에 따라 grasp 실행"""
        if self.current_phase == 1:
            self.phase1.auto_grasp()
        elif self.current_phase == 2:
            self.phase2.auto_grasp()
        elif self.current_phase == 3:
            if not self.phase3_calibrated:
                print("\n[Setup] Running camera calibration for Phase 3...")
                if self.phase3.calibrate_homography():
                    self.phase3_calibrated = True
                    print("[Setup] ✓ Calibration complete!\n")
                else:
                    print("[Warning] Calibration failed. Trying anyway...\n")
                    
            self.phase3.auto_grasp()
        elif self.current_phase == 4:
            print("[Phase 4] RL - 준비중")
    
    def execute_place(self):
        """현재 Phase에 따라 place 실행"""
        if self.current_phase == 1:
            self.phase1.place(self.target_position)
        elif self.current_phase == 2:
            self.phase2.place(self.target_position)
        elif self.current_phase == 3:
            self.phase3.place(self.target_position)
        elif self.current_phase == 4:
            print("[Phase 4] RL - 준비중")
    
    def execute_multi_cube(self):
        """Multi-cube 모드 - 모든 큐브 처리"""
        cubes = get_all_cubes()
        
        if not cubes:
            print("[Error] No cubes found!")
            return
        
        print(f"\n{'='*60}")
        print(f"MULTI-CUBE MODE (Phase {self.current_phase})")
        print(f"Processing {len(cubes)} cubes")
        print(f"{'='*60}\n")
        
        base_x = 0.5  # 테이블 중앙
        base_y = -0.1  # 중앙에서 시작
        spacing = 0.08
        
        for i, cube_info in enumerate(cubes):
            print(f"\n--- Cube {i+1}/{len(cubes)} ---")
            
            # 목표 위치 설정 (테이블 위)
            self.target_position = np.array([
                base_x,
                base_y + (i * spacing),
                0.52  # 테이블 바로 위
            ])
            self.update_target_marker()
            
            # Grasp
            if self.current_phase == 1:
                success = self.phase1.auto_grasp(cube_info['index'])
            elif self.current_phase == 2:
                success = self.phase2.auto_grasp(cube_info['index'])
            elif self.current_phase == 3:
                success = self.phase3.auto_grasp(cube_info['index'])
            else:
                print(f"[Phase {self.current_phase}] 준비중")
                break
            
            if not success:
                print(f"[Warning] Failed to grasp Cube_{cube_info['index']}")
                continue
            
            # 대기
            for _ in range(20):
                self.world.step(render=True)
            
            # Place
            if self.current_phase == 1:
                self.phase1.place(self.target_position)
            elif self.current_phase == 2:
                self.phase2.place(self.target_position)
            elif self.current_phase == 3:
                self.phase3.place(self.target_position)
            
            # 대기
            for _ in range(30):
                self.world.step(render=True)
        
        print(f"\n{'='*60}")
        print(f"✓ MULTI-CUBE COMPLETE")
        print(f"{'='*60}\n")
        
        # Phase 2, 3이면 자동으로 성능 출력
        if self.current_phase == 2:
            self.phase2.print_performance()
        elif self.current_phase == 3:
            self.phase3.print_performance()
    
    def show_performance(self):
        """성능 로그 출력"""
        if self.current_phase == 2:
            self.phase2.print_performance()
        elif self.current_phase == 3:
            self.phase3.print_performance()
        else:
            print(f"[Phase {self.current_phase}] Performance tracking not available")
    
    def compare_phases(self):
        """Phase 간 성능 비교"""
        print(f"\n{'='*60}")
        print("PHASE COMPARISON")
        print(f"{'='*60}\n")
        
        # Phase 2 성능
        if self.phase2.performance['grasp_times']:
            summary2 = self.phase2.get_performance_summary()
            print("Phase 2: IK Control")
            print(f"  Grasps: {summary2['total_grasps']}")
            if 'avg_grasp_time' in summary2:
                print(f"  Avg Grasp Time: {summary2['avg_grasp_time']:.2f}s")
            if 'avg_position_error' in summary2:
                print(f"  Avg Error: {summary2['avg_position_error']*1000:.2f} mm")
            print()
        else:
            print("Phase 2: No data yet\n")
        
        # Phase 3 성능
        if self.phase3.performance['grasp_times']:
            summary3 = self.phase3.get_performance_summary()
            print("Phase 3: Vision Control")
            print(f"  Grasps: {summary3['total_grasps']}")
            if 'avg_grasp_time' in summary3:
                print(f"  Avg Grasp Time: {summary3['avg_grasp_time']:.2f}s")
            if 'avg_detection_error' in summary3:
                print(f"  Avg Detection Error: {summary3['avg_detection_error']*1000:.2f} mm")
            print()
        else:
            print("Phase 3: No data yet\n")
        
        print("Phase 1: Joint Control - Manual tracking needed")
        print("Phase 4: RL - Not implemented")
        
        print(f"\n{'='*60}\n")
    
    def update(self):
        """매 프레임 업데이트"""
        if self.current_phase == 1:
            self.phase1.update()
        elif self.current_phase == 2:
            self.phase2.update()
        elif self.current_phase == 3:
            self.phase3.update()
        # Phase 4는 나중에
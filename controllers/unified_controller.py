# controllers/unified_controller.py
"""
- 모든 Phase (1~4) 통합 관리
- 키보드로 모드 전환
- 각 Phase별 성능 비교

[수정사항]
- 키보드 콜백에서 직접 실행 → pending_action 플래그 방식으로 변경
  (콜백에서 직접 실행하면 app.update()가 안 불려서 UI가 frozen됨)
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

        # Phase 3 보정 플래그
        self.phase3_calibrated = False

        # 현재 모드
        self.current_phase = 1
        
        # 목표 위치
        self.target_position = np.array([0.3, 0.3, 0.55])
        
        # ✅ [수정] 키보드 콜백에서 직접 실행하지 않고 플래그만 세팅
        # None / 'grasp' / 'place' / 'multi'
        self.pending_action = None

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
        print("  3         : Phase 3 - Vision (카메라 인식)")
        print("  4         : Phase 4 - RL (강화학습) [준비중]")
        print("")
        print("ACTIONS:")
        print("  SPACE     : Auto-Grasp (현재 모드)")
        print("  P         : Place")
        print("  M         : Multi-Cube Mode (모든 큐브 처리)")
        print("  L         : Show Performance (Phase 2/3)")
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
        """
        키보드 이벤트 핸들러

        ✅ [수정 핵심]
        여기서 execute_grasp() 같은 긴 작업을 직접 호출하면
        app.update()가 17초 동안 안 불려서 UI가 frozen됨.
        → 플래그(pending_action)만 세팅하고, 실제 실행은 update()에서.
        """
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
            
            # ===== 액션: 직접 실행 X → 플래그만 세팅 =====
            elif event.input == carb.input.KeyboardInput.SPACE:
                if self.pending_action is None:
                    print("[Input] SPACE → grasp 예약됨")
                    self.pending_action = 'grasp'
                else:
                    print(f"[Input] 이미 '{self.pending_action}' 실행 중, 무시됨")
            
            elif event.input == carb.input.KeyboardInput.P:
                if self.pending_action is None:
                    print("[Input] P → place 예약됨")
                    self.pending_action = 'place'
                else:
                    print(f"[Input] 이미 '{self.pending_action}' 실행 중, 무시됨")
            
            elif event.input == carb.input.KeyboardInput.M:
                if self.pending_action is None:
                    print("[Input] M → multi-cube 예약됨")
                    self.pending_action = 'multi'
                else:
                    print(f"[Input] 이미 '{self.pending_action}' 실행 중, 무시됨")
            
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
                print("\n" + "="*60)
                print("PHASE 3: INITIAL CALIBRATION")
                print("="*60)
                print("[Setup] Running camera calibration...")
                if self.phase3.calibrate_homography():
                    self.phase3_calibrated = True
                    print("[Setup] ✓ Calibration complete!")
                    print("="*60 + "\n")
                else:
                    print("[Warning] Calibration failed. Trying anyway...")
                    print("="*60 + "\n")

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
        """
        Multi-cube 모드 - 모든 큐브 처리
        ✅ 상태머신 완료를 기다리도록 수정
        """
        cubes = get_all_cubes()
        
        if not cubes:
            print("[Error] No cubes found!")
            return
        
        print(f"\n{'='*60}")
        print(f"MULTI-CUBE MODE (Phase {self.current_phase})")
        print(f"Processing {len(cubes)} cubes")
        print(f"{'='*60}\n")
        
        # ✅ 테이블 위 목표 위치 (y=0.0 중심으로 양옆으로 배치)
        base_x = 0.5
        base_y = 0.0
        spacing = 0.08
        
        for i, cube_info in enumerate(cubes):
            print(f"\n--- Cube {i+1}/{len(cubes)} ---")
            
            # 목표 위치 설정
            self.target_position = np.array([
                base_x,
                base_y + ((i - len(cubes)/2) * spacing),  # 중심 기준 양옆
                0.52
            ])
            self.update_target_marker()
            
            # ✅ Grasp 시작
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
                print(f"[Warning] Failed to start grasp Cube_{cube_info['index']}")
                continue
            
            # ✅ 상태머신 완료 대기 (Phase 3인 경우)
            if self.current_phase == 3:
                # is_busy()가 False될 때까지 update() 계속 호출
                while self.phase3.is_busy():
                    self.phase3.update()
                    self.world.step(render=True)
                    self.app.update()
            else:
                # Phase 1, 2는 블로킹 방식이라 바로 완료됨
                for _ in range(20):
                    self.world.step(render=True)
            
            if not self.phase3.cube_attached if self.current_phase == 3 else False:
                print(f"[Warning] Grasp failed for Cube_{cube_info['index']}")
                continue
            
            # ✅ Place 시작
            if self.current_phase == 1:
                self.phase1.place(self.target_position)
            elif self.current_phase == 2:
                self.phase2.place(self.target_position)
            elif self.current_phase == 3:
                self.phase3.place(self.target_position)
            
            # ✅ Place 완료 대기
            if self.current_phase == 3:
                while self.phase3.is_busy():
                    self.phase3.update()
                    self.world.step(render=True)
                    self.app.update()
            else:
                for _ in range(30):
                    self.world.step(render=True)
            
            print(f"[Multi-cube] ✓ Cube {i+1} complete")
        
        print(f"\n{'='*60}")
        print(f"✓ MULTI-CUBE COMPLETE")
        print(f"{'='*60}\n")
        
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
        """
        매 프레임 업데이트 - main.py의 루프에서 호출됨

        ✅ [수정 핵심]
        pending_action 플래그를 확인해서 메인 루프 안에서 실행.
        이렇게 해야 execute_grasp() 내부 루프 사이에
        main.py의 app.update()가 정상적으로 불림.
        """
        # 예약된 액션이 있으면 실행
        if self.pending_action is not None:
            action = self.pending_action
            self.pending_action = None  # 먼저 초기화 (중복 실행 방지)

            if action == 'grasp':
                self.execute_grasp()
            elif action == 'place':
                self.execute_place()
            elif action == 'multi':
                self.execute_multi_cube()

        # 각 Phase별 프레임 업데이트
        if self.current_phase == 1:
            self.phase1.update()
        elif self.current_phase == 2:
            self.phase2.update()
        elif self.current_phase == 3:
            self.phase3.update()
        # Phase 4는 나중에
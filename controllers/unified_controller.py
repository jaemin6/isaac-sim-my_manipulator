# controllers/unified_controller.py
"""
- 모든 Phase (1~4) 통합 관리
- 키보드로 모드 전환
- 각 Phase별 성능 비교

[수정사항]
- 키보드 콜백에서 직접 실행 → pending_action 플래그 방식으로 변경
- execute_multi_cube(): 2x2 그리드 → 단일 위치 수직 스태킹으로 변경
"""

import sys
import os
import numpy as np
import carb.input
from omni.isaac.core.objects import VisualSphere
from pxr import UsdPhysics
from omni.isaac.core.utils.stage import get_current_stage

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
utils_dir = os.path.join(parent_dir, 'utils')

sys.path.insert(0, parent_dir)
sys.path.insert(0, utils_dir)

from controllers.joint_control import JointController
from controllers.ik_control import IKController
from controllers.vision_control import VisionController
from cube_utils import get_all_cubes


# ✅ 스태킹 설정 - 씬에 맞게 조정
STACK_TARGET_XY = (0.50, 0.00)   # 쌓을 XY 목표 위치
STACK_BASE_Z    = 0.52            # 테이블 위 기본 Z 높이
CUBE_HEIGHT     = 0.05            # 큐브 한 개 높이


class UnifiedController:
    """모든 Phase를 통합한 컨트롤러"""
    
    def __init__(self, franka, world, camera):
        self.franka = franka
        self.world = world
        self.camera = camera
        
        import omni.kit.app
        self.app = omni.kit.app.get_app()
        
        self.phase1 = JointController(franka, world)
        self.phase2 = IKController(franka, world)
        self.phase3 = VisionController(franka, world, camera)

        self.phase3_calibrated = False
        self.current_phase = 1
        self.target_position = np.array([0.3, 0.3, 0.55])
        self.pending_action = None

        self.create_target_marker()
        self.setup_keyboard()
        self.print_controls()
    
    def print_controls(self):
        print("\n" + "="*60)
        print("통합 로봇 학습 시스템")
        print("="*60)
        print("MODE SELECTION:")
        print("  1         : Phase 1 - Joint Control")
        print("  2         : Phase 2 - IK Control")
        print("  3         : Phase 3 - Vision")
        print("  4         : Phase 4 - RL [준비중]")
        print("")
        print("ACTIONS:")
        print("  SPACE     : Auto-Grasp")
        print("  P         : Place")
        print("  M         : Multi-Cube Stacking Mode")
        print("  L         : Show Performance")
        print("  C         : Compare All Phases")
        print("")
        print("MANUAL CONTROL (Phase 1 only):")
        print("  W/S/A/D/Q/E/R/F/G")
        print("")
        print("TARGET POSITION:")
        print("  Arrow Keys: Move target (X/Y)")
        print("  [ / ]     : Adjust height (Z)")
        print("="*60)
        print(f"Current Phase: {self.current_phase}")
        print("="*60 + "\n")
    
    def create_target_marker(self):
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
    
    def setup_keyboard(self):
        import omni.appwindow
        appwindow = omni.appwindow.get_default_app_window()
        input_iface = carb.input.acquire_input_interface()
        self.keyboard = appwindow.get_keyboard()
        self.sub_keyboard = input_iface.subscribe_to_keyboard_events(
            self.keyboard, self._on_keyboard_event
        )
    
    def _on_keyboard_event(self, event, *args, **kwargs):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            
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
                    print("[Input] M → multi-cube 스태킹 예약됨")
                    self.pending_action = 'multi'
                else:
                    print(f"[Input] 이미 '{self.pending_action}' 실행 중, 무시됨")
            
            elif event.input == carb.input.KeyboardInput.L:
                self.show_performance()
            elif event.input == carb.input.KeyboardInput.C:
                self.compare_phases()
            
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
    
    def print_phase_info(self):
        if self.current_phase == 1:
            print("  - Joint angle 기반 제어")
        elif self.current_phase == 2:
            print("  - Inverse Kinematics 기반")
        elif self.current_phase == 3:
            print("  - 카메라 비전 기반 인식")
            print("  - RGB 색상으로 큐브 감지")
    
    # ===== 공통 인터페이스 =====
    
    def execute_grasp(self):
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
                else:
                    print("[Warning] Calibration failed. Trying anyway...")
                print("="*60 + "\n")
            self.phase3.auto_grasp()
        elif self.current_phase == 4:
            print("[Phase 4] RL - 준비중")
    
    def execute_place(self):
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
        ✅ Multi-cube 스태킹 모드
        모든 큐브를 동일한 XY 위치에 수직으로 쌓음
        """
        cubes = get_all_cubes()
        
        if not cubes:
            print("[Error] No cubes found!")
            return
        
        num_cubes = len(cubes)
        
        print(f"\n{'='*60}")
        print(f"STACKING MODE (Phase {self.current_phase})")
        print(f"Stacking {num_cubes} cubes at XY={STACK_TARGET_XY}")
        print(f"{'='*60}\n")

        # ✅ Phase 3 캘리브레이션 먼저
        if self.current_phase == 3 and not self.phase3_calibrated:
            print("[Setup] Running camera calibration...")
            if self.phase3.calibrate_homography():
                self.phase3_calibrated = True
                print("[Setup] ✓ Calibration complete!")
            else:
                print("[Warning] Calibration failed. Trying anyway...")

        placed_cubes = set()   # 이미 집어서 올린 큐브 인덱스

        for stack_i in range(num_cubes):
            print(f"\n--- Cube {stack_i + 1}/{num_cubes} ---")

            # ✅ 목표 Z = 기본 높이 + 쌓인 큐브 수 × 큐브 높이
            target_z = STACK_BASE_Z + stack_i * CUBE_HEIGHT
            self.target_position = np.array([STACK_TARGET_XY[0], STACK_TARGET_XY[1], target_z])
            print(f"[Target] Stack layer {stack_i + 1}: "
                  f"XY=({STACK_TARGET_XY[0]:.2f}, {STACK_TARGET_XY[1]:.2f}), Z={target_z:.3f}m")
            self.update_target_marker()

            # ── GRASP ─────────────────────────────────────────────
            if self.current_phase == 1:
                success = self.phase1.auto_grasp()
            elif self.current_phase == 2:
                success = self.phase2.auto_grasp()
            elif self.current_phase == 3:
                # ✅ 이미 집은 큐브 제외하고 감지
                success = self.phase3.auto_grasp(exclude_cubes=placed_cubes)
            else:
                print(f"[Phase {self.current_phase}] 준비중")
                break
            
            if not success:
                print(f"[Warning] Grasp 시작 실패, 건너뜀")
                continue

            # ✅ 상태머신 완료 대기
            if self.current_phase == 3:
                # ✅ grasp 시작 직후 어떤 큐브를 잡는지 기록
                grabbed_index = self.phase3.current_cube_index

                while self.phase3.is_busy():
                    self.phase3.update()
                    self.world.step(render=True)
                    self.app.update()

                if not self.phase3.cube_attached:
                    print(f"[Warning] Grasp 실패 (cube not attached)")
                    continue

                placed_cubes.add(grabbed_index)
                print(f"[Stack] Cube_{grabbed_index} 집음. 제외 목록: {placed_cubes}")
            else:
                for _ in range(20):
                    self.world.step(render=True)

            # ── PLACE ─────────────────────────────────────────────
            if self.current_phase == 1:
                self.phase1.place(self.target_position)
            elif self.current_phase == 2:
                self.phase2.place(self.target_position)
            elif self.current_phase == 3:
                # ✅ stack_index 전달 → place()에서 Z 높이 자동 보정
                self.phase3.place(self.target_position, stack_index=stack_i)

            # ✅ Place 완료 대기
            if self.current_phase == 3:
                while self.phase3.is_busy():
                    self.phase3.update()
                    self.world.step(render=True)
                    self.app.update()
            else:
                for _ in range(30):
                    self.world.step(render=True)

            print(f"[Stack] ✓ Cube {stack_i + 1} placed at layer {stack_i + 1} (Z={target_z:.3f}m)")

        print(f"\n{'='*60}")
        print(f"✓ STACKING COMPLETE - {len(placed_cubes)}/{num_cubes} cubes stacked!")
        print(f"{'='*60}\n")

        if self.current_phase == 2:
            self.phase2.print_performance()
        elif self.current_phase == 3:
            self.phase3.print_performance()
    
    def show_performance(self):
        if self.current_phase == 2:
            self.phase2.print_performance()
        elif self.current_phase == 3:
            self.phase3.print_performance()
        else:
            print(f"[Phase {self.current_phase}] Performance tracking not available")
    
    def compare_phases(self):
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
        """매 프레임 업데이트"""
        if self.pending_action is not None:
            action = self.pending_action
            self.pending_action = None

            if action == 'grasp':
                self.execute_grasp()
            elif action == 'place':
                self.execute_place()
            elif action == 'multi':
                self.execute_multi_cube()

        if self.current_phase == 1:
            self.phase1.update()
        elif self.current_phase == 2:
            self.phase2.update()
        elif self.current_phase == 3:
            self.phase3.update()
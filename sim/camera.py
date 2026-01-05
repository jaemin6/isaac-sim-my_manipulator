# sim/camera.py - Replicator 기반으로 완전 교체
import omni.replicator.core as rep
import numpy as np
import carb


class SimulationCamera:
    """
    Omni Replicator를 사용한 안정적인 카메라
    """
    def __init__(
        self,
        prim_path="/World/Camera",
        position=(0.0, 0.0, 2.0),
        look_at=(0.0, 0.0, 0.0),
        orientation=None,  # 하위 호환성을 위해 유지 (사용 안 함)
        target_position=None,  # 하위 호환성
        resolution=(640, 480),
        frequency=30,  # 하위 호환성 (사용 안 함)
    ):
        """
        Replicator 기반 카메라
        
        Args:
            prim_path: 카메라 경로 (참고용)
            position: 카메라 위치 (x, y, z)
            look_at: 바라볼 타겟 위치 (x, y, z)
            orientation: 사용 안 함 (하위 호환성)
            target_position: look_at의 별칭
            resolution: (width, height)
            frequency: 사용 안 함 (하위 호환성)
        """
        self.prim_path = prim_path
        self.resolution = resolution
        self.position = position
        
        # target_position이 주어진 경우 look_at으로 사용
        if target_position is not None:
            look_at = target_position
        
        self.look_at = look_at
        
        print(f"Creating Replicator camera at {prim_path}")
        print(f"  Position: {position}")
        print(f"  Look at: {look_at}")
        print(f"  Resolution: {resolution}")
        
        # Replicator 카메라 생성
        self.camera = rep.create.camera(
            position=position,
            look_at=look_at,
        )
        
        # 렌더 프로덕트 생성
        self.render_product = rep.create.render_product(
            self.camera,
            resolution=resolution,
        )
        
        # RGB annotator 생성
        self.rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        self.rgb_annotator.attach([self.render_product])
        
        self._last_rgb = None
        
        print("Replicator camera initialized successfully")
    
    def initialize(self):
        """
        하위 호환성을 위한 메서드 (이미 __init__에서 초기화됨)
        """
        pass
    
    def get_rgb(self):
        """
        RGB 이미지 가져오기
        
        Returns:
            numpy.ndarray: RGB 이미지 (H, W, 3) 또는 None
        """
        try:
            # Replicator 렌더링 트리거
            rep.orchestrator.step(rt_subframes=4)
            
            # 데이터 가져오기
            rgb_data = self.rgb_annotator.get_data()
            
            if rgb_data is None:
                return self._last_rgb
            
            # RGBA -> RGB 변환
            if len(rgb_data.shape) == 3 and rgb_data.shape[2] == 4:
                rgb = rgb_data[:, :, :3].copy()
            else:
                rgb = rgb_data.copy()
            
            self._last_rgb = rgb
            return rgb
            
        except Exception as e:
            carb.log_warn(f"Error getting RGB: {e}")
            return self._last_rgb
    
    def get_rgba(self):
        """
        RGBA 이미지 가져오기
        
        Returns:
            numpy.ndarray: RGBA 이미지 (H, W, 4) 또는 None
        """
        try:
            rep.orchestrator.step(rt_subframes=4)
            rgba_data = self.rgb_annotator.get_data()
            return rgba_data
        except Exception as e:
            carb.log_warn(f"Error getting RGBA: {e}")
            return None
    
    def get_camera_info(self):
        """
        카메라 정보 출력 (디버깅용)
        """
        print("\n=== Camera Info ===")
        print(f"Prim path: {self.prim_path}")
        print(f"Position: {self.position}")
        print(f"Look at: {self.look_at}")
        print(f"Resolution: {self.resolution}")
        print(f"Backend: Omni Replicator")
        print("===================\n")
    
    def get_world_pose(self):
        """
        하위 호환성을 위한 메서드
        Returns: (position, orientation)
        """
        return (self.position, (1.0, 0.0, 0.0, 0.0))
    
    def get_resolution(self):
        """
        해상도 반환
        """
        return self.resolution
    
    def get_current_frame(self):
        """
        현재 프레임 번호 (Replicator에서는 사용 안 함)
        """
        return 0
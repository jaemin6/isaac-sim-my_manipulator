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
        
        # Depth annotator 추가
        self.depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
        self.depth_annotator.attach([self.render_product])
        
        self._last_rgb = None
        self._last_depth = None
        
        print("Replicator camera initialized successfully (RGB + Depth)")
    
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
    
    def get_depth(self):
        """
        Depth 이미지 가져오기 (카메라로부터의 거리)
        
        Returns:
            numpy.ndarray: Depth 이미지 (H, W) - 단위: 미터
            None: 실패 시
        """
        try:
            # RGB와 함께 렌더링됨 (이미 orchestrator.step 호출됨)
            depth_data = self.depth_annotator.get_data()
            
            if depth_data is None:
                return self._last_depth
            
            # Depth는 보통 (H, W) 또는 (H, W, 1) 형태
            if len(depth_data.shape) == 3:
                depth = depth_data[:, :, 0].copy()
            else:
                depth = depth_data.copy()
            
            self._last_depth = depth
            return depth
            
        except Exception as e:
            carb.log_warn(f"Error getting depth: {e}")
            return self._last_depth
    
    def get_rgb_depth(self):
        """
        RGB와 Depth를 동시에 가져오기 (효율적)
        
        Returns:
            tuple: (rgb, depth) 또는 (None, None)
        """
        try:
            # 한 번의 렌더링으로 둘 다 가져오기
            rep.orchestrator.step(rt_subframes=4)
            
            # RGB
            rgb_data = self.rgb_annotator.get_data()
            if rgb_data is not None:
                if len(rgb_data.shape) == 3 and rgb_data.shape[2] == 4:
                    rgb = rgb_data[:, :, :3].copy()
                else:
                    rgb = rgb_data.copy()
                self._last_rgb = rgb
            else:
                rgb = self._last_rgb
            
            # Depth
            depth_data = self.depth_annotator.get_data()
            if depth_data is not None:
                if len(depth_data.shape) == 3:
                    depth = depth_data[:, :, 0].copy()
                else:
                    depth = depth_data.copy()
                self._last_depth = depth
            else:
                depth = self._last_depth
            
            return rgb, depth
            
        except Exception as e:
            carb.log_warn(f"Error getting RGB+Depth: {e}")
            return self._last_rgb, self._last_depth
    
    def visualize_depth(self, depth, min_depth=0.0, max_depth=5.0):
        """
        Depth를 시각화용 이미지로 변환
        
        Args:
            depth: Depth 배열 (H, W)
            min_depth: 최소 깊이 (미터)
            max_depth: 최대 깊이 (미터)
        
        Returns:
            numpy.ndarray: 컬러맵이 적용된 depth 이미지 (H, W, 3)
        """
        if depth is None:
            return None
        
        # 깊이를 0-255로 정규화
        depth_normalized = np.clip((depth - min_depth) / (max_depth - min_depth), 0, 1)
        depth_uint8 = (depth_normalized * 255).astype(np.uint8)
        
        # Colormap 적용 (TURBO가 더 선명함)
        import cv2
        depth_colored = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_TURBO)
        
        return depth_colored
    
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
    
    def pixel_to_camera(self, u, v, depth):
        """
        Pixel 좌표 + depth → 카메라 좌표계 (meters)
        """
        width, height = self.resolution

        # FOV 가정 (Isaac 기본값 근처, 나중에 튜닝 가능)
        fov = np.deg2rad(60.0)
        fx = width / (2 * np.tan(fov / 2))
        fy = fx
        
        cx = width / 2
        cy = height / 2

        X = (u - cx) * depth / fx
        Y = -(v - cy) * depth / fy
        Z = depth

        return np.array([X, Y, Z])
    
    def camera_to_world(self, cam_point):
        """
        카메라 좌표계 → 월드 좌표계
        """
        cam_pos = np.array(self.position, dtype=float)
        target = np.array(self.look_at, dtype=float)

        forward = target - cam_pos
        forward = forward / np.linalg.norm(forward)

        up = np.array([0.0, 0.0, 1.0])
        right = np.cross(forward, up)
        right = right / np.linalg.norm(right)
        up = np.cross(right, forward)

        R = np.vstack([right, up, forward]).T
        world_point = cam_pos + R @ cam_point

        return world_point
    
    def pixel_depth_to_world(selfm, u, v, depth):
        # Pixel (u, v) + depth → World 좌표
        cam_point = self.pixel_to_camera(u, v, depth)
        world_point = self.camera_to_world(cam_point)
        return world_point


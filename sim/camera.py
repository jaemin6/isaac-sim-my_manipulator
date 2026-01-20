# sim/camera.py
import numpy as np
import omni.replicator.core as rep
from omni.isaac.core.prims import XFormPrim
from omni.isaac.sensor import Camera


class SimulationCamera:
    """Isaac Sim 카메라 (Omni Replicator 사용)"""
    
    def __init__(self, prim_path, position, look_at=None, resolution=(640, 480)):
        self.prim_path = prim_path
        self.position = position
        self.look_at = look_at
        self.resolution = resolution
        
        self.camera = None
        self.render_product = None
        
        self._initialize_camera()

    def _initialize_camera(self):
        """카메라 초기화"""
        print(f"Creating Replicator camera at {self.prim_path}")
        print(f"  Position: {self.position}")
        if self.look_at:
            print(f"  Look at: {self.look_at}")
        print(f"  Resolution: {self.resolution}")
        
        # Replicator 카메라 생성
        self.camera = rep.create.camera(
            position=self.position,
            look_at=self.look_at if self.look_at else (0, 0, 0),
        )
        
        # Render product 생성
        self.render_product = rep.create.render_product(
            self.camera,
            self.resolution
        )
        
        # RGB annotator
        self.rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        self.rgb_annotator.attach([self.render_product])
        
        # Depth annotator
        self.depth_annotator = rep.AnnotatorRegistry.get_annotator(
            "distance_to_camera"
        )
        self.depth_annotator.attach([self.render_product])
        
        print("Replicator camera initialized successfully (RGB + Depth)")

    def get_rgb_depth(self):
        """RGB와 Depth 이미지 획득"""
        try:
            # RGB
            rgb_data = self.rgb_annotator.get_data()
            if rgb_data is None:
                return None, None
            rgb = np.array(rgb_data)
            
            # Depth
            depth_data = self.depth_annotator.get_data()
            if depth_data is None:
                return rgb, None
            depth = np.array(depth_data)
            
            return rgb, depth
            
        except Exception as e:
            print(f"[Camera] Error getting data: {e}")
            return None, None

    def get_camera_info(self):
        """카메라 정보 출력"""
        print("\n=== Camera Info ===")
        print(f"Prim path: {self.prim_path}")
        print(f"Position: {self.position}")
        if self.look_at:
            print(f"Look at: {self.look_at}")
        print(f"Resolution: {self.resolution}")
        print(f"Backend: Omni Replicator")
        print("===================\n")

    def pixel_depth_to_world(self, u, v, depth):
        """
        픽셀 좌표와 depth를 world 좌표로 변환
        카메라가 위에서 아래를 내려다보는 경우를 올바르게 처리
        """
        # 카메라 intrinsics
        fx = self.resolution[0] / 2.0
        fy = self.resolution[1] / 2.0
        cx = self.resolution[0] / 2.0
        cy = self.resolution[1] / 2.0
        
        # 카메라 좌표계에서의 3D 점
        # OpenCV/Isaac Sim 좌표계: X right, Y down, Z forward
        x_cam = (u - cx) * depth / fx
        y_cam = (v - cy) * depth / fy
        z_cam = depth
        
        point_cam = np.array([x_cam, y_cam, z_cam])
        
        # 카메라 위치
        cam_pos = np.array(self.position)
        
        if self.look_at:
            look_at_pos = np.array(self.look_at)
            
            # 카메라 방향 벡터 (Z축 - forward)
            forward = look_at_pos - cam_pos
            forward = forward / np.linalg.norm(forward)
            
            # 월드 좌표계의 up vector
            world_up = np.array([0, 0, 1])
            
            # Right vector (X축)
            right = np.cross(world_up, forward)
            if np.linalg.norm(right) < 1e-6:
                # forward와 world_up이 평행한 경우
                right = np.array([1, 0, 0])
            else:
                right = right / np.linalg.norm(right)
            
            # Up vector (Y축) - 카메라 좌표계
            up = np.cross(forward, right)
            up = up / np.linalg.norm(up)
            
            # 회전 행렬 구성: [right, up, forward]
            # 각 열이 카메라 좌표계의 축을 월드 좌표계로 변환
            R = np.column_stack([right, up, forward])
            
            # 카메라 좌표계 → 월드 좌표계
            point_world = cam_pos + R @ point_cam
        else:
            # look_at이 없으면 단순 변환
            point_world = cam_pos + point_cam
        
        return point_world


class RobotMountedCamera:
    """로봇 End-Effector에 부착된 카메라"""
    
    def __init__(self, robot, offset=(0.0, 0.0, 0.05), resolution=(640, 480)):
        """
        Args:
            robot: FrankaRobot 인스턴스
            offset: EE 기준 카메라 오프셋 [x, y, z]
            resolution: 카메라 해상도
        """
        self.robot = robot
        self.offset = np.array(offset)
        self.resolution = resolution
        
        # 카메라를 EE에 부착
        self.prim_path = "/World/Franka/panda_hand/Camera"
        self.camera = None
        self.render_product = None
        
        self._initialize_camera()

    def _initialize_camera(self):
        """EE에 카메라 생성"""
        print(f"Creating camera on robot end-effector")
        print(f"  Offset from EE: {self.offset}")
        print(f"  Resolution: {self.resolution}")
        
        # EE에 부착된 카메라 생성
        self.camera = rep.create.camera(
            position=self.offset.tolist(),
            parent=self.robot.franka.end_effector.prim_path
        )
        
        # Render product
        self.render_product = rep.create.render_product(
            self.camera,
            self.resolution
        )
        
        # Annotators
        self.rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        self.rgb_annotator.attach([self.render_product])
        
        self.depth_annotator = rep.AnnotatorRegistry.get_annotator(
            "distance_to_camera"
        )
        self.depth_annotator.attach([self.render_product])
        
        print("Robot-mounted camera initialized successfully")

    def get_rgb_depth(self):
        """RGB와 Depth 이미지 획득"""
        try:
            rgb_data = self.rgb_annotator.get_data()
            if rgb_data is None:
                return None, None
            rgb = np.array(rgb_data)
            
            depth_data = self.depth_annotator.get_data()
            if depth_data is None:
                return rgb, None
            depth = np.array(depth_data)
            
            return rgb, depth
            
        except Exception as e:
            print(f"[RobotCamera] Error: {e}")
            return None, None

    def get_camera_pose(self):
        """현재 카메라의 world pose"""
        ee_pos, ee_ori = self.robot.get_ee_pose()
        # 간단하게 EE 위치 + 오프셋
        cam_pos = ee_pos + self.offset
        return cam_pos, ee_ori

    def pixel_depth_to_world(self, u, v, depth):
        """픽셀을 world 좌표로 변환"""
        fx = self.resolution[0] / 2.0
        fy = self.resolution[1] / 2.0
        cx = self.resolution[0] / 2.0
        cy = self.resolution[1] / 2.0
        
        # 카메라 좌표계
        x_cam = (u - cx) * depth / fx
        y_cam = (v - cy) * depth / fy
        z_cam = depth
        
        # World 좌표로 변환 (간단한 버전)
        cam_pos, cam_ori = self.get_camera_pose()
        point_world = cam_pos + np.array([x_cam, y_cam, z_cam])
        
        return point_world


# ============================================================
# 메인 파일용 간단한 setup 함수
# ============================================================

def setup_camera(world):
    """
    탑다운 뷰 카메라 설치 (메인 파일용)
    
    Args:
        world: World 객체
    
    Returns:
        Camera 객체
    """
    print("[Camera] Setting up top-down camera...")
    
    camera = Camera(
        prim_path="/World/Camera",
        position=np.array([0.5, 0.0, 1.2]),
        frequency=20,
        resolution=(512, 512),
        name="top_camera"
    )
    world.scene.add(camera)
    
    # 카메라를 아래로 향하게 (top-down view)
    camera.set_local_pose(
        translation=np.array([0.5, 0.0, 1.2]),
        orientation=np.array([0.7071, 0, 0, -0.7071])  # 90도 회전
    )
    
    print("[Camera] Top-down camera setup complete!")
    return camera
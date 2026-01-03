# sim/camera.py

import numpy as np
from omni.isaac.sensor import Camera

class SimulationCamera:
    def __init__(
        self,
        prim_path="/World/Camera",
        position=(1.0, 0.0, 1.2),
        look_at=(0.5, 0.0, 0.6),
        resolution=(640, 480),
    ):
        # 1. Isaac 센서 카메라 초기화
        self.camera = Camera(
            prim_path=prim_path,
            position=position,
            resolution=resolution,
        )
        self.camera.initialize()
        
        # 2. 카메라 시점 설정 (필요 시 구현)
        self._set_look_at(position, look_at)

    def get_rgb(self):
        """RGB 이미지 취득 (H, W, 3)"""
        rgba = self.camera.get_rgba()
        return rgba[:, :, :3] if rgba is not None else None

    def get_depth(self):
        """Depth 이미지 취득 (H, W)"""
        return self.camera.get_depth()

    def get_world_pose(self):
        """카메라의 현재 위치 및 자세 반환"""
        return self.camera.get_world_pose()

    def _set_look_at(self, position, target):
        """카메라가 특정 목표를 바라보도록 설정 (내부 로직 생략)"""
        pass
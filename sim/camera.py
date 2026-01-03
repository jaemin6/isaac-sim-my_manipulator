# sim/camera.py
import numpy as np
from pxr import UsdGeom, Gf
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.sensor import Camera


class SimulationCamera:
    def __init__(
        self,
        prim_path="/World/Camera",
        position=(0.6, 0.0, 0.8),
        look_at=(0.0, 0.0, 0.4),
        resolution=(640, 480),
    ):
        self.prim_path = prim_path
        self.resolution = resolution

        # Camera sensor
        self.camera = Camera(
            prim_path=prim_path,
            position=position,
            frequency=30,
            resolution=resolution,
        )

        self.camera.initialize()

        # 방향 설정
        self._set_look_at(position, look_at)

    def _set_look_at(self, position, target):
        stage = get_current_stage()
        cam_prim = stage.GetPrimAtPath(self.prim_path)

        pos = Gf.Vec3d(*position)
        tgt = Gf.Vec3d(*target)

        direction = tgt - pos
        direction.Normalize()

        # Isaac Sim 기준: Z-up, -Z forward
        up = Gf.Vec3d(0, 0, 1)

        right = Gf.Cross(direction, up)
        right.Normalize()

        up_corrected = Gf.Cross(right, direction)
        up_corrected.Normalize()

        # 3x3 회전 행렬 생성
        rot = Gf.Matrix3d(
            right[0], up_corrected[0], -direction[0],
            right[1], up_corrected[1], -direction[1],
            right[2], up_corrected[2], -direction[2]
        )

        # Quaternion으로 변환
        rotation = rot.ExtractRotation()
        quat = rotation.GetQuaternion()

        # 직접 속성 설정
        xformable = UsdGeom.Xformable(cam_prim)
        
        # 기존 xform ops 가져오기
        xform_ops = xformable.GetOrderedXformOps()
        
        # translate와 orient op 찾기 또는 생성
        translate_op = None
        orient_op = None
        
        for op in xform_ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
            elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                orient_op = op
        
        # 없으면 생성
        if translate_op is None:
            translate_op = xformable.AddTranslateOp()
        if orient_op is None:
            orient_op = xformable.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
        
        # 값 설정
        translate_op.Set(pos)
        orient_op.Set(Gf.Quatd(quat.GetReal(), *quat.GetImaginary()))

    def get_rgb(self):
        """RGB 이미지 가져오기 (RGBA에서 RGB만 추출)"""
        rgba = self.camera.get_rgba()
        
        # 데이터가 없거나 비어있으면 None 반환
        if rgba is None or rgba.size == 0:
            return None
        
        # 1D array이면 reshape
        if rgba.ndim == 1:
            h, w = self.resolution
            # 크기가 맞지 않으면 None 반환
            if rgba.size != h * w * 4:
                return None
            rgba = rgba.reshape(h, w, 4)

        return rgba[:, :, :3]
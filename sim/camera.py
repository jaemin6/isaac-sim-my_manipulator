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

        xform = UsdGeom.XformCommonAPI(cam_prim)

        pos = Gf.Vec3d(*position)
        tgt = Gf.Vec3d(*target)

        direction = tgt - pos
        direction.Normalize()

        # Isaac Sim 기준: Z-up, -Z forward
        up = Gf.Vec3d(0, 0, 1)

        right = direction.Cross(up)
        right.Normalize()

        up_corrected = right.Cross(direction)
        up_corrected.Normalize()

        rot = Gf.Matrix3d(
            right[0], up_corrected[0], -direction[0], 0,
            right[1], up_corrected[1], -direction[1], 0,
            right[2], up_corrected[2], -direction[2], 0,
            0,        0,               0,            1,
        )

        xform.SetTranslate(pos)
        xform.SetRotate(rot.ExtractRotation())

    def get_rgb(self):
        return self.camera.get_rgba()[:, :, :3]

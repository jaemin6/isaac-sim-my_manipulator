# sim/camera.py

import numpy as np
from omni.isaac.sensor import Camera
from pxr import Gf


class SimulationCamera:
    def __init__(
        self,
        prim_path="/World/Camera",
        position=(1.0, 0.0, 1.2),
        look_at=(0.5, 0.0, 0.6),
        resolution=(640, 480),
        frequency=30,
    ):
        """
        Fixed external camera for Simulation 1
        """

        self.camera = Camera(
            prim_path=prim_path,
            position=position,
            frequency=frequency,
            resolution=resolution,
        )

        self.camera.initialize()
        self._set_look_at(position, look_at)

    # -------------------------------------------------
    # Public API
    # -------------------------------------------------
    def get_rgb(self):
        """
        Returns:
            rgb: (H, W, 3) uint8
        """
        rgba = self.camera.get_rgba()
        if rgba is None:
            return None
        return rgba[:, :, :3]

    def get_depth(self):
        """
        Returns:
            depth: (H, W) float32 (meters)
        """
        return self.camera.get_depth()

    def get_data(self):
        """
        Returns:
            rgb, depth
        """
        return self.get_rgb(), self.get_depth()

    def get_intrinsics(self):
        """
        Returns camera intrinsics
        """
        fx, fy = self.camera.get_focal_length()
        cx, cy = self.camera.get_principal_point()
        return fx, fy, cx, cy

    def get_world_pose(self):
        """
        Returns:
            position (3,)
            orientation quaternion (4,)
        """
        pos, quat = self.camera.get_world_pose()
        return np.array(pos), np.array(quat)

    def pixel_to_camera(self, u, v, depth):
        """
        Convert pixel coordinate to camera frame
        """
        fx, fy, cx, cy = self.get_intrinsics()

        z = depth[v, u]
        if z <= 0:
            return None

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        return np.array([x, y, z])

    def camera_to_world(self, point_cam):
        """
        Convert camera frame point to world frame
        """
        pos, quat = self.get_world_pose()
        rot = Gf.Quatf(quat[3], Gf.Vec3f(quat[0], quat[1], quat[2]))
        mat = Gf.Matrix4f(rot)
        mat.SetTranslateOnly(Gf.Vec3f(*pos))

        p = mat.Transform(Gf.Vec3f(*point_cam))
        return np.array([p[0], p[1], p[2]])

    # -------------------------------------------------
    # Private
    # -------------------------------------------------
    def _set_look_at(self, position, target):
        """
        Orient camera to look at target
        """
        direction = np.array(target) - np.array(position)
        direction = direction / np.linalg.norm(direction)

        up = np.array([0, 0, 1])
        right = np.cross(up, direction)
        up = np.cross(direction, right)

        rot = Gf.Matrix3f(
            right[0], up[0], -direction[0],
            right[1], up[1], -direction[1],
            right[2], up[2], -direction[2],
        )

        quat = Gf.Quatf(rot)
        self.camera.set_world_pose(position, quat)

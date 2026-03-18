"""
utils/cube_utils.py

[v5 핵심 개선]
get_cube_position():
  - 기존: UsdGeom.Xformable.ComputeLocalToWorldTransform(0)
    → 시뮬 시작 시점 고정값 → 큐브가 밀려나도 감지 못함
  - v5: RigidPrim.get_world_pose() 우선 (PhysX 실시간)
    → 큐브가 어디로 이동했든 실제 위치 반환
    → fallback: USD current_time → timecode=0

attach_cube_to_ee() [v4 유지]:
  - RigidPrim API로 실시간 transform 읽기
  - velocity 리셋으로 snap 방지
"""

import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf
from omni.isaac.core.utils.stage import get_current_stage


# ------------------------------------------------------------------ #
#  보조: transform 변환
# ------------------------------------------------------------------ #

def _gf_matrix_to_np(gf_mat):
    return np.array([[gf_mat[r][c] for c in range(4)] for r in range(4)])


def _np_to_gf_vec3f(v):
    return Gf.Vec3f(float(v[0]), float(v[1]), float(v[2]))


def _rotation_matrix_to_quat(R):
    """3x3 rotation matrix → Gf.Quatf (w, x, y, z), Shepperd method"""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return Gf.Quatf(float(w), float(x), float(y), float(z))


def _quat_wxyz_to_rotation_matrix(quat_wxyz):
    """Isaac Sim 쿼터니언 (w,x,y,z) → 3x3 rotation matrix"""
    w, x, y, z = (float(quat_wxyz[0]), float(quat_wxyz[1]),
                  float(quat_wxyz[2]), float(quat_wxyz[3]))

    # 검증: norm이 1에 가까워야 함
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    if abs(norm - 1.0) > 0.01:
        w, x, y, z = w/norm, x/norm, y/norm, z/norm

    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


# ------------------------------------------------------------------ #
#  [v5] RigidPrim으로 실시간 world pose 읽기
# ------------------------------------------------------------------ #

def _get_rigid_world_pos(prim_path):
    """
    RigidPrim.get_world_pose()로 PhysX 실시간 위치 반환.
    반환: pos [3] 또는 실패 시 None
    """
    try:
        from omni.isaac.core.prims import RigidPrim
        rigid = RigidPrim(prim_path=prim_path)
        pos, _ = rigid.get_world_pose()
        return np.array(pos, dtype=float)
    except Exception:
        return None


def _get_rigid_world_pose(prim_path):
    """
    RigidPrim.get_world_pose()로 PhysX 실시간 위치+회전 반환.
    반환: (pos [3], rot_mat [3x3]) 또는 실패 시 (None, None)
    """
    try:
        from omni.isaac.core.prims import RigidPrim
        rigid = RigidPrim(prim_path=prim_path)
        pos, quat = rigid.get_world_pose()
        pos  = np.array(pos,  dtype=float)
        quat = np.array(quat, dtype=float)
        rot  = _quat_wxyz_to_rotation_matrix(quat)
        det  = np.linalg.det(rot)
        if abs(det - 1.0) > 0.1:
            return None, None
        return pos, rot
    except Exception as e:
        print(f"[Cube] RigidPrim failed ({prim_path}): {e}")
        return None, None


def _get_usd_world_pose(prim, label=""):
    """USD Xformable fallback: current_time → timecode=0"""
    try:
        import omni.timeline
        current_time = omni.timeline.get_timeline_interface().get_current_time()
    except Exception:
        current_time = 0.0

    xform = UsdGeom.Xformable(prim)
    for tc in [current_time, 0.0]:
        try:
            mat = _gf_matrix_to_np(xform.ComputeLocalToWorldTransform(tc))
            pos = mat[:3, 3]
            rot = mat[:3, :3]
            if abs(np.linalg.det(rot) - 1.0) < 0.1:
                return pos, rot
        except Exception:
            continue
    return None, None


# ------------------------------------------------------------------ #
#  [v5] get_cube_position: RigidPrim 실시간 위치 우선
# ------------------------------------------------------------------ #

def get_cube_position(cube_index=0):
    """
    큐브 실시간 위치 반환.
    [v5] RigidPrim(PhysX) 우선 → USD fallback
    → 큐브가 밀려나거나 이동해도 실제 위치 반환
    """
    cube_path = f"/World/Cube_{cube_index}"

    # 1순위: RigidPrim (PhysX 실시간)
    pos = _get_rigid_world_pos(cube_path)
    if pos is not None:
        return pos

    # 2순위: USD fallback
    stage = get_current_stage()
    cube = stage.GetPrimAtPath(cube_path)
    if not cube or not cube.IsValid():
        return None
    try:
        import omni.timeline
        current_time = omni.timeline.get_timeline_interface().get_current_time()
    except Exception:
        current_time = 0.0

    xform = UsdGeom.Xformable(cube)
    for tc in [current_time, 0.0]:
        try:
            mat_gf = xform.ComputeLocalToWorldTransform(tc)
            pos_gf = mat_gf.ExtractTranslation()
            return np.array([pos_gf[0], pos_gf[1], pos_gf[2]])
        except Exception:
            continue
    return None


def get_all_cubes():
    """모든 큐브 실시간 위치 반환"""
    cubes_info = []
    i = 0
    while True:
        cube_path = f"/World/Cube_{i}"
        stage = get_current_stage()
        cube = stage.GetPrimAtPath(cube_path)
        if not cube or not cube.IsValid():
            break
        pos = get_cube_position(i)
        if pos is not None:
            cubes_info.append({
                'index': i,
                'path': cube_path,
                'position': pos
            })
        i += 1
    return cubes_info


def find_nearest_cube(reference_position):
    cubes = get_all_cubes()
    if not cubes:
        return None
    nearest, min_distance = None, float('inf')
    for cube_info in cubes:
        pos = cube_info['position']
        distance = np.sqrt(
            (pos[0] - reference_position[0])**2 +
            (pos[1] - reference_position[1])**2
        )
        if distance < min_distance:
            min_distance = distance
            nearest = cube_info
    return nearest


# ------------------------------------------------------------------ #
#  velocity 리셋
# ------------------------------------------------------------------ #

def _reset_rigid_body_velocity(cube_prim):
    """선속도/각속도 강제 0 리셋. attach/detach 직전 호출."""
    try:
        vel_attr = cube_prim.GetAttribute("physics:velocity")
        ang_attr = cube_prim.GetAttribute("physics:angularVelocity")
        if vel_attr and vel_attr.IsValid():
            vel_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
        if ang_attr and ang_attr.IsValid():
            ang_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
        print(f"[Cube] ✅ Velocity reset to zero")
    except Exception as e:
        print(f"[Cube] ⚠️  Velocity reset failed: {e}")


# ------------------------------------------------------------------ #
#  attach_cube_to_ee
# ------------------------------------------------------------------ #

def attach_cube_to_ee(cube_index):
    """
    큐브를 EE에 FixedJoint로 부착.
    Transform 읽기: RigidPrim(PhysX) → USD fallback
    attach 직전 velocity 리셋으로 snap 방지.
    """
    stage = get_current_stage()
    cube_path = f"/World/Cube_{cube_index}"
    ee_path   = "/World/Franka/panda_hand"

    cube_prim = stage.GetPrimAtPath(cube_path)
    ee_prim   = stage.GetPrimAtPath(ee_path)

    if not cube_prim or not cube_prim.IsValid():
        print(f"[Error] Cube_{cube_index} not found")
        return False
    if not ee_prim or not ee_prim.IsValid():
        print(f"[Error] End effector not found")
        return False

    # 1순위: RigidPrim API
    cube_pos, cube_rot = _get_rigid_world_pose(cube_path)
    ee_pos,   ee_rot   = _get_rigid_world_pose(ee_path)

    # ── DEBUG: 쿼터니언 원본값 확인 ──────────────────────────────
    if ee_pos is not None:
        from omni.isaac.core.prims import RigidPrim
        _rigid_ee   = RigidPrim(prim_path=ee_path)
        _rigid_cube = RigidPrim(prim_path=cube_path)
        _, quat_ee   = _rigid_ee.get_world_pose()
        _, quat_cube = _rigid_cube.get_world_pose()
        print(f"[DEBUG] EE   quat raw  = {quat_ee}")
        print(f"[DEBUG] EE   quat norm = {np.linalg.norm(quat_ee):.4f}")
        print(f"[DEBUG] Cube quat raw  = {quat_cube}")
        print(f"[DEBUG] Cube quat norm = {np.linalg.norm(quat_cube):.4f}")
        print(f"[DEBUG] EE   rot_mat =\n{ee_rot.round(4)}")
        print(f"[DEBUG] EE   rot trace = {np.trace(ee_rot):.4f}")
        print(f"[DEBUG] Cube rot_mat =\n{cube_rot.round(4)}")
        print(f"[DEBUG] Cube rot trace = {np.trace(cube_rot):.4f}")
    # ──────────────────────────────────────────────────────────────

    if cube_pos is not None and ee_pos is not None:
        print(f"[Cube] ✅ Using RigidPrim API (PhysX direct)")
        print(f"[Cube] cube_world_pos = {cube_pos.round(4)}")
        print(f"[Cube] ee_world_pos   = {ee_pos.round(4)}")
        rel_pos = ee_rot.T @ (cube_pos - ee_pos)
        rel_rot = ee_rot.T @ cube_rot
    else:
        # 2순위: USD fallback
        print(f"[Cube] ⚠️  RigidPrim failed → USD fallback")
        cube_pos, cube_rot = _get_usd_world_pose(cube_prim, "cube")
        ee_pos,   ee_rot   = _get_usd_world_pose(ee_prim,   "ee")
        if cube_pos is None or ee_pos is None:
            print(f"[Error] All transform methods failed.")
            return False
        rel_pos = ee_rot.T @ (cube_pos - ee_pos)
        rel_rot = ee_rot.T @ cube_rot

    trace = np.trace(rel_rot)
    if trace < 2.5:
        print(f"[Cube] ⚠️  rot_trace={trace:.3f} (정상=~3.0)")
    else:
        print(f"[Cube] ✅ rot_trace={trace:.3f}")

    _reset_rigid_body_velocity(cube_prim)

    # 기존 joint 제거 후 재생성
    joint_path = f"/World/GraspJoint_{cube_index}"
    existing = stage.GetPrimAtPath(joint_path)
    if existing and existing.IsValid():
        stage.RemovePrim(existing.GetPath())

    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([cube_prim.GetPath()])
    joint.CreateBody1Rel().SetTargets([ee_prim.GetPath()])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr().Set(_np_to_gf_vec3f(rel_pos))
    joint.CreateLocalRot1Attr().Set(_rotation_matrix_to_quat(rel_rot))

    print(f"[Cube] Attached Cube_{cube_index} | "
          f"pos={rel_pos.round(4)}, rot_trace={trace:.3f}")
    return True


# ------------------------------------------------------------------ #
#  detach_cube
# ------------------------------------------------------------------ #

def detach_cube(cube_index):
    """joint 제거. detach 직전 velocity 리셋 → 튕김 방지."""
    stage = get_current_stage()
    joint_path = f"/World/GraspJoint_{cube_index}"
    joint_prim = stage.GetPrimAtPath(joint_path)

    if joint_prim and joint_prim.IsValid():
        cube_path = f"/World/Cube_{cube_index}"
        cube_prim = stage.GetPrimAtPath(cube_path)
        if cube_prim and cube_prim.IsValid():
            _reset_rigid_body_velocity(cube_prim)
        stage.RemovePrim(joint_prim.GetPath())
        print(f"[Cube] Detached Cube_{cube_index}")
        return True

    print(f"[Cube] ⚠️  Joint not found for Cube_{cube_index}")
    return False


# ------------------------------------------------------------------ #
#  기타 유틸
# ------------------------------------------------------------------ #

def get_cube_color(cube_index):
    color_map = {0: 'red', 1: 'green', 2: 'blue', 3: 'yellow'}
    return color_map.get(cube_index, 'unknown')


def print_cubes_status():
    cubes = get_all_cubes()
    print(f"\n{'='*60}")
    print(f"CUBES STATUS: {len(cubes)} cubes detected")
    print(f"{'='*60}")
    for cube in cubes:
        pos   = cube['position']
        color = get_cube_color(cube['index'])
        print(f"  Cube_{cube['index']} ({color:6s}): "
              f"({pos[0]:5.3f}, {pos[1]:6.3f}, {pos[2]:5.3f})")
    print(f"{'='*60}\n")
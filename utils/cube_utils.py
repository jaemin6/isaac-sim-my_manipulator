"""
utils/cube_utils.py
- 위치 가져오기
- 가장 가까운 큐브 찾기
- Attach/Detach
"""

import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf
from omni.isaac.core.utils.stage import get_current_stage


def get_cube_position(cube_index=0):
    """
    특정 큐브의 월드 위치 가져오기 (Ground Truth)
    
    Args:
        cube_index (int): 큐브 인덱스 (0, 1, 2, ...)
    
    Returns:
        np.array: [x, y, z] 위치 or None
    """
    stage = get_current_stage()
    cube_path = f"/World/Cube_{cube_index}"
    cube = stage.GetPrimAtPath(cube_path)
    
    if not cube or not cube.IsValid():
        return None
    
    xform = UsdGeom.Xformable(cube)
    pos = xform.ComputeLocalToWorldTransform(0).ExtractTranslation()
    
    return np.array([pos[0], pos[1], pos[2]])


def get_all_cubes():
    """
    모든 큐브의 정보 가져오기
    
    Returns:
        list: [{'index': 0, 'path': '/World/Cube_0', 'position': [x,y,z]}, ...]
    """
    stage = get_current_stage()
    cubes_info = []
    i = 0
    
    while True:
        cube_path = f"/World/Cube_{i}"
        cube = stage.GetPrimAtPath(cube_path)
        
        if not cube or not cube.IsValid():
            break
        
        xform = UsdGeom.Xformable(cube)
        pos = xform.ComputeLocalToWorldTransform(0).ExtractTranslation()
        world_pos = np.array([pos[0], pos[1], pos[2]])
        
        cubes_info.append({
            'index': i,
            'path': cube_path,
            'position': world_pos
        })
        i += 1
    
    return cubes_info


def find_nearest_cube(reference_position):
    """
    기준 위치에서 가장 가까운 큐브 찾기 (XY 평면 거리만)
    
    Args:
        reference_position (np.array): 기준 위치 [x, y, z]
    
    Returns:
        dict: {'index': 0, 'path': '...', 'position': [...]} or None
    """
    cubes = get_all_cubes()
    
    if not cubes:
        return None
    
    nearest = None
    min_distance = float('inf')
    
    for cube_info in cubes:
        pos = cube_info['position']
        # XY 평면 거리만 계산 (높이 무시)
        distance = np.sqrt(
            (pos[0] - reference_position[0])**2 + 
            (pos[1] - reference_position[1])**2
        )
        
        if distance < min_distance:
            min_distance = distance
            nearest = cube_info
    
    return nearest


def attach_cube_to_ee(cube_index):
    """
    큐브를 end effector에 물리적으로 부착
    
    Args:
        cube_index (int): 부착할 큐브 인덱스
    
    Returns:
        bool: 성공 여부
    """
    stage = get_current_stage()
    cube_path = f"/World/Cube_{cube_index}"
    ee_path = "/World/Franka/panda_hand"
    
    cube = stage.GetPrimAtPath(cube_path)
    ee = stage.GetPrimAtPath(ee_path)
    
    if not cube or not ee:
        print(f"[Error] Cannot find cube or end effector")
        return False
    
    # Fixed Joint 생성
    joint_path = f"/World/GraspJoint_{cube_index}"
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([cube.GetPath()])
    joint.CreateBody1Rel().SetTargets([ee.GetPath()])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    
    print(f"[Cube] Attached Cube_{cube_index} to end effector")
    return True


def detach_cube(cube_index):
    """
    큐브를 end effector에서 분리
    
    Args:
        cube_index (int): 분리할 큐브 인덱스
    
    Returns:
        bool: 성공 여부
    """
    stage = get_current_stage()
    joint_path = f"/World/GraspJoint_{cube_index}"
    joint_prim = stage.GetPrimAtPath(joint_path)
    
    if joint_prim:
        stage.RemovePrim(joint_prim.GetPath())
        print(f"[Cube] Detached Cube_{cube_index}")
        return True
    
    return False


def get_cube_color(cube_index):
    """
    큐브의 색상 가져오기 (디버깅용)
    
    Args:
        cube_index (int): 큐브 인덱스
    
    Returns:
        str: 'red', 'blue', 'yellow' or 'unknown'
    """
    # 기본 색상 매핑 (main.py에서 설정한 순서)
    color_map = {
        0: 'red',
        1: 'blue', 
        2: 'yellow'
    }
    
    return color_map.get(cube_index, 'unknown')


def print_cubes_status():
    """
    모든 큐브의 상태 출력 (디버깅용)
    """
    cubes = get_all_cubes()
    
    print(f"\n{'='*60}")
    print(f"CUBES STATUS: {len(cubes)} cubes detected")
    print(f"{'='*60}")
    
    for cube in cubes:
        pos = cube['position']
        color = get_cube_color(cube['index'])
        print(f"  Cube_{cube['index']} ({color:6s}): ({pos[0]:5.2f}, {pos[1]:6.2f}, {pos[2]:5.2f})")
    
    print(f"{'='*60}\n")


# 테스트용 코드
if __name__ == "__main__":
    # 이 파일을 직접 실행하면 테스트 가능
    print("Cube Utils Module Test")
    print("Import this module in your main code")
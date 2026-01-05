# simple_test.py - 가장 기본적인 카메라 테스트
from isaacsim.simulation_app import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from omni.isaac.sensor import Camera
import numpy as np
import cv2

def main():
    # World 생성
    world = World()
    world.scene.add_default_ground_plane()
    
    print("Adding test objects...")
    
    # 여러 개의 큐브 추가 (더 눈에 띄게)
    colors = [
        [1.0, 0.0, 0.0],  # 빨강
        [0.0, 1.0, 0.0],  # 초록
        [0.0, 0.0, 1.0],  # 파랑
        [1.0, 1.0, 0.0],  # 노랑
    ]
    
    positions = [
        [0.0, 0.0, 0.5],
        [0.5, 0.0, 0.5],
        [0.0, 0.5, 0.5],
        [-0.5, 0.0, 0.5],
    ]
    
    cubes = []
    for i, (pos, color) in enumerate(zip(positions, colors)):
        cube = VisualCuboid(
            prim_path=f"/World/Cube_{i}",
            name=f"cube_{i}",
            position=np.array(pos),
            size=0.3,  # 크기 증가
            color=np.array(color)
        )
        cubes.append(cube)
        print(f"  Added cube {i} at {pos} with color {color}")
    
    world.reset()
    
    print("\nWarming up world...")
    for _ in range(60):
        world.step(render=True)
    
    print("\nWorld contains:")
    print(f"  Ground plane: {world.scene.get_object('default_ground_plane')}")
    for i in range(len(cubes)):
        print(f"  Cube {i}: {world.scene.get_object(f'cube_{i}')}")
    
    # 카메라 설정들을 하나씩 테스트
    camera_tests = [
        {
            "name": "very_high_topdown",
            "position": np.array([0.0, 0.0, 5.0]),
            "target": np.array([0.0, 0.0, 0.0]),
        },
        {
            "name": "medium_topdown", 
            "position": np.array([0.0, 0.0, 3.0]),
            "target": np.array([0.0, 0.0, 0.5]),
        },
        {
            "name": "close_topdown",
            "position": np.array([0.0, 0.0, 1.5]),
            "target": np.array([0.0, 0.0, 0.5]),
        },
        {
            "name": "angled",
            "position": np.array([2.0, 2.0, 2.0]),
            "target": np.array([0.0, 0.0, 0.5]),
        },
    ]
    
    for config in camera_tests:
        print(f"\n{'='*60}")
        print(f"Testing: {config['name']}")
        print(f"Position: {config['position']}")
        print(f"Target: {config['target']}")
        print(f"{'='*60}")
        
        # 카메라 생성
        camera = Camera(
            prim_path=f"/World/Camera_{config['name']}",
            position=config['position'],
            resolution=(640, 480),
        )
        camera.initialize()
        
        # look_at 사용
        camera.set_world_pose(
            position=config['position'],
        )
        
        # 타겟을 향하도록 설정
        from pxr import Gf
        look_at = Gf.Vec3d(config['target'][0], config['target'][1], config['target'][2])
        up = Gf.Vec3d(0, 0, 1)
        
        # 카메라 방향 계산
        forward = look_at - Gf.Vec3d(config['position'][0], config['position'][1], config['position'][2])
        forward = forward.GetNormalized()
        
        print(f"Forward direction: {forward}")
        
        # 워밍업
        for _ in range(30):
            world.step(render=True)
        
        # 이미지 캡처
        rgb = camera.get_rgb()
        
        if rgb is not None:
            print(f"✓ Captured: shape={rgb.shape}, dtype={rgb.dtype}")
            print(f"  Stats: min={rgb.min()}, max={rgb.max()}, mean={rgb.mean():.2f}")
            
            # 저장
            if rgb.shape[2] == 4:
                rgb = rgb[:, :, :3]
            
            save_path = f"/tmp/test_{config['name']}.png"
            cv2.imwrite(save_path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            print(f"  Saved: {save_path}")
        else:
            print(f"✗ Failed to capture")
        
        # 다음 테스트를 위해 대기
        for _ in range(10):
            world.step(render=True)
    
    print("\n" + "="*60)
    print("All tests completed!")
    print("Check images at /tmp/test_*.png")
    print("="*60)
    
    # 마지막으로 GUI에서 확인할 수 있도록 대기
    print("\nPress Ctrl+C to exit...")
    try:
        while simulation_app.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        simulation_app.close()
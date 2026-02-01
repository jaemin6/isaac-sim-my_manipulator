# camera_debug.py - Replicator 안정화 버전
from isaacsim.simulation_app import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
import omni.replicator.core as rep
import numpy as np
import cv2
import os
from pxr import Gf, UsdGeom, UsdShade, Sdf

def create_colored_cube(stage, path, position, size, color):
    cube_geom = UsdGeom.Cube.Define(stage, path)
    cube_geom.CreateSizeAttr(size)
    cube_geom.AddTranslateOp().Set(Gf.Vec3d(position[0], position[1], position[2]))
    material_path = f"{path}/Material"
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(color[0], color[1], color[2]))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(cube_geom).Bind(material)

def main():
    world = World()
    world.scene.add_default_ground_plane()
    stage = world.stage
    
    # 조명 설정
    from pxr import UsdLux
    dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome_light.CreateIntensityAttr(1000.0)
    
    print("\n[Setup] Creating colored cubes...")
    colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0]]
    positions = [[0.3, 0.3, 0.2], [0.3, -0.3, 0.2], [-0.3, 0.3, 0.2], [-0.3, -0.3, 0.2]]
    
    for i, (pos, color) in enumerate(zip(positions, colors)):
        create_colored_cube(stage, f"/World/Cube_{i}", pos, 0.2, color) # 크기 0.2로 확대
    
    world.reset()
    
    # 카메라 설정 (해상도 1024x768로 상향하여 DLSS 오류 방지)
    camera_configs = [
        {"name": "topdown", "pos": (0.0, 0.0, 2.5), "look": (0.0, 0.0, 0.0)},
        {"name": "angled", "pos": (1.5, 1.5, 1.5), "look": (0.0, 0.0, 0.0)},
    ]

    for config in camera_configs:
        print(f"\n[Rendering] Testing camera: {config['name']}")
        
        # 1. Replicator 카메라 및 렌더 프러덕트 생성
        cam = rep.create.camera(position=config["pos"], look_at=config["look"])
        rp = rep.create.render_product(cam, (1024, 768)) # 해상도 상향
        
        # 2. RGB 어노테이터 설정 및 연결
        rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb_annot.attach([rp])
        
        # 3. 중요: Replicator가 한 프레임을 렌더링하도록 대기
        # 단순 world.step보다 Orchestrator 조작이 더 확실합니다.
        rep.orchestrator.step() 
        
        # 4. 데이터 획득 시도 (여러 번 시도)
        rgb = None
        for i in range(10): # 최대 10프레임 대기하며 데이터 확인
            world.step(render=True)
            rgb = rgb_annot.get_data()
            if rgb is not None and len(rgb) > 0:
                break
        
        if rgb is not None:
            img = np.array(rgb)
            if img.shape[2] == 4: img = img[:, :, :3] # RGBA -> RGB
            
            # 타입 변환
            if img.dtype != np.uint8:
                img = (np.clip(img, 0, 1) * 255).astype(np.uint8)
            
            # 저장 경로를 현재 폴더로 변경
            save_path = os.path.join(os.getcwd(), f"debug_{config['name']}.png")
            cv2.imwrite(save_path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            print(f"  ✓ SUCCESS: Saved to {save_path}")
        else:
            print(f"  ✗ FAILED: Could not get RGB data for {config['name']}")
    
    print("\nCheck the current folder for debug_*.png files.")
    simulation_app.close()

if __name__ == "__main__":
    main()
"""
generate_dataset.py v4
vision_control.py의 동작하는 카메라 패턴 그대로 사용
semantic_segmentation 마스크 → bbox → YOLO 라벨

실행:
  cd ~/isaac-sim
  ./python.sh standalone_examples/my_manipulator/yolo/generate_dataset.py
"""

from isaacsim import SimulationApp
app = SimulationApp(launch_config={"headless": True, "renderer": "RaytracedLighting"})

import numpy as np
import cv2
import json
from pathlib import Path

import omni.replicator.core as rep
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid, GroundPlane
from omni.isaac.core.utils.semantics import add_update_semantics
from omni.isaac.core.utils.stage import get_current_stage
from pxr import UsdShade, Sdf, Gf

# ── 설정 ──────────────────────────────────────────────────────────────
OUTPUT_DIR   = Path(__file__).parent / "dataset"
NUM_TRAIN    = 1500
NUM_VAL      = 300
IMG_W, IMG_H = 1024, 768

CLASSES = {"red": 0, "green": 1, "blue": 2, "yellow": 3}

CUBE_COLORS = {
    "red":    np.array([1.0, 0.05, 0.05]),
    "green":  np.array([0.05, 1.0, 0.05]),
    "blue":   np.array([0.05, 0.05, 1.0]),
    "yellow": np.array([1.0, 1.0, 0.05]),
}

TABLE_X = (0.28, 0.72)
TABLE_Y = (-0.32, 0.32)
TABLE_Z = 0.526

# 카메라 기본 위치 (vision_control.py와 동일)
CAM_POSITION = [1.0, 0.0, 1.5]
CAM_LOOK_AT  = [0.5, 0.0, 0.0]
CAM_FOCAL    = 12.0


def make_dirs():
    for split in ["train", "val"]:
        (OUTPUT_DIR / "images" / split).mkdir(parents=True, exist_ok=True)
        (OUTPUT_DIR / "labels" / split).mkdir(parents=True, exist_ok=True)


def setup_scene():
    """vision_control.py / world.py 패턴으로 씬 구성"""
    world = World()

    world.scene.add(GroundPlane("/World/Ground", z_position=0))
    world.scene.add(FixedCuboid(
        prim_path="/World/Table", name="table",
        position=np.array([0.5, 0.0, 0.25]),
        scale=np.array([0.6, 0.8, 0.5]),
        color=np.array([0.6, 0.4, 0.2])
    ))

    # 큐브 생성
    cube_prims = {}
    for i, (color_name, color) in enumerate(CUBE_COLORS.items()):
        prim_path = f"/World/Cube_{color_name}"
        world.scene.add(DynamicCuboid(
            prim_path=prim_path,
            name=f"cube_{color_name}",
            position=np.array([0.4 + i * 0.1, 0.0, TABLE_Z]),
            scale=np.array([0.05, 0.05, 0.05]),
            color=color,
            mass=0.1,
        ))
        cube_prims[color_name] = prim_path

    world.reset()

    # 시맨틱 라벨 부착 (reset 후에 붙여야 안정적)
    stage = get_current_stage()
    for color_name, prim_path in cube_prims.items():
        prim = stage.GetPrimAtPath(prim_path)
        add_update_semantics(prim, color_name)
        print(f"[Scene] semantic: {color_name} → {prim_path}")

    # 안정화 (vision_control.py처럼 충분히)
    print("[Scene] Stabilizing (120 frames)...")
    for _ in range(120):
        world.step(render=True)

    return world, cube_prims


def setup_camera():
    """vision_control.py _ensure_camera_ready() 패턴 그대로"""
    print("[Camera] Setting up replicator camera...")

    rep_cam = rep.create.camera(
        position=CAM_POSITION,
        look_at=CAM_LOOK_AT,
        focal_length=CAM_FOCAL,
    )
    rp = rep.create.render_product(rep_cam, (IMG_W, IMG_H))

    # ★ 카메라 Prim 직접 저장
    cam_prim = rep_cam.get_output_prims()["prims"][0]

    rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    seg_annot = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")

    rgb_annot.attach([rp])
    seg_annot.attach([rp])

    print(f"[Camera] Ready. prim={cam_prim}")
    return cam_prim, rp, rgb_annot, seg_annot


def get_frames(world, rgb_annot, seg_annot):
    """vision_control.py _get_frames() 패턴 - 5프레임 후 캡처"""
    for _ in range(5):
        world.step(render=True)

    rgb_data = rgb_annot.get_data()
    seg_data = seg_annot.get_data()

    rgb = None
    if rgb_data is not None and len(rgb_data) > 0:
        rgb = np.array(rgb_data)
        if rgb.ndim == 3 and rgb.shape[2] == 4:
            rgb = rgb[:, :, :3]
        if rgb.dtype != np.uint8:
            rgb = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)

    return rgb, seg_data


def randomize_scene(world, cube_prims):
    """큐브 위치 랜덤화 - set_world_pose 사용"""
    for color_name in cube_prims.keys():
        x = np.random.uniform(*TABLE_X)
        y = np.random.uniform(*TABLE_Y)
        cube_obj = world.scene.get_object(f"cube_{color_name}")
        if cube_obj is not None:
            cube_obj.set_world_pose(position=np.array([x, y, TABLE_Z]))

    # 카메라 위치 랜덤화
    cam_x = np.random.uniform(0.3, 0.8)
    cam_y = np.random.uniform(-0.2, 0.2)
    cam_z = np.random.uniform(1.1, 1.8)
    return [cam_x, cam_y, cam_z]


def seg_to_yolo(seg_data):
    """semantic segmentation 데이터 → YOLO bbox 라벨"""
    lines = []

    if seg_data is None:
        return lines

    seg_array = seg_data.get("data")
    info      = seg_data.get("info", {})
    id_labels = info.get("idToLabels", {})

    if seg_array is None or len(seg_array) == 0:
        return lines

    seg_img = np.array(seg_array)

    for id_str, label_info in id_labels.items():
        class_name = label_info.get("class", "").lower()
        if class_name not in CLASSES:
            continue

        try:
            seg_id = int(id_str)
        except Exception:
            continue

        # 해당 ID 픽셀 마스크
        if seg_img.ndim == 3:
            # RGBA 형태면 R채널이 instance id
            mask = seg_img[:, :, 0] == seg_id
        else:
            mask = seg_img == seg_id

        if not mask.any():
            continue

        rows = np.any(mask, axis=1)
        cols = np.any(mask, axis=0)
        y_min, y_max = np.where(rows)[0][[0, -1]]
        x_min, x_max = np.where(cols)[0][[0, -1]]

        cx = ((x_min + x_max) / 2.0) / IMG_W
        cy = ((y_min + y_max) / 2.0) / IMG_H
        bw = (x_max - x_min) / IMG_W
        bh = (y_max - y_min) / IMG_H

        if bw > 0.005 and bh > 0.005:
            lines.append(f"{CLASSES[class_name]} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")

    return lines


def debug_frame(seg_data):
    """첫 프레임 디버그"""
    if seg_data is None:
        print("[Debug] seg_data = None")
        return False

    info     = seg_data.get("info", {})
    id_labels = info.get("idToLabels", {})
    arr      = seg_data.get("data")

    print(f"\n[Debug] seg data shape : {np.array(arr).shape if arr is not None else 'None'}")
    print(f"[Debug] idToLabels     : {id_labels}")

    lines = seg_to_yolo(seg_data)
    print(f"[Debug] YOLO lines     : {lines}\n")
    return len(lines) > 0


def generate(split, num_images, world, cube_prims, cam_prim, rgb_annot, seg_annot):
    img_dir = OUTPUT_DIR / "images" / split
    lbl_dir = OUTPUT_DIR / "labels" / split
    saved = skipped = 0

    print(f"[Dataset] Generating {num_images} {split} images...")

    while saved < num_images:
        # 랜덤화
        new_cam_pos = randomize_scene(world, cube_prims)

        # 카메라 위치 업데이트
        from pxr import UsdGeom as UsdGeomLocal, Gf as GfLocal
        cam_xf = UsdGeomLocal.Xformable(cam_prim)
        for op in cam_xf.GetOrderedXformOps():
            if "translate" in op.GetOpName().lower():
                op.Set(GfLocal.Vec3d(*new_cam_pos))
                break

        # 프레임 취득
        rgb, seg_data = get_frames(world, rgb_annot, seg_annot)

        if rgb is None:
            skipped += 1
            continue

        yolo_lines = seg_to_yolo(seg_data)
        if not yolo_lines:
            skipped += 1
            continue

        # 저장
        fname = f"{split}_{saved:05d}"
        img_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(img_dir / f"{fname}.jpg"), img_bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
        (lbl_dir / f"{fname}.txt").write_text("\n".join(yolo_lines))

        saved += 1
        if saved % 100 == 0:
            print(f"  [{split}] {saved}/{num_images} (skipped={skipped})")

    print(f"[Dataset] ✓ {split}: {saved}장 저장 완료")


def save_yaml():
    content = f"""# YOLO 큐브 데이터셋
path: {OUTPUT_DIR.absolute()}
train: images/train
val:   images/val

nc: {len(CLASSES)}
names: {list(CLASSES.keys())}
"""
    (OUTPUT_DIR / "cubes.yaml").write_text(content)
    print(f"[Dataset] cubes.yaml 저장 완료")


def main():
    make_dirs()

    world, cube_prims = setup_scene()
    cam_prim, rp, rgb_annot, seg_annot = setup_camera()

    # 워밍업 (vision_control.py처럼 충분히)
    print("[Dataset] Warming up camera (60 frames)...")
    for _ in range(60):
        world.step(render=True)

    # 디버그 첫 프레임
    rgb, seg_data = get_frames(world, rgb_annot, seg_annot)
    ok = debug_frame(seg_data)
    if not ok:
        print("[Dataset] ⚠️  시맨틱 감지 실패, 계속 진행...")

    generate("train", NUM_TRAIN, world, cube_prims, cam_prim, rgb_annot, seg_annot)
    generate("val",   NUM_VAL,   world, cube_prims, cam_prim, rgb_annot, seg_annot)
    save_yaml()

    print(f"\n[Dataset] ✅ 완료! → {OUTPUT_DIR}")
    print(f"\n다음 단계 (학습):")
    print(f"  cd ~/isaac-sim")
    print(f"  ./python.sh -c \"from ultralytics import YOLO; model = YOLO('yolov8n.pt'); model.train(data='{OUTPUT_DIR}/cubes.yaml', epochs=100, imgsz=640, batch=16)\"")

    app.close()


if __name__ == "__main__":
    main()
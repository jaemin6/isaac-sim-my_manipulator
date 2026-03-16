# 1. 초기화 (SimulationApp 실행 및 환경 설정)
app = SimulationApp({"headless": True}) # 그래픽 화면 없이 빠르게 생성
make_dirs() # dataset/images, labels 폴더 생성

def main():
    # 2. 씬 구성 (Table, Light, Cubes 배치)
    sim_context = SimulationContext()
    cube_prims = build_scene()
    
    # 3. 시맨틱 라벨링 (YOLO가 '무엇'인지 알게 이름표 달기)
    set_semantic_labels(cube_prims) # Red_Cube -> "red"

    # 4. Replicator 설정 (무한 랜덤 생성기 가동)
    #    - 카메라 위치/각도 랜덤화
    #    - 조명 강도 랜덤화
    #    - 큐브 위치 랜덤화
    rgb_annot, bbox_annot = setup_replicator(cube_prims)

    # 5. 데이터 생성 루프 (핵심 반복 구간)
    for i in range(total_images):
        # (1) 한 스텝 진행 (물체들이 랜덤한 위치로 순간이동)
        rep.orchestrator.step() 
        
        # (2) 데이터 추출 (RGB 이미지와 2D 바운딩 박스 정보 가져오기)
        rgb_data = rgb_annot.get_data()
        bbox_data = bbox_annot.get_data()
        
        # (3) YOLO 포맷 변환 (Isaac Sim 좌표 -> 0~1 사이의 중심점, 가로, 세로)
        yolo_lines = bbox_to_yolo(bbox_data)
        
        # (4) 파일 저장 (.jpg 이미지와 .txt 라벨 파일)
        save_to_disk(rgb_data, yolo_lines)

    # 6. 학습 설정 파일 생성 (cubes.yaml)
    save_yaml()
    app.close()
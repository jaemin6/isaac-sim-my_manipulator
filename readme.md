# 🦾 Isaac Sim Manipulator Tutorial (Franka + YOLO + ROS2)

Isaac Sim 환경에서 Franka 로봇을 이용하여  
**객체 인식 → 집기 → 목표 위치 이동**까지 수행하는 End-to-End 프로젝트입니다

---

## 📌 프로젝트 개요

- Isaac Sim 기반 로봇 시뮬레이션
- Replicator를 활용한 데이터셋 자동 생성
- YOLO 기반 객체 인식
- Franka Manipulator를 이용한 Pick & Place
- ROS2 Bridge를 통한 데이터 퍼블리싱

---

## 🧱 프로젝트 구조

```
my_manipulator/
├── main.py                  # 메인 실행 (Phase 기반 제어)
├── sim/
│   └── world.py             # 시뮬레이션 환경 + ROS2 Bridge
├── controllers/
│   └── unified_controller.py # 전체 제어 로직 (Phase 관리)
│
├── vision/
│   └── detector.py          # 객체 인식 (HSV / YOLO)
├── yolo/
│   └── generate_dataset.py  # 데이터셋 생성
│
├── utils/
│   └── cube_utils.py        # 큐브 위치 / attach / detach
│
├── data_collection/         # 데이터 수집 관련
├── config/                  # 설정 파일
├── rl/                      # 강화학습 관련 코드
├── train_rl.py              # RL 학습
├── test_rl.py               # RL 테스트
│
├── camera_debug.py          # 카메라 디버깅
├── img/                     # 결과 이미지
└── readme.md
```

---

# 🚀 Phase 1: Simulation Environment 구축
<img width="640" height="360" alt="Screenshot from 2026-03-18 18-26-06" src="https://github.com/user-attachments/assets/2ea9f964-144c-4feb-93c0-e7713beb7f1d" />

### ✔️ 목표
- Franka 로봇 생성
- 테이블 + 큐브 배치
- 카메라 및 ROS2 Bridge 설정

### 🔧 핵심 코드
- `world.py`

### 📌 주요 기능
- Ground / Table / Cube 생성
- Franka 로봇 초기화
- Top Camera 생성
- ROS2 Topic 퍼블리시

```
/clock
/joint_states
/tf
/isaac/rgb
/isaac/depth
/isaac/camera_info
```

### 📷 구조
(여기에 시뮬레이션 화면 이미지 추가)

---

# 🧠 Phase 2: Dataset 생성 (Replicator)

### ✔️ 목표
- 자동 데이터셋 생성
- semantic segmentation → YOLO 라벨 변환
![test_result](https://github.com/user-attachments/assets/81ad0a09-c544-43c0-8a72-32295ce6a728)

### 🔧 실행 방법

```bash
cd ~/isaac-sim
./python.sh standalone_examples/my_manipulator/yolo/generate_dataset.py
```

### 📌 특징
- 큐브 위치 랜덤화
- 카메라 위치 랜덤화
- semantic → bbox 자동 변환

### 📁 결과 구조

```
dataset/
├── images/
│   ├── train/
│   └── val/
├── labels/
│   ├── train/
│   └── val/
└── cubes.yaml
```

---

# 🤖 Phase 3: YOLO 학습 & Detection
![output](https://github.com/user-attachments/assets/c220dd08-12ea-4e4e-a285-2a98c0cb73df)

### ✔️ 목표
- 생성된 데이터로 YOLO 학습
- 실시간 객체 위치 검출

### 🔧 학습

```bash
./python.sh -c "
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
model.train(data='dataset/cubes.yaml', epochs=100)
"
```

### 🔍 Detection 방식

#### 1️⃣ YOLO 기반 (권장)
- 학습 모델 사용

#### 2️⃣ HSV 기반 (현재 코드)
- `detector.py`

### 📌 처리 과정
```
RGB → HSV → Threshold → Contour → Bounding Box
```

---

# 🎯 Phase 4: Grasp & Manipulation


### ✔️ 목표
- 인식된 객체를 Franka가 집기
- 원하는 위치로 이동

### 🔧 실행

```bash
./python.sh standalone_examples/my_manipulator/main.py
```

### 🎮 조작

```
1 / 2 / 3 / 4 → Phase 선택
SPACE → Grasp 실행
```

---

## ⚙️ 동작 흐름

```
1. 카메라 이미지 획득
2. 객체 위치 추출
3. 가장 가까운 큐브 선택
4. End Effector 이동
5. attach (grasp)
6. 목표 위치 이동
7. detach (release)
```

---

## 🧩 핵심 로직

### 📍 실시간 위치 추적
- `RigidPrim.get_world_pose()` 사용

### 📍 Grasp 방식
- Fixed Joint 생성

### 📍 안정성 처리
- Velocity Reset (튕김 방지)

---

# 🔗 ROS2 연동

### 퍼블리시 토픽

```
/clock
/joint_states
/tf
/isaac/rgb
/isaac/depth
```

👉 실제 로봇 시스템 확장 가능

---

# 🏁 결과

- 색상별 큐브 인식 성공
- Franka 자동 Pick & Place 수행
- ROS2 기반 확장 가능 구조 완성

---

# 📌 향후 개선

- YOLO → 실시간 추적 적용
- Grasp Planning 고도화
- Mobile Robot 결합 (Navigation)
- 실제 카메라 연동

---

# 💡 한줄 요약

👉 **"데이터 생성 → 학습 → 인식 → 로봇 제어까지 이어지는 완전한 로봇 AI 파이프라인"**

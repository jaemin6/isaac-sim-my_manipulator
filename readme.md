# 그리퍼 기본 시퀀스

1. EE → 큐브 위
2. EE → 큐브 접촉 위치
3. 그리퍼 닫기
4. EE 위로 들어올리기

# isaac-sim simulation 
[Camera]
   ↓
[Computer Vision]
(Object Detection / Pose)
   ↓
[Target Pose in World Frame]
   ↓
[Franka Manipulation]
(Pick)
   ↓
[Mobile Base Navigation]
(Move)
   ↓
[Franka Manipulation]
(Place)

# simulation 구조


```
simulation_1/
│
├── main.py                # 전체 실행 루프
│
├── sim/
│   ├── world.py           # 월드, 테이블, 큐브 생성
│   ├── robot.py           # Franka 로딩 & 제어
│   └── camera.py          # Isaac Camera 래퍼
│
├── vision/
│   ├── detector.py        # OpenCV 큐브 검출
│   └── geometry.py        # pixel → world 변환
│
├── control/
│   ├── ik_solver.py       # IK 계산
│   └── pick_place.py      # pick 위치 생성
│
└── config/
    └── params.py          # 카메라, 임계값, 오프셋
```
---

## 전체 동작 흐름 요약

1. `main.py` 실행
2. `world.py` → 시뮬레이션 환경 생성
3. `robot.py` → Franka 로봇 로드
4. `camera.py` → 카메라 이미지 획득
5. `detector.py` → 큐브 위치(픽셀 좌표) 검출
6. `geometry.py` → 픽셀 좌표를 월드 좌표로 변환
7. `ik_solver.py` → 로봇이 도달할 관절 각도 계산
8. `pick_place.py` → 집기 & 놓기 동작 수행

---

## 설계 의도

- **역할별 폴더 분리**로 코드 가독성 향상
- 비전 / 제어 / 시뮬레이션을 독립적으로 수정 가능
- 실제 로봇 또는 모바일 로봇과 결합 시 재사용 가능 구조



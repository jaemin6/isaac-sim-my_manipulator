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

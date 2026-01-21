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


## Simulation 1-8 Result

- Pre-grasp XY error: ~0.002 m
- Z error at grasp: ~0.127 m
- FixedJoint grasp success
- Lift successful

Console log:
```text
[Init] EE: ...
...
✓ GRASP SEQUENCE COMPLETED
```

## 테스트 순서

테스트 순서
Phase 1 테스트 (현재 활성화됨):

1. SPACE 키 누르기 → 자동 grasp
2. P 키 누르기 → 자동 place
3. M 키 누르기 → 모든 큐브 자동 처리

Phase 2 테스트 (IK):

1. 2 키 → Phase 2로 전환
2. SPACE → IK로 정밀하게 grasp
3. L 키 → 성능 로그 확인 (시간, 오차 측정)
4. M 키 → 모든 큐브 처리 후 자동으로 성능 리포트!
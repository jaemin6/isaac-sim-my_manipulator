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

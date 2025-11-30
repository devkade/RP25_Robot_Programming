import pybullet as p
import pybullet_data
import time
import numpy as np
import math

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

p.loadURDF("plane.urdf")
table = p.loadURDF("table/table.urdf", [0.5, 0, 0], useFixedBase=True)
robot = p.loadURDF("franka_panda/panda.urdf", basePosition=[0.0, 0, 0.66], useFixedBase=True)

box_6 = p.loadURDF("models/box6.xacro", basePosition=[0.3, -0.1, 0.66])
box_5 = p.loadURDF("models/box5.xacro", basePosition=[0.3, 0.0, 0.66])
box_4 = p.loadURDF("models/box4.xacro", basePosition=[0.3, 0.1, 0.66])

box_3 = p.loadURDF("models/box3.xacro", basePosition=[0.4, -0.1, 0.66])
cylinder_0 = p.loadURDF("models/cylinder1.xacro", basePosition=[0.4, 0.0, 0.66])
box_2 = p.loadURDF("models/box2.xacro", basePosition=[0.4, 0.1, 0.66])

box_0 = p.loadURDF("models/box.xacro", basePosition=[0.5, 0.0, 0.66])
triangle = p.loadURDF("models/triangle.xacro", basePosition=[0.5, 0.1, 0.66])

case_collision = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    fileName="models/case.obj", 
    flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
    meshScale=[1, 1, 1]
)

case_visual = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName="models/case.obj",
    meshScale=[1, 1, 1]
)

case = p.createMultiBody(
    baseMass=0.05,
    baseCollisionShapeIndex=case_collision,
    baseVisualShapeIndex=case_visual,
    basePosition=[0.0, 0.3, 0.64],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 3.14159])
)


p.changeVisualShape(box_6, -1, rgbaColor=[1, 0, 1, 1])  # box
p.changeVisualShape(box_5, -1, rgbaColor=[1, 0, 0, 1])  # box2
p.changeVisualShape(box_4, -1, rgbaColor=[1, 0, 0, 1])  # box3

p.changeVisualShape(box_3, -1, rgbaColor=[1, 0, 0, 1])  # box4
p.changeVisualShape(cylinder_0, -1, rgbaColor=[0, 0, 1, 1])  # box5
p.changeVisualShape(box_2, -1, rgbaColor=[1, 0, 0, 1])  # box6

p.changeVisualShape(box_0, -1, rgbaColor=[1, 0, 0, 1])  # box5
p.changeVisualShape(triangle, -1, rgbaColor=[1, 0, 1, 1])  # box6

p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0.3, 0, 0.5])

## 기본적인 Scene 코드 이후 부분을 제공된 Task를 수행하도록 구현하세요 ##

# ===== 그리퍼 방향 정의 =====
DEFAULT_ORN = p.getQuaternionFromEuler([math.pi, 0, 0])  # 아래로 향함
TRIANGLE_ORN = p.getQuaternionFromEuler([math.pi, 0, -math.pi/2])  # 삼각기둥용

# ===== 객체 데이터 정의 =====
# (물체#, 변수명, pick_pos, place_pos, grasp_z, grip_width, orientation)
objects_data = [
    # 순서: 가까운 것부터, 어려운 것 마지막
    (1, "box_4",      (0.3,  0.1),  (-0.15, 0.35), 0.69,  0.015, DEFAULT_ORN),
    (2, "box_5",      (0.3,  0.0),  (-0.15, 0.25), 0.69,  0.015, DEFAULT_ORN),
    (3, "box_6",      (0.3, -0.1),  ( 0.15, 0.25), 0.69,  0.015, DEFAULT_ORN),
    (4, "box_2",      (0.4,  0.1),  ( 0.05, 0.35), 0.69,  0.015, DEFAULT_ORN),
    (5, "cylinder_0", (0.4,  0.0),  (-0.05, 0.25), 0.69,  0.015, DEFAULT_ORN),
    (6, "box_3",      (0.4, -0.1),  (-0.05, 0.35), 0.69,  0.015, DEFAULT_ORN),
    (8, "box_0",      (0.5,  0.0),  ( 0.15, 0.35), 0.705, 0.015, DEFAULT_ORN),  # 높이 0.09
    (7, "triangle",   (0.5,  0.1),  ( 0.05, 0.25), 0.69,  0.010, TRIANGLE_ORN), # 무게 5배
]


# ===== 헬퍼 함수 =====
def open_gripper():
    """그리퍼 열기 (최대)"""
    p.setJointMotorControl2(robot, 9, p.POSITION_CONTROL, targetPosition=0.04)
    p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL, targetPosition=0.04)


def close_gripper(width):
    """그리퍼 닫기 (지정 너비)"""
    p.setJointMotorControl2(robot, 9, p.POSITION_CONTROL, targetPosition=width)
    p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL, targetPosition=width)


def move_to_position(target_pos, target_orn):
    """IK 계산 후 조인트 이동"""
    joint_positions = p.calculateInverseKinematics(
        robot,
        endEffectorLinkIndex=11,
        targetPosition=target_pos,
        targetOrientation=target_orn
    )

    for i in range(7):
        p.setJointMotorControl2(
            robot,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_positions[i]
        )


def wait_for_motion(steps):
    """시뮬레이션 스텝 대기"""
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1/240)


# ===== 일반 객체 Pick-Place 함수 =====
def pick_object(pick_pos, grasp_z, grip_width, orientation):
    """일반 객체 집기"""
    approach_z = grasp_z + 0.15

    # 1. Approach
    move_to_position([pick_pos[0], pick_pos[1], approach_z], orientation)
    wait_for_motion(100)

    # 2. Descend
    move_to_position([pick_pos[0], pick_pos[1], grasp_z], orientation)
    wait_for_motion(100)

    # 3. Grasp
    close_gripper(grip_width)
    wait_for_motion(50)

    # 4. Lift
    move_to_position([pick_pos[0], pick_pos[1], approach_z], orientation)
    wait_for_motion(100)


def place_object(place_pos, place_z, orientation):
    """일반 객체 배치"""
    approach_z = place_z + 0.15

    # 5. Move
    move_to_position([place_pos[0], place_pos[1], approach_z], orientation)
    wait_for_motion(100)

    # 6. Lower
    move_to_position([place_pos[0], place_pos[1], place_z], orientation)
    wait_for_motion(100)

    # 7. Release
    open_gripper()
    wait_for_motion(50)

    # 8. Retreat
    move_to_position([place_pos[0], place_pos[1], approach_z], orientation)
    wait_for_motion(100)


# ===== 삼각기둥 전용 함수 (다단계 이동) =====
def pick_triangle(pick_pos, grasp_z=0.69, grip_width=0.010):
    """삼각기둥 전용 pick - 다단계 접근"""
    approach_z = 0.85  # 더 높은 안전 높이
    mid_z = (approach_z + grasp_z) / 2

    # 1. 높은 위치에서 접근
    move_to_position([pick_pos[0], pick_pos[1], approach_z], TRIANGLE_ORN)
    wait_for_motion(150)

    # 2. 중간 높이로 천천히 하강
    move_to_position([pick_pos[0], pick_pos[1], mid_z], TRIANGLE_ORN)
    wait_for_motion(100)

    # 3. grasp 높이로 최종 하강
    move_to_position([pick_pos[0], pick_pos[1], grasp_z], TRIANGLE_ORN)
    wait_for_motion(150)

    # 4. 그리퍼 닫기 (더 세게, 더 오래 대기)
    close_gripper(grip_width)
    wait_for_motion(100)

    # 5. 천천히 들어올리기
    move_to_position([pick_pos[0], pick_pos[1], mid_z], TRIANGLE_ORN)
    wait_for_motion(150)

    move_to_position([pick_pos[0], pick_pos[1], approach_z], TRIANGLE_ORN)
    wait_for_motion(150)


def place_triangle_with_waypoints(place_pos, place_z=0.70):
    """삼각기둥 전용 place - waypoint 사용"""
    approach_z = 0.85
    mid_z = (approach_z + place_z) / 2

    # Waypoints: pick 위치 → 케이스
    waypoints = [
        (0.35, 0.15, approach_z),   # WP1: 테이블 위 중간
        (0.15, 0.20, approach_z),   # WP2: 케이스 방향
        (place_pos[0], place_pos[1], approach_z),  # WP3: 목표 위
    ]

    # 각 waypoint 순차 이동
    for i, wp in enumerate(waypoints):
        move_to_position(list(wp), TRIANGLE_ORN)
        wait_for_motion(100)
        print(f"Triangle waypoint {i+1} reached")

    # 하강 (2단계)
    move_to_position([place_pos[0], place_pos[1], mid_z], TRIANGLE_ORN)
    wait_for_motion(100)

    move_to_position([place_pos[0], place_pos[1], place_z], TRIANGLE_ORN)
    wait_for_motion(150)

    # 그리퍼 열기
    open_gripper()
    wait_for_motion(100)

    # 천천히 후퇴
    move_to_position([place_pos[0], place_pos[1], approach_z], TRIANGLE_ORN)
    wait_for_motion(150)


# ===== 메인 실행 =====
def main():
    # 초기화
    open_gripper()
    wait_for_motion(50)

    # place_z 추정값
    place_z = 0.70

    # 8개 객체 처리
    for obj_num, name, pick_pos, place_pos, grasp_z, grip_width, orn in objects_data:
        print(f"\n=== Processing Object {obj_num}: {name} ===")
        print(f"Pick: {pick_pos} -> Place: {place_pos}")

        if name == "triangle":
            # 삼각기둥: 특별 처리
            pick_triangle(pick_pos, grasp_z, grip_width)
            place_triangle_with_waypoints(place_pos, place_z)
        else:
            # 일반 객체
            pick_object(pick_pos, grasp_z, grip_width, orn)
            place_object(place_pos, place_z, orn)

        print(f"Object {obj_num} ({name}) completed!")

    print("\n=== All objects placed! ===")


# 실행
main()

# 시뮬레이션 유지
for _ in range(1000):
    p.stepSimulation()
    time.sleep(1/60)

p.disconnect()


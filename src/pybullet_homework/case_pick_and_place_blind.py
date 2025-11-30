import pybullet as p
import pybullet_data
import time
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

# ===== Home Position 설정 및 End-Effector Orientation (MoveIt2 스타일) =====
# home position 조인트 값으로 로봇 초기화
HOME_JOINTS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
for i in range(7):
    p.resetJointState(robot, i, HOME_JOINTS[i])

# Home position에서 end-effector의 실제 orientation 획득
# Franka Panda의 home position이 이미 위에서 아래를 향하는 자세임
link_state = p.getLinkState(robot, 11)
DEFAULT_ORN = link_state[1]  # Home position의 실제 quaternion 사용

# 삼각기둥용: DEFAULT_ORN에서 yaw만 -90도 회전 (quaternion 곱셈)
yaw_rotation = p.getQuaternionFromEuler([0, 0, -math.pi/2])
TRIANGLE_ORN = p.multiplyTransforms([0,0,0], DEFAULT_ORN, [0,0,0], yaw_rotation)[1]

print(f"DEFAULT_ORN (from home position): {DEFAULT_ORN}")
print(f"TRIANGLE_ORN (with yaw -90): {TRIANGLE_ORN}")

# ===== 객체 데이터 정의 =====
# (물체#, 변수명, pick_pos, place_pos, grasp_z, grip_width, use_triangle_orn)
# use_triangle_orn=False면 DEFAULT_ORN, True면 TRIANGLE_ORN 사용
objects_data = [
    # 순서: 가까운 것부터, 어려운 것 마지막
    (1, "box_4",      (0.3,  0.1),  (-0.15, 0.35), 0.69,  0.015, False),
    (2, "box_5",      (0.3,  0.0),  (-0.15, 0.25), 0.69,  0.015, False),
    (3, "box_6",      (0.3, -0.1),  ( 0.15, 0.25), 0.69,  0.015, False),
    (4, "box_2",      (0.4,  0.1),  ( 0.05, 0.35), 0.69,  0.015, False),
    (5, "cylinder_0", (0.4,  0.0),  (-0.05, 0.25), 0.69,  0.015, False),
    (6, "box_3",      (0.4, -0.1),  (-0.05, 0.35), 0.69,  0.015, False),
    (8, "box_0",      (0.5,  0.0),  ( 0.15, 0.35), 0.705, 0.015, False),  # 높이 0.09
    (7, "triangle",   (0.5,  0.1),  ( 0.05, 0.25), 0.69,  0.010, True),   # 무게 5배
]


# ===== 속도 설정 (느리게) =====
SIM_SPEED = 1/480  # 시뮬레이션 속도 (느리게)
MOTION_STEPS = 200  # 기본 동작 스텝 (늘림)
GRIPPER_STEPS = 100  # 그리퍼 동작 스텝


# ===== 헬퍼 함수 =====
def euler_to_quaternion_moveit_style(roll, pitch, yaw):
    """
    MoveIt2 TF2와 동일한 방식으로 RPY를 Quaternion으로 변환

    두 라이브러리 모두 동일한 extrinsic XYZ (fixed-axis) 컨벤션을 사용합니다.
    따라서 [roll, pitch, yaw] 입력이 직접 호환됩니다.

    반환값: (x, y, z, w) 형식의 quaternion (PyBullet 표준)
    """
    return p.getQuaternionFromEuler([roll, pitch, yaw])


def open_gripper():
    """그리퍼 열기 (최대)"""
    p.setJointMotorControl2(robot, 9, p.POSITION_CONTROL, targetPosition=0.04, force=20)
    p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL, targetPosition=0.04, force=20)


def close_gripper(width):
    """그리퍼 닫기 (지정 너비)"""
    p.setJointMotorControl2(robot, 9, p.POSITION_CONTROL, targetPosition=width, force=40)
    p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL, targetPosition=width, force=40)


def move_to_position(target_pos, target_orn):
    """
    IK 계산 후 조인트 이동 - MoveIt2 스타일
    위에서 접근하는 자세로 제약 추가
    """
    # Franka Panda 조인트 제한
    lower_limits = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
    upper_limits = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
    joint_ranges = [5.8, 3.5, 5.8, 3.0, 5.8, 3.8, 5.8]
    # 위에서 접근하는 기본 자세 (MoveIt2 home position과 유사)
    rest_poses = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

    joint_positions = p.calculateInverseKinematics(
        robot,
        endEffectorLinkIndex=11,
        targetPosition=target_pos,
        targetOrientation=target_orn,
        lowerLimits=lower_limits,
        upperLimits=upper_limits,
        jointRanges=joint_ranges,
        restPoses=rest_poses,
        maxNumIterations=200,
        residualThreshold=1e-6
    )

    # 조인트 이동 (느린 속도)
    for i in range(7):
        p.setJointMotorControl2(
            robot,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_positions[i],
            force=240,
            maxVelocity=0.5  # 더 느린 속도
        )


def wait_for_motion(steps=None):
    """시뮬레이션 스텝 대기"""
    if steps is None:
        steps = MOTION_STEPS
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(SIM_SPEED)


# ===== Cartesian Path 함수 (MoveIt2 스타일) =====
def compute_cartesian_path(waypoints, eef_step=0.005):
    """
    MoveIt2의 computeCartesianPath를 PyBullet에서 구현

    Args:
        waypoints: [[x, y, z], ...] 형식의 위치 리스트
        eef_step: end-effector 이동 단위 (기본 5mm, MoveIt2와 동일)

    Returns:
        보간된 waypoint 리스트
    """
    interpolated_waypoints = []

    for i in range(len(waypoints) - 1):
        start = waypoints[i]
        end = waypoints[i + 1]

        # 두 점 사이 거리
        distance = math.sqrt(
            (end[0] - start[0])**2 +
            (end[1] - start[1])**2 +
            (end[2] - start[2])**2
        )

        # 보간 포인트 수
        num_steps = max(int(distance / eef_step), 1)

        for j in range(num_steps + 1):
            t = j / num_steps
            point = [
                start[0] + t * (end[0] - start[0]),
                start[1] + t * (end[1] - start[1]),
                start[2] + t * (end[2] - start[2])
            ]
            interpolated_waypoints.append(point)

    return interpolated_waypoints


def execute_cartesian_path(waypoints, orientation, step_delay=50):
    """
    보간된 Cartesian 경로 실행

    Args:
        waypoints: 보간된 위치 리스트
        orientation: 유지할 end-effector orientation
        step_delay: 각 waypoint 간 대기 스텝
    """
    for wp in waypoints:
        move_to_position(wp, orientation)
        wait_for_motion(step_delay)


# ===== 일반 객체 Pick-Place 함수 =====
def pick_object(pick_pos, grasp_z, grip_width, orientation):
    """
    일반 객체 집기 - 위에서 접근
    1. 물체 위로 이동 (높이)
    2. 아래로 하강
    3. 그리퍼로 잡기
    4. 위로 들어올리기
    """
    safe_z = 0.95  # 안전 높이 (충분히 높게)
    approach_z = grasp_z + 0.10  # 물체 바로 위

    # 1. 안전 높이에서 물체 위치로 이동
    print(f"  [1] Moving to safe height above object...")
    move_to_position([pick_pos[0], pick_pos[1], safe_z], orientation)
    wait_for_motion(MOTION_STEPS)

    # 2. 물체 바로 위로 하강
    print(f"  [2] Descending to approach height...")
    move_to_position([pick_pos[0], pick_pos[1], approach_z], orientation)
    wait_for_motion(MOTION_STEPS)

    # 3. 잡는 높이까지 천천히 하강
    print(f"  [3] Lowering to grasp height...")
    move_to_position([pick_pos[0], pick_pos[1], grasp_z], orientation)
    wait_for_motion(MOTION_STEPS)

    # 4. 그리퍼 닫기 (물체 잡기)
    print(f"  [4] Closing gripper...")
    close_gripper(grip_width)
    wait_for_motion(GRIPPER_STEPS)

    # 5. 물체를 들어올리기
    print(f"  [5] Lifting object...")
    move_to_position([pick_pos[0], pick_pos[1], safe_z], orientation)
    wait_for_motion(MOTION_STEPS)


def place_object(place_pos, place_z, orientation):
    """
    일반 객체 배치 - 위에서 접근
    1. 목표 위치 위로 이동 (높이)
    2. 아래로 하강
    3. 그리퍼 열기 (놓기)
    4. 위로 후퇴
    """
    safe_z = 0.95  # 안전 높이
    approach_z = place_z + 0.10  # 배치 위치 바로 위

    # 6. 안전 높이에서 목표 위치로 이동
    print(f"  [6] Moving to place position...")
    move_to_position([place_pos[0], place_pos[1], safe_z], orientation)
    wait_for_motion(MOTION_STEPS)

    # 7. 배치 위치 위로 하강
    print(f"  [7] Descending to approach height...")
    move_to_position([place_pos[0], place_pos[1], approach_z], orientation)
    wait_for_motion(MOTION_STEPS)

    # 8. 배치 높이까지 천천히 하강
    print(f"  [8] Lowering to place height...")
    move_to_position([place_pos[0], place_pos[1], place_z], orientation)
    wait_for_motion(MOTION_STEPS)

    # 9. 그리퍼 열기 (물체 놓기)
    print(f"  [9] Opening gripper (releasing object)...")
    open_gripper()
    wait_for_motion(GRIPPER_STEPS)

    # 10. 위로 후퇴
    print(f"  [10] Retreating upward...")
    move_to_position([place_pos[0], place_pos[1], safe_z], orientation)
    wait_for_motion(MOTION_STEPS)


# ===== 삼각기둥 전용 함수 (다단계 이동) =====
def pick_triangle(pick_pos, grasp_z=0.69, grip_width=0.010):
    """
    삼각기둥 전용 pick - 위에서 다단계 접근
    무거운 물체이므로 더 천천히, 더 조심스럽게
    """
    safe_z = 0.95  # 안전 높이
    approach_z = grasp_z + 0.10

    # 1. 안전 높이에서 물체 위로 이동
    print(f"  [1] Moving to safe height above triangle...")
    move_to_position([pick_pos[0], pick_pos[1], safe_z], TRIANGLE_ORN)
    wait_for_motion(MOTION_STEPS * 2)  # 더 느리게

    # 2. 접근 높이로 하강
    print(f"  [2] Descending to approach height...")
    move_to_position([pick_pos[0], pick_pos[1], approach_z], TRIANGLE_ORN)
    wait_for_motion(MOTION_STEPS * 2)

    # 3. 잡는 높이까지 천천히 하강
    print(f"  [3] Lowering to grasp height...")
    move_to_position([pick_pos[0], pick_pos[1], grasp_z], TRIANGLE_ORN)
    wait_for_motion(MOTION_STEPS * 2)

    # 4. 그리퍼 닫기 (더 세게)
    print(f"  [4] Closing gripper firmly...")
    close_gripper(grip_width)
    wait_for_motion(GRIPPER_STEPS * 2)

    # 5. 천천히 들어올리기
    print(f"  [5] Lifting triangle slowly...")
    move_to_position([pick_pos[0], pick_pos[1], safe_z], TRIANGLE_ORN)
    wait_for_motion(MOTION_STEPS * 2)


def place_triangle_with_waypoints(place_pos, place_z=0.70):
    """
    삼각기둥 전용 place - Cartesian path로 안전하게 이동
    MoveIt2의 computeCartesianPath와 유사한 방식 사용
    """
    safe_z = 0.95
    approach_z = place_z + 0.10

    # 6. Cartesian path로 목표 위치까지 이동 (pick 위치 → 케이스)
    # 삼각기둥 pick 위치: (0.5, 0.1)
    current_pos = [0.5, 0.1, safe_z]  # pick 후 현재 위치 (대략)

    # Waypoints 정의: 테이블 → 케이스 방향으로 안전하게 이동
    waypoints = [
        current_pos,
        [0.35, 0.15, safe_z],  # WP1: 테이블 위 중간
        [0.15, 0.20, safe_z],  # WP2: 케이스 방향
        [place_pos[0], place_pos[1], safe_z],  # WP3: 목표 위
    ]

    print(f"  [6] Moving via Cartesian path to place position...")
    # Cartesian path 계산 및 실행 (eef_step=0.01로 더 촘촘하게)
    interpolated = compute_cartesian_path(waypoints, eef_step=0.01)
    execute_cartesian_path(interpolated, TRIANGLE_ORN, step_delay=30)

    # 7. 접근 높이로 하강 (Cartesian path)
    print(f"  [7] Descending to approach height via Cartesian path...")
    descent_waypoints = [
        [place_pos[0], place_pos[1], safe_z],
        [place_pos[0], place_pos[1], approach_z],
    ]
    descent_interpolated = compute_cartesian_path(descent_waypoints, eef_step=0.005)
    execute_cartesian_path(descent_interpolated, TRIANGLE_ORN, step_delay=50)

    # 8. 배치 높이까지 천천히 하강 (Cartesian path)
    print(f"  [8] Lowering to place height via Cartesian path...")
    final_descent = [
        [place_pos[0], place_pos[1], approach_z],
        [place_pos[0], place_pos[1], place_z],
    ]
    final_interpolated = compute_cartesian_path(final_descent, eef_step=0.002)
    execute_cartesian_path(final_interpolated, TRIANGLE_ORN, step_delay=80)

    # 9. 그리퍼 열기
    print(f"  [9] Opening gripper...")
    open_gripper()
    wait_for_motion(GRIPPER_STEPS * 2)

    # 10. 위로 후퇴
    print(f"  [10] Retreating upward...")
    move_to_position([place_pos[0], place_pos[1], safe_z], TRIANGLE_ORN)
    wait_for_motion(MOTION_STEPS * 2)


# ===== 초기 자세 설정 =====
def set_home_position():
    """로봇을 home position으로 설정 (MoveIt2 스타일)"""
    for i in range(7):
        p.resetJointState(robot, i, HOME_JOINTS[i])


# ===== 메인 실행 =====
def main():
    # 초기화 - home position 설정 후 그리퍼 열기
    print("Initializing robot to home position...")
    set_home_position()
    open_gripper()
    wait_for_motion(MOTION_STEPS)

    # place_z 추정값
    place_z = 0.70

    # 8개 객체 처리
    for obj_num, name, pick_pos, place_pos, grasp_z, grip_width, use_triangle_orn in objects_data:
        print(f"\n=== Processing Object {obj_num}: {name} ===")
        print(f"Pick: {pick_pos} -> Place: {place_pos}")

        # orientation 선택
        orn = TRIANGLE_ORN if use_triangle_orn else DEFAULT_ORN

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


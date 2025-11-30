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

# ===== Home Position 설정 및 End-Effector Orientation =====
HOME_JOINTS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
for i in range(7):
    p.resetJointState(robot, i, HOME_JOINTS[i])

# Home position에서 end-effector orientation 획득 후 yaw -90도 회전
link_state = p.getLinkState(robot, 11)
home_orn = link_state[1]
yaw_rotation = p.getQuaternionFromEuler([0, 0, -math.pi/2])
GRIPPER_ORN_BASE = p.multiplyTransforms([0,0,0], home_orn, [0,0,0], yaw_rotation)[1]


def get_gripper_orientation(yaw_deg=0):
    """기본 orientation에서 yaw 각도(도)만큼 회전된 orientation 반환"""
    if yaw_deg == 0:
        return GRIPPER_ORN_BASE
    yaw_rad = math.radians(yaw_deg)
    rotation = p.getQuaternionFromEuler([0, 0, yaw_rad])
    return p.multiplyTransforms([0,0,0], GRIPPER_ORN_BASE, [0,0,0], rotation)[1]


# ===== 객체 데이터 정의 =====
# (물체#, 변수명, pick_pos, place_pos, pick_z, place_z, grip_width, pick_yaw, place_yaw)
# pick_yaw: 집을 때 그리퍼 회전 각도 (도), place_yaw: 놓을 때 그리퍼 회전 각도 (도)
objects_data = [
    (1, "box_4",      (0.3,  0.12),  (-0.15, 0.36), 0.68, 0.70, 0.015, 0, 0),
    (2, "box_5",      (0.3,  0.02),  (-0.16, 0.25), 0.66, 0.70, 0.015, 90, 0),
    (3, "box_6",      (0.3, -0.07),  ( 0.15, 0.255), 0.65, 0.70, 0.005, 90, 0),
    (4, "box_2",      (0.4,  0.1),  ( 0.05, 0.35), 0.68, 0.70, 0.015, 0, 0),
    (5, "cylinder_0", (0.4,  0.0),  (-0.048, 0.26), 0.68, 0.70, 0.015, 0, 0),
    (6, "box_3",      (0.4, -0.1),  (-0.062, 0.35), 0.68, 0.70, 0.015, 0, 0),
    (7, "triangle",   (0.5,  0.1),  ( 0.07, 0.245), 0.68, 0.70, 0.010, 0, -39),
    (8, "box_0",      (0.5,  0.0),  ( 0.16, 0.35), 0.69, 0.70, 0.015, 0, 0),
]

# ===== 상수 정의 =====
APPROACH_OFFSET = 0.15  # 접근 오프셋 (10cm 위에서 접근)
SIM_SPEED = 1/480
MOTION_STEPS = 20
GRIPPER_STEPS = 100
PAUSE_TIME = 0.5  # 각 동작 사이 정지 시간 (초)


# ===== 헬퍼 함수 =====
def pause():
    """동작 사이 정지"""
    time.sleep(PAUSE_TIME)


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


def move_vertical(x, y, z_start, z_end, orientation, step_delay=30):
    """
    수직 이동 (Z축만 변경) - Cartesian path 사용

    Args:
        x, y: 고정된 X, Y 좌표
        z_start: 시작 Z 높이
        z_end: 목표 Z 높이
        orientation: 그리퍼 orientation
        step_delay: 각 waypoint 간 대기 스텝
    """
    waypoints = [
        [x, y, z_start],
        [x, y, z_end],
    ]
    interpolated = compute_cartesian_path(waypoints, eef_step=0.005)
    execute_cartesian_path(interpolated, orientation, step_delay)


# ===== Pick-Place 함수 =====
def pick_and_place(pick_pos, place_pos, pick_z, place_z, grip_width, pick_yaw=0, place_yaw=0):
    """
    물체 집어서 놓기 - 명확한 단계별 동작

    동작 순서:
    1. Home에서 시작, 그리퍼 열기
    2. pick 위치의 10cm 위로 이동
    3. pick 위치로 하강 (Z만 변경)
    4. 그리퍼 닫기 (물체 잡기)
    5. 10cm 위로 들어올리기
    6. waypoint를 따라 place 위치의 10cm 위로 이동
    7. place 위치로 하강 (Z만 변경)
    8. 그리퍼 열기 (물체 놓기)
    9. Home으로 복귀

    Args:
        pick_yaw: 집을 때 그리퍼 회전 각도 (도 단위)
        place_yaw: 놓을 때 그리퍼 회전 각도 (도 단위)
    """
    pick_approach_z = pick_z + APPROACH_OFFSET   # pick 위치 10cm 위
    place_approach_z = place_z + APPROACH_OFFSET  # place 위치 10cm 위

    # orientation 계산 (pick/place 별도)
    pick_orn = get_gripper_orientation(pick_yaw)
    place_orn = get_gripper_orientation(place_yaw)

    # === PICK 단계 ===

    # 1. 그리퍼 열기
    print(f"  [1] Opening gripper...")
    open_gripper()
    wait_for_motion(GRIPPER_STEPS)

    # 2. pick 위치의 10cm 위로 이동
    print(f"  [2] Moving to pick approach (10cm above)...")
    move_to_position([pick_pos[0], pick_pos[1], pick_approach_z], pick_orn)
    wait_for_motion(MOTION_STEPS)

    # 3. pick 위치로 수직 하강
    print(f"  [3] Descending vertically to pick position...")
    move_vertical(pick_pos[0], pick_pos[1], pick_approach_z, pick_z, pick_orn)

    # 4. 그리퍼 닫기 (물체 잡기)
    print(f"  [4] Closing gripper (grasping)...")
    close_gripper(grip_width)
    wait_for_motion(GRIPPER_STEPS)

    # 5. 수직으로 10cm 위로 들어올리기
    print(f"  [5] Lifting object vertically (10cm up)...")
    move_vertical(pick_pos[0], pick_pos[1], pick_z, pick_approach_z, pick_orn)

    # === PLACE 단계 ===

    # 6. waypoint를 따라 place 위치의 10cm 위로 이동
    print(f"  [6] Moving to place approach via waypoint...")
    mid_x = (pick_pos[0] + place_pos[0]) / 2
    mid_y = (pick_pos[1] + place_pos[1]) / 2

    waypoints = [
        [pick_pos[0], pick_pos[1], pick_approach_z],    # 현재 위치
        [mid_x, mid_y, pick_approach_z],                 # 중간 waypoint
        [place_pos[0], place_pos[1], place_approach_z],  # place 위치 10cm 위
    ]
    interpolated = compute_cartesian_path(waypoints, eef_step=0.01)
    execute_cartesian_path(interpolated, place_orn, step_delay=10)

    # 7. place 위치로 수직 하강
    print(f"  [7] Descending vertically to place position...")
    move_vertical(place_pos[0], place_pos[1], place_approach_z, place_z, place_orn)

    # 8. 그리퍼 열기 (물체 놓기)
    print(f"  [8] Opening gripper (releasing)...")
    open_gripper()
    wait_for_motion(GRIPPER_STEPS)
    
    print(f"  [9] Descending vertically to place position...")
    move_vertical(place_pos[0], place_pos[1], place_z, place_approach_z, place_orn)

    # 9. Home으로 복귀
    go_to_home_position()


# ===== 초기 자세 설정 =====
def set_home_position():
    """로봇을 home position으로 즉시 설정 (resetJointState 사용)"""
    for i in range(7):
        p.resetJointState(robot, i, HOME_JOINTS[i])


def go_to_home_position():
    """로봇을 home position으로 부드럽게 이동 (position control 사용)"""
    print(f"  [Home] Returning to home position...")
    for i in range(7):
        p.setJointMotorControl2(
            robot,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=HOME_JOINTS[i],
            force=240,
            maxVelocity=1
        )
    wait_for_motion(MOTION_STEPS)


# ===== 메인 실행 =====
def main():
    # 초기화 - home position 설정
    print("Initializing robot to home position...")
    set_home_position()
    open_gripper()
    wait_for_motion(MOTION_STEPS)

    # 8개 객체 처리
    for obj_num, name, pick_pos, place_pos, pick_z, place_z, grip_width, pick_yaw, place_yaw in objects_data:
        #if obj_num == 1 or obj_num==2: continue
        print(f"\n=== Processing Object {obj_num}: {name} ===")
        print(f"Pick: {pick_pos}, z={pick_z}, yaw={pick_yaw}°")
        print(f"Place: {place_pos}, z={place_z}, yaw={place_yaw}°")

        # Home → Pick → Place → Home 사이클
        pick_and_place(pick_pos, place_pos, pick_z, place_z, grip_width, pick_yaw, place_yaw)

        print(f"Object {obj_num} ({name}) completed!")

    print("\n=== All objects placed! ===")


# 실행
main()

# 시뮬레이션 유지
for _ in range(1000):
    p.stepSimulation()
    time.sleep(1/60)

p.disconnect()


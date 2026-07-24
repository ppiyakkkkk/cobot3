#!/usr/bin/env python3
"""산림 구조 시뮬레이션에서 공통으로 사용하는 설정값 모음.

이 파일은 ``SimulationApp`` 생성 전에도 안전하게 import할 수 있도록
표준 라이브러리 외의 Isaac Sim/Pegasus 모듈을 import하지 않는다.
경로, 드론 배치, 카메라, 사람 충돌체, 수색 경로 관련 값을 바꾸려면
우선 이 파일을 수정하면 된다.
"""

from pathlib import Path


# ---------------------------------------------------------------------------
# 카메라 및 센서 설정
# ---------------------------------------------------------------------------
CAMERA_FOCAL_LENGTH_MM = 10.0
# 드론 body에 고정된 RGB/Depth 카메라의 하향각이다.
# 0도는 기체 정면, 90도는 수직으로 지면을 바라본다.
# 산림 수색에서 전방과 지면을 함께 담도록 40도로 설정한다.
CAMERA_DOWN_TILT_DEG = 40.0
# 작은 원거리 사람의 픽셀 크기를 확보하기 위해 4:3 비율을 유지하며 상향한다.
CAMERA_RESOLUTION = [960, 720]

# 왼쪽 메인 Viewport가 따라갈 드론과 3인칭 추적 카메라 설정이다.
# 추적 대상을 바꾸려면 quadrotor_01을 quadrotor_02 또는 03으로 변경한다.
FOLLOW_DRONE_PRIM_PATH = "/World/quadrotor_01/body"
FOLLOW_CAMERA_PRIM_PATH = "/World/FollowCamera"

# 드론의 실제 진행방향을 기준으로 카메라를 뒤쪽·위쪽에 배치한다.
# 카메라는 드론보다 앞쪽 지점을 바라보므로 비행 진행방향이 화면에 보인다.
FOLLOW_CAMERA_BACK_DISTANCE_M = 12.0
FOLLOW_CAMERA_HEIGHT_M = 7.0
FOLLOW_CAMERA_LOOK_AHEAD_M = 10.0
# 드론 높이를 기준으로 추적 카메라가 바라볼 목표점의 상대 Z이다.
# 음수 절댓값이 커질수록 왼쪽 Viewport가 지면을 더 내려다본다.
FOLLOW_CAMERA_TARGET_HEIGHT_M = -4.0

# 위치 변화가 이 값보다 클 때만 실제 이동방향을 새로 계산한다.
# 정지 중에는 드론 body의 전방축을 사용한다.
FOLLOW_CAMERA_MIN_MOVEMENT_M = 0.01

# 방향 변화가 너무 급하게 화면에 반영되지 않도록 보간한다.
# 0에 가까울수록 부드럽고, 1에 가까울수록 즉시 방향이 바뀐다.
FOLLOW_CAMERA_DIRECTION_SMOOTHING = 0.15


# ---------------------------------------------------------------------------
# 드론 및 사람 배치 설정
# ---------------------------------------------------------------------------
MIN_DRONE_COUNT = 1
MAX_DRONE_COUNT = 4
DEFAULT_DRONE_COUNT = 3

# 실행 모드는 Isaac Sim과 ROS 2 Launch에서 동일하게 지정한다.
SUPPORTED_OPERATION_MODES = (
    "rescue_search",
    "mapping_3d",
)
DEFAULT_OPERATION_MODE = "rescue_search"
OPERATION_MODE = DEFAULT_OPERATION_MODE

# 1~3대 설정은 V1의 위치와 vehicle_id를 그대로 유지한다.
# 4번 드론은 기존 기체와 5 m 간격을 유지하면서 Terrain 안쪽에 배치한다.
_AVAILABLE_DRONE_CONFIGS = [
    ("/World/quadrotor_01", 0, [-34.0, 40.0, 31.0]),
    ("/World/quadrotor_02", 1, [-29.0, 40.0, 31.0]),
    ("/World/quadrotor_03", 2, [-39.0, 40.0, 31.0]),
    ("/World/quadrotor_04", 3, [-34.0, 45.0, 31.0]),
]

DRONE_COUNT = DEFAULT_DRONE_COUNT
DRONE_CONFIGS = []
DRONE_IDS = []
CAMERA_PRIM_PATHS = []


def configure_drone_count(drone_count=DEFAULT_DRONE_COUNT):
    """1~4 범위의 실행 드론 수를 공통 설정에 반영한다.

    ``final_24.py``가 다른 Isaac Sim 역할 모듈을 import하기 전에 이 함수를
    호출해야 한다. 인자를 생략하면 기본값 3대를 사용한다.
    """
    global DRONE_COUNT, DRONE_CONFIGS, DRONE_IDS, CAMERA_PRIM_PATHS

    try:
        count = int(drone_count)
    except (TypeError, ValueError) as error:
        raise RuntimeError(
            f"drone_count는 정수여야 합니다: {drone_count!r}"
        ) from error

    if not MIN_DRONE_COUNT <= count <= MAX_DRONE_COUNT:
        raise RuntimeError(
            f"drone_count는 {MIN_DRONE_COUNT}~{MAX_DRONE_COUNT} "
            f"범위여야 합니다: {count}"
        )
    if len(_AVAILABLE_DRONE_CONFIGS) < count:
        raise RuntimeError(
            f"요청한 {count}대에 필요한 드론 설정이 부족합니다: "
            f"available={len(_AVAILABLE_DRONE_CONFIGS)}"
        )

    selected = list(_AVAILABLE_DRONE_CONFIGS[:count])
    prim_paths = [item[0] for item in selected]
    vehicle_ids = [int(item[1]) for item in selected]
    if len(set(prim_paths)) != count:
        raise RuntimeError(f"드론 prim_path가 중복됩니다: {prim_paths}")
    if len(set(vehicle_ids)) != count:
        raise RuntimeError(f"PX4 vehicle_id가 중복됩니다: {vehicle_ids}")
    for prim_path, vehicle_id, position in selected:
        if len(position) != 3:
            raise RuntimeError(
                f"{prim_path} 스폰 좌표는 XYZ 3개여야 합니다: {position}"
            )
        if vehicle_id < 0:
            raise RuntimeError(
                f"{prim_path} vehicle_id는 0 이상이어야 합니다: {vehicle_id}"
            )

    DRONE_COUNT = count
    DRONE_CONFIGS = selected
    DRONE_IDS = [
        prim_path.rsplit("/", 1)[-1]
        for prim_path, _, _ in DRONE_CONFIGS
    ]
    CAMERA_PRIM_PATHS = [
        f"{prim_path}/body/Camera"
        for prim_path, _, _ in DRONE_CONFIGS
    ]
    return DRONE_COUNT


def configure_operation_mode(operation_mode=DEFAULT_OPERATION_MODE):
    """운용 모드를 공통 설정에 반영한다."""
    global OPERATION_MODE

    mode = str(operation_mode).strip().lower()
    if mode not in SUPPORTED_OPERATION_MODES:
        supported = ", ".join(SUPPORTED_OPERATION_MODES)
        raise RuntimeError(
            f"operation_mode는 다음 중 하나여야 합니다: {supported}. "
            f"입력값={operation_mode!r}"
        )

    OPERATION_MODE = mode
    return OPERATION_MODE


def should_spawn_people():
    """구조 수색 모드에서만 조난자와 구조자를 생성한다."""
    return OPERATION_MODE == "rescue_search"


# 다른 모듈이 sim_config만 직접 import해도 기본 설정이 준비되게 한다.
configure_drone_count(DEFAULT_DRONE_COUNT)
configure_operation_mode(DEFAULT_OPERATION_MODE)

# 일반 실행에서는 아래 후보 좌표 중 한 곳에 조난자를 생성한다.
VICTIM_SPAWN_POSITIONS = [
    [33.0, 29.0, 13.7],
    # [-0.9, -1.8, -0.9],
    # [33.0, -22.0, 50.6],
]

# 착륙 복귀 시험용 조난자 위치다.
# World ENU 기준 (X, Y, Z)를 한 줄에서 직접 지정한다.
FOR_TEST_VICTIM_SPAWN_ENABLED = True
FOR_TEST_VICTIM_WORLD_XYZ = (2.0, 38.0, 20.0)

# True이면 X·Y만 그대로 사용하고 Z는 실제 Terrain 표면으로 자동 보정한다.
# 사람이 경사면 위에서 뜨거나 묻히지 않게 하는 기본 시험 모드다.
FOR_TEST_VICTIM_KEEP_ON_GROUND = True

# 구조자 충돌체도 초기 LiDAR 팽창영역에 들어오지 않도록 6 m 떨어뜨린다.
# 구조자의 발 높이는 첫 번째 드론의 초기 World Z와 동일하게 맞춘다.
RESCUER_XY = (-34.0, 34.0)
RESCUER_FOOT_Z = float(_AVAILABLE_DRONE_CONFIGS[0][2][2])

# 지형 보간 오차로 발이 지면에 묻히지 않도록 아주 조금 띄운다.
PERSON_GROUND_CLEARANCE_M = 0.08

# 정지한 사람을 물리 장애물로 취급하기 위한 캡슐 충돌체 크기다.
# Capsule의 전체 높이 = cylinder height + 2 * radius = 1.8 m이다.
PERSON_COLLIDER_RADIUS_M = 0.30
PERSON_COLLIDER_CYLINDER_HEIGHT_M = 1.20
PERSON_COLLIDER_TOTAL_HEIGHT_M = (
    PERSON_COLLIDER_CYLINDER_HEIGHT_M
    + 2.0 * PERSON_COLLIDER_RADIUS_M
)


# ---------------------------------------------------------------------------
# 수색 경로 설정
# ---------------------------------------------------------------------------
SEARCH_AREA_MARGIN_M = 6.0
SEARCH_LANE_SPACING_M = 7.0
# 강가·급경사 구간에서 고도 변화가 한 번에 커지지 않도록 수색점 간격을
# 기존 7m보다 촘촘하게 둔다.
SEARCH_SAMPLE_SPACING_M = 5.0
SEARCH_CLEARANCE_M = 6.0

# 급격한 하천 사면과 다리 진입부의 높이 변화를 놓치지 않도록 선분을
# 0.5m 간격으로 검사한다.
SEARCH_TERRAIN_PROFILE_SPACING_M = 0.5

# 연속 Waypoint 사이의 목표 고도 변화량을 제한한다. 상승이 더 필요하면
# 이전 Waypoint들을 미리 높여 완만하게 준비하고, 하강은 다음 지점들에
# 걸쳐 단계적으로 수행한다.
SEARCH_MAX_CLIMB_PER_WAYPOINT_M = 2.5
SEARCH_MAX_DESCENT_PER_WAYPOINT_M = 2.0

# 협동 수색 진입 경로는 전역 최고고도에서 수직 상승 후 이동하지 않고,
# 현재 위치부터 첫 소구역까지 지형을 따라 이동하면서 고도를 바꾼다.
COOPERATIVE_TRANSIT_PROFILE_SPACING_M = 3.0
COOPERATIVE_MAX_CLIMB_PER_WAYPOINT_M = 2.5
COOPERATIVE_MAX_DESCENT_PER_WAYPOINT_M = 2.0

# Terrain과 분리된 다리 구조물도 사전 경로 높이에 포함하기 위한 이름
# 후보들이다. 실제 USD Prim 경로에 아래 문자열이 들어가면 구조물 상단을
# navigation surface로 취급한다.
NAVIGATION_STRUCTURE_ALIASES = (
    "bridge",
    "footbridge",
    "woodbridge",
    "woodenbridge",
    "crossing",
    "deck",
)
NAVIGATION_STRUCTURE_XY_MARGIN_M = 1.5

# 복귀 고도는 더 이상 지도 전체 최고점으로 고정하지 않는다. ROS 2
# 컨트롤러가 RETURN_HOME 수신 시점의 실제 위치부터 홈까지 지형만 검사하고,
# 그 구간의 최고 지형보다 아래 여유 높이만큼 높은 고도를 선택한다.
RETURN_PATH_CLEARANCE_M = 8.0
RETURN_PATH_SAMPLE_SPACING_M = 1.0
RETURN_PATH_CORRIDOR_RADIUS_M = 2.0
RETURN_OBSTACLE_CLEARANCE_M = 3.0


# ---------------------------------------------------------------------------
# 입력 USD와 자동 생성 파일 경로
# ---------------------------------------------------------------------------
SCRIPT_DIR = Path(__file__).resolve().parent
FOREST_WORLD_PATH = SCRIPT_DIR / "worlds" / "my_forest.usdc"
GENERATED_SEARCH_PLAN_PATH = SCRIPT_DIR / "generated_search_plan.json"
GENERATED_GROUND_TRUTH_PATH = SCRIPT_DIR / "generated_ground_truth.json"
GENERATED_TERRAIN_MESH_PATH = SCRIPT_DIR / "generated_terrain_mesh.npz"
# Terrain과 이름으로 확인된 다리 구조물 상단을 합친 경로계획 전용 표면이다.
GENERATED_NAVIGATION_SURFACE_PATH = (
    SCRIPT_DIR / "generated_navigation_surface.npz"
)
GENERATED_ENVIRONMENT_MESH_PATH = (
    SCRIPT_DIR / "generated_environment_meshes.npz"
)

# RViz 지형은 실제 USD Terrain 높이를 이 간격으로 샘플링한다.
# 값이 작을수록 산이 부드럽지만 Marker 메시지가 커진다.
RVIZ_TERRAIN_SAMPLE_SPACING_M = 1.5

# RViz에 실제 형상으로 내보낼 USD 그룹이다.
# Stage 경로의 어느 조상 Prim 이름이라도 아래 이름과 일치하면 분류한다.
RVIZ_ENVIRONMENT_GROUPS = {
    "pineforest": ("pineforest",),
    "broadleafforest": ("broadleafforest",),
    "bushes": ("bushes",),
    "rocks": ("rocks",),
    # Wooden_bridge1, Wooden_bridge2처럼 부모 Prim 이름에 아래 문자열이
    # 들어가면 실제 다리 Mesh를 별도 그룹으로 추출한다.
    "bridges": (
        "woodenbridge",
        "woodbridge",
        "footbridge",
        "bridge",
        "crossing",
        "deck",
    ),
    # 강 Prim이나 Material 이름이 아래 별칭 중 하나를 포함하면 강으로
    # 분류한다. 이름이 일반적인 Mesh/Plane인 경우에는 sim_terrain.py가
    # 파란 재질과 넓고 평평한 형상을 함께 검사해 보조 분류한다.
    "river": (
        "river",
        "water",
        "stream",
        "creek",
        "brook",
        "canal",
        "channel",
        "waterway",
        "watersurface",
        "riversurface",
        "lake",
        "pond",
    ),
}

# 강 Prim 이름이 전혀 드러나지 않는 USD를 위한 명시적 Prim 경로다.
# Stage에서 강을 선택해 확인한 경로를 여기에 추가하면 최우선으로 분류된다.
RVIZ_RIVER_EXPLICIT_PRIM_PATHS = ()

# 파란색 재질을 사용하는 넓고 평평한 Mesh를 강으로 자동 분류한다.
# 나무·바위의 파란 소품이 잘못 분류되지 않도록 색상과 형상을 함께 검사한다.
RVIZ_RIVER_AUTO_COLOR_CLASSIFICATION = True
RVIZ_RIVER_AUTO_MIN_HORIZONTAL_SPAN_M = 3.0
RVIZ_RIVER_AUTO_MAX_VERTICAL_THICKNESS_M = 1.5
RVIZ_RIVER_AUTO_MAX_THICKNESS_RATIO = 0.15
RVIZ_RIVER_AUTO_MIN_BLUE = 0.30
RVIZ_RIVER_AUTO_MIN_BLUE_MINUS_RED = 0.12
RVIZ_RIVER_AUTO_MIN_COLOR_RANGE = 0.15

# TRIANGLE_LIST 메시지가 지나치게 커지는 것을 방지하는 그룹별 상한이다.
# 원본 삼각형 수가 이 값을 넘을 때만 균일하게 일부 면을 선택한다.
RVIZ_ENVIRONMENT_MAX_TRIANGLES_PER_GROUP = 120_000

#!/usr/bin/env python3
"""수색 경로, Ground Truth JSON, 장면 조명·검증 관련 공통 로직."""

import json
import math
from pathlib import Path

import carb
import numpy as np
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdLux

from sim_config import (
    DRONE_CONFIGS,
    FOR_TEST_VICTIM_KEEP_ON_GROUND,
    FOR_TEST_VICTIM_SPAWN_ENABLED,
    FOR_TEST_VICTIM_WORLD_XYZ,
    FOREST_WORLD_PATH,
    GENERATED_GROUND_TRUTH_PATH,
    GENERATED_SEARCH_PLAN_PATH,
    PERSON_GROUND_CLEARANCE_M,
    RETURN_PATH_CLEARANCE_M,
    RETURN_PATH_CORRIDOR_RADIUS_M,
    RETURN_PATH_SAMPLE_SPACING_M,
    RETURN_OBSTACLE_CLEARANCE_M,
    SEARCH_AREA_MARGIN_M,
    SEARCH_CLEARANCE_M,
    SEARCH_LANE_SPACING_M,
    SEARCH_SAMPLE_SPACING_M,
    SEARCH_TERRAIN_PROFILE_SPACING_M,
)


class SearchPlanGenerator:
    """Terrain 높이를 반영해 3대 드론의 수색 경로 JSON을 생성한다."""

    def __init__(self, terrain):
        self.terrain = terrain
        self.test_victim_spawn_world_enu = None

    @staticmethod
    def _sample_axis(start, stop, spacing):
        """양 끝점을 포함하도록 일정 간격의 좌표 목록을 만든다.

        인스턴스 상태를 사용하지 않는 계산 함수이므로 ``@staticmethod``로
        선언한다. 이 데코레이터가 없으면 ``self``가 자동 전달되어 인자 수
        불일치(TypeError)가 발생한다.
        """
        distance = abs(float(stop) - float(start))
        count = max(2, int(math.ceil(distance / float(spacing))) + 1)
        return [
            float(value)
            for value in np.linspace(float(start), float(stop), count)
        ]

    def write_generated_search_plan(self):
        """Terrain 높이를 반영한 드론 3대의 3차원 지그재그 경로를 저장한다."""
        x_low = self.terrain.x_min + SEARCH_AREA_MARGIN_M
        x_high = self.terrain.x_max - SEARCH_AREA_MARGIN_M
        y_low = self.terrain.y_min + SEARCH_AREA_MARGIN_M
        y_high = self.terrain.y_max - SEARCH_AREA_MARGIN_M

        if x_low >= x_high or y_low >= y_high:
            raise RuntimeError("수색 경로를 만들 Terrain 영역이 부족합니다.")

        # 그림과 같이 위·가운데·아래의 가로 구역 3개로 균등 분할한다.
        y_edges = np.linspace(y_low, y_high, 4)
        plan = {
            "format_version": 3,
            "map_frame": "map",
            "coordinate_convention": "world_enu_and_local_ned",
            "terrain_bounds": {
                "x_min": self.terrain.x_min,
                "x_max": self.terrain.x_max,
                "y_min": self.terrain.y_min,
                "y_max": self.terrain.y_max,
                "z_min": self.terrain.z_min,
                "z_max": self.terrain.z_max,
            },
            "search_clearance_m": SEARCH_CLEARANCE_M,
            # 실제 복귀 목표 고도는 드론의 현재 위치가 정해지는
            # RETURN_HOME 시점에 ROS 2 컨트롤러가 계산한다.
            "return_height_mode": "current_to_home_terrain_profile",
            "return_path_clearance_m": RETURN_PATH_CLEARANCE_M,
            "return_path_sample_spacing_m": RETURN_PATH_SAMPLE_SPACING_M,
            "return_path_corridor_radius_m": RETURN_PATH_CORRIDOR_RADIUS_M,
            "return_obstacle_clearance_m": RETURN_OBSTACLE_CLEARANCE_M,
            "test_victim_spawn": None,
            "drones": {},
        }

        # 시험에서는 지정한 World ENU XYZ를 사용한다.
        if FOR_TEST_VICTIM_SPAWN_ENABLED:
            if len(FOR_TEST_VICTIM_WORLD_XYZ) != 3:
                raise RuntimeError(
                    "FOR_TEST_VICTIM_WORLD_XYZ는 (X, Y, Z) 3개 값이어야 합니다."
                )

            test_x = float(FOR_TEST_VICTIM_WORLD_XYZ[0])
            test_y = float(FOR_TEST_VICTIM_WORLD_XYZ[1])
            configured_z = float(FOR_TEST_VICTIM_WORLD_XYZ[2])

            if not (
                self.terrain.x_min <= test_x <= self.terrain.x_max
                and self.terrain.y_min <= test_y <= self.terrain.y_max
            ):
                raise RuntimeError(
                    "시험용 조난자 XY가 Terrain 범위를 벗어났습니다: "
                    f"XY=({test_x:.2f}, {test_y:.2f})"
                )

            terrain_z = self.terrain.height(test_x, test_y)
            ground_spawn_z = (
                terrain_z + PERSON_GROUND_CLEARANCE_M
            )

            if FOR_TEST_VICTIM_KEEP_ON_GROUND:
                test_z = float(ground_spawn_z)
                z_mode = "terrain_snap"
            else:
                test_z = configured_z
                z_mode = "exact_xyz"

                height_error = test_z - ground_spawn_z
                if abs(height_error) > 0.30:
                    carb.log_warn(
                        "시험 조난자 Z가 지면과 크게 다릅니다: "
                        f"configured_Z={test_z:.2f}, "
                        f"ground_spawn_Z={ground_spawn_z:.2f}, "
                        f"difference={height_error:.2f}m"
                    )

            self.test_victim_spawn_world_enu = [
                test_x,
                test_y,
                test_z,
            ]
            plan["test_victim_spawn"] = {
                "enabled": True,
                "selection": "hardcoded_xyz",
                "z_mode": z_mode,
                "configured_world_enu": [
                    test_x,
                    test_y,
                    configured_z,
                ],
                "terrain_z": float(terrain_z),
                "world_enu": list(
                    self.test_victim_spawn_world_enu
                ),
            }
            print(
                "[TEST] 조난자 시험 XYZ 설정: "
                f"configured=({test_x:.2f}, {test_y:.2f}, "
                f"{configured_z:.2f}), "
                f"terrain_Z={terrain_z:.2f}, "
                f"actual_spawn=({test_x:.2f}, {test_y:.2f}, "
                f"{test_z:.2f}), mode={z_mode}"
            )

        # drone_01은 상단, 02는 중앙, 03은 하단 구역을 맡는다.
        zone_indices = (2, 1, 0)
        for config, zone_index in zip(DRONE_CONFIGS, zone_indices):
            prim_path, vehicle_id, home = config
            drone_name = prim_path.rsplit("/", 1)[-1]
            home_x, home_y, home_z = [float(value) for value in home]
            zone_min_y = float(y_edges[zone_index])
            zone_max_y = float(y_edges[zone_index + 1])

            # 시작점이 담당 구역 안에 있으면 현재 Y에서 바로 수색을
            # 시작한다. 특히 quadrotor_01은 zone_max_y로 0.67m 이동했다가
            # 다시 꺾는 불필요한 초기 동작 없이 곧바로 왼쪽으로 진입한다.
            entry_y = min(max(home_y, zone_min_y), zone_max_y)
            rows = self._sample_axis(
                entry_y,
                zone_min_y,
                SEARCH_LANE_SPACING_M,
            )

            route_xy = []

            def append_route_point(world_x, world_y):
                """연속 중복을 제거해 수색 경로의 XY 뼈대를 만든다."""
                point = (float(world_x), float(world_y))
                if route_xy:
                    previous = route_xy[-1]
                    if (
                        abs(previous[0] - point[0]) < 1.0e-6
                        and abs(previous[1] - point[1]) < 1.0e-6
                    ):
                        return
                route_xy.append(point)

            # 출발점도 경로에 넣어 첫 이동 구간의 지형 최고점을 검사한다.
            append_route_point(home_x, home_y)

            # 각 드론은 자기 X 통로를 유지한 채 담당 구역의 첫 행으로
            # 이동한다. 그 뒤 첫 행의 양 끝 중 현재 위치에서 가까운 쪽으로
            # 진입해 불필요하게 반대쪽으로 갔다가 되돌아오는 동작을 줄인다.
            for world_y in self._sample_axis(
                home_y,
                entry_y,
                SEARCH_SAMPLE_SPACING_M,
            )[1:]:
                append_route_point(home_x, world_y)

            left_distance = abs(home_x - x_low)
            right_distance = abs(home_x - x_high)
            first_row_starts_left = left_distance <= right_distance
            first_entry_x = x_low if first_row_starts_left else x_high
            for world_x in self._sample_axis(
                home_x,
                first_entry_x,
                SEARCH_SAMPLE_SPACING_M,
            )[1:]:
                append_route_point(world_x, entry_y)

            for row_index, world_y in enumerate(rows):
                starts_left = (
                    first_row_starts_left
                    if row_index % 2 == 0
                    else not first_row_starts_left
                )
                if starts_left:
                    row_x = self._sample_axis(
                        x_low,
                        x_high,
                        SEARCH_SAMPLE_SPACING_M,
                    )
                else:
                    row_x = self._sample_axis(
                        x_high,
                        x_low,
                        SEARCH_SAMPLE_SPACING_M,
                    )

                for world_x in row_x:
                    append_route_point(world_x, world_y)

            # 각 선분의 최고 지형고도를 구하고 양 끝 Waypoint를 모두 그보다
            # SEARCH_CLEARANCE_M만큼 높인다. 따라서 PX4가 두 점 사이를 직선
            # 보간해도 선분 중간의 산봉우리와 충돌하지 않는다.
            point_ground_z = [
                self.terrain.height(world_x, world_y)
                for world_x, world_y in route_xy
            ]
            segment_safe_z = []
            for start_xy, end_xy in zip(route_xy, route_xy[1:]):
                distance = math.hypot(
                    end_xy[0] - start_xy[0],
                    end_xy[1] - start_xy[1],
                )
                sample_count = max(
                    2,
                    int(math.ceil(
                        distance / SEARCH_TERRAIN_PROFILE_SPACING_M
                    )) + 1,
                )
                terrain_max = max(
                    self.terrain.height(
                        start_xy[0]
                        + (end_xy[0] - start_xy[0]) * ratio,
                        start_xy[1]
                        + (end_xy[1] - start_xy[1]) * ratio,
                    )
                    for ratio in np.linspace(0.0, 1.0, sample_count)
                )
                segment_safe_z.append(terrain_max + SEARCH_CLEARANCE_M)

            waypoints = []
            for point_index, ((world_x, world_y), ground_z) in enumerate(
                zip(route_xy, point_ground_z)
            ):
                required_z = ground_z + SEARCH_CLEARANCE_M
                if point_index > 0:
                    required_z = max(
                        required_z,
                        segment_safe_z[point_index - 1],
                    )
                if point_index < len(segment_safe_z):
                    required_z = max(
                        required_z,
                        segment_safe_z[point_index],
                    )
                waypoints.append(
                    {
                        "north_m": world_y - home_y,
                        "east_m": world_x - home_x,
                        "down_m": -(required_z - home_z),
                        "world_enu": [world_x, world_y, required_z],
                    }
                )

            plan["drones"][drone_name] = {
                "vehicle_id": int(vehicle_id),
                "home_world_enu": [home_x, home_y, home_z],
                "zone_y_min": zone_min_y,
                "zone_y_max": zone_max_y,
                "waypoints": waypoints,
            }

        GENERATED_SEARCH_PLAN_PATH.write_text(
            json.dumps(plan, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        print(
            "[INFO] Terrain 기반 3드론 수색 경로 저장: "
            f"{GENERATED_SEARCH_PLAN_PATH}"
        )
        for drone_name, drone_plan in plan["drones"].items():
            print(
                f"[INFO] {drone_name} zone: "
                f"Y=({drone_plan['zone_y_min']:.2f}, "
                f"{drone_plan['zone_y_max']:.2f}), "
                f"waypoints={len(drone_plan['waypoints'])}"
            )


class SceneEnvironmentManager:
    """USD 환경 검증, 조명 설정, 초기 Viewport 배치를 담당한다."""

    def __init__(self, pegasus_interface):
        self.pg = pegasus_interface

    def configure_sky_background(self):
        """RTX 배경을 하늘색으로 강제하고 Dome Light로 장면을 밝힌다."""
        stage = omni.usd.get_context().get_stage()

        # Dome Light는 검은색 드론과 지형을 밝히는 장면 조명으로 사용한다.
        dome_light = UsdLux.DomeLight.Define(
            stage,
            "/World/SkyDome",
        )
        dome_light.CreateColorAttr().Set(
            Gf.Vec3f(0.45, 0.70, 1.0)
        )
        dome_light.CreateIntensityAttr().Set(400.0)
        dome_light.CreateExposureAttr().Set(0.0)

        # Dome Light 색만 지정하면 RTX Real-Time에서 배경이 검정으로
        # 남을 수 있으므로, 렌더러의 배경 소스를 Color로 강제한다.
        settings = carb.settings.get_settings()
        settings.set_int(
            "/rtx/background/source/type",
            2,
        )
        settings.set_float_array(
            "/rtx/background/source/color",
            [0.45, 0.70, 1.0],
        )

        print(
            "[INFO] RTX sky background configured: "
            "source=color, color=(0.45, 0.70, 1.0), "
            "dome_intensity=400"
        )

    def verify_loaded_environment(self):
        """요청한 USD가 실제 Root Layer로 열렸는지 확인한다."""
        stage = omni.usd.get_context().get_stage()
        root_layer = stage.GetRootLayer()
        root_path = root_layer.realPath or root_layer.identifier

        print(f"[CHECK] Stage root layer: {root_path}")

        if root_layer.realPath:
            loaded_path = Path(root_layer.realPath).resolve()
            expected_path = FOREST_WORLD_PATH.resolve()
            if loaded_path != expected_path:
                raise RuntimeError(
                    "요청한 산악 환경과 실제로 열린 Stage가 다릅니다.\n"
                    f"Expected: {expected_path}\n"
                    f"Loaded: {loaded_path}"
                )

        default_prim = stage.GetDefaultPrim()
        if default_prim and default_prim.IsValid():
            print(f"[CHECK] Default Prim: {default_prim.GetPath()}")
        else:
            carb.log_warn(
                "산악 환경 USD에 Default Prim이 없어 /World 또는 첫 번째 "
                "상위 Prim을 환경 루트로 사용합니다."
            )

        top_level_prims = [
            str(prim.GetPath())
            for prim in stage.GetPseudoRoot().GetChildren()
        ]
        print(f"[CHECK] Top-level Prims: {top_level_prims}")

    @staticmethod
    def _get_environment_root_prim():
        """환경 Bounding Box 계산에 사용할 유효한 루트 Prim을 찾는다."""
        stage = omni.usd.get_context().get_stage()

        default_prim = stage.GetDefaultPrim()
        if default_prim and default_prim.IsValid():
            return default_prim

        world_prim = stage.GetPrimAtPath("/World")
        if world_prim and world_prim.IsValid():
            return world_prim

        top_level_prims = list(stage.GetPseudoRoot().GetChildren())
        if top_level_prims:
            return top_level_prims[0]

        raise RuntimeError("산악 환경 USD에 불러올 Prim이 없습니다.")

    def fit_viewport_to_environment(self):
        """환경 크기를 계산해 산 전체가 보이도록 Viewport를 맞춘다."""
        target_prim = self._get_environment_root_prim()

        bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [
                UsdGeom.Tokens.default_,
                UsdGeom.Tokens.render,
                UsdGeom.Tokens.proxy,
            ],
        )
        aligned_range = bbox_cache.ComputeWorldBound(
            target_prim
        ).ComputeAlignedRange()

        if aligned_range.IsEmpty():
            carb.log_warn(
                "환경 Bounding Box를 계산하지 못해 기본 Viewport를 "
                "사용합니다."
            )
            self.pg.set_viewport_camera(
                [30.0, 30.0, 25.0],
                [0.0, 0.0, 0.0],
            )
            return

        minimum = aligned_range.GetMin()
        maximum = aligned_range.GetMax()
        center = 0.5 * (minimum + maximum)
        size = maximum - minimum
        largest_dimension = max(
            float(size[0]),
            float(size[1]),
            float(size[2]),
        )

        values = [
            float(center[0]),
            float(center[1]),
            float(center[2]),
            largest_dimension,
        ]
        if not all(np.isfinite(value) for value in values):
            raise RuntimeError(
                "산악 환경의 Bounding Box 값이 유효하지 않습니다: "
                f"center={center}, size={size}"
            )

        camera_distance = max(20.0, largest_dimension * 0.75)
        eye = [
            float(center[0]) + camera_distance,
            float(center[1]) + camera_distance,
            max(
                float(maximum[2]) + camera_distance * 0.35,
                float(center[2]) + camera_distance * 0.65,
            ),
        ]
        target = [
            float(center[0]),
            float(center[1]),
            float(center[2]),
        ]

        self.pg.set_viewport_camera(eye, target)
        print(
            "[CHECK] Environment bounds: "
            f"min={minimum}, max={maximum}"
        )
        print(f"[CHECK] Viewport eye={eye}, target={target}")


def write_ground_truth(victim_position, victim_index):
    """드론 시작점과 실제 조난자 스폰 위치를 원자적으로 저장한다."""
    drone_starts = []
    for prim_path, vehicle_id, position in DRONE_CONFIGS:
        drone_starts.append(
            {
                "drone_id": prim_path.rsplit("/", 1)[-1],
                "vehicle_id": int(vehicle_id),
                "world_enu": [
                    float(position[0]),
                    float(position[1]),
                    float(position[2]),
                ],
            }
        )

    payload = {
        "format_version": 1,
        "map_frame": "map",
        "coordinate_convention": "world_enu",
        "drone_starts": drone_starts,
        "victim": {
            "victim_id": "victim_01",
            "spawn_candidate_index": int(victim_index),
            "world_enu": [
                float(victim_position[0]),
                float(victim_position[1]),
                float(victim_position[2]),
            ],
        },
    }

    # ROS 노드가 저장 중인 JSON을 읽지 않도록 임시 파일을 완성한 뒤
    # 최종 파일명으로 교체한다.
    temporary_path = GENERATED_GROUND_TRUTH_PATH.with_suffix(
        ".json.tmp"
    )
    temporary_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    temporary_path.replace(GENERATED_GROUND_TRUTH_PATH)

    print(
        "[INFO] RViz Ground Truth 저장: "
        f"{GENERATED_GROUND_TRUTH_PATH}, "
        f"victim={payload['victim']['world_enu']}"
    )


def remove_previous_generated_files(*paths):
    """이전 실행의 RViz 출력이 잠시 표시되지 않도록 파일을 제거한다."""
    for path in paths:
        try:
            Path(path).unlink(missing_ok=True)
        except OSError as error:
            carb.log_warn(
                "이전 RViz 시각화 파일을 제거하지 못했습니다: "
                f"{path}: {error}"
            )

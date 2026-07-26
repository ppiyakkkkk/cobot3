#!/usr/bin/env python3
"""rescue_search 전용 조난자·구조자 생성과 구조자 경로 추종.

구조자 이동은 두 계층으로 분리한다.

1. Pegasus ``Person`` / Animation Graph:
   - 기존 걷기 애니메이션과 XY 목표 추종을 담당한다.
2. 이 모듈의 ground follower:
   - 현재 구조자 World XY 아래의 보행 가능 PhysX 표면을 찾는다.
   - Animation Graph Character root의 World Z를 직접 보정한다.

Pegasus ``Person.update_target_position()``은 목표 XYZ를 저장하지만,
실제 이동은 Animation Graph의 PathPoints가 담당한다. 산악 지형에서 이
경로가 Z를 따라가지 않는 경우가 있으므로, 공식 Animation Graph API인
``character.set_world_transform()``으로 root Z만 별도 적용한다.
"""

from dataclasses import dataclass
import math
import time

import carb
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path as PathMessage
import numpy as np
from scipy import ndimage
import omni.anim.graph.core as ag
import omni.physx
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from pegasus.simulator.logic.people.person import Person

from sim_config import (
    FOR_TEST_VICTIM_SPAWN_ENABLED,
    FOR_TEST_VICTIM_WORLD_XYZ,
    GENERATED_ENVIRONMENT_MESH_PATH,
    GENERATED_NAVIGATION_SURFACE_PATH,
    NAVIGATION_STRUCTURE_ALIASES,
    RESCUER_GROUND_BRIDGE_MAX_STEP_HEIGHT_M,
    RESCUER_GROUND_MAX_STEP_HEIGHT_M,
    PERSON_COLLIDER_CYLINDER_HEIGHT_M,
    PERSON_COLLIDER_RADIUS_M,
    PERSON_COLLIDER_TOTAL_HEIGHT_M,
    PERSON_GROUND_CLEARANCE_M,
    PERSON_CLOTHING_EXCLUDE_KEYWORDS,
    PERSON_CLOTHING_FALLBACK_TO_NON_SKIN_PARTS,
    PERSON_CLOTHING_INCLUDE_KEYWORDS,
    PERSON_ROLE_CLOTHING_COLOR_ENABLED,
    RESCUER_CHARACTER_KEYWORDS,
    RESCUER_BLOCK_ROCKS,
    RESCUER_BLOCK_VEGETATION,
    RESCUER_BRIDGE_MAX_SLOPE_DEG,
    RESCUER_BRIDGE_MAX_STEP_HEIGHT_M,
    RESCUER_MOVE_SPEED_M_S,
    RESCUER_MAX_SLOPE_DEG,
    RESCUER_MAX_STEP_HEIGHT_M,
    RESCUER_OBSTACLE_CLEARANCE_M,
    RESCUER_PATH_TOPIC,
    RESCUER_POSE_PUBLISH_PERIOD_SEC,
    RESCUER_POSITION_TOPIC,
    RESCUER_STATUS_TOPIC,
    RESCUER_PLATFORM_CLEARANCE_M,
    RESCUER_RIVER_CLEARANCE_M,
    RESCUER_SPAWN_CLEARANCE_M,
    RESCUER_SPAWN_MAX_LOCAL_STEP_M,
    RESCUER_SPAWN_PLATFORM_MIN_DISTANCE_M,
    RESCUER_SPAWN_PLATFORM_PATH,
    RESCUER_SPAWN_RAYCAST_RETRY_COUNT,
    RESCUER_SPAWN_REQUIRE_BRIDGE_REACHABLE,
    RESCUER_SPAWN_SEARCH_MAX_RADIUS_M,
    RESCUER_WAYPOINT_TOLERANCE_M,
    RESCUER_XY,
    RESCUER_CLOTHING_COLOR_RGB,
    VICTIM_CLOTHING_COLOR_RGB,
    VICTIM_PREFERRED_CHARACTER,
    VICTIM_SPAWN_POSITIONS,
)
from sim_utils import write_ground_truth
from forest_rescue_system.rescuer.rescuer_route_utils import (
    build_rescuer_grid_map,
    component_id_at,
    nearest_walkable_cell,
    walkable_component_labels,
)


# 구조자 Z 보정은 경로계획과 독립된 실행 계층이다.
# 60 Hz에서 8 m/s는 프레임당 약 0.133 m로, 내리막을 부드럽게 따라가면서
# 수 미터 공중에 오래 남지 않게 한다.
RESCUER_GROUND_FOLLOW_MAX_SPEED_M_S = 8.0
RESCUER_GROUND_SNAP_TOLERANCE_M = 0.015

# navigation surface가 가리키는 높이와 PhysX hit가 크게 다르면,
# 다리 collision이 없는데 아래 Terrain을 잘못 지면으로 선택한 것으로 본다.
RESCUER_GROUND_NAV_RAYCAST_MAX_ERROR_M = 2.5

# 지나치게 작은 dt 또는 첫 프레임에서도 제한된 보정이 가능하도록 사용한다.
RESCUER_GROUND_FOLLOW_MIN_DT_SEC = 1.0 / 120.0

# Terrain/등록 다리 외에 지면으로 잘못 선택되면 위험한 이름들이다.
NON_WALKABLE_GROUND_KEYWORDS = (
    "river",
    "water",
    "stream",
    "creek",
    "brook",
    "canal",
    "channel",
    "lake",
    "pond",
    "rock",
    "boulder",
    "personcollider",
    "personcolliders",
    "victim",
    "rescuer",
)


@dataclass(frozen=True)
class WalkableGroundHit:
    """현재 XY에서 선택한 보행 가능 PhysX 표면."""

    z: float
    prim_path: str
    surface_type: str
    navigation_z: float
    navigation_error_m: float


class PeopleManager:
    """두 사람과 충돌 프록시, 구조자 ROS Path 추종을 관리한다."""

    def __init__(
        self,
        terrain,
        rng,
        test_victim_spawn_world_enu=None,
        physics_update_callback=None,
    ):
        self.terrain = terrain
        self.rng = rng
        self.test_victim_spawn_world_enu = test_victim_spawn_world_enu
        self._physics_update_callback = physics_update_callback
        self._person_physics_proxies = {}
        self.victim = None
        self.rescuer = None

        self._ros_node = None
        self._owns_rclpy_context = False
        self._rescuer_path = []
        self._rescuer_waypoint_index = 0
        self._rescuer_status = "NOT_INITIALIZED"
        self._last_pose_publish_at = float("-inf")
        self._last_navigation_log_sim_time = float("-inf")
        self._last_navigation_sim_time = None
        self._last_ground_loss_log_at = float("-inf")
        self._last_ground_loss_reason = None
        self._last_ground_transition_log_at = float("-inf")
        self._last_large_ground_error_log_sim_time = float("-inf")
        self._ground_query_error_logged = False
        self._character_transform_error_logged = False
        self._rescuer_xform_ops_logged = False

        self._rescuer_foot_offset_m = float(PERSON_GROUND_CLEARANCE_M)
        self._terrain_prim_path = str(self.terrain._terrain_prim.GetPath())
        self._timeline = omni.timeline.get_timeline_interface()
        self._scene_query = omni.physx.get_physx_scene_query_interface()

        self._path_subscription = None
        self._pose_publisher = None
        self._status_publisher = None

        # 지면 추종 진단 상태
        self._last_ground_hit = None
        # 진단값을 지워도 마지막으로 실제 적용에 성공한 지면은 유지한다.
        # 플랫폼 가장자리의 큰 낙차를 한 프레임만 차단한 뒤 다음 프레임에
        # 허용하는 문제를 막기 위한 안전 기준이다.
        self._last_stable_ground_hit = None
        self._last_ground_z = None
        self._last_navigation_surface_z = None
        self._last_desired_z = None
        self._last_requested_z_correction = 0.0
        self._last_applied_z_correction = 0.0
        self._last_foot_error_m = None
        self._last_move_command_applied = False
        self._last_waypoint_surface_z = None
        self._last_waypoint_raycast_z = None
        self._last_waypoint_ground_prim = "NONE"

        self._navigation_structures = tuple(
            dict(structure)
            for structure in getattr(self.terrain, "_navigation_structures", [])
            if str(structure.get("path", "")).strip()
        )
        self._navigation_structure_paths = tuple(
            str(structure["path"])
            for structure in self._navigation_structures
        )
        self._navigation_structure_aliases = tuple(
            self._normalize_token(value)
            for value in NAVIGATION_STRUCTURE_ALIASES
            if str(value).strip()
        )
        self._non_walkable_ground_keywords = tuple(
            self._normalize_token(value)
            for value in NON_WALKABLE_GROUND_KEYWORDS
            if str(value).strip()
        )

    # ------------------------------------------------------------------
    # 사람 asset 및 Material
    # ------------------------------------------------------------------
    @staticmethod
    def _select_people_assets(available_assets):
        """가능한 경우 조난자와 구조자에 서로 다른 모델을 선택한다."""
        available_assets = [str(value) for value in available_assets]
        if not available_assets:
            raise RuntimeError("No Pegasus person assets were found.")

        if VICTIM_PREFERRED_CHARACTER in available_assets:
            victim_asset = VICTIM_PREFERRED_CHARACTER
        else:
            victim_asset = available_assets[0]
            carb.log_warn(
                "Preferred victim asset was not found. "
                f"Using {victim_asset} instead."
            )

        candidates = [item for item in available_assets if item != victim_asset]
        if not candidates:
            carb.log_warn(
                "구조자에 사용할 다른 Character asset이 없어 조난자 모델을 "
                "같이 사용합니다."
            )
            return victim_asset, victim_asset

        keywords = tuple(
            str(value).lower() for value in RESCUER_CHARACTER_KEYWORDS
        )

        def score(asset_name):
            lowered = asset_name.lower()
            keyword_score = 0
            for index, keyword in enumerate(keywords):
                if keyword in lowered:
                    keyword_score = max(
                        keyword_score,
                        len(keywords) - index,
                    )
            return keyword_score

        rescuer_asset = max(candidates, key=lambda item: (score(item), item))
        return victim_asset, rescuer_asset

    @staticmethod
    def _normalize_material_name(value):
        """Prim/Material 이름을 키워드 비교용 소문자 문자열로 바꾼다."""
        return "".join(
            character.lower()
            for character in str(value)
            if character.isalnum()
        )

    @staticmethod
    def _normalize_token(value):
        return "".join(
            character.lower()
            for character in str(value)
            if character.isalnum()
        )

    @staticmethod
    def _bound_material_path(prim):
        """Prim에 현재 바인딩된 Material 경로를 반환한다."""
        try:
            material, _ = UsdShade.MaterialBindingAPI(
                prim
            ).ComputeBoundMaterial()
        except Exception:
            return ""

        if not material:
            return ""
        material_prim = material.GetPrim()
        if not material_prim or not material_prim.IsValid():
            return ""
        return str(material_prim.GetPath())

    @staticmethod
    def _create_role_color_material(stage, material_path, rgb):
        """UsdPreviewSurface 기반 단색 Material을 생성하거나 갱신한다."""
        red, green, blue = [
            min(1.0, max(0.0, float(value)))
            for value in rgb
        ]

        material = UsdShade.Material.Define(stage, material_path)
        shader = UsdShade.Shader.Define(
            stage,
            f"{material_path}/PreviewSurface",
        )
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput(
            "diffuseColor",
            Sdf.ValueTypeNames.Color3f,
        ).Set(Gf.Vec3f(red, green, blue))
        shader.CreateInput(
            "roughness",
            Sdf.ValueTypeNames.Float,
        ).Set(0.65)
        shader.CreateInput(
            "metallic",
            Sdf.ValueTypeNames.Float,
        ).Set(0.0)
        material.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(),
            "surface",
        )
        return material

    @staticmethod
    def _bind_role_material(prim, material):
        """기존 하위 Material보다 우선하도록 역할 Material을 바인딩한다."""
        try:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                material,
                UsdShade.Tokens.strongerThanDescendants,
            )
            return True
        except Exception:
            try:
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
                return True
            except Exception:
                return False

    def _apply_person_clothing_color(self, person, role_name, rgb):
        """Character의 피부를 제외한 의상 Material을 역할 색상으로 덮는다."""
        if not PERSON_ROLE_CLOTHING_COLOR_ENABLED:
            return

        stage = omni.usd.get_context().get_stage()
        root_path = str(
            getattr(person, "_stage_prefix", "")
            or getattr(person, "character_skel_root_stage_path", "")
        )
        root_prim = stage.GetPrimAtPath(root_path)
        if not root_path or not root_prim.IsValid():
            carb.log_warn(
                f"{role_name} Character Prim을 찾지 못해 의상 색상을 "
                f"적용하지 못했습니다: {root_path!r}"
            )
            return

        role_token = self._normalize_material_name(role_name)
        material_path = (
            f"/World/Looks/ForestRescuePeople/{role_token}_clothing"
        )
        material = self._create_role_color_material(stage, material_path, rgb)

        include_keywords = tuple(
            self._normalize_material_name(value)
            for value in PERSON_CLOTHING_INCLUDE_KEYWORDS
            if str(value).strip()
        )
        exclude_keywords = tuple(
            self._normalize_material_name(value)
            for value in PERSON_CLOTHING_EXCLUDE_KEYWORDS
            if str(value).strip()
        )

        preferred_targets = []
        fallback_targets = []
        for prim in Usd.PrimRange(root_prim):
            if not (prim.IsA(UsdGeom.Subset) or prim.IsA(UsdGeom.Mesh)):
                continue

            description = self._normalize_material_name(
                f"{prim.GetPath()} {self._bound_material_path(prim)}"
            )
            if any(keyword in description for keyword in exclude_keywords):
                continue

            if any(keyword in description for keyword in include_keywords):
                preferred_targets.append(prim)
            else:
                fallback_targets.append(prim)

        targets = preferred_targets
        selection_mode = "clothing_keyword"
        if not targets and PERSON_CLOTHING_FALLBACK_TO_NON_SKIN_PARTS:
            subset_targets = [
                prim
                for prim in fallback_targets
                if prim.IsA(UsdGeom.Subset)
            ]
            targets = subset_targets or [
                prim
                for prim in fallback_targets
                if prim.IsA(UsdGeom.Mesh)
            ]
            selection_mode = "non_skin_fallback"

        applied_paths = []
        failed_paths = []
        for prim in targets:
            if self._bind_role_material(prim, material):
                applied_paths.append(str(prim.GetPath()))
            else:
                failed_paths.append(str(prim.GetPath()))

        if not applied_paths:
            carb.log_warn(
                f"{role_name} 의상 색상 적용 대상이 없습니다. "
                f"root={root_path}, candidates={len(targets)}, "
                f"failed={len(failed_paths)}"
            )
            return

        preview = ", ".join(applied_paths[:8])
        if len(applied_paths) > 8:
            preview += f", ...(+{len(applied_paths) - 8})"

        print(
            f"[PERSON COLOR] {role_name}: "
            f"rgb={tuple(float(value) for value in rgb)}, "
            f"mode={selection_mode}, applied={len(applied_paths)}, "
            f"failed={len(failed_paths)}, targets=[{preview}]"
        )

    # ------------------------------------------------------------------
    # 사람 생성 및 ROS bridge
    # ------------------------------------------------------------------
    @staticmethod
    def _distance_from_aabb(x, y, bounds):
        x_min, x_max, y_min, y_max = [float(value) for value in bounds]
        dx = max(x_min - float(x), 0.0, float(x) - x_max)
        dy = max(y_min - float(y), 0.0, float(y) - y_max)
        return math.hypot(dx, dy)

    def _platform_bounds(self):
        for structure in self._navigation_structures:
            configured = str(structure.get("configured_path", "")).rstrip("/")
            path = str(structure.get("path", "")).rstrip("/")
            expected = str(RESCUER_SPAWN_PLATFORM_PATH).rstrip("/")
            if configured == expected or self._paths_overlap(path, expected):
                return (
                    float(structure["x_min"]),
                    float(structure["x_max"]),
                    float(structure["y_min"]),
                    float(structure["y_max"]),
                )
        raise RuntimeError(
            "[RESCUER SPAWN SEARCH] 회색 플랫폼 AABB를 찾지 못했습니다: "
            f"platform_path={RESCUER_SPAWN_PLATFORM_PATH}"
        )

    def _raycast_spawn_terrain_once(self, x, y):
        """후보 XY에서 Terrain collision을 한 번 조회한다."""
        ray_top = float(getattr(self.terrain, "z_max", 100.0)) + 30.0
        ray_bottom = float(getattr(self.terrain, "z_min", -100.0)) - 20.0
        hits = []

        def collect(hit):
            position = self._hit_position(hit)
            if position is None:
                return True
            try:
                z = float(position[2])
            except (TypeError, ValueError, IndexError):
                return True
            if math.isfinite(z):
                hits.append((z, self._hit_prim_path(hit)))
            return True

        try:
            self._scene_query.raycast_all(
                carb.Float3(float(x), float(y), ray_top),
                carb.Float3(0.0, 0.0, -1.0),
                ray_top - ray_bottom,
                collect,
            )
        except Exception:
            return None, "no_physx_terrain_hit"

        terrain_hits = [
            item
            for item in hits
            if self._paths_overlap(item[1], self._terrain_prim_path)
        ]
        if not terrain_hits:
            return None, "no_physx_terrain_hit"
        terrain_z = max(float(item[0]) for item in terrain_hits)

        # Terrain보다 위에서 먼저 맞는 collider는 나무·바위·난간 등으로 본다.
        blocking = [
            item
            for item in hits
            if float(item[0]) > terrain_z + 0.20
            and not self._paths_overlap(item[1], self._terrain_prim_path)
        ]
        if blocking:
            return None, "obstacle"
        return terrain_z, None

    def _raycast_spawn_terrain(self, x, y):
        """PhysX cooking 지연을 고려해 Terrain collision을 제한적으로 재조회한다."""
        attempts = max(1, int(RESCUER_SPAWN_RAYCAST_RETRY_COUNT))
        for attempt in range(attempts):
            terrain_z, error = self._raycast_spawn_terrain_once(x, y)
            if error != "no_physx_terrain_hit":
                return terrain_z, error
            if attempt + 1 >= attempts or self._physics_update_callback is None:
                break
            self._physics_update_callback()
        return None, "no_physx_terrain_hit"

    def _find_safe_rescuer_spawn(self, victim_position):
        """플랫폼 밖의 가장 가까운 안전 Terrain 셀을 자동 선택한다."""
        grid_map = build_rescuer_grid_map(
            GENERATED_NAVIGATION_SURFACE_PATH,
            GENERATED_ENVIRONMENT_MESH_PATH,
            max_slope_deg=float(RESCUER_MAX_SLOPE_DEG),
            max_step_height_m=float(RESCUER_MAX_STEP_HEIGHT_M),
            bridge_max_slope_deg=float(RESCUER_BRIDGE_MAX_SLOPE_DEG),
            bridge_max_step_height_m=float(RESCUER_BRIDGE_MAX_STEP_HEIGHT_M),
            river_clearance_m=float(RESCUER_RIVER_CLEARANCE_M),
            obstacle_clearance_m=float(RESCUER_OBSTACLE_CLEARANCE_M),
            platform_clearance_m=float(RESCUER_PLATFORM_CLEARANCE_M),
            block_rocks=bool(RESCUER_BLOCK_ROCKS),
            block_vegetation=bool(RESCUER_BLOCK_VEGETATION),
        )
        bounds = self._platform_bounds()
        platform_center = (
            0.5 * (bounds[0] + bounds[1]),
            0.5 * (bounds[2] + bounds[3]),
        )
        requested_xy = (float(RESCUER_XY[0]), float(RESCUER_XY[1]))
        # 사용자가 지정한 판 앞쪽 Terrain 위치를 가장 먼저 선택한다.
        # 플랫폼 중심 기준으로 정렬하면 사진의 요청 위치와 무관한 판
        # 가장자리 후보가 먼저 선택될 수 있으므로 requested_xy를 기준으로
        # 가까운 안전 셀부터 검사한다.
        search_origin = requested_xy
        labels = walkable_component_labels(grid_map)

        requested_goal = grid_map.world_to_grid(
            float(victim_position[0]), float(victim_position[1])
        )
        victim_cell = nearest_walkable_cell(
            grid_map, requested_goal, max_radius_cells=30
        )
        victim_component = component_id_at(labels, victim_cell)
        if victim_component <= 0:
            raise RuntimeError(
                "[RESCUER SPAWN SEARCH] 조난자 측 보행 컴포넌트를 찾지 못함"
            )

        bridge_components = {
            int(labels[cell])
            for cell in zip(*np.where(grid_map.bridge_core_mask))
            if int(labels[cell]) > 0
        }
        mean_spacing = 0.5 * (
            float(grid_map.spacing_x) + float(grid_map.spacing_y)
        )
        clearance_cells = max(
            1,
            int(math.ceil(float(RESCUER_SPAWN_CLEARANCE_M) / mean_spacing)),
        )
        free_clearance = ndimage.distance_transform_edt(grid_map.walkable)
        local_step_limit = min(
            float(RESCUER_SPAWN_MAX_LOCAL_STEP_M),
            float(grid_map.max_step_height_m),
        )

        candidates = []
        requested_cell = grid_map.world_to_grid(
            requested_xy[0], requested_xy[1]
        )
        requested_row, requested_column = requested_cell
        requested_in_bounds = (
            0 <= requested_row < grid_map.walkable.shape[0]
            and 0 <= requested_column < grid_map.walkable.shape[1]
        )
        requested_walkable = bool(
            requested_in_bounds and grid_map.walkable[requested_cell]
        )
        if requested_walkable:
            # 격자 중심으로 좌표를 바꾸지 않고 사용자가 지정한 정확한 XY를
            # 첫 후보로 검사한다. 모든 안전 조건과 Terrain raycast를
            # 통과하면 selected_xy가 requested_xy와 동일하게 유지된다.
            candidates.append(
                (
                    0.0,
                    requested_cell,
                    requested_xy[0],
                    requested_xy[1],
                )
            )
        for row, column in zip(*np.where(grid_map.walkable)):
            cell = (int(row), int(column))
            if cell == requested_cell:
                continue
            x = float(grid_map.x_values[column])
            y = float(grid_map.y_values[row])
            distance_origin = math.hypot(
                x - search_origin[0], y - search_origin[1]
            )
            if distance_origin <= float(RESCUER_SPAWN_SEARCH_MAX_RADIUS_M):
                candidates.append((distance_origin, cell, x, y))
        candidates.sort(key=lambda item: item[0])
        print(
            "[RESCUER SPAWN SEARCH] "
            f"requested_cell={requested_cell}, "
            f"requested_walkable={requested_walkable}"
        )

        rejected = {}
        selected = None
        # 가까운 22 m를 먼저 검사하고, 실패할 때만 설정된 최대 반경까지
        # 넓힌다. 경사·단차·장애물·연결성 조건은 어느 단계에서도 완화하지
        # 않는다.
        initial_radius = min(22.0, float(RESCUER_SPAWN_SEARCH_MAX_RADIUS_M))
        search_radii = [initial_radius]
        if float(RESCUER_SPAWN_SEARCH_MAX_RADIUS_M) > initial_radius:
            search_radii.append(float(RESCUER_SPAWN_SEARCH_MAX_RADIUS_M))

        searched_cells = set()
        searched_count = 0
        for active_radius in search_radii:
            print(
                "[RESCUER SPAWN SEARCH] "
                f"active_radius={active_radius:.1f}m"
            )
            for distance_origin, cell, x, y in candidates:
                if distance_origin > active_radius or cell in searched_cells:
                    continue
                searched_cells.add(cell)
                searched_count += 1

                def reject(reason):
                    rejected[reason] = rejected.get(reason, 0) + 1

                platform_distance = self._distance_from_aabb(x, y, bounds)
                if platform_distance < float(
                    RESCUER_SPAWN_PLATFORM_MIN_DISTANCE_M
                ):
                    reject("inside_platform")
                    continue
                if grid_map.slope_deg[cell] > float(RESCUER_MAX_SLOPE_DEG):
                    reject("slope_too_high")
                    continue
                if grid_map.river_mask[cell]:
                    reject("river")
                    continue
                if grid_map.obstacle_mask[cell] or grid_map.platform_mask[cell]:
                    reject("obstacle")
                    continue
                if free_clearance[cell] < clearance_cells:
                    reject("obstacle")
                    continue
                row, column = cell
                # A*와 같은 4방향 인접 셀만 검사한다. 기존 3x3 검사는
                # 대각선(거리 약 2.12m)의 높이 차까지 1.25m 제한과 직접
                # 비교해, 경사 45도 미만인 (-29, 28)도 잘못 거부했다.
                neighbor_steps = []
                for d_row, d_column in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    n_row = row + d_row
                    n_column = column + d_column
                    if (
                        0 <= n_row < grid_map.z_grid.shape[0]
                        and 0 <= n_column < grid_map.z_grid.shape[1]
                    ):
                        neighbor_steps.append(
                            abs(
                                float(grid_map.z_grid[n_row, n_column])
                                - float(grid_map.z_grid[cell])
                            )
                        )
                if neighbor_steps and max(neighbor_steps) > local_step_limit:
                    reject("slope_too_high")
                    continue
                component = component_id_at(labels, cell)
                bridge_reachable = component in bridge_components
                if component != victim_component:
                    reject("disconnected_walkable_region")
                    continue
                if (
                    bool(RESCUER_SPAWN_REQUIRE_BRIDGE_REACHABLE)
                    and bridge_components
                    and not bridge_reachable
                ):
                    reject("disconnected_walkable_region")
                    continue
                terrain_z, raycast_error = self._raycast_spawn_terrain(x, y)
                if raycast_error == "no_physx_terrain_hit":
                    # 초기화 도중 World.reset()을 호출하지 않기 위해 최초
                    # 스폰 Z는 이미 생성한 Terrain heightfield를 사용한다.
                    # 모든 Prim 생성 후 단 한 번의 reset으로 PhysX를
                    # 시작하며, 실행 중 ground follower가 PhysX 표면으로
                    # 계속 보정한다.
                    terrain_z = float(grid_map.z_grid[cell])
                    raycast_error = None
                    print(
                        "[RESCUER SPAWN SEARCH] "
                        f"PhysX pre-reset hit 없음: heightfield Z 사용 "
                        f"xy=({x:.3f}, {y:.3f}), z={terrain_z:.3f}"
                    )
                if raycast_error is not None:
                    reject(raycast_error)
                    continue
                selected = (
                    x,
                    y,
                    float(terrain_z),
                    float(grid_map.slope_deg[cell]),
                    platform_distance,
                    component,
                    bridge_reachable,
                )
                break
            if selected is not None:
                break

        print("[RESCUER SPAWN SEARCH]")
        print(f"platform_path={RESCUER_SPAWN_PLATFORM_PATH}")
        print(
            "platform_bounds="
            f"({bounds[0]:.3f}, {bounds[1]:.3f}, "
            f"{bounds[2]:.3f}, {bounds[3]:.3f})"
        )
        print(f"requested_xy=({requested_xy[0]:.3f}, {requested_xy[1]:.3f})")
        for reason in (
            "inside_platform",
            "slope_too_high",
            "river",
            "obstacle",
            "disconnected_walkable_region",
            "no_physx_terrain_hit",
        ):
            if rejected.get(reason, 0):
                print(
                    f"candidate rejected: {reason} "
                    f"(count={rejected[reason]})"
                )
        if selected is None:
            raise RuntimeError(
                "[RESCUER SPAWN SEARCH] 안전한 Terrain 스폰 위치를 찾지 "
                f"못했습니다: searched={searched_count}, rejected={rejected}"
            )
        x, y, terrain_z, slope, distance, component, bridge_reachable = selected
        print(f"selected_xy=({x:.3f}, {y:.3f})")
        print(f"terrain_z={terrain_z:.3f}")
        print(f"slope_deg={slope:.3f}")
        print(f"distance_from_platform={distance:.3f}")
        print(f"walkable_component={component}")
        print(f"bridge_reachable={bridge_reachable}")
        return x, y, terrain_z

    def spawn_people(self):
        """rescue_search 모드에서 조난자와 구조자를 생성한다."""
        victim_asset, rescuer_asset = self._select_people_assets(
            Person.get_character_asset_list()
        )
        print(f"[PERSON] Victim asset: {victim_asset}")
        print(f"[PERSON] Rescuer asset: {rescuer_asset}")

        if FOR_TEST_VICTIM_SPAWN_ENABLED:
            victim_position = getattr(
                self,
                "test_victim_spawn_world_enu",
                None,
            )
            if victim_position is None:
                raise RuntimeError(
                    "시험용 조난자 위치가 생성되지 않았습니다. "
                    "FOR_TEST_VICTIM_WORLD_XYZ를 확인하세요."
                )
            victim_position = [float(value) for value in victim_position]
            victim_index = -1
            spawn_description = (
                "TEST hardcoded XYZ="
                f"({FOR_TEST_VICTIM_WORLD_XYZ[0]:.1f}, "
                f"{FOR_TEST_VICTIM_WORLD_XYZ[1]:.1f}, "
                f"{FOR_TEST_VICTIM_WORLD_XYZ[2]:.1f})"
            )
        else:
            victim_index = int(
                self.rng.integers(len(VICTIM_SPAWN_POSITIONS))
            )
            victim_candidate = VICTIM_SPAWN_POSITIONS[victim_index]
            victim_x = float(victim_candidate[0])
            victim_y = float(victim_candidate[1])
            victim_ground_z = self.terrain.height(victim_x, victim_y)
            victim_position = [
                victim_x,
                victim_y,
                victim_ground_z + PERSON_GROUND_CLEARANCE_M,
            ]
            spawn_description = f"candidate {victim_index + 1}"

        write_ground_truth(victim_position, victim_index)
        self.victim = Person(
            "victim_01",
            victim_asset,
            init_pos=victim_position,
            init_yaw=0.0,
        )
        self._apply_person_clothing_color(
            self.victim,
            role_name="victim",
            rgb=VICTIM_CLOTHING_COLOR_RGB,
        )
        self._create_person_physics_proxy(
            "victim_01",
            self.victim,
            victim_position,
        )
        print(
            f"[INFO] Spawned victim at {spawn_description}: "
            f"({victim_position[0]:.3f}, "
            f"{victim_position[1]:.3f}, "
            f"{victim_position[2]:.3f})"
        )

        rescuer_x, rescuer_y, rescuer_ground_z = (
            self._find_safe_rescuer_spawn(victim_position)
        )
        rescuer_position = [
            float(rescuer_x),
            float(rescuer_y),
            rescuer_ground_z + float(PERSON_GROUND_CLEARANCE_M),
        ]
        self.rescuer = Person(
            "rescuer_01",
            rescuer_asset,
            init_pos=rescuer_position,
            init_yaw=0.0,
        )
        self._apply_person_clothing_color(
            self.rescuer,
            role_name="rescuer",
            rgb=RESCUER_CLOTHING_COLOR_RGB,
        )
        self._create_person_physics_proxy(
            "rescuer_01",
            self.rescuer,
            rescuer_position,
        )
        self._log_rescuer_xform_ops_once()
        self._measure_rescuer_foot_offset()
        print(
            "[INFO] Spawned rescuer on safe Terrain at "
            f"({rescuer_position[0]:.3f}, "
            f"{rescuer_position[1]:.3f}, "
            f"{rescuer_position[2]:.3f})"
        )
        print(
            "[RESCUER] 보행 가능 표면 등록: "
            f"terrain={self._terrain_prim_path}, "
            f"structures={list(self._navigation_structure_paths)}"
        )
        self._initialize_ros_bridge()

    def _initialize_ros_bridge(self):
        """ROS Path 수신과 구조자 위치·상태 발행 노드를 준비한다."""
        if self._ros_node is not None:
            return
        if not rclpy.ok():
            rclpy.init(args=[])
            self._owns_rclpy_context = True

        self._ros_node = rclpy.create_node("isaac_rescuer_bridge")
        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._path_subscription = self._ros_node.create_subscription(
            PathMessage,
            RESCUER_PATH_TOPIC,
            self._rescuer_path_callback,
            transient_qos,
        )
        self._pose_publisher = self._ros_node.create_publisher(
            PointStamped,
            RESCUER_POSITION_TOPIC,
            10,
        )
        self._status_publisher = self._ros_node.create_publisher(
            String,
            RESCUER_STATUS_TOPIC,
            transient_qos,
        )
        self._set_rescuer_status("IDLE")
        print(
            "[OK] Isaac 구조자 ROS bridge 준비: "
            f"path={RESCUER_PATH_TOPIC}, pose={RESCUER_POSITION_TOPIC}, "
            f"status={RESCUER_STATUS_TOPIC}"
        )

    # ------------------------------------------------------------------
    # 경로 수신 및 XY 추종
    # ------------------------------------------------------------------
    def _rescuer_path_callback(self, message):
        if self.rescuer is None:
            return
        if message.header.frame_id and message.header.frame_id != "map":
            carb.log_error(
                "구조자 Path frame이 map이 아닙니다: "
                f"{message.header.frame_id}"
            )
            self._set_rescuer_status("PATH_REJECTED_FRAME")
            return

        waypoints = []
        for pose_stamped in message.poses:
            point = pose_stamped.pose.position
            waypoint = np.asarray(
                # Path의 Z는 navigation surface 높이로 보관한다.
                # Person에는 XY 진행만 맡기고, 실제 Z는 ground follower가 적용한다.
                [point.x, point.y, point.z],
                dtype=np.float64,
            )
            if waypoint.shape != (3,) or not np.all(np.isfinite(waypoint)):
                continue
            if (
                waypoints
                and np.linalg.norm(waypoint - waypoints[-1]) < 0.05
            ):
                continue
            waypoints.append(waypoint)

        if not waypoints:
            carb.log_error(
                "수신한 구조자 Path에 유효한 Waypoint가 없습니다."
            )
            self._set_rescuer_status("PATH_REJECTED_EMPTY")
            return

        self._rescuer_path = waypoints
        self._rescuer_waypoint_index = 0
        self._set_rescuer_status("PATH_RECEIVED")
        self._skip_reached_waypoints()
        self._send_current_waypoint()
        print(
            "[RESCUER] Path 수신: "
            f"waypoints={len(self._rescuer_path)}, "
            f"speed={RESCUER_MOVE_SPEED_M_S:.1f}m/s"
        )

    def _ensure_character_graph(self):
        """Play 상태의 Animation Graph Character handle을 반환한다."""
        if self.rescuer is None:
            return None

        graph = getattr(self.rescuer, "character_graph", None)
        if graph is not None:
            return graph

        skel_path = str(
            getattr(self.rescuer, "character_skel_root_stage_path", "")
        )
        if not skel_path:
            return None

        try:
            graph = ag.get_character(skel_path)
        except Exception:
            graph = None

        if graph is not None:
            self.rescuer.character_graph = graph
        return graph

    def _read_character_world_transform(self):
        """Animation Graph의 실제 World 위치와 회전을 읽는다."""
        graph = self._ensure_character_graph()
        if graph is None:
            return None

        position = carb.Float3(0.0, 0.0, 0.0)
        rotation = carb.Float4(0.0, 0.0, 0.0, 1.0)
        try:
            graph.get_world_transform(position, rotation)
        except Exception as error:
            if not self._character_transform_error_logged:
                carb.log_error(
                    "[RESCUER] Character World Transform 읽기 실패: "
                    f"{type(error).__name__}: {error}"
                )
                self._character_transform_error_logged = True
            return None

        values = np.asarray(
            [position[0], position[1], position[2]],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(values)):
            return None
        self._character_transform_error_logged = False
        return values, rotation

    def _current_rescuer_position(self):
        """가능하면 Character root의 실제 World 위치를 사용한다."""
        transform = self._read_character_world_transform()
        if transform is not None:
            position, _rotation = transform
            self._synchronize_person_state(position)
            return position

        if self.rescuer is None:
            return None
        position = np.asarray(
            self.rescuer.state.position,
            dtype=np.float64,
        )
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            return None
        return position

    def _synchronize_person_state(self, position):
        """즉시 pose 발행과 proxy 동기화를 위해 Person state도 맞춘다."""
        if self.rescuer is None:
            return
        position = np.asarray(position, dtype=np.float64)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            return

        state = getattr(self.rescuer, "_state", None)
        if state is not None:
            state.position = position.copy()

        # Pegasus의 속도 계산에서 Z 강제 보정이 수직 속도 spike로 잡히지 않게
        # 직전 위치의 Z만 같이 맞춘다. XY 속도 계산은 그대로 유지한다.
        previous = getattr(self.rescuer, "_previous_position", None)
        if isinstance(previous, np.ndarray) and previous.shape == (3,):
            previous[2] = float(position[2])

    def _set_character_world_z(self, desired_z):
        """공식 Animation Graph API로 Character root의 Z만 변경한다."""
        transform = self._read_character_world_transform()
        if transform is None:
            return False

        current, rotation = transform
        updated = np.asarray(
            [current[0], current[1], float(desired_z)],
            dtype=np.float64,
        )
        graph = self._ensure_character_graph()
        if graph is None:
            return False

        try:
            graph.set_world_transform(
                carb.Float3(
                    float(updated[0]),
                    float(updated[1]),
                    float(updated[2]),
                ),
                rotation,
            )
        except Exception as error:
            if not self._character_transform_error_logged:
                carb.log_error(
                    "[RESCUER] Character root Z 적용 실패: "
                    f"{type(error).__name__}: {error}"
                )
                self._character_transform_error_logged = True
            return False

        self._character_transform_error_logged = False
        self._synchronize_person_state(updated)
        return True

    def _skip_reached_waypoints(self):
        position = self._current_rescuer_position()
        if position is None:
            return
        tolerance = float(RESCUER_WAYPOINT_TOLERANCE_M)
        while self._rescuer_waypoint_index < len(self._rescuer_path):
            target = self._rescuer_path[self._rescuer_waypoint_index]
            if float(np.linalg.norm(target[:2] - position[:2])) > tolerance:
                break
            self._rescuer_waypoint_index += 1

    def _send_current_waypoint(self):
        """다음 XY 목표를 보내고 Path Z와 실제 표면 Z를 함께 검증한다."""
        if self.rescuer is None or not self._rescuer_path:
            return

        if self._rescuer_waypoint_index >= len(self._rescuer_path):
            current_position = self._current_rescuer_position()
            if current_position is not None:
                self.rescuer.update_target_position(
                    current_position.tolist(),
                    walk_speed=0.0,
                )
            self._set_rescuer_status("ARRIVED")
            print("[RESCUER] 조난자 주변 최종 목표에 도착했습니다.")
            return

        target = self._rescuer_path[self._rescuer_waypoint_index]
        path_surface_z = float(target[2])
        target_hit = self._query_walkable_ground(
            float(target[0]),
            float(target[1]),
            navigation_hint_z=path_surface_z,
        )
        if target_hit is None:
            self._stop_rescuer_for_ground_loss(
                "목표 waypoint 아래에서 navigation surface와 일치하는 "
                "보행 가능 collision을 찾지 못함"
            )
            return

        self._last_waypoint_surface_z = path_surface_z
        self._last_waypoint_raycast_z = float(target_hit.z)
        self._last_waypoint_ground_prim = target_hit.prim_path

        current_position = self._current_rescuer_position()
        if current_position is None:
            return

        # Animation Graph에는 XY 보행만 맡긴다. 목표 Z는 현재 실제 root Z로
        # 유지하여 3D PathPoints가 캐릭터를 공중으로 끌어당기지 않게 한다.
        animation_target = [
            float(target[0]),
            float(target[1]),
            float(current_position[2]),
        ]
        self.rescuer.update_target_position(
            animation_target,
            walk_speed=float(RESCUER_MOVE_SPEED_M_S),
        )
        self._last_move_command_applied = True
        self._set_rescuer_status(
            f"WALKING:{self._rescuer_waypoint_index + 1}/"
            f"{len(self._rescuer_path)}"
        )

        surface_difference = float(target_hit.z) - path_surface_z
        if abs(surface_difference) > 0.5:
            carb.log_warn(
                "[RESCUER] Waypoint navigation/PhysX 높이 차이: "
                f"waypoint_z={path_surface_z:.3f}, "
                f"raycast_z={target_hit.z:.3f}, "
                f"diff={surface_difference:+.3f}m, "
                f"prim={target_hit.prim_path}"
            )

    def _refresh_current_xy_target(self, current_z):
        """현재 Waypoint의 XY와 보정된 현재 Z로 Animation Graph 목표를 갱신한다."""
        if self._rescuer_waypoint_index >= len(self._rescuer_path):
            return
        target = self._rescuer_path[self._rescuer_waypoint_index]
        self.rescuer.update_target_position(
            [
                float(target[0]),
                float(target[1]),
                float(current_z),
            ],
            walk_speed=float(RESCUER_MOVE_SPEED_M_S),
        )
        self._last_move_command_applied = True
        if self._rescuer_status == "GROUND_LOST":
            self._last_ground_loss_reason = None
            self._set_rescuer_status(
                f"WALKING:{self._rescuer_waypoint_index + 1}/"
                f"{len(self._rescuer_path)}"
            )

    # ------------------------------------------------------------------
    # 보행 가능 지면 판정
    # ------------------------------------------------------------------
    @staticmethod
    def _hit_prim_path(hit):
        """Isaac Sim 버전별 raycast hit에서 Prim 경로를 꺼낸다."""
        for key in ("collision", "rigidBody", "material"):
            if isinstance(hit, dict):
                value = hit.get(key)
            else:
                value = getattr(hit, key, None)
            if value:
                return str(value)
        return ""

    @staticmethod
    def _hit_position(hit):
        """Raycast hit 위치를 dict/바인딩 객체 양쪽에서 안전하게 꺼낸다."""
        if isinstance(hit, dict):
            return hit.get("position")
        return getattr(hit, "position", None)

    @staticmethod
    def _paths_overlap(first, second):
        first = str(first).rstrip("/")
        second = str(second).rstrip("/")
        if not first or not second:
            return False
        return (
            first == second
            or first.startswith(f"{second}/")
            or second.startswith(f"{first}/")
        )

    def _classify_walkable_prim(self, prim_path):
        """Terrain 또는 navigation surface에 등록된 구조물만 허용한다."""
        candidate = str(prim_path).strip()
        if not candidate:
            return None

        normalized = self._normalize_token(candidate)
        if any(
            keyword and keyword in normalized
            for keyword in self._non_walkable_ground_keywords
        ):
            return None

        if self._paths_overlap(candidate, self._terrain_prim_path):
            return "terrain"

        for structure in self._navigation_structures:
            # 명시적 회색 플랫폼은 드론 전용이며 구조자 지면으로 허용하지 않는다.
            if str(structure.get("source_type", "alias")) == "explicit":
                continue
            structure_path = str(structure["path"])
            if self._paths_overlap(candidate, structure_path):
                return "navigation_structure"

        # 등록된 구조물의 collision Prim이 별도 경로로 생성되는 환경을 위한
        # fallback이다. sim_config.py에서 navigation structure로 명시한
        # bridge/deck 계열 이름만 허용한다.
        if any(
            alias and alias in normalized
            for alias in self._navigation_structure_aliases
        ):
            return "navigation_structure_alias"

        return None

    def _navigation_height(self, x, y):
        """실행 중 TerrainHeightField가 사용한 navigation surface 높이."""
        try:
            value = float(self.terrain.navigation_height(float(x), float(y)))
        except (AttributeError, RuntimeError, TypeError, ValueError):
            try:
                value = float(self.terrain.height(float(x), float(y)))
            except (RuntimeError, TypeError, ValueError):
                return float("nan")
        return value if math.isfinite(value) else float("nan")


    def _navigation_structure_at(self, x, y, alias_margin_m=0.10):
        """현재 XY가 실제 플랫폼 또는 다리 상판 안인지 확인한다.

        새 TerrainHeightField는 다리를 AABB 전체가 아니라 추출된 상판 core로
        판정한다. 이 경로를 우선 사용하면 난간·교각과 다리 주변 빈 공간을
        구조물 내부로 오인하지 않는다. 구형 TerrainHeightField를 위한 AABB
        fallback은 그대로 남긴다.
        """
        x = float(x)
        y = float(y)

        try:
            structure = self.terrain.navigation_structure_at(x, y)
        except (AttributeError, RuntimeError, TypeError, ValueError):
            structure = None
        if structure is not None:
            return structure

        alias_margin = max(0.0, float(alias_margin_m))
        matches = []
        for structure in self._navigation_structures:
            source_type = str(structure.get("source_type", "alias"))
            # 새 다리 데이터가 있는데 core 밖이면 AABB fallback을 사용하지
            # 않는다. 접근부에서는 Terrain과 다리 PhysX hit를 모두 허용한다.
            if (
                source_type == "alias"
                and getattr(self.terrain, "_bridge_core_mask", None) is not None
            ):
                continue
            margin = 0.0 if source_type == "explicit" else alias_margin
            if (
                float(structure["x_min"]) - margin
                <= x
                <= float(structure["x_max"]) + margin
                and float(structure["y_min"]) - margin
                <= y
                <= float(structure["y_max"]) + margin
            ):
                matches.append(structure)

        if not matches:
            return None
        return max(
            matches,
            key=lambda item: float(item.get("z_max", -math.inf)),
        )

    def _is_bridge_surface(self, prim_path, surface_type):
        """등록된 다리 Prim 또는 bridge/deck 별칭인지 확인한다."""
        if not str(surface_type).startswith("navigation_structure"):
            return False
        normalized = self._normalize_token(prim_path)
        return any(
            alias and alias in normalized
            for alias in self._navigation_structure_aliases
        )

    def _query_walkable_ground(
        self,
        x,
        y,
        navigation_hint_z=None,
        enforce_local_step=False,
    ):
        """현재 XY의 PhysX hit 중 navigation surface와 맞는 지면을 선택한다.

        ``enforce_local_step``이 True이면 직전 실제 지면에서 다른 Prim으로
        넘어갈 때 높이차를 제한한다. 경로의 먼 waypoint를 미리 검증할 때는
        전체 경사 누적 높이차를 단차로 오인하지 않도록 이 검사를 사용하지
        않고, 현재 구조자 위치를 갱신할 때만 활성화한다.
        """
        current = self._current_rescuer_position()
        current_z = (
            float(current[2])
            if current is not None and math.isfinite(float(current[2]))
            else float(getattr(self.terrain, "z_max", 100.0))
        )
        ray_top = max(
            float(getattr(self.terrain, "z_max", 100.0)) + 20.0,
            current_z + 20.0,
        )
        ray_bottom = float(getattr(self.terrain, "z_min", -100.0)) - 20.0
        accepted_hits = []

        def report_hit(hit):
            prim_path = self._hit_prim_path(hit)
            surface_type = self._classify_walkable_prim(prim_path)
            if surface_type is None:
                return True

            position = self._hit_position(hit)
            if position is None:
                return True
            try:
                hit_z = float(position[2])
            except (TypeError, ValueError, IndexError):
                return True
            if math.isfinite(hit_z):
                accepted_hits.append((hit_z, prim_path, surface_type))
            return True

        try:
            self._scene_query.raycast_all(
                carb.Float3(float(x), float(y), ray_top),
                carb.Float3(0.0, 0.0, -1.0),
                ray_top - ray_bottom,
                report_hit,
            )
        except Exception as error:
            if not self._ground_query_error_logged:
                carb.log_error(
                    "[RESCUER] PhysX 보행 지면 raycast 실패: "
                    f"{type(error).__name__}: {error}"
                )
                self._ground_query_error_logged = True
            return None

        if navigation_hint_z is None or not math.isfinite(
            float(navigation_hint_z)
        ):
            navigation_z = self._navigation_height(x, y)
        else:
            navigation_z = float(navigation_hint_z)

        if not accepted_hits:
            return None

        expected_structure = self._navigation_structure_at(x, y)
        if expected_structure is not None:
            # 구조물 실제 AABB 안에서는 아래 Terrain을 대신 선택하지 않는다.
            # Collider가 누락된 경우 구조자를 판 아래로 내리지 않고 정지한다.
            structure_hits = [
                item
                for item in accepted_hits
                if item[2].startswith("navigation_structure")
            ]
            if not structure_hits:
                return None
            candidate_hits = structure_hits
        else:
            # 명시적 플랫폼은 AABB margin을 0으로 사용한다. 판을 실제로
            # 벗어난 순간에는 Terrain hit를 허용하며, 아래 local step 검사로
            # 안전한 낮은 연결부와 큰 낙차를 구분한다.
            candidate_hits = accepted_hits

        # navigation surface와 가장 가까운 물리 표면을 우선한다.
        # navigation_z가 없으면 가장 높은 허용 표면을 선택한다.
        if math.isfinite(navigation_z):
            selected = min(
                candidate_hits,
                key=lambda item: (
                    abs(float(item[0]) - navigation_z),
                    -float(item[0]),
                ),
            )
            navigation_error = abs(float(selected[0]) - navigation_z)
            if (
                expected_structure is not None
                and navigation_error
                > float(RESCUER_GROUND_NAV_RAYCAST_MAX_ERROR_M)
            ):
                return None
        else:
            selected = max(candidate_hits, key=lambda item: float(item[0]))
            navigation_error = float("nan")

        if enforce_local_step and self._last_stable_ground_hit is not None:
            previous_hit = self._last_stable_ground_hit
            selected_prim_path = str(selected[1])
            selected_surface_type = str(selected[2])
            changed_surface = (
                not self._paths_overlap(
                    selected_prim_path,
                    previous_hit.prim_path,
                )
                or selected_surface_type != previous_hit.surface_type
            )
            step_height = abs(float(selected[0]) - float(previous_hit.z))
            bridge_transition = (
                self._is_bridge_surface(
                    previous_hit.prim_path,
                    previous_hit.surface_type,
                )
                or self._is_bridge_surface(
                    selected_prim_path,
                    selected_surface_type,
                )
            )
            step_limit = (
                float(RESCUER_GROUND_BRIDGE_MAX_STEP_HEIGHT_M)
                if bridge_transition
                else float(RESCUER_GROUND_MAX_STEP_HEIGHT_M)
            )
            if changed_surface and step_height > step_limit:
                now = time.monotonic()
                if now - self._last_ground_transition_log_at >= 1.0:
                    carb.log_error(
                        "[RESCUER] 지면 전환 차단: "
                        f"from={previous_hit.prim_path}"
                        f"({previous_hit.z:.3f}m), "
                        f"to={selected_prim_path}({float(selected[0]):.3f}m), "
                        f"step={step_height:.3f}m > "
                        f"limit={step_limit:.3f}m, "
                        f"bridge_transition={bridge_transition}"
                    )
                    self._last_ground_transition_log_at = now
                return None

        self._ground_query_error_logged = False
        return WalkableGroundHit(
            z=float(selected[0]),
            prim_path=str(selected[1]),
            surface_type=str(selected[2]),
            navigation_z=float(navigation_z),
            navigation_error_m=float(navigation_error),
        )

    # ------------------------------------------------------------------
    # 실제 Character root Z 보정
    # ------------------------------------------------------------------
    def _apply_ground_following(self, position, ground_hit, sim_dt):
        """지면 높이에 맞춰 Animation Graph Character root Z를 직접 보정한다."""
        desired_z = float(ground_hit.z) + self._rescuer_foot_offset_m
        current_z = float(position[2])
        requested = desired_z - current_z

        effective_dt = max(
            float(RESCUER_GROUND_FOLLOW_MIN_DT_SEC),
            float(sim_dt),
        )
        max_step = (
            float(RESCUER_GROUND_FOLLOW_MAX_SPEED_M_S)
            * effective_dt
        )
        if abs(requested) <= float(RESCUER_GROUND_SNAP_TOLERANCE_M):
            applied = requested
        else:
            applied = float(np.clip(requested, -max_step, max_step))

        new_z = current_z + applied
        applied_ok = self._set_character_world_z(new_z)

        self._last_ground_hit = ground_hit
        self._last_stable_ground_hit = ground_hit
        self._last_ground_z = float(ground_hit.z)
        self._last_navigation_surface_z = float(ground_hit.navigation_z)
        self._last_desired_z = desired_z
        self._last_requested_z_correction = requested
        self._last_applied_z_correction = applied if applied_ok else 0.0

        if not applied_ok:
            self._last_foot_error_m = requested
            return False

        corrected_position = self._current_rescuer_position()
        corrected_z = (
            float(corrected_position[2])
            if corrected_position is not None
            else new_z
        )
        self._last_foot_error_m = corrected_z - desired_z

        if abs(requested) >= 0.75:
            sim_time = float(self._timeline.get_current_time())
            if (
                sim_time - self._last_large_ground_error_log_sim_time
                >= 1.0
            ):
                carb.log_warn(
                    "[RESCUER] 큰 지면 Z 오차를 단계 보정 중: "
                    f"current_z={current_z:.3f}, "
                    f"ground_z={ground_hit.z:.3f}, "
                    f"desired_z={desired_z:.3f}, "
                    f"requested={requested:+.3f}m, "
                    f"applied={applied:+.3f}m, "
                    f"prim={ground_hit.prim_path}"
                )
                self._last_large_ground_error_log_sim_time = sim_time
        return True

    def update_rescuer_navigation(self):
        """매 시뮬레이션 프레임 ROS, XY 추종, 실제 지면 Z를 갱신한다."""
        if self._ros_node is None or self.rescuer is None:
            return

        rclpy.spin_once(self._ros_node, timeout_sec=0.0)

        # 경로가 없을 때도 실제 현재 pose는 계속 발행한다.
        if not self._rescuer_path or self._rescuer_status == "ARRIVED":
            self._publish_rescuer_pose_if_due()
            return

        position = self._current_rescuer_position()
        if position is None:
            return

        sim_time = float(self._timeline.get_current_time())
        if self._last_navigation_sim_time is None:
            sim_dt = 0.0
        else:
            sim_dt = max(0.0, sim_time - self._last_navigation_sim_time)
        self._last_navigation_sim_time = sim_time

        target = self._rescuer_path[self._rescuer_waypoint_index]
        distance_xy = float(np.linalg.norm(target[:2] - position[:2]))

        # 현재 XY의 navigation surface 높이를 hint로 사용한다.
        current_navigation_z = self._navigation_height(
            float(position[0]),
            float(position[1]),
        )
        ground_hit = self._query_walkable_ground(
            float(position[0]),
            float(position[1]),
            navigation_hint_z=current_navigation_z,
            enforce_local_step=True,
        )

        self._last_move_command_applied = False
        if ground_hit is None:
            self._clear_ground_diagnostics()
            self._stop_rescuer_for_ground_loss(
                "현재 구조자 XY 아래에서 navigation surface와 일치하는 "
                "Terrain/등록 구조물 collision을 찾지 못함"
            )
            self._publish_rescuer_pose_if_due()
            return

        if not self._apply_ground_following(position, ground_hit, sim_dt):
            self._stop_rescuer_for_ground_loss(
                "보행 지면은 찾았지만 Animation Graph root Z를 적용하지 못함"
            )
            self._publish_rescuer_pose_if_due()
            return

        corrected_position = self._current_rescuer_position()
        if corrected_position is None:
            corrected_position = position
        self._refresh_current_xy_target(float(corrected_position[2]))

        self._log_navigation_diagnostics(
            sim_time=sim_time,
            position=corrected_position,
            target=target,
            distance_xy=distance_xy,
            sim_dt=sim_dt,
        )

        # 보정된 동일한 XYZ를 ROS와 physics proxy가 사용한다.
        self._publish_rescuer_pose_if_due()

        if distance_xy <= float(RESCUER_WAYPOINT_TOLERANCE_M):
            self._rescuer_waypoint_index += 1
            self._skip_reached_waypoints()
            self._send_current_waypoint()

    def _clear_ground_diagnostics(self):
        self._last_ground_hit = None
        self._last_ground_z = None
        self._last_navigation_surface_z = None
        self._last_desired_z = None
        self._last_requested_z_correction = 0.0
        self._last_applied_z_correction = 0.0
        self._last_foot_error_m = None

    def _stop_rescuer_for_ground_loss(self, reason):
        position = self._current_rescuer_position()
        if position is not None:
            self.rescuer.update_target_position(
                position.tolist(),
                walk_speed=0.0,
            )
        self._last_move_command_applied = False

        reason = str(reason)
        entering_ground_lost = self._rescuer_status != "GROUND_LOST"
        reason_changed = reason != self._last_ground_loss_reason
        self._set_rescuer_status("GROUND_LOST")

        # 같은 위치에서 매초 동일한 오류를 반복하지 않는다. 상태에 처음
        # 진입하거나 원인이 달라졌을 때만 기록하고, 지면을 회복하면 다시
        # 출력할 수 있도록 _refresh_current_xy_target()에서 초기화한다.
        if entering_ground_lost or reason_changed:
            carb.log_error(f"[RESCUER] 이동 정지: {reason}")
            self._last_ground_loss_log_at = time.monotonic()
            self._last_ground_loss_reason = reason

    @staticmethod
    def _format_optional(value):
        if value is None:
            return "NONE"
        try:
            value = float(value)
        except (TypeError, ValueError):
            return str(value)
        return f"{value:.3f}" if math.isfinite(value) else "NONE"

    def _log_navigation_diagnostics(
        self,
        sim_time,
        position,
        target,
        distance_xy,
        sim_dt,
    ):
        if sim_time - self._last_navigation_log_sim_time < 1.0:
            return
        self._last_navigation_log_sim_time = sim_time

        hit = self._last_ground_hit
        ground_prim = hit.prim_path if hit is not None else "NONE"
        ground_type = hit.surface_type if hit is not None else "NONE"

        print(
            "[RESCUER][GROUND] "
            f"current=({position[0]:.3f},{position[1]:.3f},"
            f"{position[2]:.3f}), "
            f"current_z={position[2]:.3f}, "
            f"ground_prim={ground_prim}, "
            f"ground_type={ground_type}, "
            f"ground_z={self._format_optional(self._last_ground_z)}, "
            f"navigation_z="
            f"{self._format_optional(self._last_navigation_surface_z)}, "
            f"desired_z={self._format_optional(self._last_desired_z)}, "
            f"requested_z="
            f"{self._last_requested_z_correction:+.3f}m, "
            f"applied_z={self._last_applied_z_correction:+.3f}m, "
            f"foot_error="
            f"{self._format_optional(self._last_foot_error_m)}m, "
            f"waypoint=({target[0]:.3f},{target[1]:.3f},"
            f"{target[2]:.3f}), "
            f"waypoint_navigation_z="
            f"{self._format_optional(self._last_waypoint_surface_z)}, "
            f"waypoint_raycast_z="
            f"{self._format_optional(self._last_waypoint_raycast_z)}, "
            f"waypoint_ground_prim={self._last_waypoint_ground_prim}, "
            f"distance_xy={distance_xy:.3f}m, "
            f"index={self._rescuer_waypoint_index + 1}/"
            f"{len(self._rescuer_path)}, "
            f"move_command={self._last_move_command_applied}, "
            f"sim_dt={sim_dt:.4f}s"
        )

    # ------------------------------------------------------------------
    # 구조자 발 오프셋 및 상태/pose
    # ------------------------------------------------------------------
    def _measure_rescuer_foot_offset(self):
        """Character visual 최저점과 root state 원점 사이 오프셋을 측정한다."""
        try:
            stage = omni.usd.get_context().get_stage()
            root_path = str(self.rescuer._stage_prefix)
            root_prim = stage.GetPrimAtPath(root_path)
            bbox_cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
            )
            world_range = (
                bbox_cache.ComputeWorldBound(root_prim).ComputeAlignedRange()
            )
            visual_min_z = float(world_range.GetMin()[2])
            state_z = float(self.rescuer.state.position[2])
            measured = state_z - visual_min_z
            if math.isfinite(measured) and -1.0 <= measured <= 1.0:
                self._rescuer_foot_offset_m = (
                    measured + float(PERSON_GROUND_CLEARANCE_M)
                )
            print(
                "[RESCUER] 발 기준 측정: "
                f"root={root_path}, state_z={state_z:.3f}, "
                f"visual_min_z={visual_min_z:.3f}, "
                f"foot_offset={self._rescuer_foot_offset_m:.3f}m"
            )
        except Exception as error:
            carb.log_warn(
                "[RESCUER] visual 발바닥 높이 측정 실패, 기본 여유 높이 사용: "
                f"{type(error).__name__}: {error}"
            )

    def _log_rescuer_xform_ops_once(self):
        """Character Prim의 기존 xformOp를 변경하지 않고 기록만 한다."""
        if self._rescuer_xform_ops_logged or self.rescuer is None:
            return
        self._rescuer_xform_ops_logged = True

        root_path = str(self.rescuer._stage_prefix)
        stage = omni.usd.get_context().get_stage()
        root_prim = stage.GetPrimAtPath(root_path)
        if not root_prim.IsValid():
            carb.log_warn(
                f"[RESCUER] Character root Prim을 찾지 못했습니다: {root_path}"
            )
            return

        try:
            xformable = UsdGeom.Xformable(root_prim)
            ordered_ops = xformable.GetOrderedXformOps()
            op_names = [
                str(op.GetAttr().GetName())
                for op in ordered_ops
            ]
            print(
                "[RESCUER] 기존 Character xformOp 유지: "
                f"root={root_path}, ordered_ops={op_names}"
            )
        except Exception as error:
            carb.log_warn(
                "[RESCUER] Character xformOp 확인 실패: "
                f"{type(error).__name__}: {error}"
            )

    def _publish_rescuer_pose_if_due(self):
        now = time.monotonic()
        if now - self._last_pose_publish_at < float(
            RESCUER_POSE_PUBLISH_PERIOD_SEC
        ):
            return
        position = self._current_rescuer_position()
        if position is None:
            return

        message = PointStamped()
        message.header.frame_id = "map"
        message.header.stamp = self._ros_node.get_clock().now().to_msg()
        message.point.x = float(position[0])
        message.point.y = float(position[1])
        message.point.z = float(position[2])
        self._pose_publisher.publish(message)
        self._last_pose_publish_at = now

    def _set_rescuer_status(self, status):
        status = str(status)
        if status == self._rescuer_status:
            return
        self._rescuer_status = status
        if self._status_publisher is not None:
            message = String()
            message.data = status
            self._status_publisher.publish(message)
        print(f"[RESCUER] 상태: {status}")

    def shutdown_ros(self):
        if self._ros_node is not None:
            self._ros_node.destroy_node()
            self._ros_node = None
        if self._owns_rclpy_context and rclpy.ok():
            rclpy.shutdown()
        self._owns_rclpy_context = False

    # ------------------------------------------------------------------
    # 사람용 kinematic collision proxy
    # ------------------------------------------------------------------
    def _create_person_physics_proxy(
        self,
        person_name,
        person,
        foot_position,
    ):
        """사람을 따라 움직이는 보이지 않는 캡슐형 충돌체를 만든다."""
        stage = omni.usd.get_context().get_stage()
        collider_path = f"/World/person_colliders/{person_name}"

        capsule = UsdGeom.Capsule.Define(stage, collider_path)
        capsule.CreateAxisAttr().Set(UsdGeom.Tokens.z)
        capsule.CreateRadiusAttr().Set(PERSON_COLLIDER_RADIUS_M)
        capsule.CreateHeightAttr().Set(PERSON_COLLIDER_CYLINDER_HEIGHT_M)

        capsule_center = Gf.Vec3d(
            float(foot_position[0]),
            float(foot_position[1]),
            float(foot_position[2])
            + PERSON_COLLIDER_TOTAL_HEIGHT_M * 0.5,
        )
        translate_op = capsule.AddTranslateOp()
        translate_op.Set(capsule_center)

        capsule.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        collider_prim = capsule.GetPrim()
        UsdPhysics.CollisionAPI.Apply(collider_prim)

        rigid_body = UsdPhysics.RigidBodyAPI.Apply(collider_prim)
        rigid_body.CreateRigidBodyEnabledAttr(True)
        rigid_body.CreateKinematicEnabledAttr(True)

        self._person_physics_proxies[person_name] = {
            "person": person,
            "translate_op": translate_op,
            "collider_path": collider_path,
        }

        print(
            f"[INFO] Physics proxy created: {collider_path}, "
            f"foot={foot_position}, center={tuple(capsule_center)}"
        )

    def sync_person_physics_proxies(self):
        """보정된 Person World 위치로 물리 충돌체를 이동한다."""
        for person_name, proxy in self._person_physics_proxies.items():
            person = proxy["person"]

            # 구조자는 Animation Graph root에서 읽어 Person state와 동기화한
            # 값이 우선이다. 조난자는 기존 state 위치를 사용한다.
            if person is self.rescuer:
                person_position = self._current_rescuer_position()
            else:
                person_position = np.asarray(
                    person.state.position,
                    dtype=np.float64,
                )

            if (
                person_position is None
                or person_position.shape != (3,)
                or not np.all(np.isfinite(person_position))
            ):
                carb.log_warn(
                    f"Invalid person position for {person_name}: "
                    f"{person_position}"
                )
                continue

            capsule_center = Gf.Vec3d(
                float(person_position[0]),
                float(person_position[1]),
                float(person_position[2])
                + PERSON_COLLIDER_TOTAL_HEIGHT_M * 0.5,
            )
            proxy["translate_op"].Set(capsule_center)

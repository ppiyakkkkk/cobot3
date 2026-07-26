#!/usr/bin/env python3
"""USD Terrain 높이 분석과 RViz용 실제 환경 Mesh 추출.

이 파일에는 정점 변환, 보간, 삼각분할처럼 코드가 길지만 서로 밀접한
지형 처리 로직만 모았다. 일반 실행 흐름에서는 직접 호출하지 않고
``final_24.py``가 TerrainHeightField와 EnvironmentMeshExporter를 사용한다.
"""

import math
from pathlib import Path

import carb
import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade
from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator

from sim_config import (
    NAVIGATION_STRUCTURE_ALIASES,
    NAVIGATION_STRUCTURE_EXPLICIT_PRIM_PATHS,
    NAVIGATION_STRUCTURE_EXPLICIT_XY_MARGIN_M,
    NAVIGATION_STRUCTURE_XY_MARGIN_M,
    RVIZ_ENVIRONMENT_GROUPS,
    RVIZ_ENVIRONMENT_MAX_TRIANGLES_PER_GROUP,
    RVIZ_RIVER_AUTO_COLOR_CLASSIFICATION,
    RVIZ_RIVER_AUTO_MAX_THICKNESS_RATIO,
    RVIZ_RIVER_AUTO_MAX_VERTICAL_THICKNESS_M,
    RVIZ_RIVER_AUTO_MIN_BLUE,
    RVIZ_RIVER_AUTO_MIN_BLUE_MINUS_RED,
    RVIZ_RIVER_AUTO_MIN_COLOR_RANGE,
    RVIZ_RIVER_AUTO_MIN_HORIZONTAL_SPAN_M,
    RVIZ_RIVER_EXPLICIT_PRIM_PATHS,
)


class TerrainHeightField:
    def __init__(self, stage):
        self._stage = stage
        self._terrain_prim = self._find_terrain_mesh()
        self._build_interpolator()
        self._navigation_structures = []
        self._build_navigation_structures()

    def _find_terrain_mesh(self):
        meshes = [
            prim
            for prim in self._stage.Traverse()
            if prim.IsA(UsdGeom.Mesh)
        ]
        if not meshes:
            raise RuntimeError("Stage에서 Mesh Prim을 찾지 못했습니다.")

        named_candidates = [
            prim
            for prim in meshes
            if "terrain" in prim.GetName().lower()
            or "terrain" in str(prim.GetPath()).lower()
        ]

        candidates = named_candidates if named_candidates else meshes

        def point_count(prim):
            points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            return len(points) if points is not None else 0

        terrain_prim = max(candidates, key=point_count)
        if not named_candidates:
            carb.log_warn(
                "이름에 Terrain이 들어간 Mesh를 찾지 못해 가장 큰 Mesh를 "
                f"지형으로 사용합니다: {terrain_prim.GetPath()}"
            )

        print(f"[TERRAIN] Mesh Prim: {terrain_prim.GetPath()}")
        return terrain_prim

    def _build_interpolator(self):
        mesh = UsdGeom.Mesh(self._terrain_prim)
        local_points = mesh.GetPointsAttr().Get()
        if local_points is None or len(local_points) < 3:
            raise RuntimeError(
                f"Terrain Mesh 정점이 부족합니다: {self._terrain_prim.GetPath()}"
            )

        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        world_matrix = xform_cache.GetLocalToWorldTransform(self._terrain_prim)

        world_points = np.asarray(
            [
                tuple(
                    world_matrix.Transform(
                        Gf.Vec3d(
                            float(point[0]),
                            float(point[1]),
                            float(point[2]),
                        )
                    )
                )
                for point in local_points
            ],
            dtype=np.float64,
        )

        xy = world_points[:, :2]
        z = world_points[:, 2]

        rounded_xy = np.round(xy, decimals=5)
        _, unique_indices = np.unique(
            rounded_xy,
            axis=0,
            return_index=True,
        )
        xy = xy[unique_indices]
        z = z[unique_indices]

        if len(xy) < 3:
            raise RuntimeError("Terrain 높이 보간에 사용할 정점이 부족합니다.")

        self._linear = LinearNDInterpolator(xy, z, fill_value=np.nan)
        self._nearest = NearestNDInterpolator(xy, z)

        self.x_min = float(np.min(xy[:, 0]))
        self.x_max = float(np.max(xy[:, 0]))
        self.y_min = float(np.min(xy[:, 1]))
        self.y_max = float(np.max(xy[:, 1]))
        self.z_min = float(np.min(z))
        self.z_max = float(np.max(z))

        print(
            "[TERRAIN] XY bounds: "
            f"X=({self.x_min:.2f}, {self.x_max:.2f}), "
            f"Y=({self.y_min:.2f}, {self.y_max:.2f})"
        )
        print(
            "[TERRAIN] Z bounds: "
            f"Z=({self.z_min:.2f}, {self.z_max:.2f})"
        )

    def height(self, x, y):
        if not (
            self.x_min <= x <= self.x_max
            and self.y_min <= y <= self.y_max
        ):
            raise ValueError(
                f"좌표가 Terrain 범위를 벗어났습니다: X={x:.2f}, Y={y:.2f}"
            )

        value = self._linear(float(x), float(y))
        value = float(np.asarray(value))
        if not np.isfinite(value):
            value = float(self._nearest(float(x), float(y)))
        if not np.isfinite(value):
            raise RuntimeError(
                f"Terrain 높이를 계산하지 못했습니다: X={x:.2f}, Y={y:.2f}"
            )
        return value

    @staticmethod
    def _normalize_name(value):
        return "".join(
            character.lower()
            for character in str(value)
            if character.isalnum()
        )

    @staticmethod
    def _paths_overlap(first, second):
        """두 Prim 경로가 동일하거나 부모·자식 관계인지 확인한다."""
        first = str(first).rstrip("/")
        second = str(second).rstrip("/")
        if not first or not second:
            return False
        return (
            first == second
            or first.startswith(f"{second}/")
            or second.startswith(f"{first}/")
        )

    @staticmethod
    def _has_collision_api_on_path(prim):
        """현재 Prim 또는 부모 Prim에 Collision API가 있는지 검사한다."""
        current = prim
        while current and current.IsValid():
            try:
                if current.HasAPI(UsdPhysics.CollisionAPI):
                    return True
            except Exception:
                pass
            parent = current.GetParent()
            if not parent or parent == current:
                break
            current = parent
        return False

    def _build_navigation_structures(self):
        """다리와 명시적 스폰 판의 World AABB 및 상단 높이를 수집한다.

        다리는 기존처럼 이름 별칭으로 찾는다. 이름이 일반적인 회색 Cube는
        정확한 Prim 경로로만 등록해 다른 Cube가 보행 지면으로 오인되지 않게
        한다. 구조물마다 별도의 XY margin을 저장하므로 다리 접근부와 높은
        플랫폼 가장자리를 서로 다르게 처리할 수 있다.
        """
        aliases = tuple(
            self._normalize_name(value)
            for value in NAVIGATION_STRUCTURE_ALIASES
            if str(value).strip()
        )
        explicit_paths = tuple(
            str(value).strip().rstrip("/")
            for value in NAVIGATION_STRUCTURE_EXPLICIT_PRIM_PATHS
            if str(value).strip()
        )

        if not aliases and not explicit_paths:
            return

        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        unmatched_large_meshes = []
        matched_explicit_paths = set()

        for prim in self._stage.Traverse():
            if not prim.IsA(UsdGeom.Mesh):
                continue
            if prim == self._terrain_prim:
                continue

            mesh = UsdGeom.Mesh(prim)
            local_points = mesh.GetPointsAttr().Get()
            if local_points is None or len(local_points) < 3:
                continue

            prim_path = str(prim.GetPath())
            path_text = self._normalize_name(prim_path)
            matched_explicit_path = next(
                (
                    configured_path
                    for configured_path in explicit_paths
                    if self._paths_overlap(prim_path, configured_path)
                ),
                None,
            )
            matched_alias = any(alias in path_text for alias in aliases)
            if matched_explicit_path is None and not matched_alias:
                if len(local_points) >= 100:
                    unmatched_large_meshes.append(
                        (len(local_points), prim_path)
                    )
                continue

            imageable = UsdGeom.Imageable(prim)
            if imageable and (
                imageable.ComputeVisibility() == UsdGeom.Tokens.invisible
                or imageable.ComputePurpose() == UsdGeom.Tokens.proxy
            ):
                continue

            world_matrix = xform_cache.GetLocalToWorldTransform(prim)
            world_points = np.asarray(
                [
                    tuple(
                        world_matrix.Transform(
                            Gf.Vec3d(
                                float(point[0]),
                                float(point[1]),
                                float(point[2]),
                            )
                        )
                    )
                    for point in local_points
                ],
                dtype=np.float64,
            )
            finite = np.all(np.isfinite(world_points), axis=1)
            world_points = world_points[finite]
            if len(world_points) < 3:
                continue

            source_type = (
                "explicit"
                if matched_explicit_path is not None
                else "alias"
            )
            xy_margin_m = (
                float(NAVIGATION_STRUCTURE_EXPLICIT_XY_MARGIN_M)
                if source_type == "explicit"
                else float(NAVIGATION_STRUCTURE_XY_MARGIN_M)
            )
            structure = {
                "path": prim_path,
                "configured_path": (
                    str(matched_explicit_path)
                    if matched_explicit_path is not None
                    else ""
                ),
                "source_type": source_type,
                "xy_margin_m": max(0.0, xy_margin_m),
                "has_collision_api": self._has_collision_api_on_path(prim),
                "x_min": float(np.min(world_points[:, 0])),
                "x_max": float(np.max(world_points[:, 0])),
                "y_min": float(np.min(world_points[:, 1])),
                "y_max": float(np.max(world_points[:, 1])),
                "z_max": float(np.max(world_points[:, 2])),
            }
            self._navigation_structures.append(structure)

            if matched_explicit_path is not None:
                matched_explicit_paths.add(str(matched_explicit_path))

            print(
                "[NAV STRUCTURE] "
                f"type={source_type}, path={structure['path']}, "
                f"configured={structure['configured_path'] or 'ALIAS'}, "
                f"XY=({structure['x_min']:.2f}, {structure['x_max']:.2f}, "
                f"{structure['y_min']:.2f}, {structure['y_max']:.2f}), "
                f"top_Z={structure['z_max']:.2f}, "
                f"margin={structure['xy_margin_m']:.2f}m, "
                f"collision_api={structure['has_collision_api']}"
            )

            if source_type == "explicit" and not structure["has_collision_api"]:
                carb.log_warn(
                    "명시적으로 등록한 스폰 판에서 Collision API를 확인하지 "
                    "못했습니다. 실제 Collider가 자식 Prim에 따로 있다면 "
                    "ground raycast 로그도 함께 확인하세요: "
                    f"{structure['path']}"
                )

        missing_explicit_paths = [
            configured_path
            for configured_path in explicit_paths
            if configured_path not in matched_explicit_paths
        ]
        if missing_explicit_paths:
            carb.log_warn(
                "명시적으로 등록한 navigation structure를 Stage에서 찾지 "
                f"못했습니다: {missing_explicit_paths}"
            )

        if self._navigation_structures:
            print(
                "[NAV STRUCTURE] 경로계획에 반영할 구조물 수: "
                f"{len(self._navigation_structures)}"
            )
            return

        unmatched_large_meshes.sort(reverse=True)
        if unmatched_large_meshes:
            candidate_text = ", ".join(
                f"{path}(points={count})"
                for count, path in unmatched_large_meshes[:12]
            )
            carb.log_warn(
                "다리 또는 명시적 스폰 판을 찾지 못했습니다. "
                "Prim 이름이나 경로가 바뀌었다면 sim_config.py의 "
                "NAVIGATION_STRUCTURE_ALIASES 또는 "
                "NAVIGATION_STRUCTURE_EXPLICIT_PRIM_PATHS를 수정하세요. "
                f"큰 Mesh 후보: {candidate_text}"
            )

    def navigation_height(self, x, y):
        """현재 XY에서 실제로 걸어야 할 navigation surface 높이를 반환한다.

        다리는 Terrain과 일부 겹칠 수 있으므로 기존처럼 더 높은 표면을
        사용한다. 정확한 Prim 경로로 등록한 스폰 플랫폼은 그 자체가 실제
        보행 바닥이므로, 플랫폼 AABB 안에서는 아래 Terrain 보간값보다
        플랫폼의 PhysX 상단 높이를 우선한다.
        """
        x = float(x)
        y = float(y)
        terrain_z = float(self.height(x, y))

        explicit_heights = []
        alias_heights = []
        for structure in self._navigation_structures:
            margin = max(
                0.0,
                float(
                    structure.get(
                        "xy_margin_m",
                        NAVIGATION_STRUCTURE_XY_MARGIN_M,
                    )
                ),
            )
            if not (
                float(structure["x_min"]) - margin
                <= x
                <= float(structure["x_max"]) + margin
                and float(structure["y_min"]) - margin
                <= y
                <= float(structure["y_max"]) + margin
            ):
                continue

            structure_z = float(structure["z_max"])
            if structure.get("source_type") == "explicit":
                explicit_heights.append(structure_z)
            else:
                alias_heights.append(structure_z)

        # 명시적 플랫폼이 겹치는 영역에서는 Terrain과 max()를 취하지 않는다.
        # 로그에서 판 PhysX 상단은 30.500m인데 Terrain 보간이 약 31.0m로
        # 더 높게 계산되어 경로 Z가 실제 판보다 0.5m 높아졌던 문제를 막는다.
        if explicit_heights:
            return max(explicit_heights)

        result = terrain_z
        if alias_heights:
            result = max(result, max(alias_heights))
        return result

    def _log_navigation_structure_edges(
        self,
        x_values,
        y_values,
        z_grid,
    ):
        """구조물 상단과 바깥 셀 사이의 최소·최대 높이차를 출력한다."""
        grid_x, grid_y = np.meshgrid(x_values, y_values)
        row_count, column_count = z_grid.shape
        neighbor_offsets = ((-1, 0), (1, 0), (0, -1), (0, 1))

        for structure in self._navigation_structures:
            inside = (
                (grid_x >= float(structure["x_min"]))
                & (grid_x <= float(structure["x_max"]))
                & (grid_y >= float(structure["y_min"]))
                & (grid_y <= float(structure["y_max"]))
            )
            rows, columns = np.where(inside)
            edge_height_changes = []

            for row, column in zip(rows, columns):
                for row_offset, column_offset in neighbor_offsets:
                    neighbor_row = int(row + row_offset)
                    neighbor_column = int(column + column_offset)
                    if not (
                        0 <= neighbor_row < row_count
                        and 0 <= neighbor_column < column_count
                    ):
                        continue
                    if inside[neighbor_row, neighbor_column]:
                        continue
                    edge_height_changes.append(
                        abs(
                            float(z_grid[row, column])
                            - float(z_grid[neighbor_row, neighbor_column])
                        )
                    )

            if not edge_height_changes:
                carb.log_warn(
                    "[NAV STRUCTURE EDGE] 구조물 경계의 높이차를 계산하지 "
                    f"못했습니다: path={structure['path']}"
                )
                continue

            print(
                "[NAV STRUCTURE EDGE] "
                f"type={structure.get('source_type', 'alias')}, "
                f"path={structure['path']}, "
                f"top_Z={float(structure['z_max']):.3f}, "
                f"samples={len(edge_height_changes)}, "
                f"min_step={min(edge_height_changes):.3f}m, "
                f"max_step={max(edge_height_changes):.3f}m. "
                "ROS 구조자 경로의 max_step_height_m보다 큰 연결은 "
                "통행 불가로 처리됩니다."
            )

    def write_navigation_surface(self, output_path, sample_spacing_m):
        """Terrain과 구조물 상단을 합친 경로계획 전용 규칙 격자를 저장한다."""
        spacing = max(0.25, float(sample_spacing_m))
        x_count = max(
            2,
            int(math.ceil((self.x_max - self.x_min) / spacing)) + 1,
        )
        y_count = max(
            2,
            int(math.ceil((self.y_max - self.y_min) / spacing)) + 1,
        )
        x_values = np.linspace(self.x_min, self.x_max, x_count)
        y_values = np.linspace(self.y_min, self.y_max, y_count)

        vertices = np.empty((x_count * y_count, 3), dtype=np.float32)
        vertex_index = 0
        for world_y in y_values:
            for world_x in x_values:
                vertices[vertex_index] = (
                    float(world_x),
                    float(world_y),
                    float(self.navigation_height(world_x, world_y)),
                )
                vertex_index += 1

        z_grid = vertices[:, 2].reshape(y_count, x_count)
        self._log_navigation_structure_edges(
            x_values,
            y_values,
            z_grid,
        )

        triangle_count = (x_count - 1) * (y_count - 1) * 2
        triangles = np.empty((triangle_count, 3), dtype=np.int32)
        triangle_index = 0
        for row in range(y_count - 1):
            for column in range(x_count - 1):
                lower_left = row * x_count + column
                lower_right = lower_left + 1
                upper_left = (row + 1) * x_count + column
                upper_right = upper_left + 1
                triangles[triangle_index] = (
                    lower_left,
                    lower_right,
                    upper_right,
                )
                triangle_index += 1
                triangles[triangle_index] = (
                    lower_left,
                    upper_right,
                    upper_left,
                )
                triangle_index += 1

        structure_bounds = np.asarray(
            [
                [
                    structure["x_min"],
                    structure["x_max"],
                    structure["y_min"],
                    structure["y_max"],
                ]
                for structure in self._navigation_structures
            ],
            dtype=np.float32,
        ).reshape(-1, 4)
        structure_top_z = np.asarray(
            [
                structure["z_max"]
                for structure in self._navigation_structures
            ],
            dtype=np.float32,
        )
        structure_margins = np.asarray(
            [
                structure.get("xy_margin_m", 0.0)
                for structure in self._navigation_structures
            ],
            dtype=np.float32,
        )

        output_path = Path(output_path)
        temporary_path = output_path.with_suffix(".npz.tmp")
        with temporary_path.open("wb") as file_handle:
            np.savez_compressed(
                file_handle,
                vertices=vertices,
                triangles=triangles,
                map_frame=np.asarray(["map"]),
                coordinate_convention=np.asarray(["world_enu"]),
                sample_spacing_m=np.asarray([spacing], dtype=np.float32),
                navigation_structure_count=np.asarray(
                    [len(self._navigation_structures)],
                    dtype=np.int32,
                ),
                navigation_structure_paths=np.asarray(
                    [
                        structure["path"]
                        for structure in self._navigation_structures
                    ],
                    dtype=str,
                ),
                navigation_structure_configured_paths=np.asarray(
                    [
                        structure.get("configured_path", "")
                        for structure in self._navigation_structures
                    ],
                    dtype=str,
                ),
                navigation_structure_source_types=np.asarray(
                    [
                        structure.get("source_type", "alias")
                        for structure in self._navigation_structures
                    ],
                    dtype=str,
                ),
                navigation_structure_bounds_xy=structure_bounds,
                navigation_structure_top_z=structure_top_z,
                navigation_structure_xy_margin_m=structure_margins,
                navigation_structure_collision_api=np.asarray(
                    [
                        bool(structure.get("has_collision_api", False))
                        for structure in self._navigation_structures
                    ],
                    dtype=np.bool_,
                ),
            )
        temporary_path.replace(output_path)
        print(
            "[INFO] 경로계획 Navigation Surface 저장: "
            f"{output_path}, vertices={len(vertices)}, "
            f"structures={len(self._navigation_structures)}, "
            f"spacing={spacing:.2f}m"
        )

    def write_rviz_terrain_mesh(self, output_path, sample_spacing_m):
        """실제 USD Terrain 높이로 RViz용 삼각형 표면을 저장한다."""
        spacing = max(0.25, float(sample_spacing_m))

        # 양 끝 경계를 반드시 포함하는 규칙 격자를 만든다.
        x_count = max(
            2,
            int(math.ceil((self.x_max - self.x_min) / spacing)) + 1,
        )
        y_count = max(
            2,
            int(math.ceil((self.y_max - self.y_min) / spacing)) + 1,
        )
        x_values = np.linspace(self.x_min, self.x_max, x_count)
        y_values = np.linspace(self.y_min, self.y_max, y_count)

        vertices = np.empty(
            (x_count * y_count, 3),
            dtype=np.float32,
        )

        vertex_index = 0
        for world_y in y_values:
            for world_x in x_values:
                vertices[vertex_index] = (
                    float(world_x),
                    float(world_y),
                    float(self.height(world_x, world_y)),
                )
                vertex_index += 1

        # 각 격자 셀을 위쪽에서 보았을 때 반시계 방향인 삼각형 2개로 만든다.
        triangle_count = (x_count - 1) * (y_count - 1) * 2
        triangles = np.empty((triangle_count, 3), dtype=np.int32)
        triangle_index = 0

        for row in range(y_count - 1):
            for column in range(x_count - 1):
                lower_left = row * x_count + column
                lower_right = lower_left + 1
                upper_left = (row + 1) * x_count + column
                upper_right = upper_left + 1

                triangles[triangle_index] = (
                    lower_left,
                    lower_right,
                    upper_right,
                )
                triangle_index += 1
                triangles[triangle_index] = (
                    lower_left,
                    upper_right,
                    upper_left,
                )
                triangle_index += 1

        output_path = Path(output_path)
        temporary_path = output_path.with_suffix(".npz.tmp")
        with temporary_path.open("wb") as file_handle:
            np.savez_compressed(
                file_handle,
                vertices=vertices,
                triangles=triangles,
                map_frame=np.asarray(["map"]),
                coordinate_convention=np.asarray(["world_enu"]),
                source_prim=np.asarray([str(self._terrain_prim.GetPath())]),
                sample_spacing_m=np.asarray([spacing], dtype=np.float32),
                bounds=np.asarray(
                    [
                        self.x_min,
                        self.x_max,
                        self.y_min,
                        self.y_max,
                        self.z_min,
                        self.z_max,
                    ],
                    dtype=np.float32,
                ),
            )
        temporary_path.replace(output_path)

        print(
            "[INFO] RViz Terrain Mesh 저장: "
            f"{output_path}, vertices={len(vertices)}, "
            f"triangles={len(triangles)}, spacing={spacing:.2f}m"
        )

    def random_surface_position(
        self,
        rng,
        x_range,
        y_range,
        max_slope_deg,
        attempts=500,
    ):
        x_low = max(float(x_range[0]), self.x_min)
        x_high = min(float(x_range[1]), self.x_max)
        y_low = max(float(y_range[0]), self.y_min)
        y_high = min(float(y_range[1]), self.y_max)

        if x_low >= x_high or y_low >= y_high:
            raise RuntimeError(
                "PERSON_INCLUDE 영역과 Terrain 영역이 겹치지 않습니다."
            )

        for _ in range(attempts):
            x = float(rng.uniform(x_low, x_high))
            y = float(rng.uniform(y_low, y_high))
            z = self.height(x, y)
            return np.array([x, y, z], dtype=np.float64)

        raise RuntimeError(
            "산 표면 위에 사람을 생성할 수 있는 위치를 찾지 못했습니다."
        )


class EnvironmentMeshExporter:
    """지정된 USD 그룹 아래의 실제 Mesh를 RViz용으로 추출한다."""

    def __init__(self, stage):
        self._stage = stage
        self._xform_cache = UsdGeom.XformCache(
            Usd.TimeCode.Default()
        )

    @staticmethod
    def _normalize_name(value):
        """공백·언더바·대소문자를 무시할 수 있는 이름으로 바꾼다."""
        return "".join(
            character.lower()
            for character in str(value)
            if character.isalnum()
        )

    @staticmethod
    def _normalized_aliases(values):
        return tuple(
            EnvironmentMeshExporter._normalize_name(value)
            for value in values
            if str(value).strip()
        )

    @staticmethod
    def _color_triplet(value):
        """USD 색상 값을 평균 RGB 3개로 변환한다."""
        if value is None:
            return None
        try:
            array = np.asarray(value, dtype=np.float64)
        except (TypeError, ValueError):
            return None
        if array.size < 3:
            return None

        if array.ndim == 1:
            color = array[:3]
        else:
            if array.shape[-1] < 3:
                return None
            color = np.mean(array.reshape(-1, array.shape[-1])[:, :3], axis=0)

        if color.shape != (3,) or not np.all(np.isfinite(color)):
            return None
        return color

    def _bound_material(self, prim):
        """Prim에 최종 바인딩된 Material을 안전하게 반환한다."""
        try:
            material, _ = UsdShade.MaterialBindingAPI(
                prim
            ).ComputeBoundMaterial()
        except Exception:
            return None
        if not material:
            return None
        material_prim = material.GetPrim()
        if not material_prim or not material_prim.IsValid():
            return None
        return material

    def _material_matches_alias(self, prim, aliases):
        material = self._bound_material(prim)
        if material is None:
            return False

        material_text = self._normalize_name(str(material.GetPath()))
        if any(alias in material_text for alias in aliases):
            return True

        for child in Usd.PrimRange(material.GetPrim()):
            child_text = self._normalize_name(
                f"{child.GetName()} {child.GetTypeName()}"
            )
            if any(alias in child_text for alias in aliases):
                return True
        return False

    def _mesh_color_candidates(self, prim):
        """Mesh 표시색과 Material Shader의 색상 입력을 수집한다."""
        colors = []

        try:
            display_color = UsdGeom.PrimvarsAPI(prim).GetPrimvar(
                "displayColor"
            )
            if display_color and display_color.HasValue():
                color = self._color_triplet(display_color.Get())
                if color is not None:
                    colors.append(color)
        except Exception:
            pass

        material = self._bound_material(prim)
        if material is None:
            return colors

        color_input_tokens = (
            "basecolor",
            "diffusecolor",
            "albedo",
            "reflectioncolor",
            "watercolor",
            "tintcolor",
            "color",
        )
        for child in Usd.PrimRange(material.GetPrim()):
            if not child.IsA(UsdShade.Shader):
                continue
            shader = UsdShade.Shader(child)
            for shader_input in shader.GetInputs():
                input_name = self._normalize_name(
                    shader_input.GetBaseName()
                )
                if not any(
                    token in input_name for token in color_input_tokens
                ):
                    continue
                try:
                    value = shader_input.Get()
                except Exception:
                    continue
                color = self._color_triplet(value)
                if color is not None:
                    colors.append(color)
        return colors

    @staticmethod
    def _looks_like_river_color(color):
        red, green, blue = [float(value) for value in color]
        color_range = max(red, green, blue) - min(red, green, blue)
        return (
            blue >= float(RVIZ_RIVER_AUTO_MIN_BLUE)
            and blue - red
            >= float(RVIZ_RIVER_AUTO_MIN_BLUE_MINUS_RED)
            and color_range >= float(RVIZ_RIVER_AUTO_MIN_COLOR_RANGE)
            # 청록색 물도 허용하되 녹색이 파란색보다 지나치게 크면 제외한다.
            and green <= blue * 1.35
        )

    def _looks_like_broad_flat_water_mesh(self, prim):
        """파란색이며 넓고 평평한 Mesh인지 검사한다."""
        if not bool(RVIZ_RIVER_AUTO_COLOR_CLASSIFICATION):
            return False
        if not any(
            self._looks_like_river_color(color)
            for color in self._mesh_color_candidates(prim)
        ):
            return False

        local_points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        if local_points is None or len(local_points) < 3:
            return False

        world_matrix = self._xform_cache.GetLocalToWorldTransform(prim)
        world_points = np.asarray(
            [
                tuple(
                    world_matrix.Transform(
                        Gf.Vec3d(
                            float(point[0]),
                            float(point[1]),
                            float(point[2]),
                        )
                    )
                )
                for point in local_points
            ],
            dtype=np.float64,
        )
        finite = np.all(np.isfinite(world_points), axis=1)
        world_points = world_points[finite]
        if len(world_points) < 3:
            return False

        extents = np.ptp(world_points, axis=0)
        horizontal_span = float(max(extents[0], extents[1]))
        vertical_thickness = float(extents[2])
        if horizontal_span < float(RVIZ_RIVER_AUTO_MIN_HORIZONTAL_SPAN_M):
            return False

        thickness_limit = max(
            float(RVIZ_RIVER_AUTO_MAX_VERTICAL_THICKNESS_M),
            horizontal_span * float(RVIZ_RIVER_AUTO_MAX_THICKNESS_RATIO),
        )
        return vertical_thickness <= thickness_limit

    def _classify_mesh(self, prim):
        """Prim 경로·Material·색상과 형상으로 환경 그룹을 결정한다."""
        prim_path = str(prim.GetPath())
        explicit_river_paths = tuple(
            str(value).strip()
            for value in RVIZ_RIVER_EXPLICIT_PRIM_PATHS
            if str(value).strip()
        )
        if any(
            prim_path == path or prim_path.startswith(path.rstrip("/") + "/")
            for path in explicit_river_paths
        ):
            print(f"[RIVER] 명시적 Prim 경로로 분류: {prim_path}")
            return "river"

        normalized_segments = [
            self._normalize_name(segment)
            for segment in prim_path.split("/")
            if segment
        ]

        for category, raw_aliases in RVIZ_ENVIRONMENT_GROUPS.items():
            aliases = self._normalized_aliases(raw_aliases)
            for segment in normalized_segments:
                if any(
                    segment == alias or segment.startswith(alias)
                    for alias in aliases
                ):
                    if category == "bridges":
                        print(
                            "[BRIDGE] Prim 경로로 분류: "
                            f"{prim_path}"
                        )
                    return category
            # 강은 Prim 이름 대신 Material 이름에 Water/River가 들어간
            # 경우가 많으므로 바인딩 Material 경로도 함께 검사한다.
            if category == "river" and self._material_matches_alias(
                prim,
                aliases,
            ):
                print(f"[RIVER] Material 이름으로 분류: {prim_path}")
                return category

        # 이름과 Material 모두 일반적이어도 파란색의 넓고 평평한 Mesh면
        # 강 표면으로 자동 분류한다.
        if self._looks_like_broad_flat_water_mesh(prim):
            print(f"[RIVER AUTO] 파란 평면 Mesh로 분류: {prim_path}")
            return "river"

        return None

    @staticmethod
    def _triangulate_mesh(mesh):
        """USD 다각형 Face를 삼각형 인덱스로 변환한다."""
        counts = mesh.GetFaceVertexCountsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()

        if counts is None or indices is None:
            return np.empty((0, 3), dtype=np.int64)

        counts = [int(value) for value in counts]
        indices = [int(value) for value in indices]
        triangles = []
        offset = 0

        orientation = mesh.GetOrientationAttr().Get()
        left_handed = orientation == UsdGeom.Tokens.leftHanded

        for vertex_count in counts:
            face = indices[offset:offset + vertex_count]
            offset += vertex_count

            if vertex_count < 3 or len(face) != vertex_count:
                continue

            first = face[0]
            for index in range(1, vertex_count - 1):
                second = face[index]
                third = face[index + 1]
                if left_handed:
                    second, third = third, second
                triangles.append((first, second, third))

        if not triangles:
            return np.empty((0, 3), dtype=np.int64)
        return np.asarray(triangles, dtype=np.int64)

    def _extract_mesh(self, prim):
        """Mesh의 정점과 면을 World ENU 좌표로 변환한다."""
        mesh = UsdGeom.Mesh(prim)
        local_points = mesh.GetPointsAttr().Get()
        if local_points is None or len(local_points) < 3:
            return None

        imageable = UsdGeom.Imageable(prim)
        if imageable:
            visibility = imageable.ComputeVisibility()
            if visibility == UsdGeom.Tokens.invisible:
                return None

            purpose = imageable.ComputePurpose()
            if purpose == UsdGeom.Tokens.proxy:
                return None

        path_lower = str(prim.GetPath()).lower()
        if "collision" in path_lower or "collider" in path_lower:
            return None

        world_matrix = self._xform_cache.GetLocalToWorldTransform(prim)
        world_points = np.asarray(
            [
                tuple(
                    world_matrix.Transform(
                        Gf.Vec3d(
                            float(point[0]),
                            float(point[1]),
                            float(point[2]),
                        )
                    )
                )
                for point in local_points
            ],
            dtype=np.float32,
        )
        triangles = self._triangulate_mesh(mesh)

        if triangles.size == 0:
            return None
        if np.min(triangles) < 0 or np.max(triangles) >= len(world_points):
            carb.log_warn(
                "환경 Mesh 인덱스 범위가 잘못되어 건너뜁니다: "
                f"{prim.GetPath()}"
            )
            return None

        return world_points, triangles

    @staticmethod
    def _limit_and_compact(vertices, triangles, max_triangles):
        """삼각형 수를 제한한 뒤 사용하지 않는 정점을 제거한다."""
        original_triangle_count = len(triangles)

        if original_triangle_count > max_triangles:
            selected = np.linspace(
                0,
                original_triangle_count - 1,
                max_triangles,
                dtype=np.int64,
            )
            triangles = triangles[selected]

        used_vertices = np.unique(triangles.reshape(-1))
        remap = np.full(len(vertices), -1, dtype=np.int64)
        remap[used_vertices] = np.arange(
            len(used_vertices),
            dtype=np.int64,
        )

        compact_vertices = vertices[used_vertices]
        compact_triangles = remap[triangles].astype(np.int32)

        return (
            compact_vertices.astype(np.float32),
            compact_triangles,
            original_triangle_count,
        )

    def write(self, output_path):
        """식생·바위·강·다리 Mesh를 하나의 NPZ로 저장한다."""
        grouped_vertices = {
            category: []
            for category in RVIZ_ENVIRONMENT_GROUPS
        }
        grouped_triangles = {
            category: []
            for category in RVIZ_ENVIRONMENT_GROUPS
        }
        grouped_paths = {
            category: []
            for category in RVIZ_ENVIRONMENT_GROUPS
        }
        vertex_offsets = {
            category: 0
            for category in RVIZ_ENVIRONMENT_GROUPS
        }

        for prim in self._stage.Traverse():
            if not prim.IsA(UsdGeom.Mesh):
                continue

            category = self._classify_mesh(prim)
            if category is None:
                continue

            extracted = self._extract_mesh(prim)
            if extracted is None:
                continue

            vertices, triangles = extracted
            offset = vertex_offsets[category]
            grouped_vertices[category].append(vertices)
            grouped_triangles[category].append(triangles + offset)
            grouped_paths[category].append(str(prim.GetPath()))
            vertex_offsets[category] += len(vertices)

        payload = {
            "format_version": np.asarray([3], dtype=np.int32),
            "map_frame": np.asarray(["map"]),
            "coordinate_convention": np.asarray(["world_enu"]),
        }

        found_group_count = 0
        for category in RVIZ_ENVIRONMENT_GROUPS:
            if not grouped_vertices[category]:
                carb.log_warn(
                    f"RViz 환경 그룹 Mesh를 찾지 못했습니다: {category}"
                )
                payload[f"{category}_vertices"] = np.empty(
                    (0, 3),
                    dtype=np.float32,
                )
                payload[f"{category}_triangles"] = np.empty(
                    (0, 3),
                    dtype=np.int32,
                )
                payload[f"{category}_source_paths"] = np.asarray(
                    [],
                    dtype="U1",
                )
                payload[f"{category}_original_triangle_count"] = (
                    np.asarray([0], dtype=np.int64)
                )
                continue

            vertices = np.concatenate(
                grouped_vertices[category],
                axis=0,
            )
            triangles = np.concatenate(
                grouped_triangles[category],
                axis=0,
            )
            (
                vertices,
                triangles,
                original_triangle_count,
            ) = self._limit_and_compact(
                vertices,
                triangles,
                RVIZ_ENVIRONMENT_MAX_TRIANGLES_PER_GROUP,
            )

            payload[f"{category}_vertices"] = vertices
            payload[f"{category}_triangles"] = triangles
            payload[f"{category}_source_paths"] = np.asarray(
                grouped_paths[category],
                dtype=str,
            )
            payload[f"{category}_original_triangle_count"] = np.asarray(
                [original_triangle_count],
                dtype=np.int64,
            )
            found_group_count += 1

            print(
                f"[INFO] RViz {category}: "
                f"meshes={len(grouped_paths[category])}, "
                f"vertices={len(vertices)}, "
                f"triangles={len(triangles)}, "
                f"original_triangles={original_triangle_count}"
            )

        if found_group_count == 0:
            raise RuntimeError(
                "PineForest/BroadleafForest/Bushes/Rocks/River/Bridges에서 "
                "RViz용 Mesh를 하나도 찾지 못했습니다."
            )

        output_path = Path(output_path)
        temporary_path = output_path.with_suffix(".npz.tmp")
        with temporary_path.open("wb") as file_handle:
            np.savez_compressed(file_handle, **payload)
        temporary_path.replace(output_path)

        print(
            "[INFO] RViz 환경 그룹 Mesh 저장: "
            f"{output_path}, groups={found_group_count}"
        )

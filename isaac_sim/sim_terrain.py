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
from scipy import ndimage
from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator

from sim_config import (
    NAVIGATION_BRIDGE_ACCESS_HALF_WIDTH_M,
    NAVIGATION_BRIDGE_ACCESS_LENGTH_M,
    NAVIGATION_BRIDGE_CORE_EXPANSION_M,
    NAVIGATION_BRIDGE_DECK_NORMAL_Z_MIN,
    NAVIGATION_BRIDGE_LOCAL_STEP_M,
    NAVIGATION_BRIDGE_MIN_COMPONENT_CELLS,
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
        self._bridge_grid_x_values = None
        self._bridge_grid_y_values = None
        self._bridge_core_mask = None
        self._bridge_access_mask = None
        self._bridge_label_grid = None
        self._bridge_surface_z_grid = None
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


    @staticmethod
    def _triangulate_mesh_indices(mesh):
        """USD polygon face를 삼각형 인덱스로 바꾼다."""
        counts = mesh.GetFaceVertexCountsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        if counts is None or indices is None:
            return np.empty((0, 3), dtype=np.int64)

        counts = [int(value) for value in counts]
        indices = [int(value) for value in indices]
        triangles = []
        offset = 0
        left_handed = (
            mesh.GetOrientationAttr().Get() == UsdGeom.Tokens.leftHanded
        )
        for vertex_count in counts:
            face = indices[offset : offset + vertex_count]
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

    def _build_navigation_structures(self):
        """다리와 명시적 스폰 판의 World 형상을 수집한다.

        명시적 스폰 판은 정확한 Prim 경로와 AABB 상단을 사용한다. 다리는
        AABB 최고 Z를 보행 높이로 쓰지 않고, 실제 Mesh의 위쪽 삼각형을
        ``write_navigation_surface``에서 다시 분석할 수 있도록 World 정점과
        삼각형을 보존한다.
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
            finite_points = world_points[finite]
            if len(finite_points) < 3:
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
                "x_min": float(np.min(finite_points[:, 0])),
                "x_max": float(np.max(finite_points[:, 0])),
                "y_min": float(np.min(finite_points[:, 1])),
                "y_max": float(np.max(finite_points[:, 1])),
                "z_min": float(np.min(finite_points[:, 2])),
                "z_max": float(np.max(finite_points[:, 2])),
                "bridge_label": 0,
            }

            if source_type == "alias":
                triangles = self._triangulate_mesh_indices(mesh)
                if (
                    triangles.size
                    and np.max(triangles) < len(world_points)
                    and np.min(triangles) >= 0
                    and np.all(finite)
                ):
                    structure["bridge_triangles_xyz"] = world_points[
                        triangles
                    ]
                else:
                    structure["bridge_triangles_xyz"] = np.empty(
                        (0, 3, 3),
                        dtype=np.float64,
                    )

            self._navigation_structures.append(structure)

            if matched_explicit_path is not None:
                matched_explicit_paths.add(str(matched_explicit_path))

            print(
                "[NAV STRUCTURE] "
                f"type={source_type}, path={structure['path']}, "
                f"configured={structure['configured_path'] or 'ALIAS'}, "
                f"XY=({structure['x_min']:.2f}, {structure['x_max']:.2f}, "
                f"{structure['y_min']:.2f}, {structure['y_max']:.2f}), "
                f"Z=({structure['z_min']:.2f}, {structure['z_max']:.2f}), "
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

    @staticmethod
    def _rasterize_triangle_footprint(
        x_values,
        y_values,
        triangles_xyz,
    ):
        """삼각형 XY 투영이 덮는 규칙 격자 셀을 반환한다."""
        mask = np.zeros((len(y_values), len(x_values)), dtype=bool)
        triangles_xyz = np.asarray(triangles_xyz, dtype=np.float64)
        if triangles_xyz.size == 0:
            return mask

        for triangle in triangles_xyz:
            points = triangle[:, :2]
            if not np.all(np.isfinite(points)):
                continue
            x_min, y_min = np.min(points, axis=0)
            x_max, y_max = np.max(points, axis=0)
            column_min = max(
                0,
                int(np.searchsorted(x_values, x_min, side="left")) - 1,
            )
            column_max = min(
                len(x_values) - 1,
                int(np.searchsorted(x_values, x_max, side="right")),
            )
            row_min = max(
                0,
                int(np.searchsorted(y_values, y_min, side="left")) - 1,
            )
            row_max = min(
                len(y_values) - 1,
                int(np.searchsorted(y_values, y_max, side="right")),
            )
            if column_min > column_max or row_min > row_max:
                continue

            a, b, c = points
            v0 = c - a
            v1 = b - a
            denominator = v0[0] * v1[1] - v1[0] * v0[1]
            if abs(float(denominator)) < 1.0e-10:
                continue

            local_x = x_values[column_min : column_max + 1]
            local_y = y_values[row_min : row_max + 1]
            grid_x, grid_y = np.meshgrid(local_x, local_y)
            v2x = grid_x - a[0]
            v2y = grid_y - a[1]
            u = (v2x * v1[1] - v1[0] * v2y) / denominator
            v = (v0[0] * v2y - v2x * v0[1]) / denominator
            inside = (
                (u >= -1.0e-7)
                & (v >= -1.0e-7)
                & (u + v <= 1.0 + 1.0e-7)
            )
            mask[
                row_min : row_max + 1,
                column_min : column_max + 1,
            ] |= inside
        return mask

    @staticmethod
    def _bridge_surface_candidates(
        x_values,
        y_values,
        triangles_xyz,
    ):
        """위쪽을 향하는 삼각형의 셀별 Z 후보를 수집한다."""
        candidates = {}
        triangles_xyz = np.asarray(triangles_xyz, dtype=np.float64)
        if triangles_xyz.size == 0:
            return candidates

        edge1 = triangles_xyz[:, 1] - triangles_xyz[:, 0]
        edge2 = triangles_xyz[:, 2] - triangles_xyz[:, 0]
        normals = np.cross(edge1, edge2)
        normal_length = np.linalg.norm(normals, axis=1)
        valid = normal_length > 1.0e-9
        normal_z = np.zeros(len(triangles_xyz), dtype=np.float64)
        normal_z[valid] = np.abs(
            normals[valid, 2] / normal_length[valid]
        )
        selected_triangles = triangles_xyz[
            valid
            & (
                normal_z
                >= float(NAVIGATION_BRIDGE_DECK_NORMAL_Z_MIN)
            )
        ]

        for triangle in selected_triangles:
            points = triangle[:, :2]
            x_min, y_min = np.min(points, axis=0)
            x_max, y_max = np.max(points, axis=0)
            column_min = max(
                0,
                int(np.searchsorted(x_values, x_min, side="left")) - 1,
            )
            column_max = min(
                len(x_values) - 1,
                int(np.searchsorted(x_values, x_max, side="right")),
            )
            row_min = max(
                0,
                int(np.searchsorted(y_values, y_min, side="left")) - 1,
            )
            row_max = min(
                len(y_values) - 1,
                int(np.searchsorted(y_values, y_max, side="right")),
            )
            if column_min > column_max or row_min > row_max:
                continue

            a, b, c = points
            v0 = c - a
            v1 = b - a
            denominator = v0[0] * v1[1] - v1[0] * v0[1]
            if abs(float(denominator)) < 1.0e-10:
                continue

            local_x = x_values[column_min : column_max + 1]
            local_y = y_values[row_min : row_max + 1]
            grid_x, grid_y = np.meshgrid(local_x, local_y)
            v2x = grid_x - a[0]
            v2y = grid_y - a[1]
            u = (v2x * v1[1] - v1[0] * v2y) / denominator
            v = (v0[0] * v2y - v2x * v0[1]) / denominator
            inside = (
                (u >= -1.0e-7)
                & (v >= -1.0e-7)
                & (u + v <= 1.0 + 1.0e-7)
            )
            local_rows, local_columns = np.where(inside)
            for local_row, local_column in zip(
                local_rows,
                local_columns,
            ):
                row = row_min + int(local_row)
                column = column_min + int(local_column)
                local_u = float(u[local_row, local_column])
                local_v = float(v[local_row, local_column])
                z = (
                    float(triangle[0, 2])
                    + local_u
                    * float(triangle[2, 2] - triangle[0, 2])
                    + local_v
                    * float(triangle[1, 2] - triangle[0, 2])
                )
                if math.isfinite(z):
                    candidates.setdefault((row, column), []).append(z)

        # 같은 판자의 앞·뒤 면처럼 거의 같은 높이는 하나로 합친다.
        for cell, values in tuple(candidates.items()):
            unique = []
            for value in sorted(float(item) for item in values):
                if not unique or abs(value - unique[-1]) > 0.04:
                    unique.append(value)
            candidates[cell] = unique
        return candidates

    @staticmethod
    def _select_bridge_deck_component(candidates):
        """난간·기둥이 아닌 가장 긴 매끄러운 상판 성분을 고른다."""
        nodes = []
        cell_nodes = {}
        for cell, values in candidates.items():
            for value in values:
                node_index = len(nodes)
                nodes.append((cell, float(value)))
                cell_nodes.setdefault(cell, []).append(node_index)

        if not nodes:
            return {}

        adjacency = [[] for _ in nodes]
        for cell, indices in cell_nodes.items():
            # 같은 XY의 얇은 판자 위·아래 면은 같은 구조로 묶는다.
            for first_pos, first in enumerate(indices):
                for second in indices[first_pos + 1 :]:
                    if abs(nodes[first][1] - nodes[second][1]) <= 0.35:
                        adjacency[first].append(second)
                        adjacency[second].append(first)

            row, column = cell
            for row_offset in (-1, 0, 1):
                for column_offset in (-1, 0, 1):
                    if row_offset == 0 and column_offset == 0:
                        continue
                    neighbor = (
                        row + row_offset,
                        column + column_offset,
                    )
                    for first in indices:
                        for second in cell_nodes.get(neighbor, ()):
                            if (
                                abs(nodes[first][1] - nodes[second][1])
                                <= float(NAVIGATION_BRIDGE_LOCAL_STEP_M)
                            ):
                                adjacency[first].append(second)

        visited = set()
        components = []
        for start_index in range(len(nodes)):
            if start_index in visited:
                continue
            stack = [start_index]
            visited.add(start_index)
            component = []
            while stack:
                current = stack.pop()
                component.append(current)
                for neighbor in adjacency[current]:
                    if neighbor in visited:
                        continue
                    visited.add(neighbor)
                    stack.append(neighbor)

            unique_cells = {nodes[index][0] for index in component}
            if not unique_cells:
                continue
            rows = [cell[0] for cell in unique_cells]
            columns = [cell[1] for cell in unique_cells]
            span = math.hypot(
                max(rows) - min(rows),
                max(columns) - min(columns),
            )
            median_z = float(
                np.median([nodes[index][1] for index in component])
            )
            components.append(
                (
                    len(unique_cells),
                    span,
                    len(component),
                    median_z,
                    component,
                )
            )

        if not components:
            return {}

        # 먼저 가장 넓고 긴 성분을 고르고, 규모가 같을 때는 교각 아래면보다
        # 위쪽의 실제 상판을 선택하도록 median Z를 마지막 tie-break로 쓴다.
        components.sort(
            key=lambda item: (item[0], item[1], item[2], item[3]),
            reverse=True,
        )
        (
            unique_cell_count,
            _span,
            _node_count,
            _median_z,
            selected,
        ) = components[0]
        if unique_cell_count < int(NAVIGATION_BRIDGE_MIN_COMPONENT_CELLS):
            return {}

        deck = {}
        for node_index in selected:
            cell, value = nodes[node_index]
            # 같은 XY에서는 실제 보행 상단인 더 높은 면을 사용한다.
            deck[cell] = max(value, deck.get(cell, -math.inf))
        return deck

    @staticmethod
    def _bridge_endpoint_access_mask(
        core_mask,
        x_values,
        y_values,
    ):
        """상판 주축의 양 끝에만 짧은 접속 영역을 만든다."""
        access = np.zeros_like(core_mask, dtype=bool)
        rows, columns = np.where(core_mask)
        if len(rows) < 2:
            return access

        points = np.column_stack(
            (x_values[columns], y_values[rows])
        ).astype(np.float64)
        center = np.mean(points, axis=0)
        centered = points - center
        _u, _s, vh = np.linalg.svd(centered, full_matrices=False)
        axis = vh[0]
        axis_length = float(np.linalg.norm(axis))
        if axis_length <= 1.0e-9:
            return access
        axis = axis / axis_length

        projections = centered @ axis
        minimum = float(np.min(projections))
        maximum = float(np.max(projections))
        endpoint_band = max(
            0.75,
            0.15 * max(maximum - minimum, 1.0),
        )
        low_points = points[
            projections <= minimum + endpoint_band
        ]
        high_points = points[
            projections >= maximum - endpoint_band
        ]
        low_center = np.mean(low_points, axis=0)
        high_center = np.mean(high_points, axis=0)

        grid_x, grid_y = np.meshgrid(x_values, y_values)
        for endpoint, direction in (
            (low_center, -axis),
            (high_center, axis),
        ):
            delta_x = grid_x - float(endpoint[0])
            delta_y = grid_y - float(endpoint[1])
            along = delta_x * direction[0] + delta_y * direction[1]
            lateral = np.abs(
                -delta_x * direction[1]
                + delta_y * direction[0]
            )
            access |= (
                (along >= -0.25)
                & (
                    along
                    <= float(NAVIGATION_BRIDGE_ACCESS_LENGTH_M)
                )
                & (
                    lateral
                    <= float(NAVIGATION_BRIDGE_ACCESS_HALF_WIDTH_M)
                )
            )
        return access

    def _build_bridge_navigation_layers(
        self,
        x_values,
        y_values,
        base_z_grid,
    ):
        """각 다리를 상판·접속부로 분리해 격자 보행 레이어를 만든다."""
        shape = base_z_grid.shape
        core_mask = np.zeros(shape, dtype=bool)
        access_mask = np.zeros(shape, dtype=bool)
        label_grid = np.zeros(shape, dtype=np.int32)
        surface_z_grid = np.full(shape, np.nan, dtype=np.float64)

        spacing_x = float(np.median(np.diff(x_values)))
        spacing_y = float(np.median(np.diff(y_values)))
        mean_spacing = max(1.0e-6, 0.5 * (spacing_x + spacing_y))
        expansion_iterations = max(
            1,
            int(
                round(
                    float(NAVIGATION_BRIDGE_CORE_EXPANSION_M)
                    / mean_spacing
                )
            ),
        )

        bridge_label = 0
        for structure in self._navigation_structures:
            if structure.get("source_type") != "alias":
                continue
            triangles_xyz = np.asarray(
                structure.get(
                    "bridge_triangles_xyz",
                    np.empty((0, 3, 3)),
                ),
                dtype=np.float64,
            )
            candidates = self._bridge_surface_candidates(
                x_values,
                y_values,
                triangles_xyz,
            )
            deck_cells = self._select_bridge_deck_component(candidates)
            if not deck_cells:
                carb.log_warn(
                    "[BRIDGE DECK] 실제 상판 성분을 찾지 못했습니다: "
                    f"{structure['path']}"
                )
                continue

            bridge_label += 1
            raw_mask = np.zeros(shape, dtype=bool)
            raw_z = np.full(shape, np.nan, dtype=np.float64)
            for (row, column), z in deck_cells.items():
                raw_mask[row, column] = True
                raw_z[row, column] = float(z)

            footprint = self._rasterize_triangle_footprint(
                x_values,
                y_values,
                triangles_xyz,
            )
            footprint = ndimage.binary_dilation(
                footprint,
                iterations=1,
            )
            local_core = ndimage.binary_dilation(
                raw_mask,
                iterations=expansion_iterations,
            )
            local_core &= footprint

            # 확장된 셀은 가장 가까운 실제 상판 샘플 높이를 사용한다.
            _distance, nearest = ndimage.distance_transform_edt(
                ~raw_mask,
                return_indices=True,
            )
            nearest_z = raw_z[nearest[0], nearest[1]]
            local_surface_z = np.where(
                local_core,
                nearest_z,
                np.nan,
            )

            local_access = local_core | self._bridge_endpoint_access_mask(
                local_core,
                x_values,
                y_values,
            )

            # 다른 다리와 겹칠 가능성은 낮지만 먼저 등록된 상판을 유지한다.
            write_core = local_core & ~core_mask
            core_mask |= local_core
            access_mask |= local_access
            surface_z_grid[write_core] = local_surface_z[write_core]
            label_grid[local_access & (label_grid == 0)] = bridge_label

            structure["bridge_label"] = bridge_label
            structure["deck_cell_count"] = int(np.count_nonzero(raw_mask))
            structure["core_cell_count"] = int(np.count_nonzero(local_core))
            structure["access_cell_count"] = int(
                np.count_nonzero(local_access)
            )
            structure["deck_z_min"] = float(
                np.nanmin(local_surface_z[local_core])
            )
            structure["deck_z_max"] = float(
                np.nanmax(local_surface_z[local_core])
            )
            structure["deck_z_median"] = float(
                np.nanmedian(local_surface_z[local_core])
            )

            print(
                "[BRIDGE DECK] "
                f"label={bridge_label}, path={structure['path']}, "
                f"raw_cells={structure['deck_cell_count']}, "
                f"core_cells={structure['core_cell_count']}, "
                f"access_cells={structure['access_cell_count']}, "
                f"deck_Z=({structure['deck_z_min']:.3f}, "
                f"{structure['deck_z_max']:.3f}), "
                f"mesh_top_Z={float(structure['z_max']):.3f}"
            )

        return core_mask, access_mask, label_grid, surface_z_grid

    def navigation_structure_at(self, x, y):
        """현재 XY가 실제 플랫폼 또는 다리 상판 안인지 반환한다."""
        x = float(x)
        y = float(y)

        for structure in self._navigation_structures:
            if structure.get("source_type") != "explicit":
                continue
            if (
                float(structure["x_min"]) <= x <= float(structure["x_max"])
                and float(structure["y_min"]) <= y <= float(structure["y_max"])
            ):
                return structure

        if (
            self._bridge_grid_x_values is None
            or self._bridge_grid_y_values is None
            or self._bridge_core_mask is None
            or self._bridge_label_grid is None
        ):
            return None

        column = int(
            np.argmin(np.abs(self._bridge_grid_x_values - x))
        )
        row = int(
            np.argmin(np.abs(self._bridge_grid_y_values - y))
        )
        if not self._bridge_core_mask[row, column]:
            return None

        label = int(self._bridge_label_grid[row, column])
        for structure in self._navigation_structures:
            if int(structure.get("bridge_label", 0)) == label:
                return structure
        return None

    def navigation_height(self, x, y):
        """현재 XY의 실제 보행 표면 높이를 반환한다."""
        x = float(x)
        y = float(y)
        terrain_z = float(self.height(x, y))

        # 스폰 플랫폼은 아래 Terrain보다 실제 판 상단을 우선한다.
        for structure in self._navigation_structures:
            if structure.get("source_type") != "explicit":
                continue
            if (
                float(structure["x_min"]) <= x <= float(structure["x_max"])
                and float(structure["y_min"]) <= y <= float(structure["y_max"])
            ):
                return float(structure["z_max"])

        if (
            self._bridge_grid_x_values is not None
            and self._bridge_grid_y_values is not None
            and self._bridge_core_mask is not None
            and self._bridge_surface_z_grid is not None
        ):
            column = int(
                np.argmin(np.abs(self._bridge_grid_x_values - x))
            )
            row = int(
                np.argmin(np.abs(self._bridge_grid_y_values - y))
            )
            if self._bridge_core_mask[row, column]:
                bridge_z = float(self._bridge_surface_z_grid[row, column])
                if math.isfinite(bridge_z):
                    return bridge_z
        return terrain_z


    def _log_navigation_structure_edges(
        self,
        x_values,
        y_values,
        z_grid,
    ):
        """플랫폼과 다리 접속부의 실제 격자 높이차를 출력한다."""
        grid_x, grid_y = np.meshgrid(x_values, y_values)
        row_count, column_count = z_grid.shape
        neighbor_offsets = ((-1, 0), (1, 0), (0, -1), (0, 1))

        for structure in self._navigation_structures:
            if (
                structure.get("source_type") == "alias"
                and self._bridge_label_grid is not None
                and int(structure.get("bridge_label", 0)) > 0
            ):
                inside = (
                    self._bridge_label_grid
                    == int(structure["bridge_label"])
                )
                display_z = float(
                    structure.get(
                        "deck_z_median",
                        structure.get("z_max", float("nan")),
                    )
                )
            else:
                inside = (
                    (grid_x >= float(structure["x_min"]))
                    & (grid_x <= float(structure["x_max"]))
                    & (grid_y >= float(structure["y_min"]))
                    & (grid_y <= float(structure["y_max"]))
                )
                display_z = float(structure["z_max"])

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
                f"surface_Z={display_z:.3f}, "
                f"samples={len(edge_height_changes)}, "
                f"min_step={min(edge_height_changes):.3f}m, "
                f"max_step={max(edge_height_changes):.3f}m"
            )

    def write_navigation_surface(self, output_path, sample_spacing_m):
        """Terrain·실제 다리 상판을 합친 규칙 격자를 저장한다.

        명시적 회색 스폰 플랫폼은 보행 높이에 합치지 않는다. 대신 별도의
        ``platform_mask``와 bounds 메타데이터로 저장해 구조자 스폰 및 A*가
        해당 영역을 확실히 제외할 수 있게 한다.
        """
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

        # 기본 표면은 순수 Terrain이다. 다리만 실제 상판 추출 결과로
        # 덮어쓰며, 회색 드론 플랫폼은 절대로 보행 높이에 포함하지 않는다.
        z_grid = np.empty((y_count, x_count), dtype=np.float64)
        for row, world_y in enumerate(y_values):
            for column, world_x in enumerate(x_values):
                z_grid[row, column] = float(self.height(world_x, world_y))

        grid_x, grid_y = np.meshgrid(x_values, y_values)
        platform_mask = np.zeros_like(z_grid, dtype=bool)
        for structure in self._navigation_structures:
            if structure.get("source_type") != "explicit":
                continue
            platform_mask |= (
                (grid_x >= float(structure["x_min"]))
                & (grid_x <= float(structure["x_max"]))
                & (grid_y >= float(structure["y_min"]))
                & (grid_y <= float(structure["y_max"]))
            )

        (
            bridge_core_mask,
            bridge_access_mask,
            bridge_label_grid,
            bridge_surface_z_grid,
        ) = self._build_bridge_navigation_layers(
            x_values,
            y_values,
            z_grid,
        )
        bridge_surface_valid = (
            bridge_core_mask
            & np.isfinite(bridge_surface_z_grid)
        )
        z_grid[bridge_surface_valid] = bridge_surface_z_grid[
            bridge_surface_valid
        ]

        self._bridge_grid_x_values = np.asarray(
            x_values,
            dtype=np.float64,
        )
        self._bridge_grid_y_values = np.asarray(
            y_values,
            dtype=np.float64,
        )
        self._bridge_core_mask = bridge_core_mask
        self._bridge_access_mask = bridge_access_mask
        self._bridge_label_grid = bridge_label_grid
        self._bridge_surface_z_grid = bridge_surface_z_grid

        vertices = np.empty((x_count * y_count, 3), dtype=np.float32)
        vertex_index = 0
        for row, world_y in enumerate(y_values):
            for column, world_x in enumerate(x_values):
                vertices[vertex_index] = (
                    float(world_x),
                    float(world_y),
                    float(z_grid[row, column]),
                )
                vertex_index += 1

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
                (
                    structure.get("deck_z_median", structure["z_max"])
                    if structure.get("source_type") == "alias"
                    else structure["z_max"]
                )
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
                bridge_core_mask=bridge_core_mask.astype(np.bool_),
                bridge_access_mask=bridge_access_mask.astype(np.bool_),
                bridge_label_grid=bridge_label_grid.astype(np.int32),
                bridge_surface_z_grid=bridge_surface_z_grid.astype(
                    np.float32
                ),
                platform_mask=platform_mask.astype(np.bool_),
                bridge_structure_paths=np.asarray(
                    [
                        structure["path"]
                        for structure in self._navigation_structures
                        if int(structure.get("bridge_label", 0)) > 0
                    ],
                    dtype=str,
                ),
            )
        temporary_path.replace(output_path)
        print(
            "[INFO] 경로계획 Navigation Surface 저장: "
            f"{output_path}, vertices={len(vertices)}, "
            f"structures={len(self._navigation_structures)}, "
            f"bridge_core={int(np.count_nonzero(bridge_core_mask))}, "
            f"bridge_access={int(np.count_nonzero(bridge_access_mask))}, "
            f"platform_blocked={int(np.count_nonzero(platform_mask))}, "
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

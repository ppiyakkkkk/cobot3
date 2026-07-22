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
from pxr import Gf, Usd, UsdGeom
from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator

from sim_config import (
    RVIZ_ENVIRONMENT_GROUPS,
    RVIZ_ENVIRONMENT_MAX_TRIANGLES_PER_GROUP,
)


class TerrainHeightField:
    def __init__(self, stage):
        self._stage = stage
        self._terrain_prim = self._find_terrain_mesh()
        self._build_interpolator()

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

    def _classify_mesh(self, prim):
        """Prim 경로의 조상 이름을 검사해 환경 그룹을 결정한다."""
        normalized_segments = [
            self._normalize_name(segment)
            for segment in str(prim.GetPath()).split("/")
            if segment
        ]

        for category, aliases in RVIZ_ENVIRONMENT_GROUPS.items():
            for segment in normalized_segments:
                if any(
                    segment == alias or segment.startswith(alias)
                    for alias in aliases
                ):
                    return category
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
        """Pine/Broadleaf/Bushes/Rocks Mesh를 하나의 NPZ로 저장한다."""
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
            "format_version": np.asarray([1], dtype=np.int32),
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
                "PineForest/BroadleafForest/Bushes/Rocks 아래에서 "
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

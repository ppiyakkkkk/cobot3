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
from pxr import Gf, Usd, UsdGeom, UsdShade
from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator

from sim_config import (
    NAVIGATION_STRUCTURE_ALIASES,
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

    def _build_navigation_structures(self):
        """다리처럼 Terrain과 분리된 구조물의 World AABB를 수집한다.

        구조물의 세부 삼각형을 2D 격자에 직접 투영하면 계산량이 커지므로,
        경로계획에서는 보수적으로 World XY bounding box 안의 최고점을
        구조물 표면 높이로 사용한다. 구조물 수가 적은 산림 환경에 적합하다.
        """
        aliases = tuple(
            self._normalize_name(value)
            for value in NAVIGATION_STRUCTURE_ALIASES
            if str(value).strip()
        )
        if not aliases:
            return

        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        unmatched_large_meshes = []

        for prim in self._stage.Traverse():
            if not prim.IsA(UsdGeom.Mesh):
                continue
            if prim == self._terrain_prim:
                continue

            mesh = UsdGeom.Mesh(prim)
            local_points = mesh.GetPointsAttr().Get()
            if local_points is None or len(local_points) < 3:
                continue

            path_text = self._normalize_name(str(prim.GetPath()))
            matched = any(alias in path_text for alias in aliases)
            if not matched:
                if len(local_points) >= 100:
                    unmatched_large_meshes.append(
                        (len(local_points), str(prim.GetPath()))
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

            structure = {
                "path": str(prim.GetPath()),
                "x_min": float(np.min(world_points[:, 0])),
                "x_max": float(np.max(world_points[:, 0])),
                "y_min": float(np.min(world_points[:, 1])),
                "y_max": float(np.max(world_points[:, 1])),
                "z_max": float(np.max(world_points[:, 2])),
            }
            self._navigation_structures.append(structure)
            print(
                "[NAV STRUCTURE] "
                f"{structure['path']}: "
                f"XY=({structure['x_min']:.2f}, {structure['x_max']:.2f}, "
                f"{structure['y_min']:.2f}, {structure['y_max']:.2f}), "
                f"top_Z={structure['z_max']:.2f}"
            )

        if self._navigation_structures:
            print(
                "[NAV STRUCTURE] 경로계획에 반영할 구조물 수: "
                f"{len(self._navigation_structures)}"
            )
            return

        # Bridge 이름이 다른 경우 다음 실행 로그만으로 Prim 이름을 찾을 수
        # 있도록 Terrain 외의 큰 Mesh 후보를 출력한다.
        unmatched_large_meshes.sort(reverse=True)
        if unmatched_large_meshes:
            candidate_text = ", ".join(
                f"{path}(points={count})"
                for count, path in unmatched_large_meshes[:12]
            )
            carb.log_warn(
                "bridge/deck 이름의 경로계획 구조물을 찾지 못했습니다. "
                "다리 Prim 이름이 다르면 sim_config.py의 "
                f"NAVIGATION_STRUCTURE_ALIASES에 추가하세요. 후보: {candidate_text}"
            )

    def navigation_height(self, x, y):
        """Terrain과 등록된 다리 구조물 중 더 높은 표면을 반환한다."""
        result = float(self.height(x, y))
        margin = max(0.0, float(NAVIGATION_STRUCTURE_XY_MARGIN_M))
        for structure in self._navigation_structures:
            if (
                structure["x_min"] - margin
                <= float(x)
                <= structure["x_max"] + margin
                and structure["y_min"] - margin
                <= float(y)
                <= structure["y_max"] + margin
            ):
                result = max(result, structure["z_max"])
        return result

    def write_navigation_surface(self, output_path, sample_spacing_m):
        """Terrain과 다리 상단을 합친 경로계획 전용 규칙 격자를 저장한다."""
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

#!/usr/bin/env python3
"""구조자 보행용 격자 지도와 A* 경로계획 순수 함수.

ROS 메시지나 Isaac Sim 모듈에 의존하지 않으므로 단위 테스트와 재사용이 쉽다.
강은 통행 불가로 만들고 다리 영역만 다시 통행 가능하게 열어, 강 건너편
목표로 이동할 때 경로가 반드시 다리를 지나도록 한다.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
from scipy import ndimage


GridIndex = tuple[int, int]  # (row=y, column=x)


@dataclass
class RescuerGridMap:
    x_values: np.ndarray
    y_values: np.ndarray
    z_grid: np.ndarray
    slope_deg: np.ndarray
    walkable: np.ndarray
    river_mask: np.ndarray
    bridge_mask: np.ndarray
    bridge_core_mask: np.ndarray
    bridge_access_mask: np.ndarray
    bridge_connector_mask: np.ndarray
    bridge_labels: np.ndarray
    obstacle_mask: np.ndarray
    spacing_x: float
    spacing_y: float
    max_step_height_m: float
    bridge_max_step_height_m: float

    @property
    def shape(self) -> tuple[int, int]:
        return self.walkable.shape

    def world_to_grid(self, x: float, y: float) -> GridIndex:
        column = int(np.argmin(np.abs(self.x_values - float(x))))
        row = int(np.argmin(np.abs(self.y_values - float(y))))
        return row, column

    def grid_to_world(self, cell: GridIndex) -> np.ndarray:
        row, column = cell
        return np.asarray(
            [
                float(self.x_values[column]),
                float(self.y_values[row]),
                float(self.z_grid[row, column]),
            ],
            dtype=np.float64,
        )

    def in_bounds(self, cell: GridIndex) -> bool:
        row, column = cell
        return 0 <= row < self.shape[0] and 0 <= column < self.shape[1]


def _reshape_navigation_vertices(vertices: np.ndarray):
    vertices = np.asarray(vertices, dtype=np.float64)
    if vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 4:
        raise ValueError(f"navigation vertices 형식이 잘못됐습니다: {vertices.shape}")

    x_values = np.unique(np.round(vertices[:, 0], decimals=6))
    y_values = np.unique(np.round(vertices[:, 1], decimals=6))
    x_values.sort()
    y_values.sort()

    expected = len(x_values) * len(y_values)
    if expected != len(vertices):
        raise ValueError(
            "navigation surface가 규칙 격자가 아닙니다: "
            f"vertices={len(vertices)}, expected={expected}"
        )

    z_grid = np.full((len(y_values), len(x_values)), np.nan, dtype=np.float64)
    x_index = {float(value): index for index, value in enumerate(x_values)}
    y_index = {float(value): index for index, value in enumerate(y_values)}
    for x, y, z in vertices:
        key_x = float(round(float(x), 6))
        key_y = float(round(float(y), 6))
        z_grid[y_index[key_y], x_index[key_x]] = float(z)

    if not np.all(np.isfinite(z_grid)):
        raise ValueError("navigation surface 높이 격자에 NaN/Inf가 있습니다.")
    return x_values.astype(np.float64), y_values.astype(np.float64), z_grid


def _triangle_mask(
    x_values: np.ndarray,
    y_values: np.ndarray,
    vertices: np.ndarray,
    triangles: np.ndarray,
) -> np.ndarray:
    """XY로 투영된 삼각형 내부의 격자점을 True로 만든다."""
    mask = np.zeros((len(y_values), len(x_values)), dtype=bool)
    vertices = np.asarray(vertices, dtype=np.float64)
    triangles = np.asarray(triangles, dtype=np.int64)
    if vertices.size == 0 or triangles.size == 0:
        return mask

    for triangle in triangles:
        points = vertices[triangle, :2]
        if not np.all(np.isfinite(points)):
            continue
        x_min, y_min = np.min(points, axis=0)
        x_max, y_max = np.max(points, axis=0)
        column_min = max(0, int(np.searchsorted(x_values, x_min, side="left")) - 1)
        column_max = min(
            len(x_values) - 1,
            int(np.searchsorted(x_values, x_max, side="right")),
        )
        row_min = max(0, int(np.searchsorted(y_values, y_min, side="left")) - 1)
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
        inside = (u >= -1.0e-7) & (v >= -1.0e-7) & (u + v <= 1.0 + 1.0e-7)
        mask[row_min : row_max + 1, column_min : column_max + 1] |= inside

        # 다리 폭이 격자 간격보다 좁아 중심점이 하나도 들어오지 않는 경우를
        # 대비해 세 꼭짓점과 무게중심의 최근접 셀도 표시한다.
        samples = np.vstack([points, np.mean(points, axis=0, keepdims=True)])
        for sample_x, sample_y in samples:
            sample_column = int(np.argmin(np.abs(x_values - sample_x)))
            sample_row = int(np.argmin(np.abs(y_values - sample_y)))
            mask[sample_row, sample_column] = True
    return mask


def _dilation_iterations(distance_m: float, spacing_m: float) -> int:
    if distance_m <= 0.0:
        return 0
    return max(1, int(math.ceil(float(distance_m) / max(float(spacing_m), 1.0e-6))))


def _load_group(data, name: str):
    vertex_key = f"{name}_vertices"
    triangle_key = f"{name}_triangles"
    if vertex_key not in data.files or triangle_key not in data.files:
        return np.empty((0, 3)), np.empty((0, 3), dtype=np.int64)
    return (
        np.asarray(data[vertex_key], dtype=np.float64),
        np.asarray(data[triangle_key], dtype=np.int64),
    )



def _load_navigation_bridge_layers(navigation, shape):
    """새 navigation NPZ의 다리 상판·접속부 레이어를 읽는다."""
    required = (
        "bridge_core_mask",
        "bridge_access_mask",
        "bridge_label_grid",
    )
    if not all(key in navigation.files for key in required):
        return None

    core = np.asarray(
        navigation["bridge_core_mask"],
        dtype=bool,
    )
    access = np.asarray(
        navigation["bridge_access_mask"],
        dtype=bool,
    )
    labels = np.asarray(
        navigation["bridge_label_grid"],
        dtype=np.int32,
    )
    if core.shape != shape or access.shape != shape or labels.shape != shape:
        raise ValueError(
            "navigation bridge layer shape 불일치: "
            f"core={core.shape}, access={access.shape}, "
            f"labels={labels.shape}, expected={shape}"
        )
    access |= core
    labels = np.where(access, labels, 0).astype(np.int32)
    if np.any(access & (labels <= 0)):
        fallback_labels, _ = ndimage.label(access)
        labels = np.where(
            access & (labels <= 0),
            fallback_labels,
            labels,
        ).astype(np.int32)
    return core, access, labels



def _bridge_endpoint_bands(
    core_mask: np.ndarray,
    label_grid: np.ndarray,
    bridge_label: int,
    x_values: np.ndarray,
    y_values: np.ndarray,
):
    """다리 상판의 주축과 양 끝 셀 집합을 반환한다."""
    rows, columns = np.where(
        core_mask & (label_grid == int(bridge_label))
    )
    if len(rows) < 2:
        return None

    points = np.column_stack(
        (x_values[columns], y_values[rows])
    ).astype(np.float64)
    center = np.mean(points, axis=0)
    centered = points - center
    _u, _s, vh = np.linalg.svd(centered, full_matrices=False)
    axis = vh[0]
    axis_length = float(np.linalg.norm(axis))
    if axis_length <= 1.0e-9:
        return None
    axis = axis / axis_length

    projection = centered @ axis
    minimum = float(np.min(projection))
    maximum = float(np.max(projection))
    band = max(
        0.75,
        0.18 * max(maximum - minimum, 1.0),
    )

    low_indices = np.where(projection <= minimum + band)[0]
    high_indices = np.where(projection >= maximum - band)[0]
    low_cells = [
        (int(rows[index]), int(columns[index]))
        for index in low_indices
    ]
    high_cells = [
        (int(rows[index]), int(columns[index]))
        for index in high_indices
    ]
    low_center = np.mean(points[low_indices], axis=0)
    high_center = np.mean(points[high_indices], axis=0)
    return (
        axis,
        (low_cells, low_center, -axis),
        (high_cells, high_center, axis),
    )


def _find_bridge_connector_path(
    z_grid: np.ndarray,
    finite_mask: np.ndarray,
    slope_deg: np.ndarray,
    obstacle_mask: np.ndarray,
    base_walkable: np.ndarray,
    existing_access: np.ndarray,
    start_cells: Sequence[GridIndex],
    endpoint_xy: np.ndarray,
    direction_xy: np.ndarray,
    x_values: np.ndarray,
    y_values: np.ndarray,
    *,
    max_length_m: float,
    half_width_m: float,
    max_slope_deg: float,
    max_step_height_m: float,
) -> list[GridIndex]:
    """다리 끝에서 가장 가까운 일반 보행 영역까지 좁은 연결로를 찾는다."""
    row_count, column_count = z_grid.shape
    grid_x, grid_y = np.meshgrid(x_values, y_values)
    delta_x = grid_x - float(endpoint_xy[0])
    delta_y = grid_y - float(endpoint_xy[1])
    along = (
        delta_x * float(direction_xy[0])
        + delta_y * float(direction_xy[1])
    )
    lateral = np.abs(
        -delta_x * float(direction_xy[1])
        + delta_y * float(direction_xy[0])
    )

    corridor = (
        (along >= -0.75)
        & (along <= float(max_length_m))
        & (lateral <= float(half_width_m))
        & finite_mask
        & ~obstacle_mask
        & (slope_deg <= float(max_slope_deg))
    )
    corridor |= existing_access

    # 다리에서 충분히 떨어진 일반 지면만 연결 목표로 사용한다.
    target = (
        base_walkable
        & corridor
        & (along >= min(1.0, float(max_length_m) * 0.25))
    )
    if not np.any(target):
        return []

    open_heap = []
    came_from = {}
    g_score = {}
    for cell in start_cells:
        row, column = cell
        if not (
            0 <= row < row_count
            and 0 <= column < column_count
            and corridor[cell]
        ):
            continue
        g_score[cell] = 0.0
        heapq.heappush(open_heap, (0.0, cell))

    neighbor_offsets = (
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    )

    closed = set()
    while open_heap:
        current_cost, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)

        if target[current]:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        row, column = current
        for row_offset, column_offset, distance_factor in neighbor_offsets:
            neighbor = (
                row + row_offset,
                column + column_offset,
            )
            neighbor_row, neighbor_column = neighbor
            if not (
                0 <= neighbor_row < row_count
                and 0 <= neighbor_column < column_count
                and corridor[neighbor]
            ):
                continue

            height_change = abs(
                float(z_grid[neighbor])
                - float(z_grid[current])
            )
            if height_change > float(max_step_height_m):
                continue

            # 대각선 이동 시 좁은 코너를 관통하지 않는다.
            if row_offset and column_offset:
                orthogonal_row = (row + row_offset, column)
                orthogonal_column = (row, column + column_offset)
                if not corridor[orthogonal_row] or not corridor[orthogonal_column]:
                    continue
                if (
                    abs(
                        float(z_grid[orthogonal_row])
                        - float(z_grid[current])
                    )
                    > float(max_step_height_m)
                ):
                    continue
                if (
                    abs(
                        float(z_grid[orthogonal_column])
                        - float(z_grid[current])
                    )
                    > float(max_step_height_m)
                ):
                    continue

            horizontal_distance = math.hypot(
                column_offset
                * float(np.median(np.diff(x_values))),
                row_offset
                * float(np.median(np.diff(y_values))),
            )
            slope_penalty = 1.0 + min(
                1.0,
                float(slope_deg[neighbor])
                / max(float(max_slope_deg), 1.0),
            )
            tentative = (
                current_cost
                + horizontal_distance
                * distance_factor
                * slope_penalty
            )
            if tentative >= g_score.get(neighbor, float("inf")):
                continue
            came_from[neighbor] = current
            g_score[neighbor] = tentative
            heapq.heappush(open_heap, (tentative, neighbor))

    return []


def _extend_bridge_access_to_terrain(
    z_grid: np.ndarray,
    finite_mask: np.ndarray,
    slope_deg: np.ndarray,
    obstacle_mask: np.ndarray,
    base_walkable: np.ndarray,
    bridge_core_mask: np.ndarray,
    bridge_access_mask: np.ndarray,
    bridge_labels: np.ndarray,
    x_values: np.ndarray,
    y_values: np.ndarray,
    *,
    max_length_m: float,
    half_width_m: float,
    max_slope_deg: float,
    max_step_height_m: float,
):
    """각 다리의 양 끝을 실제 일반 보행 컴포넌트까지 국소 연결한다."""
    access = bridge_access_mask.copy()
    labels = bridge_labels.copy()
    connector_mask = np.zeros_like(access, dtype=bool)

    bridge_ids = sorted(
        int(value)
        for value in np.unique(labels[bridge_core_mask])
        if int(value) > 0
    )
    for bridge_id in bridge_ids:
        endpoint_data = _bridge_endpoint_bands(
            bridge_core_mask,
            labels,
            bridge_id,
            x_values,
            y_values,
        )
        if endpoint_data is None:
            continue
        _axis, low_endpoint, high_endpoint = endpoint_data

        for start_cells, endpoint_xy, direction_xy in (
            low_endpoint,
            high_endpoint,
        ):
            local_existing = access & (labels == bridge_id)
            path = _find_bridge_connector_path(
                z_grid,
                finite_mask,
                slope_deg,
                obstacle_mask,
                base_walkable,
                local_existing,
                start_cells,
                endpoint_xy,
                direction_xy,
                x_values,
                y_values,
                max_length_m=max_length_m,
                half_width_m=half_width_m,
                max_slope_deg=max_slope_deg,
                max_step_height_m=max_step_height_m,
            )
            if not path:
                continue

            path_mask = np.zeros_like(access, dtype=bool)
            for cell in path:
                path_mask[cell] = True

            # 1셀 폭을 확보하되 같은 국소 기울기 조건 안에서만 확장한다.
            dilated = ndimage.binary_dilation(
                path_mask,
                iterations=1,
            )
            local_allowed = (
                finite_mask
                & ~obstacle_mask
                & (slope_deg <= float(max_slope_deg))
            )
            dilated &= local_allowed
            access |= dilated
            connector_mask |= dilated
            labels[dilated & (labels == 0)] = bridge_id

    return access, labels, connector_mask


def build_rescuer_grid_map(
    navigation_surface_path: str | Path,
    environment_mesh_path: str | Path,
    *,
    max_slope_deg: float = 45.0,
    max_step_height_m: float = 1.25,
    bridge_max_slope_deg: float = 60.0,
    bridge_max_step_height_m: float = 1.5,
    bridge_connector_max_length_m: float = 15.0,
    bridge_connector_half_width_m: float = 4.5,
    river_clearance_m: float = 0.75,
    bridge_expansion_m: float = 1.5,
    obstacle_clearance_m: float = 0.8,
    block_rocks: bool = True,
    block_vegetation: bool = False,
) -> RescuerGridMap:
    navigation_surface_path = Path(navigation_surface_path).expanduser()
    environment_mesh_path = Path(environment_mesh_path).expanduser()

    with np.load(navigation_surface_path, allow_pickle=False) as navigation:
        vertices = np.asarray(navigation["vertices"], dtype=np.float64)
        x_values, y_values, z_grid = _reshape_navigation_vertices(
            vertices
        )
        navigation_bridge_layers = _load_navigation_bridge_layers(
            navigation,
            z_grid.shape,
        )

    spacing_x = float(np.median(np.diff(x_values)))
    spacing_y = float(np.median(np.diff(y_values)))
    gradient_y, gradient_x = np.gradient(
        z_grid,
        spacing_y,
        spacing_x,
    )
    slope_deg = np.degrees(
        np.arctan(np.hypot(gradient_x, gradient_y))
    )
    finite_mask = np.isfinite(z_grid)
    walkable = finite_mask & (slope_deg <= float(max_slope_deg))

    with np.load(environment_mesh_path, allow_pickle=False) as environment:
        river_vertices, river_triangles = _load_group(
            environment,
            "river",
        )
        rock_vertices, rock_triangles = _load_group(
            environment,
            "rocks",
        )

        river_mask = _triangle_mask(
            x_values,
            y_values,
            river_vertices,
            river_triangles,
        )
        if navigation_bridge_layers is None:
            bridge_vertices, bridge_triangles = _load_group(
                environment,
                "bridges",
            )
            environment_bridge_mask = _triangle_mask(
                x_values,
                y_values,
                bridge_vertices,
                bridge_triangles,
            )
        else:
            environment_bridge_mask = np.zeros_like(
                walkable,
                dtype=bool,
            )
        obstacle_mask = (
            _triangle_mask(
                x_values,
                y_values,
                rock_vertices,
                rock_triangles,
            )
            if block_rocks
            else np.zeros_like(walkable)
        )

        if block_vegetation:
            for group_name in (
                "pineforest",
                "broadleafforest",
                "bushes",
            ):
                group_vertices, group_triangles = _load_group(
                    environment,
                    group_name,
                )
                obstacle_mask |= _triangle_mask(
                    x_values,
                    y_values,
                    group_vertices,
                    group_triangles,
                )

    mean_spacing = (spacing_x + spacing_y) * 0.5
    river_iterations = _dilation_iterations(
        river_clearance_m,
        mean_spacing,
    )
    obstacle_iterations = _dilation_iterations(
        obstacle_clearance_m,
        mean_spacing,
    )

    if river_iterations:
        river_mask = ndimage.binary_dilation(
            river_mask,
            iterations=river_iterations,
        )
    if obstacle_iterations:
        obstacle_mask = ndimage.binary_dilation(
            obstacle_mask,
            iterations=obstacle_iterations,
        )

    if navigation_bridge_layers is not None:
        bridge_core_mask, bridge_access_mask, bridge_labels = (
            navigation_bridge_layers
        )
    else:
        # 구형 NPZ fallback. 새 sim_terrain.py로 재생성하면 이 경로를
        # 사용하지 않고 실제 상판·양 끝 접속부 레이어를 사용한다.
        bridge_iterations = _dilation_iterations(
            bridge_expansion_m,
            mean_spacing,
        )
        bridge_access_mask = environment_bridge_mask.copy()
        if bridge_iterations:
            bridge_access_mask = ndimage.binary_dilation(
                bridge_access_mask,
                iterations=bridge_iterations,
            )
        bridge_core_mask = environment_bridge_mask.copy()
        bridge_labels, _ = ndimage.label(bridge_access_mask)

    bridge_access_mask |= bridge_core_mask

    # 일반 제한으로 이미 안전한 Terrain 셀을 기준으로 다리 양 끝만
    # 국소적으로 연결한다. 강 전체나 임의의 절벽을 여는 방식이 아니다.
    base_walkable = (
        finite_mask
        & (slope_deg <= float(max_slope_deg))
        & ~river_mask
        & ~obstacle_mask
    )
    (
        bridge_access_mask,
        bridge_labels,
        bridge_connector_mask,
    ) = _extend_bridge_access_to_terrain(
        z_grid,
        finite_mask,
        slope_deg,
        obstacle_mask,
        base_walkable,
        bridge_core_mask,
        bridge_access_mask,
        bridge_labels,
        x_values,
        y_values,
        max_length_m=float(bridge_connector_max_length_m),
        half_width_m=float(bridge_connector_half_width_m),
        max_slope_deg=float(bridge_max_slope_deg),
        max_step_height_m=max(
            float(max_step_height_m),
            float(bridge_max_step_height_m),
        ),
    )
    bridge_mask = bridge_access_mask.copy()

    # 일반 Terrain은 기존 42도/1.0m 제한을 유지한다.
    walkable = base_walkable.copy()

    # 실제 상판 core는 격자 미분으로 경사가 과장될 수 있어 직접 연다.
    # 양 끝 접속부는 별도 bridge_max_slope_deg 안에서만 연다.
    bridge_core_walkable = (
        bridge_core_mask
        & finite_mask
        & ~obstacle_mask
    )
    bridge_approach_walkable = (
        bridge_access_mask
        & ~bridge_core_mask
        & finite_mask
        & (slope_deg <= float(bridge_max_slope_deg))
        & ~obstacle_mask
    )
    walkable |= bridge_core_walkable
    walkable |= bridge_approach_walkable

    # bridge label은 실제로 열린 다리 접속 영역에만 남긴다.
    bridge_labels = np.where(
        bridge_access_mask,
        bridge_labels,
        0,
    ).astype(np.int32)

    return RescuerGridMap(
        x_values=x_values,
        y_values=y_values,
        z_grid=z_grid,
        slope_deg=slope_deg,
        walkable=walkable,
        river_mask=river_mask,
        bridge_mask=bridge_mask,
        bridge_core_mask=bridge_core_mask,
        bridge_access_mask=bridge_access_mask,
        bridge_connector_mask=bridge_connector_mask,
        bridge_labels=bridge_labels,
        obstacle_mask=obstacle_mask,
        spacing_x=spacing_x,
        spacing_y=spacing_y,
        max_step_height_m=float(max_step_height_m),
        bridge_max_step_height_m=max(
            float(max_step_height_m),
            float(bridge_max_step_height_m),
        ),
    )


def transition_is_walkable(
    grid_map: RescuerGridMap,
    start: GridIndex,
    end: GridIndex,
) -> bool:
    """두 셀 사이가 보행 가능하고 지역별 단차 제한을 만족하는지 검사한다."""
    if (
        not grid_map.in_bounds(start)
        or not grid_map.in_bounds(end)
        or not grid_map.walkable[start]
        or not grid_map.walkable[end]
    ):
        return False

    height_change = abs(
        float(grid_map.z_grid[end])
        - float(grid_map.z_grid[start])
    )
    bridge_transition = bool(
        grid_map.bridge_access_mask[start]
        or grid_map.bridge_access_mask[end]
    )
    limit = (
        float(grid_map.bridge_max_step_height_m)
        if bridge_transition
        else float(grid_map.max_step_height_m)
    )
    return height_change <= limit


def nearest_walkable_cell(
    grid_map: RescuerGridMap,
    requested: GridIndex,
    max_radius_cells: int = 12,
    *,
    requested_z: float | None = None,
    max_height_difference_m: float | None = None,
) -> GridIndex:
    """최근접 보행 셀을 찾되 필요하면 현재 높이도 함께 검사한다."""
    def accepted(cell):
        if not grid_map.in_bounds(cell) or not grid_map.walkable[cell]:
            return False
        if requested_z is None or max_height_difference_m is None:
            return True
        return (
            abs(
                float(grid_map.z_grid[cell])
                - float(requested_z)
            )
            <= float(max_height_difference_m)
        )

    if accepted(requested):
        return requested

    row0, column0 = requested
    for radius in range(1, max_radius_cells + 1):
        candidates = []
        for row in range(row0 - radius, row0 + radius + 1):
            for column in range(column0 - radius, column0 + radius + 1):
                cell = (row, column)
                if not accepted(cell):
                    continue
                distance = math.hypot(row - row0, column - column0)
                height_error = (
                    0.0
                    if requested_z is None
                    else abs(
                        float(grid_map.z_grid[cell])
                        - float(requested_z)
                    )
                )
                candidates.append((distance, height_error, cell))
        if candidates:
            candidates.sort(key=lambda item: (item[0], item[1]))
            return candidates[0][2]

    raise RuntimeError(
        "주변에서 조건을 만족하는 보행 가능 셀을 찾지 못했습니다: "
        f"requested={requested}, requested_z={requested_z}, "
        f"max_height_difference_m={max_height_difference_m}"
    )

def goal_cells_around_victim(
    grid_map: RescuerGridMap,
    victim_xyz: Sequence[float],
    min_standoff_m: float,
    max_standoff_m: float,
) -> set[GridIndex]:
    victim_x = float(victim_xyz[0])
    victim_y = float(victim_xyz[1])
    grid_x, grid_y = np.meshgrid(grid_map.x_values, grid_map.y_values)
    distance = np.hypot(grid_x - victim_x, grid_y - victim_y)
    goal_mask = (
        grid_map.walkable
        & (distance >= float(min_standoff_m))
        & (distance <= float(max_standoff_m))
    )
    rows, columns = np.where(goal_mask)
    goals = {(int(row), int(column)) for row, column in zip(rows, columns)}
    if goals:
        return goals

    requested = grid_map.world_to_grid(victim_x, victim_y)
    return {nearest_walkable_cell(grid_map, requested, max_radius_cells=20)}


_NEIGHBORS = (
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)),
    (1, 1, math.sqrt(2.0)),
)


def _heuristic_to_victim(
    grid_map: RescuerGridMap,
    cell: GridIndex,
    victim_xyz: Sequence[float],
    max_standoff_m: float,
) -> float:
    point = grid_map.grid_to_world(cell)
    distance = math.hypot(point[0] - victim_xyz[0], point[1] - victim_xyz[1])
    return max(0.0, distance - float(max_standoff_m))


def astar_to_goal_set(
    grid_map: RescuerGridMap,
    start: GridIndex,
    goals: set[GridIndex],
    victim_xyz: Sequence[float],
    *,
    max_standoff_m: float,
    slope_cost_weight: float = 1.5,
) -> list[GridIndex]:
    if start in goals:
        return [start]
    if not grid_map.walkable[start]:
        raise RuntimeError(f"A* 시작 셀이 보행 불가입니다: {start}")

    open_heap: list[tuple[float, float, GridIndex]] = []
    heapq.heappush(
        open_heap,
        (
            _heuristic_to_victim(grid_map, start, victim_xyz, max_standoff_m),
            0.0,
            start,
        ),
    )
    came_from: dict[GridIndex, GridIndex] = {}
    g_score = {start: 0.0}
    closed: set[GridIndex] = set()

    while open_heap:
        _f_score, current_cost, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current in goals:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        closed.add(current)

        row, column = current
        for d_row, d_column, _distance_factor in _NEIGHBORS:
            neighbor = (row + d_row, column + d_column)
            if not transition_is_walkable(grid_map, current, neighbor):
                continue
            # 대각선으로 막힌 셀 모서리를 뚫고 지나가지 않는다.
            if d_row and d_column:
                orthogonal_row = (row + d_row, column)
                orthogonal_column = (row, column + d_column)
                if not transition_is_walkable(
                    grid_map, current, orthogonal_row
                ):
                    continue
                if not transition_is_walkable(
                    grid_map, current, orthogonal_column
                ):
                    continue

            horizontal_distance = math.hypot(
                d_column * grid_map.spacing_x,
                d_row * grid_map.spacing_y,
            )
            slope_ratio = min(
                1.0,
                float(grid_map.slope_deg[neighbor]) / 45.0,
            )
            step_cost = horizontal_distance * (
                1.0 + float(slope_cost_weight) * slope_ratio * slope_ratio
            )
            tentative = current_cost + step_cost
            if tentative >= g_score.get(neighbor, float("inf")):
                continue
            came_from[neighbor] = current
            g_score[neighbor] = tentative
            heuristic = _heuristic_to_victim(
                grid_map, neighbor, victim_xyz, max_standoff_m
            )
            heapq.heappush(
                open_heap,
                (tentative + heuristic, tentative, neighbor),
            )

    reachable_bridge_core = sum(
        1 for cell in closed if grid_map.bridge_core_mask[cell]
    )
    reachable_bridge_access = sum(
        1 for cell in closed if grid_map.bridge_access_mask[cell]
    )
    walkable_goals = sum(
        1
        for cell in goals
        if grid_map.in_bounds(cell) and grid_map.walkable[cell]
    )
    nearest_goal_distance_m = float("inf")
    if closed and goals:
        for current in closed:
            current_world = grid_map.grid_to_world(current)
            for goal in goals:
                if not grid_map.in_bounds(goal):
                    continue
                goal_world = grid_map.grid_to_world(goal)
                nearest_goal_distance_m = min(
                    nearest_goal_distance_m,
                    float(
                        np.linalg.norm(
                            goal_world[:2] - current_world[:2]
                        )
                    ),
                )
    nearest_text = (
        "NONE"
        if not math.isfinite(nearest_goal_distance_m)
        else f"{nearest_goal_distance_m:.2f}m"
    )
    raise RuntimeError(
        "구조자 A* 경로를 찾지 못했습니다. "
        f"reachable={len(closed)}, "
        f"reachable_bridge_core={reachable_bridge_core}, "
        f"reachable_bridge_access={reachable_bridge_access}, "
        f"goal_cells={len(goals)}, "
        f"walkable_goals={walkable_goals}, "
        f"nearest_goal_distance={nearest_text}"
    )


def _bresenham_cells(start: GridIndex, end: GridIndex) -> list[GridIndex]:
    row0, column0 = start
    row1, column1 = end
    x0, y0 = column0, row0
    x1, y1 = column1, row1
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    result = []
    while True:
        result.append((y0, x0))
        if x0 == x1 and y0 == y1:
            break
        twice = 2 * error
        if twice >= dy:
            error += dy
            x0 += sx
        if twice <= dx:
            error += dx
            y0 += sy
    return result


def line_of_sight(grid_map: RescuerGridMap, start: GridIndex, end: GridIndex) -> bool:
    cells = _bresenham_cells(start, end)
    previous = cells[0]
    for cell in cells:
        if not transition_is_walkable(grid_map, previous, cell):
            return False
        d_row = cell[0] - previous[0]
        d_column = cell[1] - previous[1]
        if d_row and d_column:
            if not grid_map.walkable[previous[0] + d_row, previous[1]]:
                return False
            if not grid_map.walkable[previous[0], previous[1] + d_column]:
                return False
        previous = cell
    return True


def path_height_statistics(
    grid_map: RescuerGridMap,
    path: Sequence[GridIndex],
) -> tuple[float, float]:
    """경로가 통과하는 최대 경사와 인접 셀 최대 높이 차를 반환한다."""
    if not path:
        return 0.0, 0.0
    maximum_slope = max(float(grid_map.slope_deg[cell]) for cell in path)
    maximum_step = 0.0
    for start, end in zip(path, path[1:]):
        maximum_step = max(
            maximum_step,
            abs(
                float(grid_map.z_grid[end])
                - float(grid_map.z_grid[start])
            ),
        )
    return maximum_slope, maximum_step


def simplify_grid_path(
    grid_map: RescuerGridMap,
    path: Sequence[GridIndex],
    max_segment_m: float = 4.5,
) -> list[GridIndex]:
    """시야가 트인 점은 줄이되 지면 높이를 따르도록 구간 길이를 제한한다."""
    if len(path) <= 2:
        return list(path)
    simplified = [path[0]]
    anchor_index = 0
    while anchor_index < len(path) - 1:
        next_index = anchor_index + 1
        for candidate_index in range(anchor_index + 2, len(path)):
            start_world = grid_map.grid_to_world(path[anchor_index])
            candidate_world = grid_map.grid_to_world(path[candidate_index])
            segment_length = float(
                np.linalg.norm(candidate_world - start_world)
            )
            if segment_length > float(max_segment_m):
                break
            if line_of_sight(
                grid_map, path[anchor_index], path[candidate_index]
            ):
                next_index = candidate_index
            else:
                break
        simplified.append(path[next_index])
        anchor_index = next_index
    return simplified


def bridge_required_for_path(
    grid_map: RescuerGridMap,
    start: GridIndex,
    goal: GridIndex,
) -> bool:
    walkable_without_bridges = grid_map.walkable.copy()
    walkable_without_bridges[grid_map.bridge_mask] = False
    labels, _ = ndimage.label(walkable_without_bridges)
    start_label = int(labels[start])
    goal_label = int(labels[goal])
    return start_label > 0 and goal_label > 0 and start_label != goal_label


def validate_bridge_usage(
    grid_map: RescuerGridMap,
    path: Sequence[GridIndex],
) -> tuple[bool, int]:
    if not path:
        return False, 0
    required = bridge_required_for_path(grid_map, path[0], path[-1])
    bridge_ids = [
        int(grid_map.bridge_labels[cell])
        for cell in path
        if int(grid_map.bridge_labels[cell]) > 0
    ]
    selected_bridge = bridge_ids[0] if bridge_ids else 0
    if required and selected_bridge == 0:
        raise RuntimeError(
            "강 건너편 목표인데 생성 경로가 다리 영역을 지나지 않습니다."
        )
    return required, selected_bridge


def path_length_m(grid_map: RescuerGridMap, path: Sequence[GridIndex]) -> float:
    if len(path) < 2:
        return 0.0
    points = np.asarray([grid_map.grid_to_world(cell) for cell in path])
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))

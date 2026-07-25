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
    bridge_labels: np.ndarray
    obstacle_mask: np.ndarray
    spacing_x: float
    spacing_y: float
    max_step_height_m: float

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


def build_rescuer_grid_map(
    navigation_surface_path: str | Path,
    environment_mesh_path: str | Path,
    *,
    max_slope_deg: float = 42.0,
    max_step_height_m: float = 1.0,
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
    x_values, y_values, z_grid = _reshape_navigation_vertices(vertices)

    spacing_x = float(np.median(np.diff(x_values)))
    spacing_y = float(np.median(np.diff(y_values)))
    gradient_y, gradient_x = np.gradient(z_grid, spacing_y, spacing_x)
    slope_deg = np.degrees(np.arctan(np.hypot(gradient_x, gradient_y)))
    walkable = np.isfinite(z_grid) & (slope_deg <= float(max_slope_deg))

    with np.load(environment_mesh_path, allow_pickle=False) as environment:
        river_vertices, river_triangles = _load_group(environment, "river")
        bridge_vertices, bridge_triangles = _load_group(environment, "bridges")
        rock_vertices, rock_triangles = _load_group(environment, "rocks")

        river_mask = _triangle_mask(
            x_values, y_values, river_vertices, river_triangles
        )
        bridge_mask = _triangle_mask(
            x_values, y_values, bridge_vertices, bridge_triangles
        )
        obstacle_mask = _triangle_mask(
            x_values, y_values, rock_vertices, rock_triangles
        ) if block_rocks else np.zeros_like(walkable)

        if block_vegetation:
            for group_name in ("pineforest", "broadleafforest", "bushes"):
                group_vertices, group_triangles = _load_group(environment, group_name)
                obstacle_mask |= _triangle_mask(
                    x_values, y_values, group_vertices, group_triangles
                )

    mean_spacing = (spacing_x + spacing_y) * 0.5
    river_iterations = _dilation_iterations(river_clearance_m, mean_spacing)
    bridge_iterations = _dilation_iterations(bridge_expansion_m, mean_spacing)
    obstacle_iterations = _dilation_iterations(obstacle_clearance_m, mean_spacing)

    if river_iterations:
        river_mask = ndimage.binary_dilation(river_mask, iterations=river_iterations)
    if bridge_iterations:
        bridge_mask = ndimage.binary_dilation(bridge_mask, iterations=bridge_iterations)
    if obstacle_iterations:
        obstacle_mask = ndimage.binary_dilation(
            obstacle_mask, iterations=obstacle_iterations
        )

    # 강과 장애물은 닫고, 다리 영역은 마지막에 다시 연다.
    walkable &= ~river_mask
    walkable &= ~obstacle_mask
    walkable |= bridge_mask & np.isfinite(z_grid)

    bridge_labels, _ = ndimage.label(bridge_mask)
    return RescuerGridMap(
        x_values=x_values,
        y_values=y_values,
        z_grid=z_grid,
        slope_deg=slope_deg,
        walkable=walkable,
        river_mask=river_mask,
        bridge_mask=bridge_mask,
        bridge_labels=bridge_labels,
        obstacle_mask=obstacle_mask,
        spacing_x=spacing_x,
        spacing_y=spacing_y,
        max_step_height_m=float(max_step_height_m),
    )


def transition_is_walkable(
    grid_map: RescuerGridMap,
    start: GridIndex,
    end: GridIndex,
) -> bool:
    """두 셀 사이가 보행 가능하고 절벽성 높이 단차가 없는지 검사한다."""
    if (
        not grid_map.in_bounds(start)
        or not grid_map.in_bounds(end)
        or not grid_map.walkable[start]
        or not grid_map.walkable[end]
    ):
        return False

    height_change = abs(
        float(grid_map.z_grid[end]) - float(grid_map.z_grid[start])
    )
    return height_change <= float(grid_map.max_step_height_m)


def nearest_walkable_cell(
    grid_map: RescuerGridMap,
    requested: GridIndex,
    max_radius_cells: int = 12,
) -> GridIndex:
    if grid_map.in_bounds(requested) and grid_map.walkable[requested]:
        return requested
    row0, column0 = requested
    for radius in range(1, max_radius_cells + 1):
        candidates = []
        for row in range(row0 - radius, row0 + radius + 1):
            for column in range(column0 - radius, column0 + radius + 1):
                cell = (row, column)
                if not grid_map.in_bounds(cell) or not grid_map.walkable[cell]:
                    continue
                distance = math.hypot(row - row0, column - column0)
                candidates.append((distance, cell))
        if candidates:
            candidates.sort(key=lambda item: item[0])
            return candidates[0][1]
    raise RuntimeError(f"주변에서 보행 가능 셀을 찾지 못했습니다: {requested}")


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

    raise RuntimeError("구조자 A* 경로를 찾지 못했습니다.")


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

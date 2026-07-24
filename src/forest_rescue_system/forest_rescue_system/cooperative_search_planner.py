#!/usr/bin/env python3

"""특정 드론 담당 구역을 활성 드론 수만큼 재분할하는 협동 수색 계획기."""

from itertools import permutations
import json
import math
from pathlib import Path
import uuid

import numpy as np


class CooperativeSearchPlanner:
    """기본 JSON과 Terrain mesh를 사용해 런타임 협동 경로를 생성한다."""

    def __init__(
        self,
        search_plan_path,
        terrain_mesh_path,
        navigation_surface_path=None,
        lane_spacing_m=4.0,
        sample_spacing_m=4.0,
        terrain_profile_spacing_m=1.0,
        transit_altitude_step_m=1.0,
        transit_profile_spacing_m=3.0,
        max_climb_step_m=2.5,
        max_descent_step_m=2.0,
        transit_climb_only=True,
        transit_skip_current_waypoint=True,
        subzone_margin_m=0.6,
    ):
        self.search_plan_path = Path(search_plan_path).expanduser()
        self.terrain_mesh_path = Path(terrain_mesh_path).expanduser()
        self.navigation_surface_path = (
            Path(navigation_surface_path).expanduser()
            if navigation_surface_path
            else self.terrain_mesh_path
        )
        self.lane_spacing_m = max(1.0, float(lane_spacing_m))
        self.sample_spacing_m = max(0.5, float(sample_spacing_m))
        self.terrain_profile_spacing_m = max(
            0.25,
            float(terrain_profile_spacing_m),
        )
        self.transit_altitude_step_m = max(
            0.0,
            float(transit_altitude_step_m),
        )
        self.transit_profile_spacing_m = max(
            1.0,
            float(transit_profile_spacing_m),
        )
        self.max_climb_step_m = max(
            0.5,
            float(max_climb_step_m),
        )
        self.max_descent_step_m = max(
            0.5,
            float(max_descent_step_m),
        )
        # 협동 진입은 담당 구역 도착이 우선이다. 지형이 낮아져도
        # 절대고도를 낮추지 않고, 앞쪽 지형·다리 때문에 필요할 때만
        # 상승한다. 첫 현재 위치 점도 생략해 제자리 고도 변경을 줄인다.
        self.transit_climb_only = bool(transit_climb_only)
        self.transit_skip_current_waypoint = bool(
            transit_skip_current_waypoint
        )
        self.subzone_margin_m = max(0.0, float(subzone_margin_m))

        self.base_plan = None
        self.terrain_vertices = None
        self._grid_x = None
        self._grid_y = None
        self._grid_z = None

    def reload(self):
        self.base_plan = json.loads(
            self.search_plan_path.read_text(encoding="utf-8")
        )
        drone_plans = self.base_plan.get("drones")
        if not isinstance(drone_plans, dict) or not drone_plans:
            raise ValueError("기본 수색 계획의 drones 항목이 비어 있습니다.")

        mesh_path = (
            self.navigation_surface_path
            if self.navigation_surface_path.is_file()
            else self.terrain_mesh_path
        )
        with np.load(mesh_path, allow_pickle=False) as mesh:
            vertices = np.asarray(mesh["vertices"], dtype=np.float64)
        if vertices.ndim != 2 or vertices.shape[1] != 3:
            raise ValueError(
                f"Terrain vertices 형식이 잘못됐습니다: {vertices.shape}"
            )
        if not np.all(np.isfinite(vertices)):
            raise ValueError("Terrain vertices에 유한하지 않은 값이 있습니다.")
        self.terrain_vertices = vertices
        self._build_height_grid()
        return self.base_plan

    def _build_height_grid(self):
        vertices = self.terrain_vertices
        unique_x = np.unique(vertices[:, 0])
        unique_y = np.unique(vertices[:, 1])
        if len(unique_x) * len(unique_y) != len(vertices):
            return

        z_grid = np.full((len(unique_y), len(unique_x)), np.nan)
        x_indices = np.searchsorted(unique_x, vertices[:, 0])
        y_indices = np.searchsorted(unique_y, vertices[:, 1])
        z_grid[y_indices, x_indices] = vertices[:, 2]
        if np.any(~np.isfinite(z_grid)):
            return
        self._grid_x = unique_x
        self._grid_y = unique_y
        self._grid_z = z_grid

    def terrain_height(self, x, y):
        """정규 격자는 bilinear, 그 외 mesh는 최근접 정점 높이를 쓴다."""
        x = float(x)
        y = float(y)
        if self._grid_z is None:
            delta = self.terrain_vertices[:, :2] - np.asarray([x, y])
            index = int(np.argmin(np.einsum("ij,ij->i", delta, delta)))
            return float(self.terrain_vertices[index, 2])

        x_value = min(max(x, self._grid_x[0]), self._grid_x[-1])
        y_value = min(max(y, self._grid_y[0]), self._grid_y[-1])
        x_high = int(np.searchsorted(self._grid_x, x_value, side="right"))
        y_high = int(np.searchsorted(self._grid_y, y_value, side="right"))
        x_high = min(max(1, x_high), len(self._grid_x) - 1)
        y_high = min(max(1, y_high), len(self._grid_y) - 1)
        x_low = x_high - 1
        y_low = y_high - 1

        x0, x1 = self._grid_x[x_low], self._grid_x[x_high]
        y0, y1 = self._grid_y[y_low], self._grid_y[y_high]
        tx = 0.0 if abs(x1 - x0) < 1.0e-9 else (x_value - x0) / (x1 - x0)
        ty = 0.0 if abs(y1 - y0) < 1.0e-9 else (y_value - y0) / (y1 - y0)
        z00 = self._grid_z[y_low, x_low]
        z10 = self._grid_z[y_low, x_high]
        z01 = self._grid_z[y_high, x_low]
        z11 = self._grid_z[y_high, x_high]
        return float(
            (1.0 - tx) * (1.0 - ty) * z00
            + tx * (1.0 - ty) * z10
            + (1.0 - tx) * ty * z01
            + tx * ty * z11
        )

    @staticmethod
    def _sample_axis(start, stop, spacing):
        distance = abs(float(stop) - float(start))
        count = max(2, int(math.ceil(distance / spacing)) + 1)
        return [float(value) for value in np.linspace(start, stop, count)]

    def _segment_max_height(self, start_xy, end_xy):
        distance = math.hypot(
            float(end_xy[0]) - float(start_xy[0]),
            float(end_xy[1]) - float(start_xy[1]),
        )
        count = max(
            2,
            int(math.ceil(distance / self.terrain_profile_spacing_m)) + 1,
        )
        return max(
            self.terrain_height(
                start_xy[0] + (end_xy[0] - start_xy[0]) * ratio,
                start_xy[1] + (end_xy[1] - start_xy[1]) * ratio,
            )
            for ratio in np.linspace(0.0, 1.0, count)
        )

    @staticmethod
    def _append_unique(route, x, y):
        point = (float(x), float(y))
        if route and math.hypot(
            route[-1][0] - point[0],
            route[-1][1] - point[1],
        ) < 1.0e-6:
            return
        route.append(point)

    def _lawnmower_xy(self, bounds):
        x_min, x_max, y_min, y_max = [float(value) for value in bounds]
        margin = min(
            self.subzone_margin_m,
            max(0.0, (x_max - x_min) * 0.15),
            max(0.0, (y_max - y_min) * 0.15),
        )
        x0, x1 = x_min + margin, x_max - margin
        y0, y1 = y_min + margin, y_max - margin
        if x0 >= x1:
            x0, x1 = x_min, x_max
        if y0 >= y1:
            y0, y1 = y_min, y_max

        route = []
        width = x1 - x0
        height = y1 - y0
        if width >= height:
            lanes = self._sample_axis(y0, y1, self.lane_spacing_m)
            for lane_index, lane_y in enumerate(lanes):
                xs = self._sample_axis(
                    x0 if lane_index % 2 == 0 else x1,
                    x1 if lane_index % 2 == 0 else x0,
                    self.sample_spacing_m,
                )
                for x_value in xs:
                    self._append_unique(route, x_value, lane_y)
        else:
            lanes = self._sample_axis(x0, x1, self.lane_spacing_m)
            for lane_index, lane_x in enumerate(lanes):
                ys = self._sample_axis(
                    y0 if lane_index % 2 == 0 else y1,
                    y1 if lane_index % 2 == 0 else y0,
                    self.sample_spacing_m,
                )
                for y_value in ys:
                    self._append_unique(route, lane_x, y_value)
        if len(route) < 2:
            raise ValueError(f"소구역 경로를 만들 수 없습니다: {bounds}")
        return route

    def _smooth_altitudes(self, required_z):
        if not required_z:
            return []
        smoothed = [float(value) for value in required_z]
        for index in range(len(smoothed) - 2, -1, -1):
            smoothed[index] = max(
                smoothed[index],
                smoothed[index + 1] - self.max_climb_step_m,
            )
        for index in range(1, len(smoothed)):
            smoothed[index] = max(
                smoothed[index],
                smoothed[index - 1] - self.max_descent_step_m,
            )
        return smoothed

    def _sample_segment_xy(self, start_xy, end_xy, spacing_m):
        distance = self._distance_xy(start_xy, end_xy)
        count = max(2, int(math.ceil(distance / spacing_m)) + 1)
        return [
            (
                float(start_xy[0] + (end_xy[0] - start_xy[0]) * ratio),
                float(start_xy[1] + (end_xy[1] - start_xy[1]) * ratio),
            )
            for ratio in np.linspace(0.0, 1.0, count)
        ]

    def _safe_world_route(self, route_xy, clearance_m):
        ground_z = [self.terrain_height(x, y) for x, y in route_xy]
        segment_safe_z = [
            self._segment_max_height(start, end) + clearance_m
            for start, end in zip(route_xy, route_xy[1:])
        ]
        required_z = []
        for index, ground in enumerate(ground_z):
            z_value = ground + clearance_m
            if index > 0:
                z_value = max(z_value, segment_safe_z[index - 1])
            if index < len(segment_safe_z):
                z_value = max(z_value, segment_safe_z[index])
            required_z.append(float(z_value))

        smoothed_z = self._smooth_altitudes(required_z)
        return [
            [float(x), float(y), float(z_value)]
            for (x, y), z_value in zip(route_xy, smoothed_z)
        ]

    def _terrain_following_transit_route(
        self,
        current,
        entry,
        clearance_m,
        altitude_offset_m,
    ):
        """수직 상승 후 장거리 수평이동 대신 지형을 따라 진입한다."""
        route_xy = self._sample_segment_xy(
            current[:2],
            entry[:2],
            self.transit_profile_spacing_m,
        )
        safe_route = self._safe_world_route(
            route_xy,
            clearance_m + altitude_offset_m,
        )

        # 협동 진입에서는 낮아지는 지형을 따라 하강하지 않는다. 현재
        # World Z를 기준으로 시작해, 이후 각 점은 앞쪽 지형·구조물의
        # 안전고도가 더 높을 때만 상승한다. 한 번 올라간 고도는 첫
        # 협동 수색점에 도착할 때까지 유지한다.
        if self.transit_climb_only:
            maintained_z = float(current[2])
            for point in safe_route:
                maintained_z = max(maintained_z, float(point[2]))
                point[2] = maintained_z
        else:
            safe_route[0][2] = max(
                float(current[2]),
                float(safe_route[0][2]),
            )

        safe_route[-1][2] = max(
            float(entry[2]),
            float(safe_route[-1][2]),
        )

        # 첫 점은 현재 XY와 같아서 고도만 바뀌는 제자리 명령이 될 수
        # 있다. 두 번째 점부터 보내면 전진과 필요한 상승을 동시에 한다.
        if self.transit_skip_current_waypoint and len(safe_route) > 1:
            safe_route = safe_route[1:]
        return safe_route

    @staticmethod
    def _partition(bounds, count):
        x_min, x_max, y_min, y_max = [float(value) for value in bounds]
        width = x_max - x_min
        height = y_max - y_min
        if count < 1 or width <= 0.0 or height <= 0.0:
            raise ValueError(f"협동 구역 분할 입력이 잘못됐습니다: {bounds}, {count}")
        subzones = []
        if width >= height:
            edges = np.linspace(x_min, x_max, count + 1)
            for index in range(count):
                subzones.append(
                    [float(edges[index]), float(edges[index + 1]), y_min, y_max]
                )
        else:
            edges = np.linspace(y_min, y_max, count + 1)
            for index in range(count):
                subzones.append(
                    [x_min, x_max, float(edges[index]), float(edges[index + 1])]
                )
        return subzones

    @staticmethod
    def _center(bounds):
        x_min, x_max, y_min, y_max = bounds
        return ((x_min + x_max) * 0.5, (y_min + y_max) * 0.5)

    @staticmethod
    def _contains(bounds, position):
        x_min, x_max, y_min, y_max = bounds
        return x_min <= position[0] <= x_max and y_min <= position[1] <= y_max

    @staticmethod
    def _distance_xy(first, second):
        return math.hypot(first[0] - second[0], first[1] - second[1])

    def _assign_subzones(self, owner_id, active_ids, world_positions, subzones):
        owner_position = world_positions[owner_id]
        containing = [
            index
            for index, bounds in enumerate(subzones)
            if self._contains(bounds, owner_position)
        ]
        if containing:
            owner_subzone = containing[0]
        else:
            owner_subzone = min(
                range(len(subzones)),
                key=lambda index: self._distance_xy(
                    owner_position,
                    self._center(subzones[index]),
                ),
            )

        assignments = {owner_id: owner_subzone}
        helper_ids = [item for item in active_ids if item != owner_id]
        remaining = [
            index for index in range(len(subzones)) if index != owner_subzone
        ]
        if not helper_ids:
            return assignments

        best = None
        for candidate in permutations(remaining, len(helper_ids)):
            total = 0.0
            for drone_id, subzone_index in zip(helper_ids, candidate):
                path_xy = self._lawnmower_xy(subzones[subzone_index])
                position = world_positions[drone_id]
                total += min(
                    self._distance_xy(position, path_xy[0]),
                    self._distance_xy(position, path_xy[-1]),
                )
            if best is None or total < best[0]:
                best = (total, candidate)
        for drone_id, subzone_index in zip(helper_ids, best[1]):
            assignments[drone_id] = subzone_index
        return assignments

    def create_plan(self, target_drone_id, active_drone_ids, world_positions):
        if self.base_plan is None or self.terrain_vertices is None:
            self.reload()
        drone_plans = self.base_plan["drones"]
        if target_drone_id not in drone_plans:
            raise KeyError(f"기본 계획에 {target_drone_id} 구역이 없습니다.")
        if target_drone_id not in active_drone_ids:
            raise ValueError(
                f"대상 구역 담당 드론이 협동 수색에 참여할 수 없습니다: "
                f"{target_drone_id}"
            )
        missing = [item for item in active_drone_ids if item not in world_positions]
        if missing:
            raise ValueError(f"현재 위치가 없는 드론: {missing}")

        target_bounds = [
            float(value)
            for value in drone_plans[target_drone_id]["zone_bounds_xy"]
        ]
        # zone_bounds의 X가 Terrain 전체 경계인 이전 계획과도 호환하되,
        # 실제 기본 수색 경로가 사용하는 search_area_bounds 안으로 제한한다.
        search_area = self.base_plan.get("search_area_bounds_xy")
        if isinstance(search_area, list) and len(search_area) == 4:
            target_bounds[0] = max(target_bounds[0], float(search_area[0]))
            target_bounds[1] = min(target_bounds[1], float(search_area[1]))
            target_bounds[2] = max(target_bounds[2], float(search_area[2]))
            target_bounds[3] = min(target_bounds[3], float(search_area[3]))
        subzones = self._partition(target_bounds, len(active_drone_ids))
        assignments = self._assign_subzones(
            target_drone_id,
            active_drone_ids,
            world_positions,
            subzones,
        )
        clearance = float(self.base_plan.get("search_clearance_m", 6.0))
        ordered_for_altitude = [target_drone_id] + sorted(
            item for item in active_drone_ids if item != target_drone_id
        )
        altitude_offsets = {
            drone_id: index * self.transit_altitude_step_m
            for index, drone_id in enumerate(ordered_for_altitude)
        }

        plan_id = f"coop_{uuid.uuid4().hex[:8]}"
        plan = {
            "format_version": 1,
            "plan_id": plan_id,
            "plan_type": "cooperative_search",
            "target_drone_id": target_drone_id,
            "target_zone_bounds_xy": target_bounds,
            "active_drone_ids": list(active_drone_ids),
            "search_repeat_mode": "infinite",
            "transit_altitude_mode": (
                "climb_only_absolute_world_z"
                if self.transit_climb_only
                else "terrain_following"
            ),
            "transit_skip_current_waypoint": bool(
                self.transit_skip_current_waypoint
            ),
            "assignments": {},
        }

        for drone_id in active_drone_ids:
            subzone_index = assignments[drone_id]
            bounds = subzones[subzone_index]
            route_xy = self._lawnmower_xy(bounds)
            current = world_positions[drone_id]
            if self._distance_xy(current, route_xy[-1]) < self._distance_xy(
                current,
                route_xy[0],
            ):
                route_xy.reverse()
            search_route = self._safe_world_route(route_xy, clearance)
            entry = search_route[0]
            transit_route = self._terrain_following_transit_route(
                current=current,
                entry=entry,
                clearance_m=clearance,
                altitude_offset_m=altitude_offsets[drone_id],
            )
            plan["assignments"][drone_id] = {
                "drone_id": drone_id,
                "subzone_index": int(subzone_index),
                "subzone_bounds_xy": [float(value) for value in bounds],
                "transit_altitude_offset_m": float(altitude_offsets[drone_id]),
                "entry_world_enu": list(entry),
                "transit_waypoints_world_enu": transit_route,
                "search_waypoints_world_enu": search_route,
            }
        return plan

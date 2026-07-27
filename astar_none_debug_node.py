#!/usr/bin/env python3

"""로컬 A*=NONE 원인을 터미널에 설명하는 독립 ROS 2 디버깅 노드."""

import heapq
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32


class AstarNoneDebugNode(Node):
    """빈 A* Path를 감지해 실제 플래너와 같은 조건으로 실패 단계를 재현한다."""

    def __init__(self):
        super().__init__("astar_none_debug_node")

        self.declare_parameter("drone_number", 1)
        self.declare_parameter("accumulated_cloud_topic", "")
        self.declare_parameter("local_astar_path_topic", "")
        self.declare_parameter("movement_direction_topic", "")
        self.declare_parameter("local_grid_size_m", 12.0)
        self.declare_parameter("local_grid_resolution_m", 0.25)
        self.declare_parameter("obstacle_inflation_radius_m", 0.7)
        self.declare_parameter("local_planner_goal_distance_m", 4.5)
        self.declare_parameter("local_planner_lookahead_m", 0.75)
        self.declare_parameter("planner_start_release_radius_m", 0.50)
        self.declare_parameter("local_planner_min_forward_progress_m", 0.75)
        self.declare_parameter("minimum_report_period_sec", 1.0)
        self.declare_parameter("cloud_stale_sec", 1.5)

        number = int(self.get_parameter("drone_number").value)
        prefix = f"/drone_{number:02d}"
        cloud_topic = self._topic_or_default(
            "accumulated_cloud_topic",
            f"{prefix}/obstacle/accumulated_cloud_body",
        )
        path_topic = self._topic_or_default(
            "local_astar_path_topic",
            f"{prefix}/obstacle/local_astar_path",
        )
        direction_topic = self._topic_or_default(
            "movement_direction_topic",
            f"{prefix}/navigation/direction_body_rad",
        )

        self.latest_x = np.empty(0, dtype=np.float32)
        self.latest_y = np.empty(0, dtype=np.float32)
        self.latest_cloud_wall_time = float("-inf")
        self.target_angle_rad = 0.0
        self.last_report_wall_time = float("-inf")
        self.empty_path_count = 0
        self.valid_path_count = 0

        self.create_subscription(
            PointCloud2,
            cloud_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32,
            direction_topic,
            self._direction_callback,
            10,
        )
        self.create_subscription(Path, path_topic, self._path_callback, 10)

        self.get_logger().info(
            f"[A* NONE 디버거] 드론 {number:02d} 감시 시작 | "
            f"cloud={cloud_topic}, path={path_topic}, direction={direction_topic}"
        )
        self.get_logger().info(
            "실행 중인 obstacle_monitor의 파라미터와 디버거 파라미터를 "
            "동일하게 맞춰야 정확합니다."
        )

    def _topic_or_default(self, parameter_name, default):
        value = str(self.get_parameter(parameter_name).value).strip()
        return value if value else default

    def _cloud_callback(self, message):
        try:
            points = point_cloud2.read_points_numpy(
                message,
                field_names=("x", "y"),
                skip_nans=True,
            )
            points = np.asarray(points)
            if points.size == 0:
                self.latest_x = np.empty(0, dtype=np.float32)
                self.latest_y = np.empty(0, dtype=np.float32)
            else:
                points = points.reshape(-1, 2)
                self.latest_x = points[:, 0].astype(np.float32, copy=False)
                self.latest_y = points[:, 1].astype(np.float32, copy=False)
            self.latest_cloud_wall_time = time.monotonic()
        except Exception as error:
            self.get_logger().warning(f"누적 PointCloud 읽기 실패: {error}")

    def _direction_callback(self, message):
        if math.isfinite(float(message.data)):
            self.target_angle_rad = float(message.data)

    def _path_callback(self, message):
        if message.poses:
            self.valid_path_count += 1
            return

        self.empty_path_count += 1
        now = time.monotonic()
        period = max(
            0.0,
            float(self.get_parameter("minimum_report_period_sec").value),
        )
        if now - self.last_report_wall_time < period:
            return
        self.last_report_wall_time = now
        self._diagnose(now)

    def _diagnose(self, now):
        cloud_age = now - self.latest_cloud_wall_time
        stale_limit = float(self.get_parameter("cloud_stale_sec").value)
        if self.latest_x.size == 0:
            self._report(
                "EMPTY_INPUT",
                [
                    "누적 PointCloud에 유효한 XY 포인트가 없습니다.",
                    "mapper 노드 생존 여부, 토픽 이름, TF 실패 로그를 확인하세요.",
                ],
            )
            return
        if cloud_age > stale_limit:
            self._report(
                "STALE_CLOUD",
                [
                    f"마지막 누적 PointCloud 수신 후 {cloud_age:.2f}초 경과",
                    f"허용 기준={stale_limit:.2f}초",
                    "mapper 처리 지연, TF 대기 또는 토픽 중단을 확인하세요.",
                ],
            )
            return

        size_m = float(self.get_parameter("local_grid_size_m").value)
        resolution = float(
            self.get_parameter("local_grid_resolution_m").value
        )
        if size_m <= 2.0 or resolution <= 0.05:
            self._report(
                "INVALID_GRID_PARAMETER",
                [f"grid_size={size_m:.2f}m, resolution={resolution:.3f}m"],
            )
            return

        cell_count = max(21, int(round(size_m / resolution)))
        if cell_count % 2 == 0:
            cell_count += 1
        center = cell_count // 2
        half_size = center * resolution
        raw = np.zeros((cell_count, cell_count), dtype=bool)
        valid = (
            np.isfinite(self.latest_x)
            & np.isfinite(self.latest_y)
            & (np.abs(self.latest_x) <= half_size)
            & (np.abs(self.latest_y) <= half_size)
        )
        columns = (
            np.rint(self.latest_x[valid] / resolution).astype(int) + center
        )
        rows = center - np.rint(
            self.latest_y[valid] / resolution
        ).astype(int)
        inside = (
            (rows >= 0)
            & (rows < cell_count)
            & (columns >= 0)
            & (columns < cell_count)
        )
        raw[rows[inside], columns[inside]] = True

        inflation_m = float(
            self.get_parameter("obstacle_inflation_radius_m").value
        )
        inflation_cells = int(math.ceil(inflation_m / resolution))
        occupied = self._inflate_grid(raw, inflation_cells)
        occupied_before_release = occupied.copy()
        start = (center, center)
        start_raw = bool(raw[start])
        start_inflated = bool(occupied[start])

        release_m = float(
            self.get_parameter("planner_start_release_radius_m").value
        )
        release_cells = max(0, int(math.floor(release_m / resolution)))
        for row_offset in range(-release_cells, release_cells + 1):
            for column_offset in range(
                -release_cells, release_cells + 1
            ):
                if row_offset**2 + column_offset**2 > release_cells**2:
                    continue
                cell = (center + row_offset, center + column_offset)
                if not raw[cell]:
                    occupied[cell] = False

        common = [
            f"입력 포인트={self.latest_x.size}, 격자 내 점유 셀={np.count_nonzero(raw)}",
            f"inflation={inflation_m:.2f}m({inflation_cells}셀), "
            f"팽창 점유율={100.0 * np.mean(occupied_before_release):.1f}%",
            f"시작 셀: 원본점유={start_raw}, 팽창점유={start_inflated}, "
            f"해제 후 점유={bool(occupied[start])}",
        ]
        if occupied[start]:
            self._report(
                "START_BLOCKED",
                common
                + [
                    "드론 중심 셀이 실제 장애물 포인트로 점유되어 "
                    "start-release로도 열리지 않습니다.",
                    "self filter, 높이 필터 또는 근접 포인트 오염을 확인하세요.",
                ],
            )
            return

        goal_distance = min(
            float(
                self.get_parameter(
                    "local_planner_goal_distance_m"
                ).value
            ),
            half_size - resolution,
        )
        goal_x = math.cos(self.target_angle_rad) * goal_distance
        goal_y = math.sin(self.target_angle_rad) * goal_distance
        requested_goal = (
            center - int(round(goal_y / resolution)),
            center + int(round(goal_x / resolution)),
        )
        requested_goal_blocked = bool(occupied[requested_goal])
        goal = self._nearest_free_cell(
            occupied, requested_goal, max_radius_cells=16
        )
        if goal is None:
            self._report(
                "GOAL_SURROUNDED",
                common
                + [
                    f"요청 목표={goal_distance:.2f}m, "
                    f"방향={math.degrees(self.target_angle_rad):.1f}°",
                    "목표 주변 16셀 안에서 자유 셀을 찾지 못했습니다.",
                    "inflation이 겹쳐 목표 구역 전체가 닫힌 상태입니다.",
                ],
            )
            return

        goal_shift_m = math.hypot(
            goal[0] - requested_goal[0],
            goal[1] - requested_goal[1],
        ) * resolution
        path = self._astar_grid(occupied, start, goal)
        if len(path) < 2:
            reachable = self._reachable_cell_count(occupied, start)
            free_cells = int(np.count_nonzero(~occupied))
            self._report(
                "DISCONNECTED_BY_INFLATION",
                common
                + [
                    f"목표 셀 최초점유={requested_goal_blocked}, "
                    f"가까운 자유 셀 이동={goal_shift_m:.2f}m",
                    f"시작점 연결 자유 셀={reachable}/{free_cells} "
                    f"({100.0 * reachable / max(1, free_cells):.1f}%)",
                    "시작 영역과 목표 영역 사이가 팽창 장애물로 끊겼습니다.",
                ],
            )
            return

        lookahead = max(
            resolution,
            float(self.get_parameter("local_planner_lookahead_m").value),
        )
        selected = path[1]
        traveled = 0.0
        previous = path[0]
        for cell in path[1:]:
            traveled += math.hypot(
                cell[0] - previous[0], cell[1] - previous[1]
            ) * resolution
            if traveled > lookahead + 1.0e-9:
                break
            if self._grid_line_is_free(occupied, start, cell):
                selected = cell
            previous = cell

        detour_x = (selected[1] - center) * resolution
        detour_y = (center - selected[0]) * resolution
        detour_distance = math.hypot(detour_x, detour_y)
        forward = (
            detour_x * math.cos(self.target_angle_rad)
            + detour_y * math.sin(self.target_angle_rad)
        )
        minimum_forward = float(
            self.get_parameter(
                "local_planner_min_forward_progress_m"
            ).value
        )
        path_length = self._path_length(path, resolution)
        details = common + [
            f"A* 셀 경로는 존재: 길이={path_length:.2f}m, 셀={len(path)}",
            f"lookahead 결과=({detour_x:.2f}, {detour_y:.2f})m, "
            f"거리={detour_distance:.2f}m",
            f"Waypoint 방향 전진량={forward:.2f}m, "
            f"요구={minimum_forward:.2f}m",
        ]
        if detour_distance < 0.5:
            self._report(
                "DETOUR_TOO_SHORT",
                details
                + [
                    "경로는 있지만 선택된 첫 안전 구간이 0.5m보다 짧아 "
                    "최종적으로 NONE 처리됩니다."
                ],
            )
        elif forward < minimum_forward:
            self._report(
                "INSUFFICIENT_FORWARD_PROGRESS",
                details
                + [
                    "경로는 있지만 첫 구간이 측면 우회에 가까워 "
                    "전진량 검사에서 NONE 처리됩니다."
                ],
            )
        else:
            self._report(
                "DEBUG_REPRODUCTION_OK_BUT_RUNTIME_NONE",
                details
                + [
                    "현재 누적 cloud로는 유효 경로가 재현됩니다.",
                    "실제 플래너가 latest_scan을 사용했거나 cloud 시각/파라미터가 "
                    "서로 달랐을 가능성이 큽니다.",
                ],
            )

    def _report(self, reason, details):
        summary = (
            f"[A*=NONE 원인] {reason} | "
            f"empty={self.empty_path_count}, valid={self.valid_path_count}"
        )
        self.get_logger().warning(summary)
        for detail in details:
            self.get_logger().warning(f"  - {detail}")

    @staticmethod
    def _path_length(path, resolution):
        return sum(
            math.hypot(b[0] - a[0], b[1] - a[1]) * resolution
            for a, b in zip(path, path[1:])
        )

    @staticmethod
    def _reachable_cell_count(occupied, start):
        if occupied[start]:
            return 0
        rows, columns = occupied.shape
        stack = [start]
        visited = {start}
        while stack:
            row, column = stack.pop()
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    neighbor = (row + dr, column + dc)
                    if not (
                        0 <= neighbor[0] < rows
                        and 0 <= neighbor[1] < columns
                    ):
                        continue
                    if occupied[neighbor] or neighbor in visited:
                        continue
                    visited.add(neighbor)
                    stack.append(neighbor)
        return len(visited)

    @staticmethod
    def _inflate_grid(occupied, radius_cells):
        if radius_cells <= 0 or not np.any(occupied):
            return occupied.copy()
        inflated = occupied.copy()
        rows, columns = occupied.shape
        for row_offset in range(-radius_cells, radius_cells + 1):
            for column_offset in range(
                -radius_cells, radius_cells + 1
            ):
                if row_offset**2 + column_offset**2 > radius_cells**2:
                    continue
                sr0 = max(0, -row_offset)
                sr1 = min(rows, rows - row_offset)
                sc0 = max(0, -column_offset)
                sc1 = min(columns, columns - column_offset)
                tr0, tr1 = sr0 + row_offset, sr1 + row_offset
                tc0, tc1 = sc0 + column_offset, sc1 + column_offset
                inflated[tr0:tr1, tc0:tc1] |= occupied[
                    sr0:sr1, sc0:sc1
                ]
        return inflated

    @staticmethod
    def _nearest_free_cell(occupied, goal, max_radius_cells):
        rows, columns = occupied.shape
        goal_row = min(rows - 1, max(0, int(goal[0])))
        goal_col = min(columns - 1, max(0, int(goal[1])))
        if not occupied[goal_row, goal_col]:
            return goal_row, goal_col
        for radius in range(1, max_radius_cells + 1):
            candidates = []
            for row in range(
                max(0, goal_row - radius),
                min(rows, goal_row + radius + 1),
            ):
                for column in range(
                    max(0, goal_col - radius),
                    min(columns, goal_col + radius + 1),
                ):
                    if max(
                        abs(row - goal_row), abs(column - goal_col)
                    ) != radius:
                        continue
                    if not occupied[row, column]:
                        candidates.append((row, column))
            if candidates:
                return min(
                    candidates,
                    key=lambda cell: (cell[0] - goal_row) ** 2
                    + (cell[1] - goal_col) ** 2,
                )
        return None

    @staticmethod
    def _astar_grid(occupied, start, goal):
        rows, columns = occupied.shape
        neighbors = (
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        )
        queue = [(0.0, start)]
        came_from = {}
        cost_so_far = {start: 0.0}
        while queue:
            _priority, current = heapq.heappop(queue)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            for dr, dc, move_cost in neighbors:
                neighbor = (current[0] + dr, current[1] + dc)
                if not (
                    0 <= neighbor[0] < rows
                    and 0 <= neighbor[1] < columns
                ):
                    continue
                if occupied[neighbor]:
                    continue
                if dr != 0 and dc != 0:
                    if (
                        occupied[current[0] + dr, current[1]]
                        or occupied[current[0], current[1] + dc]
                    ):
                        continue
                new_cost = cost_so_far[current] + move_cost
                if (
                    neighbor not in cost_so_far
                    or new_cost < cost_so_far[neighbor]
                ):
                    cost_so_far[neighbor] = new_cost
                    heuristic = math.hypot(
                        goal[0] - neighbor[0], goal[1] - neighbor[1]
                    )
                    heapq.heappush(
                        queue, (new_cost + heuristic, neighbor)
                    )
                    came_from[neighbor] = current
        return []

    @staticmethod
    def _grid_line_is_free(occupied, start, end):
        row_delta = int(end[0]) - int(start[0])
        column_delta = int(end[1]) - int(start[1])
        steps = max(abs(row_delta), abs(column_delta)) * 2 + 1
        rows, columns = occupied.shape
        for ratio in np.linspace(0.0, 1.0, max(2, steps)):
            row = int(round(start[0] + row_delta * ratio))
            column = int(round(start[1] + column_delta * ratio))
            if not (0 <= row < rows and 0 <= column < columns):
                return False
            if occupied[row, column]:
                return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = AstarNoneDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

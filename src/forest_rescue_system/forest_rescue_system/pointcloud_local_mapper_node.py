#!/usr/bin/env python3

"""최근 PointCloud를 map 좌표에 누적해 로컬 코스트맵을 만든다.

LiDAR 프레임을 그대로 합치면 드론 이동에 따라 같은 나무가 여러 위치에
늘어져 보인다. 이 노드는 각 스캔을 측정 시점의 TF로 map 좌표에 고정한 뒤
최근 일정 시간만 유지하고, 현재 LiDAR 좌표로 다시 변환한 누적 점군을 로컬
A*에 제공한다.
"""

from collections import deque
import math
import time

import numpy as np
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from forest_rescue_system.log_utils import TimestampedNode


class PointCloudLocalMapperNode(TimestampedNode):
    """3초 슬라이딩 PointCloud와 rolling 2D costmap을 발행한다."""

    def __init__(self):
        super().__init__("pointcloud_local_mapper_node")

        self.declare_parameter("drone_id", "quadrotor_01")
        self.declare_parameter("point_cloud_topic", "/quadrotor_01/point_cloud")
        self.declare_parameter("mission_state_topic", "/mission/state")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("body_frame", "quadrotor_01/base_scan")
        self.declare_parameter(
            "accumulated_cloud_topic",
            "/drone_01/obstacle/accumulated_cloud",
        )
        self.declare_parameter(
            "accumulated_cloud_body_topic",
            "/drone_01/obstacle/accumulated_cloud_body",
        )
        self.declare_parameter(
            "local_costmap_topic",
            "/drone_01/obstacle/local_costmap",
        )
        # 오래된 나무 voxel이 좌우 판단을 계속 끌고 가지 않도록 최근
        # 1.5초만 누적한다. A*는 이 짧은 로컬 지도를 방향 힌트로 사용한다.
        self.declare_parameter("accumulation_sec", 1.5)
        self.declare_parameter("processing_period_sec", 0.10)
        self.declare_parameter("publish_period_sec", 0.20)
        # PointCloud가 TF보다 수십 ms 먼저 도착할 수 있으므로 메시지를 잠시
        # 보류했다가 같은 시각의 TF가 들어오면 처리한다.
        self.declare_parameter("tf_retry_period_sec", 0.02)
        self.declare_parameter("tf_exact_wait_sec", 1.00)
        # 누적 지도는 위치 정확도가 중요하므로 exact timestamp TF만 허용한다.
        # 다른 시각의 latest TF로 대체하는 기능은 의도적으로 두지 않는다.
        self.declare_parameter("pending_cloud_queue_size", 30)
        self.declare_parameter("max_pending_clouds_per_cycle", 8)
        self.declare_parameter("voxel_size_m", 0.25)
        self.declare_parameter("minimum_voxel_observations", 2)
        self.declare_parameter("maximum_points_per_scan", 30000)
        self.declare_parameter("minimum_height_m", -1.2)
        self.declare_parameter("maximum_height_m", 1.8)
        self.declare_parameter("local_costmap_size_m", 14.0)
        self.declare_parameter("local_costmap_resolution_m", 0.25)
        self.declare_parameter("obstacle_inflation_radius_m", 1.10)
        self.declare_parameter("self_filter_radius_m", 0.65)
        self.declare_parameter(
            "active_mission_states",
            [
                "INITIAL_TAKEOFF",
                "INITIAL_HOVER",
                "READY",
                "SEARCHING",
                "COOP_SEARCH_PREPARING",
                "COOP_SEARCH_TRANSIT",
                "COOP_SEARCHING",
                "VICTIM_DETECTED",
                "RETURNING_NO_VICTIM",
                "COMPLETE",
                "MISSION_FAILED",
            ],
        )
        self.declare_parameter("warning_period_sec", 5.0)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.body_frame = str(self.get_parameter("body_frame").value)
        self.accumulation_sec = max(
            0.2, float(self.get_parameter("accumulation_sec").value)
        )
        self.processing_period_sec = max(
            0.02, float(self.get_parameter("processing_period_sec").value)
        )
        self.publish_period_sec = max(
            0.05, float(self.get_parameter("publish_period_sec").value)
        )
        self.warning_period_sec = max(
            0.5, float(self.get_parameter("warning_period_sec").value)
        )
        self.active_mission_states = {
            str(state).strip().upper()
            for state in self.get_parameter("active_mission_states").value
        }

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_cloud_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("accumulated_cloud_topic").value),
            qos_profile_sensor_data,
        )
        self.body_cloud_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("accumulated_cloud_body_topic").value),
            qos_profile_sensor_data,
        )
        self.costmap_publisher = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("local_costmap_topic").value),
            10,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("point_cloud_topic").value),
            self._point_cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mission_state_topic").value),
            self._mission_state_callback,
            10,
        )
        self.create_timer(self.publish_period_sec, self._publish_accumulated_map)
        self.create_timer(
            max(0.01, float(self.get_parameter("tf_retry_period_sec").value)),
            self._process_pending_clouds,
        )

        # TF가 아직 해당 PointCloud 시각까지 도착하지 않았으면 이 큐에 잠시
        # 보관한다. 센서 콜백에서 lookup_transform(timeout)을 기다리면 단일
        # executor가 TF 콜백을 처리하지 못할 수 있으므로 비동기 재시도한다.
        self.pending_clouds = deque(
            maxlen=max(
                2, int(self.get_parameter("pending_cloud_queue_size").value)
            )
        )

        # 각 원소는 (측정 시각, map 기준 voxel key 배열)이다. 한 스캔에서
        # 같은 voxel은 먼저 제거하므로 observation count는 프레임 수에 가깝다.
        self.cloud_buffer = deque()
        self.monitoring_enabled = False
        self.last_scan_stamp_sec = float("-inf")
        self.last_accepted_scan_stamp_sec = float("-inf")
        self.last_warning_wall = float("-inf")
        self.last_publish_stats_wall = float("-inf")
        self.received_cloud_count = 0
        self.sampled_cloud_count = 0
        self.exact_tf_count = 0
        self.dropped_tf_count = 0
        self.pending_overflow_count = 0
        self.exact_wait_total_sec = 0.0
        self.exact_wait_max_sec = 0.0

        self.get_logger().info(
            f"{self.get_parameter('drone_id').value} 3초 PointCloud 누적기 시작: "
            f"window={self.accumulation_sec:.1f}s, map={self.map_frame}, "
            f"body={self.body_frame}, tf_policy=exact_only"
        )

    def _mission_state_callback(self, message):
        state = str(message.data).strip().upper()
        enabled = state in self.active_mission_states
        if enabled == self.monitoring_enabled:
            return

        self.monitoring_enabled = enabled
        self.cloud_buffer.clear()
        self.pending_clouds.clear()
        self.last_scan_stamp_sec = float("-inf")
        self.last_accepted_scan_stamp_sec = float("-inf")
        if enabled:
            self._reset_diagnostics()
            self.get_logger().info(
                f"PointCloud 누적 활성화: mission_state={state}"
            )
        else:
            self._publish_empty_outputs()
            self.get_logger().info(
                f"PointCloud 누적 비활성화: mission_state={state}"
            )

    def _reset_diagnostics(self):
        self.received_cloud_count = 0
        self.sampled_cloud_count = 0
        self.exact_tf_count = 0
        self.dropped_tf_count = 0
        self.pending_overflow_count = 0
        self.exact_wait_total_sec = 0.0
        self.exact_wait_max_sec = 0.0
        self.last_publish_stats_wall = float("-inf")

    def _point_cloud_callback(self, message):
        if not self.monitoring_enabled:
            return

        self.received_cloud_count += 1
        stamp_sec = self._stamp_to_seconds(message.header.stamp)
        if stamp_sec + 1.0e-6 < self.last_scan_stamp_sec:
            # Isaac Sim 재시작 또는 /clock 되감기 시 이전 실행 데이터를 폐기한다.
            self.cloud_buffer.clear()
            self.pending_clouds.clear()
            self.last_accepted_scan_stamp_sec = float("-inf")
            self._reset_diagnostics()

        self.last_scan_stamp_sec = stamp_sec

        # wall time이 아니라 센서의 simulation timestamp로 입력을 제한한다.
        # 시뮬레이션이 느리게 실행돼도 3초 윈도우에 과도한 프레임이 쌓이지 않는다.
        if (
            stamp_sec - self.last_accepted_scan_stamp_sec
            < self.processing_period_sec - 1.0e-6
        ):
            return
        self.last_accepted_scan_stamp_sec = stamp_sec
        self.sampled_cloud_count += 1

        if len(self.pending_clouds) == self.pending_clouds.maxlen:
            self.pending_clouds.popleft()
            self.pending_overflow_count += 1
        self.pending_clouds.append((time.monotonic(), message))

    def _process_pending_clouds(self):
        """대기 중 PointCloud를 exact timestamp TF가 준비된 순서대로 처리한다.

        첫 메시지 하나가 잘못된 timestamp를 가져도 뒤 메시지를 막지 않도록
        매 주기마다 여러 항목을 독립적으로 검사하고, 아직 기다릴 항목만 큐
        뒤로 돌려보낸다. 누적 지도에는 다른 시각의 latest TF를 사용하지 않는다.
        """
        if not self.monitoring_enabled or not self.pending_clouds:
            return

        maximum_per_cycle = max(
            1,
            int(self.get_parameter("max_pending_clouds_per_cycle").value),
        )
        process_count = min(maximum_per_cycle, len(self.pending_clouds))
        exact_wait_sec = max(
            0.0, float(self.get_parameter("tf_exact_wait_sec").value)
        )
        for _ in range(process_count):
            queued_wall, message = self.pending_clouds.popleft()
            source_frame = str(message.header.frame_id).strip() or self.body_frame
            requested_time = Time.from_msg(message.header.stamp)
            transform = None

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    source_frame,
                    requested_time,
                    timeout=Duration(seconds=0.0),
                )
                wait_sec = max(0.0, time.monotonic() - queued_wall)
                self.exact_tf_count += 1
                self.exact_wait_total_sec += wait_sec
                self.exact_wait_max_sec = max(self.exact_wait_max_sec, wait_sec)
            except TransformException as exact_error:
                wait_sec = max(0.0, time.monotonic() - queued_wall)
                if wait_sec < exact_wait_sec:
                    # 아직 허용 대기시간 안이므로 뒤 메시지를 막지 않고 큐 끝으로
                    # 이동시킨다. 다음 timer 주기에서 같은 시각 TF를 다시 찾는다.
                    self.pending_clouds.append((queued_wall, message))
                    continue

                self.dropped_tf_count += 1
                requested_sec = self._stamp_to_seconds(message.header.stamp)
                latest_text = self._latest_tf_diagnostic(
                    source_frame, requested_sec
                )
                self._warn_throttled(
                    "exact timestamp TF 대기시간을 초과해 PointCloud 프레임을 "
                    "폐기했습니다: "
                    f"{source_frame}→{self.map_frame}, "
                    f"requested={requested_sec:.6f}, waited={wait_sec:.3f}s, "
                    f"latest={latest_text}, error={exact_error}"
                )
                continue

            self._accumulate_cloud(message, transform)

    def _latest_tf_diagnostic(self, source_frame, requested_sec):
        """프레임 폐기 로그에 최신 TF 시각과 차이만 안전하게 기록한다."""
        try:
            latest_transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            return "unavailable"
        latest_sec = self._stamp_to_seconds(latest_transform.header.stamp)
        return f"{latest_sec:.6f}(gap={requested_sec - latest_sec:+.3f}s)"

    def _accumulate_cloud(self, message, transform):
        stamp_sec = self._stamp_to_seconds(message.header.stamp)
        points = self._read_xyz(message)
        if points.size == 0:
            self._expire_old_frames(stamp_sec)
            return

        max_points = max(
            1000, int(self.get_parameter("maximum_points_per_scan").value)
        )
        if points.shape[0] > max_points:
            stride = int(math.ceil(points.shape[0] / max_points))
            points = points[::stride]

        finite = np.all(np.isfinite(points), axis=1)
        points = points[finite]
        if points.size == 0:
            return

        points_map = self._transform_points(points, transform)
        voxel_size = max(0.05, float(self.get_parameter("voxel_size_m").value))
        voxel_keys = np.floor(points_map / voxel_size).astype(np.int32)
        voxel_keys = np.unique(voxel_keys, axis=0)
        if voxel_keys.size:
            self.cloud_buffer.append((stamp_sec, voxel_keys))
        self._expire_old_frames(stamp_sec)

    def _publish_accumulated_map(self):
        if not self.monitoring_enabled:
            return
        if not self.cloud_buffer:
            self._publish_empty_outputs()
            return

        reference_stamp_sec = self.last_scan_stamp_sec
        self._expire_old_frames(reference_stamp_sec)
        if not self.cloud_buffer:
            self._publish_empty_outputs()
            return

        try:
            body_transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.body_frame,
                Time(),
                # latest body TF 조회는 이미 버퍼에 있는 값만 사용한다. 단일
                # executor에서 블로킹 대기해 /tf 콜백을 지연시키지 않는다.
                timeout=Duration(seconds=0.0),
            )
        except TransformException as error:
            self._warn_throttled(
                f"현재 body TF를 찾지 못해 누적맵 발행을 건너뜁니다: {error}"
            )
            return

        voxel_size = max(0.05, float(self.get_parameter("voxel_size_m").value))
        all_keys = np.vstack([frame[1] for frame in self.cloud_buffer])
        unique_keys, counts = np.unique(all_keys, axis=0, return_counts=True)
        minimum_observations = max(
            1,
            int(self.get_parameter("minimum_voxel_observations").value),
        )
        confirmed_keys = unique_keys[counts >= minimum_observations]
        points_map = (
            confirmed_keys.astype(np.float32) + 0.5
        ) * voxel_size

        translation, rotation = self._transform_components(body_transform)
        if points_map.size:
            points_body = (points_map - translation) @ rotation
        else:
            points_body = np.empty((0, 3), dtype=np.float32)

        minimum_height = float(self.get_parameter("minimum_height_m").value)
        maximum_height = float(self.get_parameter("maximum_height_m").value)
        costmap_size = max(
            4.0, float(self.get_parameter("local_costmap_size_m").value)
        )
        half_size = 0.5 * costmap_size
        self_filter_radius = max(
            0.0, float(self.get_parameter("self_filter_radius_m").value)
        )

        if points_body.size:
            local_mask = (
                (np.abs(points_body[:, 0]) <= half_size)
                & (np.abs(points_body[:, 1]) <= half_size)
                & (points_body[:, 2] >= minimum_height)
                & (points_body[:, 2] <= maximum_height)
                & (
                    np.hypot(points_body[:, 0], points_body[:, 1])
                    >= self_filter_radius
                )
            )
            points_body = points_body[local_mask]
            points_map = points_body @ rotation.T + translation

        stamp = self.get_clock().now().to_msg()
        self._publish_cloud(
            self.map_cloud_publisher,
            self.map_frame,
            stamp,
            points_map,
        )
        self._publish_cloud(
            self.body_cloud_publisher,
            self.body_frame,
            stamp,
            points_body,
        )
        self._publish_costmap(stamp, translation, points_map)

        wall_now = time.monotonic()
        if wall_now - self.last_publish_stats_wall >= 10.0:
            self.get_logger().info(
                "누적 PointCloud 갱신: "
                f"buffer_frames={len(self.cloud_buffer)}, "
                f"pending={len(self.pending_clouds)}, "
                f"confirmed_voxels={points_map.shape[0]}, "
                f"rx={self.received_cloud_count}, "
                f"sampled={self.sampled_cloud_count}, "
                f"tf_exact={self.exact_tf_count}, "
                f"exact_wait_avg={self._exact_wait_average():.3f}s, "
                f"exact_wait_max={self.exact_wait_max_sec:.3f}s, "
                f"tf_drop={self.dropped_tf_count}, "
                f"queue_overflow={self.pending_overflow_count}"
            )
            self.last_publish_stats_wall = wall_now

    def _exact_wait_average(self):
        if self.exact_tf_count <= 0:
            return 0.0
        return self.exact_wait_total_sec / float(self.exact_tf_count)

    def _publish_costmap(self, stamp, body_translation, points_map):
        resolution = max(
            0.10,
            float(self.get_parameter("local_costmap_resolution_m").value),
        )
        size_m = max(
            4.0, float(self.get_parameter("local_costmap_size_m").value)
        )
        cell_count = max(21, int(round(size_m / resolution)))
        if cell_count % 2 == 0:
            cell_count += 1
        actual_size = cell_count * resolution
        origin_x = float(body_translation[0]) - actual_size * 0.5
        origin_y = float(body_translation[1]) - actual_size * 0.5

        occupied = np.zeros((cell_count, cell_count), dtype=bool)
        if points_map.size:
            columns = np.floor((points_map[:, 0] - origin_x) / resolution).astype(int)
            rows = np.floor((points_map[:, 1] - origin_y) / resolution).astype(int)
            inside = (
                (rows >= 0)
                & (rows < cell_count)
                & (columns >= 0)
                & (columns < cell_count)
            )
            occupied[rows[inside], columns[inside]] = True

        inflation_cells = int(
            math.ceil(
                max(
                    0.0,
                    float(
                        self.get_parameter("obstacle_inflation_radius_m").value
                    ),
                )
                / resolution
            )
        )
        inflated = self._inflate_grid(occupied, inflation_cells)
        data = np.zeros((cell_count, cell_count), dtype=np.int8)
        data[inflated] = 65
        data[occupied] = 100

        message = OccupancyGrid()
        message.header.stamp = stamp
        message.header.frame_id = self.map_frame
        message.info.resolution = float(resolution)
        message.info.width = int(cell_count)
        message.info.height = int(cell_count)
        message.info.origin.position.x = origin_x
        message.info.origin.position.y = origin_y
        message.info.origin.position.z = float(body_translation[2])
        message.info.origin.orientation.w = 1.0
        message.data = data.reshape(-1).astype(int).tolist()
        self.costmap_publisher.publish(message)

    def _expire_old_frames(self, current_stamp_sec):
        cutoff = current_stamp_sec - self.accumulation_sec
        while self.cloud_buffer and self.cloud_buffer[0][0] < cutoff:
            self.cloud_buffer.popleft()

    def _publish_empty_outputs(self):
        stamp = self.get_clock().now().to_msg()
        empty = np.empty((0, 3), dtype=np.float32)
        self._publish_cloud(self.map_cloud_publisher, self.map_frame, stamp, empty)
        self._publish_cloud(self.body_cloud_publisher, self.body_frame, stamp, empty)

        resolution = max(
            0.10,
            float(self.get_parameter("local_costmap_resolution_m").value),
        )
        size_m = max(
            4.0, float(self.get_parameter("local_costmap_size_m").value)
        )
        cell_count = max(21, int(round(size_m / resolution)))
        if cell_count % 2 == 0:
            cell_count += 1
        message = OccupancyGrid()
        message.header.stamp = stamp
        message.header.frame_id = self.map_frame
        message.info.resolution = resolution
        message.info.width = cell_count
        message.info.height = cell_count
        message.info.origin.orientation.w = 1.0
        message.data = [0] * (cell_count * cell_count)
        self.costmap_publisher.publish(message)

    @staticmethod
    def _publish_cloud(publisher, frame_id, stamp, points):
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        publisher.publish(
            point_cloud2.create_cloud_xyz32(
                header,
                np.asarray(points, dtype=np.float32).reshape(-1, 3).tolist(),
            )
        )

    @staticmethod
    def _read_xyz(message):
        try:
            points = point_cloud2.read_points_numpy(
                message,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        except (AttributeError, ValueError):
            points = np.asarray(
                list(
                    point_cloud2.read_points(
                        message,
                        field_names=("x", "y", "z"),
                        skip_nans=True,
                    )
                ),
                dtype=np.float32,
            )
        points = np.asarray(points, dtype=np.float32)
        if points.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        return points.reshape(-1, 3)

    @staticmethod
    def _transform_points(points, transform):
        translation, rotation = PointCloudLocalMapperNode._transform_components(
            transform
        )
        return points @ rotation.T + translation

    @staticmethod
    def _transform_components(transform):
        translation = np.asarray(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float32,
        )
        quaternion = np.asarray(
            [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ],
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(quaternion))
        if norm <= 1.0e-12:
            rotation = np.eye(3, dtype=np.float32)
        else:
            x, y, z, w = quaternion / norm
            rotation = np.asarray(
                [
                    [
                        1.0 - 2.0 * (y * y + z * z),
                        2.0 * (x * y - z * w),
                        2.0 * (x * z + y * w),
                    ],
                    [
                        2.0 * (x * y + z * w),
                        1.0 - 2.0 * (x * x + z * z),
                        2.0 * (y * z - x * w),
                    ],
                    [
                        2.0 * (x * z - y * w),
                        2.0 * (y * z + x * w),
                        1.0 - 2.0 * (x * x + y * y),
                    ],
                ],
                dtype=np.float32,
            )
        return translation, rotation

    @staticmethod
    def _inflate_grid(occupied, radius_cells):
        if radius_cells <= 0 or not np.any(occupied):
            return occupied.copy()
        inflated = occupied.copy()
        rows, columns = occupied.shape
        for row_offset in range(-radius_cells, radius_cells + 1):
            for column_offset in range(-radius_cells, radius_cells + 1):
                if row_offset ** 2 + column_offset ** 2 > radius_cells ** 2:
                    continue
                source_row_start = max(0, -row_offset)
                source_row_end = min(rows, rows - row_offset)
                source_col_start = max(0, -column_offset)
                source_col_end = min(columns, columns - column_offset)
                target_row_start = source_row_start + row_offset
                target_row_end = source_row_end + row_offset
                target_col_start = source_col_start + column_offset
                target_col_end = source_col_end + column_offset
                inflated[
                    target_row_start:target_row_end,
                    target_col_start:target_col_end,
                ] |= occupied[
                    source_row_start:source_row_end,
                    source_col_start:source_col_end,
                ]
        return inflated

    def _warn_throttled(self, text):
        wall_now = time.monotonic()
        if wall_now - self.last_warning_wall < self.warning_period_sec:
            return
        self.get_logger().warning(text)
        self.last_warning_wall = wall_now

    @staticmethod
    def _stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) / 1.0e9


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudLocalMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

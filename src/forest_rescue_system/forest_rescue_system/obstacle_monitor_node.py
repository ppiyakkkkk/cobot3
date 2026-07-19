#!/usr/bin/env python3

"""LiDAR 수평 감시 영역의 최소 거리를 계산해 안전 정지 신호를 만든다."""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, String


class ObstacleMonitorNode(Node):
    def __init__(self):
        super().__init__("obstacle_monitor_node")

        self.declare_parameter("point_cloud_topic", "/point_cloud")
        self.declare_parameter("safety_distance_m", 1.5)
        self.declare_parameter("forward_half_angle_deg", 180.0)
        self.declare_parameter("minimum_height_m", -0.8)
        self.declare_parameter("maximum_height_m", 0.8)
        self.declare_parameter(
            "active_mission_states",
            ["READY", "SEARCHING"],
        )
        self.declare_parameter("warning_period_sec", 10.0)

        self.blocked_publisher = self.create_publisher(
            Bool,
            "/obstacle/blocked",
            10,
        )
        self.distance_publisher = self.create_publisher(
            Float32,
            "/obstacle/min_distance",
            10,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("point_cloud_topic").value,
            self._point_cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/mission/state",
            self._mission_state_callback,
            10,
        )

        self.last_blocked = False
        self.monitoring_enabled = False
        self.last_warning_time = float("-inf")
        self.warning_period_sec = max(
            0.1,
            float(self.get_parameter("warning_period_sec").value),
        )
        self.active_mission_states = {
            str(state).upper()
            for state in self.get_parameter("active_mission_states").value
        }
        self.get_logger().info(
            "LiDAR 360도 수평 장애물 감시 노드 시작: READY/SEARCHING 대기"
        )

    def _mission_state_callback(self, message):
        state = message.data.strip().upper()
        enabled = state in self.active_mission_states
        if enabled == self.monitoring_enabled:
            return

        self.monitoring_enabled = enabled
        self.last_blocked = False
        if enabled:
            self.get_logger().info(
                f"LiDAR 장애물 감시 활성화: mission_state={state}"
            )
        else:
            # 감시가 꺼질 때 이전 차단 상태가 남지 않게 해제한다.
            self._publish_result(float("inf"), False, allow_warning=False)
            self.get_logger().info(
                f"LiDAR 장애물 감시 비활성화: mission_state={state}"
            )

    def _point_cloud_callback(self, message):
        if not self.monitoring_enabled:
            return

        try:
            points = point_cloud2.read_points_numpy(
                message,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        except (AttributeError, ValueError):
            # 일부 Humble 버전에는 read_points_numpy가 없을 수 있다.
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

        points = np.asarray(points)
        if points.size == 0:
            self._publish_result(float("inf"), False)
            return
        points = points.reshape(-1, 3)

        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        angle_limit = math.radians(
            float(self.get_parameter("forward_half_angle_deg").value)
        )
        minimum_height = float(
            self.get_parameter("minimum_height_m").value
        )
        maximum_height = float(
            self.get_parameter("maximum_height_m").value
        )

        angles = np.arctan2(y, x)
        mask = (
            (x > 0.0)
            & (np.abs(angles) <= angle_limit)
            & (z >= minimum_height)
            & (z <= maximum_height)
        )
        if not np.any(mask):
            self._publish_result(float("inf"), False)
            return

        distances = np.hypot(x[mask], y[mask])
        minimum_distance = float(np.min(distances))
        safety_distance = float(
            self.get_parameter("safety_distance_m").value
        )
        self._publish_result(
            minimum_distance,
            minimum_distance < safety_distance,
        )

    def _publish_result(
        self,
        minimum_distance,
        blocked,
        allow_warning=True,
    ):
        distance_message = Float32()
        distance_message.data = minimum_distance
        self.distance_publisher.publish(distance_message)

        blocked_message = Bool()
        blocked_message.data = blocked
        self.blocked_publisher.publish(blocked_message)

        now = time.monotonic()
        if (
            allow_warning
            and blocked
            and now - self.last_warning_time >= self.warning_period_sec
        ):
            self.get_logger().warning(
                f"안전거리 내 장애물: {minimum_distance:.2f}m"
            )
            self.last_warning_time = now
        self.last_blocked = blocked


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

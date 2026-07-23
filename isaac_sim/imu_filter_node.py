#!/usr/bin/env python3
"""IMU 가속도 이상치를 직전 정상값으로 대체해서 republish한다.

숲에서 장애물 회피로 급기동할 때 linear_acceleration 크기가 순간적으로
중력(~9.8)의 몇 배까지 튀는 구간이 실제 녹화 bag에 있다(최대 65 m/s² 확인).
GTSAM IMU 사전적분은 이런 이상치가 들어오면 속도/바이어스 추정이 한 번
발산하면 스스로 복구하지 못하고 TF가 멈춰버린다. max_accel_mag를 넘는
샘플은 버리지 않고 직전 정상 가속도로 대체해서 흐름은 유지한다.

사용 예:
    ros2 run --prefix python3 . imu_filter_node.py \\
        --ros-args -p input_topic:=/quadrotor_01/imu/data \\
                   -p output_topic:=/quadrotor_01/imu/data_filtered
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuFilterNode(Node):
    def __init__(self):
        super().__init__("imu_filter_node")
        self.declare_parameter("input_topic", "imu/data")
        self.declare_parameter("output_topic", "imu/data_filtered")
        self.declare_parameter("max_accel_mag", 15.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._max_accel_mag = float(self.get_parameter("max_accel_mag").value)

        self._last_good_accel = None
        self._clamped_count = 0
        self.pub = self.create_publisher(Imu, output_topic, 10)
        self.sub = self.create_subscription(Imu, input_topic, self._callback, 10)
        self.get_logger().info(
            f"imu_filter_node: {input_topic} -> {output_topic} "
            f"(max_accel_mag={self._max_accel_mag})"
        )

    def _callback(self, msg: Imu):
        acc = msg.linear_acceleration
        mag = math.sqrt(acc.x ** 2 + acc.y ** 2 + acc.z ** 2)

        if mag > self._max_accel_mag and self._last_good_accel is not None:
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (
                self._last_good_accel
            )
            self._clamped_count += 1
            if self._clamped_count % 100 == 1:
                self.get_logger().warn(
                    f"가속도 이상치 클램핑: {mag:.1f} m/s^2 -> 직전 정상값 "
                    f"(누적 {self._clamped_count}건)"
                )
        else:
            self._last_good_accel = (acc.x, acc.y, acc.z)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

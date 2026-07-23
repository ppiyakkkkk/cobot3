#!/usr/bin/env python3
"""Isaac Sim RTX Lidar 출력에 LIO-SAM이 요구하는 ring 필드를 채워 republish한다.

LIO-SAM(imageProjection.cpp)은 point cloud에 `ring` 필드가 없으면 노드를 종료시킨다.
Isaac Sim의 RtxLidarROS2PublishPointCloud writer는 x,y,z,intensity만 채우므로,
각 포인트의 elevation 각도를 라이다 프로파일(Example_Rotary.json)의 32채널 고유
elevation 값에 매칭해 ring(채널 인덱스)을 채워 넣는다.

사용 예:
    ros2 run --prefix python3 . ring_filler_node.py \\
        --ros-args -p input_topic:=/quadrotor_01/point_cloud \\
                   -p output_topic:=/points
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2

# Example_Rotary.json emitterStates[0].elevationDeg 의 고유 32개 값 (오름차순, 채널 0~31).
# 128 emitter가 이 32개 elevation을 4개 azimuth 그룹(-3/-1/1/3deg)에서 반복하므로
# elevation만으로 채널을 32개로 구분할 수 있다.
CHANNEL_ELEVATIONS_DEG = np.array([
    -15.0, -14.19, -13.39, -12.58, -11.77, -10.97, -10.16, -9.35, -8.55, -7.74,
    -6.94, -6.13, -5.32, -4.52, -3.71, -2.9, -2.1, -1.29, -0.48, 0.32,
    1.13, 1.94, 2.74, 3.55, 4.35, 5.16, 5.97, 6.77, 7.58, 8.39, 9.19, 10.0,
], dtype=np.float32)

OUTPUT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
    PointField(name="time", offset=20, datatype=PointField.FLOAT32, count=1),
]
POINT_STEP = 24


class RingFillerNode(Node):
    def __init__(self):
        super().__init__("ring_filler_node")
        self.declare_parameter("input_topic", "point_cloud")
        self.declare_parameter("output_topic", "point_cloud/ring")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self._out_dtype = pc2.dtype_from_fields(OUTPUT_FIELDS, point_step=POINT_STEP)
        self._logged_fields = False
        self.pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.sub = self.create_subscription(PointCloud2, input_topic, self._callback, 10)
        self.get_logger().info(f"ring_filler_node: {input_topic} -> {output_topic}")

    def _callback(self, msg: PointCloud2):
        if not self._logged_fields:
            names = [f.name for f in msg.fields]
            self.get_logger().info(f"input point cloud fields: {names}")
            self._logged_fields = True

        available = {f.name for f in msg.fields}
        if not {"x", "y", "z"}.issubset(available):
            self.get_logger().error(
                f"x/y/z field missing from input cloud, got: {sorted(available)}"
            )
            return

        has_intensity = "intensity" in available
        read_fields = ("x", "y", "z", "intensity") if has_intensity else ("x", "y", "z")
        raw = pc2.read_points_numpy(msg, field_names=read_fields, skip_nans=True)
        if raw.shape[0] == 0:
            return

        if has_intensity:
            x, y, z, intensity = raw[:, 0], raw[:, 1], raw[:, 2], raw[:, 3]
        else:
            x, y, z = raw[:, 0], raw[:, 1], raw[:, 2]
            intensity = np.zeros_like(x)
        elevation_deg = np.degrees(np.arctan2(z, np.hypot(x, y))).astype(np.float32)
        ring = np.argmin(
            np.abs(elevation_deg[:, None] - CHANNEL_ELEVATIONS_DEG[None, :]), axis=1
        ).astype(np.uint16)

        out = np.zeros(x.shape[0], dtype=self._out_dtype)
        out["x"], out["y"], out["z"], out["intensity"] = x, y, z, intensity
        out["ring"] = ring
        # 프레임 단위 스냅샷이라 스캔 내 상대시간이 없음 -> 0으로 두면 LIO-SAM이
        # deskew만 비활성화하고(경고만 찍고) 계속 동작한다 (imageProjection.cpp:337).
        out["time"] = 0.0

        cloud_msg = pc2.create_cloud(msg.header, OUTPUT_FIELDS, out)
        # sensor_msgs_py.create_cloud()는 is_dense를 항상 False로 채우는데,
        # skip_nans=True로 이미 NaN을 제거했으므로 실제로는 dense가 맞다.
        # False로 두면 imageProjection.cpp가 이를 보고 즉시 shutdown한다.
        cloud_msg.is_dense = True
        self.pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RingFillerNode()
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

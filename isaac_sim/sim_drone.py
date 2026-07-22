#!/usr/bin/env python3
"""Iris 드론, PX4 backend, ROS 카메라 그래프와 LiDAR 구성."""

import math

import carb
import numpy as np
import omni.replicator.core as rep
import omni.usd
from pxr import Gf, Usd, UsdGeom
from scipy.spatial.transform import Rotation

from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.graphs import ROS2CameraGraph
from pegasus.simulator.logic.graphical_sensors.lidar import Lidar
from pegasus.simulator.logic.vehicles.multirotor import (
    Multirotor,
    MultirotorConfig,
)

from sim_config import (
    CAMERA_DOWN_TILT_DEG,
    CAMERA_FOCAL_LENGTH_MM,
    CAMERA_PRIM_PATHS,
    CAMERA_RESOLUTION,
    DRONE_CONFIGS,
)


def print_camera_direction_debug():
    """드론 body 전방과 실제 카메라 광축의 방향 차이를 출력한다.

    방향 기준
    ---------
    - 드론 body local +X: 기체 전방
    - USD Camera local -Z: 실제 영상 촬영 방향

    Transform 전체를 사용하면 부모 Prim의 Scale이 방향 벡터에 섞일 수
    있으므로, 여기서는 World Transform에서 회전 성분만 추출해 비교한다.
    """
    stage = omni.usd.get_context().get_stage()
    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())

    for camera_path in CAMERA_PRIM_PATHS:
        body_path = camera_path.rsplit("/Camera", 1)[0]
        camera_prim = stage.GetPrimAtPath(camera_path)
        body_prim = stage.GetPrimAtPath(body_path)

        if not camera_prim.IsValid():
            carb.log_warn(f"카메라 방향 검사 실패: {camera_path}")
            continue
        if not body_prim.IsValid():
            carb.log_warn(f"드론 body 방향 검사 실패: {body_path}")
            continue

        camera_world_matrix = xform_cache.GetLocalToWorldTransform(camera_prim)
        body_world_matrix = xform_cache.GetLocalToWorldTransform(body_prim)

        # 위치와 Scale을 제외하고 회전 성분만 방향 계산에 사용한다.
        body_world_rotation = body_world_matrix.ExtractRotation()
        camera_world_rotation = camera_world_matrix.ExtractRotation()

        body_forward_world = body_world_rotation.TransformDir(
            Gf.Vec3d(1.0, 0.0, 0.0)
        )
        camera_forward_world = camera_world_rotation.TransformDir(
            Gf.Vec3d(0.0, 0.0, -1.0)
        )

        body_forward = np.asarray(
            [
                float(body_forward_world[0]),
                float(body_forward_world[1]),
                float(body_forward_world[2]),
            ],
            dtype=np.float64,
        )
        camera_forward = np.asarray(
            [
                float(camera_forward_world[0]),
                float(camera_forward_world[1]),
                float(camera_forward_world[2]),
            ],
            dtype=np.float64,
        )

        body_norm = float(np.linalg.norm(body_forward))
        camera_norm = float(np.linalg.norm(camera_forward))
        if body_norm < 1.0e-9 or camera_norm < 1.0e-9:
            carb.log_warn(f"카메라 방향 벡터 계산 실패: {camera_path}")
            continue
        body_forward /= body_norm
        camera_forward /= camera_norm

        body_xy = body_forward[:2]
        camera_xy = camera_forward[:2]
        body_xy_norm = float(np.linalg.norm(body_xy))
        camera_xy_norm = float(np.linalg.norm(camera_xy))

        if body_xy_norm < 1.0e-6 or camera_xy_norm < 1.0e-6:
            carb.log_warn(f"카메라 수평 방향 계산 불가: {camera_path}")
            continue

        body_xy /= body_xy_norm
        camera_xy /= camera_xy_norm
        heading_dot = float(np.clip(np.dot(body_xy, camera_xy), -1.0, 1.0))
        heading_error_deg = math.degrees(math.acos(heading_dot))

        # World ENU에서 +Z는 위쪽이다. 따라서 -Z 성분이 하향 성분이다.
        camera_down_tilt_deg = math.degrees(
            math.atan2(
                -float(camera_forward[2]),
                float(np.linalg.norm(camera_forward[:2])),
            )
        )

        print(
            f"[CAMERA DEBUG] {camera_path}\n"
            f"  body_forward_world="
            f"({body_forward[0]:.3f}, "
            f"{body_forward[1]:.3f}, "
            f"{body_forward[2]:.3f})\n"
            f"  camera_forward_world="
            f"({camera_forward[0]:.3f}, "
            f"{camera_forward[1]:.3f}, "
            f"{camera_forward[2]:.3f})\n"
            f"  heading_error={heading_error_deg:.2f} deg\n"
            f"  camera_down_tilt={camera_down_tilt_deg:.2f} deg"
        )


class NamespacedLidar(Lidar):
    """드론마다 독립된 ROS 2 PointCloud2 토픽을 발행하는 LiDAR."""

    def __init__(
        self,
        lidar_name,
        topic_name,
        frame_id,
        config=None,
    ):
        super().__init__(lidar_name, config=config or {})
        self._topic_name = topic_name
        self._frame_id = frame_id
        self._render_product = None
        self._writer = None

    def start(self):
        # Pegasus 기본 Lidar는 모든 드론이 /point_cloud를 사용하므로
        # ROS 2 writer를 직접 생성해 드론별 토픽과 frame_id를 지정한다.
        if not self._show_render:
            return

        self._render_product = rep.create.render_product(
            self._sensor.GetPath(),
            [1, 1],
            name="Isaac",
        )
        self._writer = rep.writers.get(
            "RtxLidarROS2PublishPointCloud"
        )
        self._writer.initialize(
            topicName=self._topic_name,
            frameId=self._frame_id,
        )
        self._writer.attach([self._render_product])


class DroneManager:
    """세 대의 Iris 생성과 기체 카메라 설정을 담당한다."""

    def __init__(self, pegasus_interface):
        self.pg = pegasus_interface
        self.drones = []
        self.drone = None

    def spawn_iris(self):
        """시작 지점 주변에 카메라가 장착된 Iris 드론 3대를 생성한다."""
        self.drones = []

        for prim_path, vehicle_id, position in DRONE_CONFIGS:
            drone_name = prim_path.rsplit("/", 1)[-1]
            multirotor_config = MultirotorConfig()

            px4_config = PX4MavlinkBackendConfig(
                {
                    "vehicle_id": vehicle_id,
                    "px4_autolaunch": True,
                    "px4_dir": self.pg.px4_path,
                }
            )
            multirotor_config.backends = [
                PX4MavlinkBackend(px4_config)
            ]

            multirotor_config.graphs = [
                ROS2CameraGraph(
                    "body/Camera",
                    config={
                        "resolution": CAMERA_RESOLUTION,
                        "types": [
                            "rgb",
                            "depth",
                            "depth_pcl",
                            "camera_info",
                        ],
                        "namespace": f"/{drone_name}",
                        "topic": "/Camera",
                        "tf_frame_id": (
                            f"{drone_name}/camera_optical_frame"
                        ),
                    },
                )
            ]

            # 세 드론에 LiDAR를 각각 하나씩 장착하고 토픽을 분리한다.
            multirotor_config.graphical_sensors = [
                NamespacedLidar(
                    "lidar",
                    topic_name=f"/{drone_name}/point_cloud",
                    frame_id=f"{drone_name}/base_scan",
                    config={
                        "frequency": 10.0,
                        "position": np.array([0.0, 0.0, 0.15]),
                        "orientation": np.array([0.0, 0.0, 0.0]),
                        "sensor_configuration": {
                            "sensor_configuration": "Example_Rotary"
                        },
                        "show_render": True,
                    },
                )
            ]

            drone = Multirotor(
                prim_path,
                ROBOTS["Iris"],
                vehicle_id,
                position,
                Rotation.from_euler(
                    "XYZ",
                    [0.0, 0.0, 0.0],
                    degrees=True,
                ).as_quat(),
                config=multirotor_config,
            )
            self.drones.append(drone)
            print(
                f"[INFO] Spawned drone {vehicle_id + 1}: {prim_path} at "
                f"({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})"
            )

        # 기존 코드와의 호환성을 위해 1번 드론도 별도 참조로 남긴다.
        self.drone = self.drones[0]

    @staticmethod
    def _set_camera_translation_zero(xformable):
        """기존 회전은 유지하고 카메라 local Translate만 0,0,0으로 맞춘다."""
        translate_op = next(
            (
                op
                for op in xformable.GetOrderedXformOps()
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate
            ),
            None,
        )

        if translate_op is None:
            xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
            return

        attribute_type = str(translate_op.GetAttr().GetTypeName())
        if attribute_type == "float3":
            zero = Gf.Vec3f(0.0, 0.0, 0.0)
        elif attribute_type == "half3":
            zero = Gf.Vec3h(0.0, 0.0, 0.0)
        else:
            zero = Gf.Vec3d(0.0, 0.0, 0.0)
        translate_op.Set(zero)

    def configure_drone_cameras(self):
        """세 카메라를 기체 전방에서 지정 각도만큼 아래로 향하게 한다."""
        stage = omni.usd.get_context().get_stage()

        for camera_path in CAMERA_PRIM_PATHS:
            camera_prim = stage.GetPrimAtPath(camera_path)
            if not camera_prim.IsValid():
                raise RuntimeError(
                    f"Iris 카메라 Prim을 찾을 수 없습니다: {camera_path}"
                )

            camera = UsdGeom.Camera(camera_prim)
            camera.GetFocalLengthAttr().Set(CAMERA_FOCAL_LENGTH_MM)

            horizontal_aperture = float(
                camera.GetHorizontalApertureAttr().Get()
            )
            horizontal_fov_deg = math.degrees(
                2.0
                * math.atan(
                    horizontal_aperture
                    / (2.0 * CAMERA_FOCAL_LENGTH_MM)
                )
            )

            xformable = UsdGeom.Xformable(camera_prim)
            self._set_camera_translation_zero(xformable)

            orient_op = next(
                (
                    op
                    for op in xformable.GetOrderedXformOps()
                    if op.GetOpType() == UsdGeom.XformOp.TypeOrient
                ),
                None,
            )

            if orient_op is None:
                carb.log_warn(
                    f"Camera orient op가 없어 방향을 적용하지 못했습니다: "
                    f"{camera_path}"
                )
            else:
                # Iris USD에 저장된 기존 카메라 방향은 사용하지 않는다.
                # USD Camera의 local -Z 광축을 드론 body +X 전방에 맞춘다.
                # local +Y는 body +Z가 되어 영상의 위쪽도 정상 유지된다.
                camera_forward_rotation = Rotation.from_matrix(
                    np.array(
                        [
                            [0.0, 0.0, -1.0],
                            [-1.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0],
                        ],
                        dtype=np.float64,
                    )
                )

                # 전방 기준으로 카메라를 아래쪽으로 기울인다.
                down_rotation = Rotation.from_euler(
                    "X",
                    -CAMERA_DOWN_TILT_DEG,
                    degrees=True,
                )
                configured_xyzw = (
                    camera_forward_rotation * down_rotation
                ).as_quat()
                x, y, z, w = [
                    float(value)
                    for value in configured_xyzw
                ]

                # 기존 USD Orient 속성의 Quaternion 자료형을 유지한다.
                attribute_type = str(
                    orient_op.GetAttr().GetTypeName()
                )
                if attribute_type == "quatd":
                    configured_quaternion = Gf.Quatd(
                        w,
                        Gf.Vec3d(x, y, z),
                    )
                elif attribute_type == "quath":
                    configured_quaternion = Gf.Quath(
                        w,
                        Gf.Vec3h(x, y, z),
                    )
                else:
                    configured_quaternion = Gf.Quatf(
                        w,
                        Gf.Vec3f(x, y, z),
                    )
                orient_op.Set(configured_quaternion)

            print(
                f"[INFO] Camera configured: {camera_path}, "
                f"local_translate=(0, 0, 0), "
                f"focal={CAMERA_FOCAL_LENGTH_MM:.1f}mm, "
                f"down_tilt={CAMERA_DOWN_TILT_DEG:.1f}deg, "
                f"horizontal_fov≈{horizontal_fov_deg:.1f}deg"
            )

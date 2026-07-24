from builtin_interfaces.msg import Time as TimeMsg
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformException

from forest_rescue_system.coverage_ownership import TriangleOwnership
from forest_rescue_system.coverage_visualization_node import (
    CoverageVisualizationNode,
)
from forest_rescue_system import coverage_geometry, coverage_mesh


class _StubTfBuffer:
    """map→camera 항등 변환만 돌려주는 테스트용 TF 버퍼."""

    def lookup_transform(self, target_frame, source_frame, time, timeout=None):
        stamped = TransformStamped()
        stamped.transform.rotation.w = 1.0
        return stamped


class _RecordingTfBuffer:
    """lookup_transform 호출 시 (target_frame, source_frame, time) 인자를 기록하는 테스트용 TF 버퍼."""

    def __init__(self):
        self.calls = []

    def lookup_transform(self, target_frame, source_frame, time, timeout=None):
        self.calls.append((target_frame, source_frame, time))
        stamped = TransformStamped()
        stamped.transform.rotation.w = 1.0
        return stamped


@pytest.fixture
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node(rclpy_context, tmp_path):
    terrain_path = tmp_path / "terrain.npz"
    np.savez(
        terrain_path,
        vertices=np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
        ),
        triangles=np.array([[0, 1, 2]]),
    )
    env_path = tmp_path / "env.npz"
    np.savez(
        env_path,
        rocks_vertices=np.array(
            [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 2.0, 0.0]]
        ),
        rocks_triangles=np.array([[0, 1, 2]]),
    )
    node = CoverageVisualizationNode(
        parameter_overrides=[
            Parameter(
                "terrain_mesh_path",
                Parameter.Type.STRING,
                str(terrain_path),
            ),
            Parameter(
                "environment_mesh_path",
                Parameter.Type.STRING,
                str(env_path),
            ),
        ]
    )
    return node


def test_constructor_loads_mesh_and_builds_scene(rclpy_context, tmp_path):
    node = _make_node(rclpy_context, tmp_path)
    try:
        assert node.scene is not None
        assert sorted(node.scene.group_names) == ["rocks", "terrain"]
        assert len(node.scene.centroids) == 2
        assert node.ownership is not None
    finally:
        node.destroy_node()


def _synthetic_scene_and_ownership():
    # 카메라 원점에서 z=5(near)/z=20(far) 방향으로 겹쳐 놓인 두 삼각형.
    # 기본 ray_grid_step_px(4px) 격자가 최소 한 개 이상의 광선으로
    # near 삼각형을 확실히 맞히도록 투영 폭을 충분히 크게 잡는다
    # (변보다 작으면 격자 간격 사이로 광선이 모두 빠져나갈 수 있다).
    vertices = np.array(
        [
            [-0.5, -0.5, 5.0],
            [0.5, -0.5, 5.0],
            [0.0, 0.5, 5.0],
            [-0.5, -0.5, 20.0],
            [0.5, -0.5, 20.0],
            [0.0, 0.5, 20.0],
        ]
    )
    triangles = np.array([[0, 1, 2], [3, 4, 5]])
    scene = coverage_geometry.assemble_scene({"synthetic": (vertices, triangles)})
    ownership = TriangleOwnership(len(scene.centroids))
    return scene, ownership


def _camera_info():
    info = CameraInfo()
    info.k = [100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0]
    info.width = 100
    info.height = 100
    return info


def test_process_drone_claims_visible_triangle_and_rejects_occluded_one(
    rclpy_context, tmp_path
):
    # 카메라 원점(0,0,0)에서 +z 방향을 보는 항등 변환.
    # near 삼각형(z=5)이 far 삼각형(z=20)과 같은 시선 방향에 겹쳐 있으므로
    # 실제 오클루전으로 far는 가려져야 한다.
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        node.depth_shape_by_drone["quadrotor_01"] = (100, 100)
        node.depth_stamp_by_drone["quadrotor_01"] = TimeMsg()

        node._process_drone(0, "quadrotor_01")

        np.testing.assert_array_equal(node.ownership.owner_ids, [0, -1])
    finally:
        node.destroy_node()


def test_process_drone_does_not_overwrite_existing_owner(rclpy_context, tmp_path):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        node.ownership.claim([0], drone_index=1)
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        node.depth_shape_by_drone["quadrotor_01"] = (100, 100)
        node.depth_stamp_by_drone["quadrotor_01"] = TimeMsg()

        node._process_drone(0, "quadrotor_01")

        np.testing.assert_array_equal(node.ownership.owner_ids, [1, -1])
    finally:
        node.destroy_node()


def test_build_coverage_marker_array_uses_owner_namespaces_and_colors(
    rclpy_context, tmp_path
):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.ownership.claim([0], drone_index=0)

        marker_array = node._build_coverage_marker_array()

        assert [marker.ns for marker in marker_array.markers] == [
            "coverage_drone_01",
            "coverage_drone_02",
            "coverage_drone_03",
        ]
        assert len(marker_array.markers[0].points) == 3
        assert len(marker_array.markers[1].points) == 0

        expected_color = [
            float(value)
            for value in node.get_parameter("drone_01_color_rgb").value
        ]
        marker_color = marker_array.markers[0].color
        assert [marker_color.r, marker_color.g, marker_color.b] == expected_color
    finally:
        node.destroy_node()


def test_compute_total_area_sums_only_owned_triangles(rclpy_context, tmp_path):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()

        assert node._compute_total_area() == pytest.approx(0.0)

        node.ownership.claim([0], drone_index=0)
        expected = float(node.scene.areas[0])
        assert node._compute_total_area() == pytest.approx(expected)
    finally:
        node.destroy_node()


def test_process_drone_skips_when_camera_info_missing(rclpy_context, tmp_path):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        node.tf_buffer = _StubTfBuffer()

        node._process_drone(0, "quadrotor_01")

        np.testing.assert_array_equal(node.ownership.owner_ids, [-1, -1])
    finally:
        node.destroy_node()


def test_process_drone_skips_when_depth_missing(rclpy_context, tmp_path):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        # depth_shape_by_drone에 아무것도 등록하지 않음

        node._process_drone(0, "quadrotor_01")

        np.testing.assert_array_equal(node.ownership.owner_ids, [-1, -1])
    finally:
        node.destroy_node()


def test_process_drone_skips_when_tf_lookup_fails(rclpy_context, tmp_path):
    class _FailingTfBuffer:
        def lookup_transform(self, target_frame, source_frame, time, timeout=None):
            raise TransformException("no transform available")

    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        node.tf_buffer = _FailingTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        node.depth_shape_by_drone["quadrotor_01"] = (100, 100)
        node.depth_stamp_by_drone["quadrotor_01"] = TimeMsg()

        node._process_drone(0, "quadrotor_01")

        np.testing.assert_array_equal(node.ownership.owner_ids, [-1, -1])
    finally:
        node.destroy_node()


def test_constructor_leaves_scene_none_when_mesh_files_are_missing(
    rclpy_context, tmp_path
):
    node = CoverageVisualizationNode(
        parameter_overrides=[
            Parameter(
                "terrain_mesh_path",
                Parameter.Type.STRING,
                str(tmp_path / "missing_terrain.npz"),
            ),
            Parameter(
                "environment_mesh_path",
                Parameter.Type.STRING,
                str(tmp_path / "missing_env.npz"),
            ),
        ]
    )
    try:
        assert node.scene is None
        assert node.ownership is None
    finally:
        node.destroy_node()


def test_process_drone_looks_up_transform_with_correct_frame_order(
    rclpy_context, tmp_path
):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        recording_tf_buffer = _RecordingTfBuffer()
        node.tf_buffer = recording_tf_buffer
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        node.depth_shape_by_drone["quadrotor_01"] = (100, 100)
        node.depth_stamp_by_drone["quadrotor_01"] = TimeMsg()

        node._process_drone(0, "quadrotor_01")

        assert [(call[0], call[1]) for call in recording_tf_buffer.calls] == [
            (node.map_frame, "quadrotor_01/camera_optical_frame")
        ]
    finally:
        node.destroy_node()


def test_process_drone_looks_up_transform_at_depth_image_capture_time(
    rclpy_context, tmp_path
):
    # TF는 처리 시점의 "최신" 자세가 아니라, depth 이미지가 실제로 찍힌
    # 시각(header.stamp) 기준으로 조회해야 드론 이동 중 투영이 어긋나지
    # 않는다.
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        recording_tf_buffer = _RecordingTfBuffer()
        node.tf_buffer = recording_tf_buffer
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        node.depth_shape_by_drone["quadrotor_01"] = (100, 100)
        node.depth_stamp_by_drone["quadrotor_01"] = TimeMsg(
            sec=123, nanosec=456
        )

        node._process_drone(0, "quadrotor_01")

        used_time = recording_tf_buffer.calls[0][2]
        assert used_time == Time(seconds=123, nanoseconds=456)
    finally:
        node.destroy_node()


def test_process_drone_records_flashlight_state_with_hit_points(
    rclpy_context, tmp_path
):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        node.depth_shape_by_drone["quadrotor_01"] = (100, 100)
        node.depth_stamp_by_drone["quadrotor_01"] = TimeMsg()

        node._process_drone(0, "quadrotor_01")

        state = node.flashlight_state["quadrotor_01"]
        np.testing.assert_allclose(state["origin"], [0.0, 0.0, 0.0], atol=1e-9)
        assert state["corner_directions"].shape == (4, 3)
        assert state["hit_points"].shape[1] == 3
        assert state["hit_points"].shape[0] > 0
    finally:
        node.destroy_node()


def test_process_drone_clears_flashlight_state_when_tf_lookup_fails(
    rclpy_context, tmp_path
):
    class _FailingTfBuffer:
        def lookup_transform(self, target_frame, source_frame, time, timeout=None):
            raise TransformException("no transform available")

    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        node.depth_shape_by_drone["quadrotor_01"] = (100, 100)
        node.depth_stamp_by_drone["quadrotor_01"] = TimeMsg()
        node._process_drone(0, "quadrotor_01")
        assert "quadrotor_01" in node.flashlight_state

        node.tf_buffer = _FailingTfBuffer()
        node._process_drone(0, "quadrotor_01")

        assert "quadrotor_01" not in node.flashlight_state
    finally:
        node.destroy_node()


def test_refresh_coverage_first_seen_wins_across_drones_same_cycle(
    rclpy_context, tmp_path
):
    node = _make_node(rclpy_context, tmp_path)
    try:
        assert node.drone_ids[0] == "quadrotor_01"
        assert node.drone_ids[1] == "quadrotor_02"

        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        node.tf_buffer = _StubTfBuffer()

        for drone_id in ("quadrotor_01", "quadrotor_02"):
            node.camera_info_by_drone[drone_id] = _camera_info()
            node.depth_shape_by_drone[drone_id] = (100, 100)
            node.depth_stamp_by_drone[drone_id] = TimeMsg()

        node._refresh_coverage()

        assert node.ownership.owner_ids[0] == 0
    finally:
        node.destroy_node()


def test_load_mesh_if_ready_does_not_reload_after_first_success(
    rclpy_context, tmp_path, monkeypatch
):
    node = _make_node(rclpy_context, tmp_path)
    try:
        assert node.scene is not None

        call_count = 0
        original = coverage_mesh.load_terrain_group

        def _counting_load_terrain_group(path):
            nonlocal call_count
            call_count += 1
            return original(path)

        monkeypatch.setattr(
            coverage_mesh, "load_terrain_group", _counting_load_terrain_group
        )

        node._load_mesh_if_ready()
        node._load_mesh_if_ready()

        assert call_count == 0
    finally:
        node.destroy_node()


def test_refresh_coverage_skips_marker_publish_when_nothing_newly_claimed(
    rclpy_context, tmp_path
):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        all_indices = np.arange(len(node.scene.centroids))
        node.ownership.claim(all_indices, drone_index=0)

        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        node.depth_shape_by_drone["quadrotor_01"] = (100, 100)
        node.depth_stamp_by_drone["quadrotor_01"] = TimeMsg()

        calls = []

        def _counting_publish(msg):
            calls.append(msg)

        node.marker_publisher.publish = _counting_publish

        node._refresh_coverage()

        assert len(calls) == 0
    finally:
        node.destroy_node()


def test_refresh_coverage_publishes_marker_when_something_newly_claimed(
    rclpy_context, tmp_path
):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        node.depth_shape_by_drone["quadrotor_01"] = (100, 100)
        node.depth_stamp_by_drone["quadrotor_01"] = TimeMsg()

        calls = []

        def _counting_publish(msg):
            calls.append(msg)

        node.marker_publisher.publish = _counting_publish

        node._refresh_coverage()

        assert len(calls) == 1
    finally:
        node.destroy_node()


def test_constructor_builds_raycasting_scene(rclpy_context, tmp_path):
    node = _make_node(rclpy_context, tmp_path)
    try:
        assert node.raycasting_scene is not None
    finally:
        node.destroy_node()


def test_depth_callback_records_shape_and_stamp_without_cv_bridge(
    rclpy_context, tmp_path
):
    from sensor_msgs.msg import Image

    node = _make_node(rclpy_context, tmp_path)
    try:
        message = Image()
        message.height = 48
        message.width = 64
        message.encoding = "32FC1"
        message.header.stamp = TimeMsg(sec=1, nanosec=0)

        node._depth_callback("quadrotor_01", message)

        assert node.depth_shape_by_drone["quadrotor_01"] == (48, 64)
        assert node.depth_stamp_by_drone["quadrotor_01"] == message.header.stamp
    finally:
        node.destroy_node()

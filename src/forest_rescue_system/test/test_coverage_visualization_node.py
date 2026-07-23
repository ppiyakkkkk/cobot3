import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.parameter import Parameter
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
    """lookup_transform 호출 시 (target_frame, source_frame) 인자를 기록하는 테스트용 TF 버퍼."""

    def __init__(self):
        self.calls = []

    def lookup_transform(self, target_frame, source_frame, time, timeout=None):
        self.calls.append((target_frame, source_frame))
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
    vertices = np.array(
        [
            [-0.05, -0.05, 5.0],
            [0.05, -0.05, 5.0],
            [0.0, 0.05, 5.0],
            [-0.05, -0.05, 20.0],
            [0.05, -0.05, 20.0],
            [0.0, 0.05, 20.0],
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
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        depth_image = np.zeros((100, 100), dtype=np.float32)
        depth_image[50, 50] = 5.0
        node.depth_by_drone["quadrotor_01"] = depth_image

        node._process_drone(0, "quadrotor_01")

        np.testing.assert_array_equal(node.ownership.owner_ids, [0, -1])
    finally:
        node.destroy_node()


def test_process_drone_does_not_overwrite_existing_owner(rclpy_context, tmp_path):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.ownership.claim([0], drone_index=1)
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        depth_image = np.zeros((100, 100), dtype=np.float32)
        depth_image[50, 50] = 5.0
        node.depth_by_drone["quadrotor_01"] = depth_image

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
        node.tf_buffer = _StubTfBuffer()
        # camera_info_by_drone에 아무것도 등록하지 않음 (아직 수신 전 상황)

        node._process_drone(0, "quadrotor_01")

        np.testing.assert_array_equal(node.ownership.owner_ids, [-1, -1])
    finally:
        node.destroy_node()


def test_process_drone_skips_when_depth_missing(rclpy_context, tmp_path):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        # depth_by_drone에 아무것도 등록하지 않음 (아직 수신 전 상황)

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
        node.tf_buffer = _FailingTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        depth_image = np.zeros((100, 100), dtype=np.float32)
        depth_image[50, 50] = 5.0
        node.depth_by_drone["quadrotor_01"] = depth_image

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
        recording_tf_buffer = _RecordingTfBuffer()
        node.tf_buffer = recording_tf_buffer
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        depth_image = np.zeros((100, 100), dtype=np.float32)
        depth_image[50, 50] = 5.0
        node.depth_by_drone["quadrotor_01"] = depth_image

        node._process_drone(0, "quadrotor_01")

        assert recording_tf_buffer.calls == [
            ("quadrotor_01/camera_optical_frame", node.map_frame)
        ]
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
        node.tf_buffer = _StubTfBuffer()

        depth_image = np.zeros((100, 100), dtype=np.float32)
        depth_image[50, 50] = 5.0
        for drone_id in ("quadrotor_01", "quadrotor_02"):
            node.camera_info_by_drone[drone_id] = _camera_info()
            node.depth_by_drone[drone_id] = depth_image.copy()

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

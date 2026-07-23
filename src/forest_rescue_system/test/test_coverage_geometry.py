import numpy as np

from forest_rescue_system import coverage_geometry


def test_triangle_vertex_positions_indexes_vertices_by_triangle():
    vertices = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    triangles = np.array([[0, 1, 2]])
    positions = coverage_geometry.triangle_vertex_positions(vertices, triangles)
    assert positions.shape == (1, 3, 3)
    np.testing.assert_array_equal(positions[0], vertices)


def test_triangle_centroids_averages_the_three_vertices():
    positions = np.array([[[0.0, 0.0, 0.0], [3.0, 0.0, 0.0], [0.0, 3.0, 0.0]]])
    centroids = coverage_geometry.triangle_centroids(positions)
    np.testing.assert_allclose(centroids, [[1.0, 1.0, 0.0]])


def test_triangle_areas_computes_right_triangle_area():
    positions = np.array([[[0.0, 0.0, 0.0], [4.0, 0.0, 0.0], [0.0, 3.0, 0.0]]])
    areas = coverage_geometry.triangle_areas(positions)
    np.testing.assert_allclose(areas, [6.0])


def test_assemble_scene_concatenates_groups_in_insertion_order():
    terrain_vertices = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    terrain_triangles = np.array([[0, 1, 2]])
    rocks_vertices = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 2.0, 0.0]])
    rocks_triangles = np.array([[0, 1, 2]])

    scene = coverage_geometry.assemble_scene(
        {
            "terrain": (terrain_vertices, terrain_triangles),
            "rocks": (rocks_vertices, rocks_triangles),
        }
    )

    assert scene.group_names == ["terrain", "rocks"]
    assert scene.group_slices == {"terrain": slice(0, 1), "rocks": slice(1, 2)}
    assert scene.centroids.shape == (2, 3)
    assert scene.areas.shape == (2,)
    assert scene.triangle_positions.shape == (2, 3, 3)


def test_scaled_intrinsics_scales_when_depth_resolution_differs():
    k = [100.0, 0.0, 50.0, 0.0, 100.0, 40.0, 0.0, 0.0, 1.0]
    fx, fy, cx, cy = coverage_geometry.scaled_intrinsics(
        k, info_width=200, info_height=160, depth_width=100, depth_height=80
    )
    assert (fx, fy, cx, cy) == (50.0, 50.0, 25.0, 20.0)


def test_transform_matrix_from_tf_applies_translation_with_identity_rotation():
    matrix = coverage_geometry.transform_matrix_from_tf(
        translation=(1.0, 2.0, 3.0), quaternion=(0.0, 0.0, 0.0, 1.0)
    )
    points = coverage_geometry.apply_transform(np.array([[0.0, 0.0, 0.0]]), matrix)
    np.testing.assert_allclose(points, [[1.0, 2.0, 3.0]])


def test_transform_matrix_from_tf_rotates_90_degrees_about_z():
    half_angle = np.pi / 4.0
    quaternion = (0.0, 0.0, np.sin(half_angle), np.cos(half_angle))
    matrix = coverage_geometry.transform_matrix_from_tf(
        translation=(0.0, 0.0, 0.0), quaternion=quaternion
    )
    points = coverage_geometry.apply_transform(np.array([[1.0, 0.0, 0.0]]), matrix)
    np.testing.assert_allclose(points, [[0.0, 1.0, 0.0]], atol=1e-9)

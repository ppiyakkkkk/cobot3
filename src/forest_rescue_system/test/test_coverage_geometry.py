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

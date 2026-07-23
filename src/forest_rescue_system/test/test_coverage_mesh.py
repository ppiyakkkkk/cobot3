import numpy as np

from forest_rescue_system import coverage_mesh


def test_scan_dynamic_groups_detects_pairs_and_ignores_other_suffixes():
    keys = [
        "pineforest_vertices",
        "pineforest_triangles",
        "pineforest_source_paths",
        "pineforest_original_triangle_count",
        "rocks_vertices",
        "rocks_triangles",
        "coordinate_convention",
        "map_frame",
        "newgroup_vertices",
        "newgroup_triangles",
    ]
    assert coverage_mesh.scan_dynamic_groups(keys) == [
        "newgroup",
        "pineforest",
        "rocks",
    ]


def test_scan_dynamic_groups_requires_both_keys():
    keys = ["onlyvertices_vertices", "onlytriangles_triangles_extra"]
    assert coverage_mesh.scan_dynamic_groups(keys) == []


def test_load_environment_groups_reads_all_dynamic_groups(tmp_path):
    path = tmp_path / "env.npz"
    np.savez(
        path,
        pineforest_vertices=np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
        ),
        pineforest_triangles=np.array([[0, 1, 2]]),
        rocks_vertices=np.array(
            [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 2.0, 0.0]]
        ),
        rocks_triangles=np.array([[0, 1, 2]]),
    )
    groups = coverage_mesh.load_environment_groups(path)
    assert set(groups.keys()) == {"pineforest", "rocks"}
    vertices, triangles = groups["pineforest"]
    assert vertices.shape == (3, 3)
    assert triangles.shape == (1, 3)


def test_load_terrain_group_reads_root_level_keys(tmp_path):
    path = tmp_path / "terrain.npz"
    np.savez(
        path,
        vertices=np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
        ),
        triangles=np.array([[0, 1, 2]]),
    )
    groups = coverage_mesh.load_terrain_group(path)
    assert set(groups.keys()) == {"terrain"}

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


def test_visibility_mask_accepts_point_matching_depth_image():
    points_camera = np.array([[0.0, 0.0, 5.0]])
    depth_image = np.full((10, 10), 100.0, dtype=np.float32)
    depth_image[5, 5] = 5.0
    mask = coverage_geometry.visibility_mask(
        points_camera,
        fx=10.0, fy=10.0, cx=5.0, cy=5.0,
        depth_image=depth_image,
        tolerance_m=0.5, min_depth_m=0.2, max_depth_m=30.0,
    )
    np.testing.assert_array_equal(mask, [True])


def test_visibility_mask_rejects_occluded_point_behind_closer_surface():
    points_camera = np.array([[0.0, 0.0, 20.0]])
    depth_image = np.full((10, 10), 5.0, dtype=np.float32)
    mask = coverage_geometry.visibility_mask(
        points_camera,
        fx=10.0, fy=10.0, cx=5.0, cy=5.0,
        depth_image=depth_image,
        tolerance_m=0.5, min_depth_m=0.2, max_depth_m=30.0,
    )
    np.testing.assert_array_equal(mask, [False])


def test_visibility_mask_rejects_point_outside_max_depth_range():
    points_camera = np.array([[0.0, 0.0, 50.0]])
    depth_image = np.full((10, 10), 50.0, dtype=np.float32)
    mask = coverage_geometry.visibility_mask(
        points_camera,
        fx=10.0, fy=10.0, cx=5.0, cy=5.0,
        depth_image=depth_image,
        tolerance_m=0.5, min_depth_m=0.2, max_depth_m=30.0,
    )
    np.testing.assert_array_equal(mask, [False])


def test_visibility_mask_rejects_point_projecting_outside_image_bounds():
    points_camera = np.array([[1000.0, 0.0, 5.0]])
    depth_image = np.full((10, 10), 5.0, dtype=np.float32)
    mask = coverage_geometry.visibility_mask(
        points_camera,
        fx=10.0, fy=10.0, cx=5.0, cy=5.0,
        depth_image=depth_image,
        tolerance_m=0.5, min_depth_m=0.2, max_depth_m=30.0,
    )
    np.testing.assert_array_equal(mask, [False])


def test_visibility_mask_samples_depth_image_as_row_v_col_u():
    # u=2, v=0으로 서로 다른 값이 나오게 만들어, depth_image 인덱싱이
    # [v, u](row=height, col=width) 순서인지 실제로 구분되게 한다.
    # 만약 구현이 실수로 [u, v]로 뒤집히면 이 테스트는 실패해야 한다.
    points_camera = np.array([[2.0, 0.0, 10.0]])
    depth_image = np.zeros((5, 10), dtype=np.float32)
    depth_image[0, 2] = 10.0
    mask = coverage_geometry.visibility_mask(
        points_camera,
        fx=10.0, fy=20.0, cx=0.0, cy=0.0,
        depth_image=depth_image,
        tolerance_m=0.5, min_depth_m=0.2, max_depth_m=30.0,
    )
    np.testing.assert_array_equal(mask, [True])


def test_triangle_sample_points_returns_vertices_plus_centroid():
    positions = np.array([[[0.0, 0.0, 0.0], [3.0, 0.0, 0.0], [0.0, 3.0, 0.0]]])
    samples = coverage_geometry.triangle_sample_points(positions)
    assert samples.shape == (1, 4, 3)
    np.testing.assert_allclose(samples[0, :3], positions[0])
    np.testing.assert_allclose(samples[0, 3], [1.0, 1.0, 0.0])


def test_visibility_mask_multi_sample_true_if_any_sample_visible():
    # centroid는 가려진 픽셀(깊이 불일치)에 투영되지만, 정점 하나는
    # depth_image와 일치하는 픽셀에 투영된다 -> 무게중심만 보던 기존
    # 방식이라면 놓쳤을 삼각형을 다중 샘플링은 잡아내야 한다.
    sample_points_camera = np.array(
        [[[0.0, 0.0, 5.0], [10.0, 0.0, 5.0], [0.0, 10.0, 5.0], [3.0, 3.0, 5.0]]]
    )
    normals_camera = np.array([[0.0, 0.0, -1.0]])  # 카메라를 정면으로 바라봄
    depth_image = np.zeros((20, 20), dtype=np.float32)
    depth_image[5, 5] = 5.0  # 정점 (0,0,5) -> u=v=5
    mask = coverage_geometry.visibility_mask_multi_sample(
        sample_points_camera,
        normals_camera,
        fx=10.0, fy=10.0, cx=5.0, cy=5.0,
        depth_image=depth_image,
        tolerance_m=0.5, min_depth_m=0.2, max_depth_m=30.0,
    )
    np.testing.assert_array_equal(mask, [True])


def test_visibility_mask_multi_sample_false_if_all_samples_occluded():
    sample_points_camera = np.array(
        [[[0.0, 0.0, 20.0], [10.0, 0.0, 20.0], [0.0, 10.0, 20.0], [3.0, 3.0, 20.0]]]
    )
    normals_camera = np.array([[0.0, 0.0, -1.0]])
    depth_image = np.full((20, 20), 5.0, dtype=np.float32)
    mask = coverage_geometry.visibility_mask_multi_sample(
        sample_points_camera,
        normals_camera,
        fx=10.0, fy=10.0, cx=5.0, cy=5.0,
        depth_image=depth_image,
        tolerance_m=0.5, min_depth_m=0.2, max_depth_m=30.0,
    )
    np.testing.assert_array_equal(mask, [False])


def test_triangle_normals_returns_unit_length_perpendicular_vector():
    positions = np.array([[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]])
    normals = coverage_geometry.triangle_normals(positions)
    np.testing.assert_allclose(normals, [[0.0, 0.0, 1.0]])


def test_transform_direction_rotates_without_applying_translation():
    half_angle = np.pi / 4.0
    quaternion = (0.0, 0.0, np.sin(half_angle), np.cos(half_angle))
    matrix = coverage_geometry.transform_matrix_from_tf(
        translation=(5.0, 5.0, 5.0), quaternion=quaternion
    )
    rotated = coverage_geometry.transform_direction(
        np.array([[1.0, 0.0, 0.0]]), matrix
    )
    np.testing.assert_allclose(rotated, [[0.0, 1.0, 0.0]], atol=1e-9)


def test_grazing_angle_tolerance_keeps_base_value_when_facing_camera():
    points_camera = np.array([[0.0, 0.0, 10.0]])
    normals_camera = np.array([[0.0, 0.0, -1.0]])  # 카메라를 정면으로 바라봄
    tolerance = coverage_geometry.grazing_angle_tolerance(
        0.1, normals_camera, points_camera
    )
    np.testing.assert_allclose(tolerance, [0.1])


def test_grazing_angle_tolerance_scales_up_near_grazing_incidence():
    points_camera = np.array([[0.0, 0.0, 10.0]])
    # 시선은 +z 방향, 법선은 이와 거의 수직(+x) -> 그레이징에 가깝다.
    normals_camera = np.array([[1.0, 0.0, 0.0]])
    tolerance = coverage_geometry.grazing_angle_tolerance(
        0.1, normals_camera, points_camera
    )
    # min_cosine=0.2 -> 1/0.2 = 5.0배로 확대(상한).
    np.testing.assert_allclose(tolerance, [0.5])


def test_visibility_mask_matches_neighboring_pixel_within_window():
    # 그레이징 각도에서 픽셀 반올림으로 (u,v)가 한 칸 어긋나도, 3x3
    # 윈도우 안에 실제로 일치하는 depth가 있으면 보이는 것으로 처리한다.
    points_camera = np.array([[0.0, 0.0, 5.0]])
    depth_image = np.full((10, 10), 100.0, dtype=np.float32)
    depth_image[6, 5] = 5.0  # 투영 픽셀(5,5) 바로 옆 칸에 실제 depth가 있음
    mask = coverage_geometry.visibility_mask(
        points_camera,
        fx=10.0, fy=10.0, cx=5.0, cy=5.0,
        depth_image=depth_image,
        tolerance_m=0.1, min_depth_m=0.2, max_depth_m=30.0,
    )
    np.testing.assert_array_equal(mask, [True])


def test_pixel_to_camera_ray_at_principal_point_points_straight_ahead():
    direction = coverage_geometry.pixel_to_camera_ray(
        u=50.0, v=50.0, fx=100.0, fy=100.0, cx=50.0, cy=50.0
    )
    np.testing.assert_allclose(direction, [0.0, 0.0, 1.0], atol=1e-9)


def test_pixel_to_camera_ray_returns_normalized_offset_direction():
    # u=cx+fx, v=cy -> 카메라 프레임에서 (1,0,1) 방향, 정규화 후 1/sqrt(2)
    direction = coverage_geometry.pixel_to_camera_ray(
        u=150.0, v=50.0, fx=100.0, fy=100.0, cx=50.0, cy=50.0
    )
    expected = np.array([1.0, 0.0, 1.0]) / np.sqrt(2.0)
    np.testing.assert_allclose(direction, expected, atol=1e-9)


def test_pixel_to_camera_ray_vectorizes_over_arrays():
    directions = coverage_geometry.pixel_to_camera_ray(
        u=np.array([50.0, 150.0]),
        v=np.array([50.0, 50.0]),
        fx=100.0, fy=100.0, cx=50.0, cy=50.0,
    )
    assert directions.shape == (2, 3)
    np.testing.assert_allclose(directions[0], [0.0, 0.0, 1.0], atol=1e-9)


def test_pixel_grid_uv_covers_full_frame_with_step_one():
    u, v = coverage_geometry.pixel_grid_uv(width=4, height=2, step_px=1)
    assert u.shape == (8,)
    assert v.shape == (8,)
    # 픽셀 중심이므로 0.5, 1.5, 2.5, 3.5 값만 나와야 한다
    np.testing.assert_allclose(sorted(np.unique(u)), [0.5, 1.5, 2.5, 3.5])
    np.testing.assert_allclose(sorted(np.unique(v)), [0.5, 1.5])


def test_pixel_grid_uv_subsamples_with_step_px():
    u, v = coverage_geometry.pixel_grid_uv(width=10, height=10, step_px=4)
    # arange(0,10,4) = [0,4,8] -> 3x3 = 9개
    assert u.shape == (9,)
    np.testing.assert_allclose(sorted(np.unique(u)), [0.5, 4.5, 8.5])

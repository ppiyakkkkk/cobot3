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


def _single_triangle_scene(z=5.0):
    vertices = np.array(
        [[-1.0, -1.0, z], [1.0, -1.0, z], [0.0, 1.0, z]]
    )
    triangles = np.array([[0, 1, 2]])
    return coverage_geometry.build_raycasting_scene(
        coverage_geometry.triangle_vertex_positions(vertices, triangles)
    )


def test_cast_visibility_rays_hits_triangle_facing_camera():
    scene = _single_triangle_scene(z=5.0)
    hit_points, triangle_indices = coverage_geometry.cast_visibility_rays(
        scene,
        ray_origin=np.array([0.0, 0.0, 0.0]),
        ray_directions=np.array([[0.0, 0.0, 1.0]]),
        min_depth_m=0.2,
        max_depth_m=30.0,
    )
    assert triangle_indices.tolist() == [0]
    np.testing.assert_allclose(hit_points, [[0.0, 0.0, 5.0]], atol=1e-5)


def test_cast_visibility_rays_misses_when_ray_points_away():
    scene = _single_triangle_scene(z=5.0)
    hit_points, triangle_indices = coverage_geometry.cast_visibility_rays(
        scene,
        ray_origin=np.array([0.0, 0.0, 0.0]),
        ray_directions=np.array([[0.0, 0.0, -1.0]]),
        min_depth_m=0.2,
        max_depth_m=30.0,
    )
    assert triangle_indices.shape == (0,)
    assert hit_points.shape == (0, 3)


def test_cast_visibility_rays_respects_max_depth_clipping():
    scene = _single_triangle_scene(z=5.0)
    hit_points, triangle_indices = coverage_geometry.cast_visibility_rays(
        scene,
        ray_origin=np.array([0.0, 0.0, 0.0]),
        ray_directions=np.array([[0.0, 0.0, 1.0]]),
        min_depth_m=0.2,
        max_depth_m=3.0,  # 삼각형(z=5)보다 가까운 far clip
    )
    assert triangle_indices.shape == (0,)


def test_cast_visibility_rays_occluder_blocks_farther_triangle():
    # 가까운 삼각형(z=5)이 먼 삼각형(z=20)과 같은 광선 방향에 겹쳐 있으면
    # 가까운 쪽만 히트되어야 한다 (실제 오클루션, tolerance 없음).
    near = coverage_geometry.triangle_vertex_positions(
        np.array([[-1.0, -1.0, 5.0], [1.0, -1.0, 5.0], [0.0, 1.0, 5.0]]),
        np.array([[0, 1, 2]]),
    )
    far = coverage_geometry.triangle_vertex_positions(
        np.array([[-1.0, -1.0, 20.0], [1.0, -1.0, 20.0], [0.0, 1.0, 20.0]]),
        np.array([[0, 1, 2]]),
    )
    both = np.concatenate([near, far], axis=0)
    scene = coverage_geometry.build_raycasting_scene(both)

    hit_points, triangle_indices = coverage_geometry.cast_visibility_rays(
        scene,
        ray_origin=np.array([0.0, 0.0, 0.0]),
        ray_directions=np.array([[0.0, 0.0, 1.0]]),
        min_depth_m=0.2,
        max_depth_m=30.0,
    )
    assert triangle_indices.tolist() == [0]  # near, not far(index 1)
    np.testing.assert_allclose(hit_points, [[0.0, 0.0, 5.0]], atol=1e-5)


def test_cast_visibility_rays_hits_grazing_angle_downslope_triangle():
    # 원래 버그 재현: 카메라 시선과 거의 평행한(그레이징) 내리막 삼각형.
    # 뎁스 리프로젝션 비교 방식은 tolerance 튜닝에 의존했지만, 레이캐스팅은
    # 정확한 ray-triangle 교차라 그레이징 각도와 무관하게 항상 히트되어야 한다.
    # 광선을 삼각형의 무게중심으로 정확히 조준한다 (무게중심은 정점 3개의
    # 평균이므로 비퇴화 삼각형이라면 항상 삼각형 내부에 있음이 보장된다 -
    # 손으로 배리센트릭 좌표를 계산하지 않아도 교차가 확실하다).
    vertices = np.array(
        [
            [-1.0, 10.0, 0.0],
            [1.0, 10.0, 0.0],
            [0.0, 11.0, -0.1],  # y(카메라로부터의 거리)가 커질수록 z가 낮아지는 완만한 내리막
        ]
    )
    triangles = np.array([[0, 1, 2]])
    scene = coverage_geometry.build_raycasting_scene(
        coverage_geometry.triangle_vertex_positions(vertices, triangles)
    )
    centroid = vertices.mean(axis=0)
    ray_direction = centroid / np.linalg.norm(centroid)

    hit_points, triangle_indices = coverage_geometry.cast_visibility_rays(
        scene,
        ray_origin=np.array([0.0, 0.0, 0.0]),
        ray_directions=np.array([ray_direction]),
        min_depth_m=0.2,
        max_depth_m=30.0,
    )
    assert triangle_indices.tolist() == [0]
    np.testing.assert_allclose(hit_points[0], centroid, atol=1e-5)


def test_cast_visibility_rays_handles_empty_ray_array():
    scene = _single_triangle_scene(z=5.0)
    hit_points, triangle_indices = coverage_geometry.cast_visibility_rays(
        scene,
        ray_origin=np.array([0.0, 0.0, 0.0]),
        ray_directions=np.zeros((0, 3)),
        min_depth_m=0.2,
        max_depth_m=30.0,
    )
    assert hit_points.shape == (0, 3)
    assert triangle_indices.shape == (0,)

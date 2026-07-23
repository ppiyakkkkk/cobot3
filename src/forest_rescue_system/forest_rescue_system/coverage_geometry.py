#!/usr/bin/env python3

"""드론 카메라 커버리지 시각화용 순수 기하 계산 함수."""

from dataclasses import dataclass

import numpy as np


@dataclass
class SceneMesh:
    group_names: list
    group_slices: dict
    triangle_positions: np.ndarray
    centroids: np.ndarray
    areas: np.ndarray


def triangle_vertex_positions(vertices, triangles):
    return np.asarray(vertices, dtype=np.float64)[
        np.asarray(triangles, dtype=np.int64)
    ]


def triangle_centroids(triangle_positions):
    return triangle_positions.mean(axis=1)


def triangle_areas(triangle_positions):
    v0 = triangle_positions[:, 0, :]
    v1 = triangle_positions[:, 1, :]
    v2 = triangle_positions[:, 2, :]
    cross = np.cross(v1 - v0, v2 - v0)
    return 0.5 * np.linalg.norm(cross, axis=1)


def assemble_scene(groups):
    group_names = []
    group_slices = {}
    position_parts = []
    offset = 0

    for name, (vertices, triangles) in groups.items():
        positions = triangle_vertex_positions(vertices, triangles)
        position_parts.append(positions)
        count = len(positions)
        group_slices[name] = slice(offset, offset + count)
        group_names.append(name)
        offset += count

    if position_parts:
        triangle_positions = np.concatenate(position_parts, axis=0)
    else:
        triangle_positions = np.zeros((0, 3, 3), dtype=np.float64)

    return SceneMesh(
        group_names=group_names,
        group_slices=group_slices,
        triangle_positions=triangle_positions,
        centroids=triangle_centroids(triangle_positions),
        areas=triangle_areas(triangle_positions),
    )


def scaled_intrinsics(k, info_width, info_height, depth_width, depth_height):
    effective_width = info_width or depth_width
    effective_height = info_height or depth_height
    scale_x = depth_width / float(effective_width)
    scale_y = depth_height / float(effective_height)
    fx = float(k[0]) * scale_x
    fy = float(k[4]) * scale_y
    cx = float(k[2]) * scale_x
    cy = float(k[5]) * scale_y
    return fx, fy, cx, cy


def transform_matrix_from_tf(translation, quaternion):
    x, y, z, w = quaternion
    matrix = np.eye(4)
    matrix[0, 0] = 1 - 2 * (y * y + z * z)
    matrix[0, 1] = 2 * (x * y - w * z)
    matrix[0, 2] = 2 * (x * z + w * y)
    matrix[1, 0] = 2 * (x * y + w * z)
    matrix[1, 1] = 1 - 2 * (x * x + z * z)
    matrix[1, 2] = 2 * (y * z - w * x)
    matrix[2, 0] = 2 * (x * z - w * y)
    matrix[2, 1] = 2 * (y * z + w * x)
    matrix[2, 2] = 1 - 2 * (x * x + y * y)
    matrix[0, 3], matrix[1, 3], matrix[2, 3] = translation
    return matrix


def apply_transform(points, matrix):
    points = np.asarray(points, dtype=np.float64)
    homogeneous = np.concatenate(
        [points, np.ones((len(points), 1))], axis=1
    )
    return (homogeneous @ matrix.T)[:, :3]


def visibility_mask(
    points_camera,
    fx,
    fy,
    cx,
    cy,
    depth_image,
    tolerance_m,
    min_depth_m,
    max_depth_m,
):
    points_camera = np.asarray(points_camera, dtype=np.float64)
    result = np.zeros(len(points_camera), dtype=bool)
    if len(points_camera) == 0:
        return result

    x = points_camera[:, 0]
    y = points_camera[:, 1]
    z = points_camera[:, 2]

    in_front = z > 0.0
    in_range = (z >= min_depth_m) & (z <= max_depth_m)

    safe_z = np.where(in_front, z, 1.0)
    u = np.round((x * fx / safe_z) + cx).astype(np.int64)
    v = np.round((y * fy / safe_z) + cy).astype(np.int64)

    height, width = depth_image.shape[:2]
    in_bounds = (u >= 0) & (u < width) & (v >= 0) & (v < height)

    candidate = in_front & in_range & in_bounds
    candidate_idx = np.where(candidate)[0]
    if candidate_idx.size == 0:
        return result

    sampled_depth = depth_image[v[candidate_idx], u[candidate_idx]]
    close_enough = np.isfinite(sampled_depth) & (
        np.abs(sampled_depth - z[candidate_idx]) < tolerance_m
    )
    result[candidate_idx[close_enough]] = True
    return result


def triangle_sample_points(triangle_positions):
    centroids = triangle_centroids(triangle_positions)
    return np.concatenate(
        [triangle_positions, centroids[:, np.newaxis, :]], axis=1
    )


def visibility_mask_multi_sample(
    sample_points_camera,
    fx,
    fy,
    cx,
    cy,
    depth_image,
    tolerance_m,
    min_depth_m,
    max_depth_m,
):
    sample_points_camera = np.asarray(sample_points_camera, dtype=np.float64)
    triangle_count, samples_per_triangle, _ = sample_points_camera.shape
    flat_points = sample_points_camera.reshape(-1, 3)
    flat_visible = visibility_mask(
        flat_points, fx, fy, cx, cy, depth_image,
        tolerance_m, min_depth_m, max_depth_m,
    )
    return flat_visible.reshape(triangle_count, samples_per_triangle).any(axis=1)

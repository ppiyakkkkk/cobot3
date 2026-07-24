#!/usr/bin/env python3

"""드론 카메라 커버리지 시각화용 순수 기하 계산 함수."""

from dataclasses import dataclass

import numpy as np
import open3d as o3d
import open3d.core as o3c


@dataclass
class SceneMesh:
    group_names: list
    group_slices: dict
    triangle_positions: np.ndarray
    centroids: np.ndarray
    areas: np.ndarray
    normals: np.ndarray


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


def triangle_normals(triangle_positions):
    v0 = triangle_positions[:, 0, :]
    v1 = triangle_positions[:, 1, :]
    v2 = triangle_positions[:, 2, :]
    cross = np.cross(v1 - v0, v2 - v0)
    norms = np.linalg.norm(cross, axis=1, keepdims=True)
    safe_norms = np.where(norms > 0.0, norms, 1.0)
    return cross / safe_norms


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
        normals=triangle_normals(triangle_positions),
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


def transform_direction(vectors, matrix):
    vectors = np.asarray(vectors, dtype=np.float64)
    return vectors @ matrix[:3, :3].T


def pixel_to_camera_ray(u, v, fx, fy, cx, cy):
    u = np.asarray(u, dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    directions = np.stack(
        [(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], axis=-1
    )
    norms = np.linalg.norm(directions, axis=-1, keepdims=True)
    return directions / norms


def pixel_grid_uv(width, height, step_px):
    u = np.arange(0, width, step_px, dtype=np.float64) + 0.5
    v = np.arange(0, height, step_px, dtype=np.float64) + 0.5
    grid_u, grid_v = np.meshgrid(u, v)
    return grid_u.ravel(), grid_v.ravel()


def build_raycasting_scene(triangle_positions):
    triangle_positions = np.asarray(triangle_positions, dtype=np.float64)
    vertices = triangle_positions.reshape(-1, 3).astype(np.float32)
    triangle_count = triangle_positions.shape[0]
    triangles = np.arange(vertices.shape[0], dtype=np.uint32).reshape(
        triangle_count, 3
    )
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(
        o3c.Tensor(vertices, dtype=o3c.float32),
        o3c.Tensor(triangles, dtype=o3c.uint32),
    )
    return scene


def cast_visibility_rays(
    scene, ray_origin, ray_directions, min_depth_m, max_depth_m
):
    ray_origin = np.asarray(ray_origin, dtype=np.float64)
    ray_directions = np.asarray(ray_directions, dtype=np.float64)
    ray_count = ray_directions.shape[0]
    if ray_count == 0:
        return (
            np.zeros((0, 3), dtype=np.float64),
            np.zeros((0,), dtype=np.int64),
        )

    norms = np.linalg.norm(ray_directions, axis=1, keepdims=True)
    safe_norms = np.where(norms > 0.0, norms, 1.0)
    unit_directions = ray_directions / safe_norms

    origins = np.broadcast_to(ray_origin, (ray_count, 3))
    rays = np.concatenate([origins, unit_directions], axis=1).astype(
        np.float32
    )

    result = scene.cast_rays(o3c.Tensor(rays, dtype=o3c.float32))
    t_hit = result["t_hit"].numpy()
    primitive_ids = result["primitive_ids"].numpy()

    valid = (
        np.isfinite(t_hit) & (t_hit >= min_depth_m) & (t_hit <= max_depth_m)
    )
    valid_t = t_hit[valid].astype(np.float64)
    hit_points = ray_origin + unit_directions[valid] * valid_t[:, np.newaxis]
    triangle_indices = primitive_ids[valid].astype(np.int64)
    return hit_points, triangle_indices

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

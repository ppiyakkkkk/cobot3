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

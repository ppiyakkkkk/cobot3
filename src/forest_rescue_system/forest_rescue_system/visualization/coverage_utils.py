#!/usr/bin/env python3

"""카메라 커버리지 평가에 사용하는 Mesh·레이캐스팅 공통 함수.

참고 브랜치의 coverage_geometry.py, coverage_mesh.py,
coverage_ownership.py 역할을 하나로 합쳤다. ROS 2 Node와 Marker 발행은
coverage_visualization_node.py에 남겨 계산 코드와 실행 코드를 분리한다.
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import open3d as o3d
import open3d.core as o3c


@dataclass(frozen=True)
class SceneMesh:
    """Terrain과 환경 그룹을 하나의 전역 삼각형 인덱스로 합친 장면."""

    group_names: list
    group_slices: dict
    triangle_positions: np.ndarray
    centroids: np.ndarray
    areas: np.ndarray
    normals: np.ndarray


class TriangleOwnership:
    """각 삼각형을 처음 관측한 드론 인덱스로 영구 점유한다."""

    def __init__(self, triangle_count):
        count = int(triangle_count)
        if count < 0:
            raise ValueError(f"triangle_count는 0 이상이어야 합니다: {count}")
        self._owner = np.full(count, -1, dtype=np.int32)

    @property
    def owner_ids(self):
        return self._owner

    def reset(self):
        self._owner.fill(-1)

    def claim(self, triangle_indices, drone_index):
        indices = np.asarray(triangle_indices, dtype=np.int64).reshape(-1)
        if indices.size == 0:
            return np.empty(0, dtype=np.int64)
        valid = (indices >= 0) & (indices < len(self._owner))
        indices = np.unique(indices[valid])
        if indices.size == 0:
            return np.empty(0, dtype=np.int64)
        unclaimed = indices[self._owner[indices] < 0]
        self._owner[unclaimed] = int(drone_index)
        return unclaimed

    def indices_for_drone(self, drone_index):
        return np.where(self._owner == int(drone_index))[0]


def _validate_mesh(vertices, triangles, name):
    vertices = np.asarray(vertices, dtype=np.float64)
    triangles = np.asarray(triangles, dtype=np.int64)
    if vertices.ndim != 2 or vertices.shape[1] != 3:
        raise ValueError(f"{name} vertices 형식 오류: {vertices.shape}")
    if triangles.ndim != 2 or triangles.shape[1] != 3:
        raise ValueError(f"{name} triangles 형식 오류: {triangles.shape}")
    if vertices.size == 0 or triangles.size == 0:
        return vertices, triangles
    if not np.all(np.isfinite(vertices)):
        raise ValueError(f"{name} vertices에 유효하지 않은 값이 있습니다.")
    if np.min(triangles) < 0 or np.max(triangles) >= len(vertices):
        raise ValueError(f"{name} triangle index가 정점 범위를 벗어났습니다.")
    return vertices, triangles


def scan_dynamic_groups(npz_keys):
    """``*_vertices``와 ``*_triangles`` 쌍을 환경 그룹으로 찾는다."""
    keys = set(npz_keys)
    groups = []
    for key in sorted(keys):
        if not key.endswith("_vertices"):
            continue
        name = key[: -len("_vertices")]
        if f"{name}_triangles" in keys:
            groups.append(name)
    return groups


def load_terrain_group(path):
    path = Path(path).expanduser()
    with np.load(path, allow_pickle=False) as data:
        vertices, triangles = _validate_mesh(
            data["vertices"], data["triangles"], "terrain"
        )
    if vertices.size == 0 or triangles.size == 0:
        return {}
    return {"terrain": (vertices, triangles)}


def load_environment_groups(path):
    path = Path(path).expanduser()
    groups = {}
    with np.load(path, allow_pickle=False) as data:
        for name in scan_dynamic_groups(data.files):
            vertices, triangles = _validate_mesh(
                data[f"{name}_vertices"],
                data[f"{name}_triangles"],
                name,
            )
            if vertices.size == 0 or triangles.size == 0:
                continue
            groups[name] = (vertices, triangles)
    return groups


def triangle_vertex_positions(vertices, triangles):
    return np.asarray(vertices, dtype=np.float64)[
        np.asarray(triangles, dtype=np.int64)
    ]


def triangle_centroids(triangle_positions):
    positions = np.asarray(triangle_positions, dtype=np.float64)
    if positions.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    return positions.mean(axis=1)


def triangle_areas(triangle_positions):
    positions = np.asarray(triangle_positions, dtype=np.float64)
    if positions.size == 0:
        return np.empty(0, dtype=np.float64)
    v0 = positions[:, 0, :]
    v1 = positions[:, 1, :]
    v2 = positions[:, 2, :]
    return 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)


def triangle_normals(triangle_positions):
    positions = np.asarray(triangle_positions, dtype=np.float64)
    if positions.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    v0 = positions[:, 0, :]
    v1 = positions[:, 1, :]
    v2 = positions[:, 2, :]
    cross = np.cross(v1 - v0, v2 - v0)
    norms = np.linalg.norm(cross, axis=1, keepdims=True)
    return cross / np.where(norms > 1.0e-12, norms, 1.0)


def assemble_scene(groups):
    """그룹 순서를 보존해 전역 삼각형 배열과 그룹 slice를 만든다."""
    group_names = []
    group_slices = {}
    position_parts = []
    offset = 0

    for name, (vertices, triangles) in groups.items():
        positions = triangle_vertex_positions(vertices, triangles)
        if positions.size == 0:
            continue
        position_parts.append(positions)
        count = len(positions)
        group_names.append(str(name))
        group_slices[str(name)] = slice(offset, offset + count)
        offset += count

    triangle_positions = (
        np.concatenate(position_parts, axis=0)
        if position_parts
        else np.empty((0, 3, 3), dtype=np.float64)
    )
    return SceneMesh(
        group_names=group_names,
        group_slices=group_slices,
        triangle_positions=triangle_positions,
        centroids=triangle_centroids(triangle_positions),
        areas=triangle_areas(triangle_positions),
        normals=triangle_normals(triangle_positions),
    )


def scaled_intrinsics(k, info_width, info_height, image_width, image_height):
    effective_width = int(info_width) or int(image_width)
    effective_height = int(info_height) or int(image_height)
    if effective_width <= 0 or effective_height <= 0:
        raise ValueError("CameraInfo 또는 영상 해상도가 유효하지 않습니다.")
    scale_x = float(image_width) / float(effective_width)
    scale_y = float(image_height) / float(effective_height)
    fx = float(k[0]) * scale_x
    fy = float(k[4]) * scale_y
    cx = float(k[2]) * scale_x
    cy = float(k[5]) * scale_y
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError(f"카메라 초점거리가 유효하지 않습니다: fx={fx}, fy={fy}")
    return fx, fy, cx, cy


def transform_matrix_from_tf(translation, quaternion):
    """XYZ translation과 normalized XYZW quaternion으로 4x4 행렬 생성."""
    x, y, z, w = [float(value) for value in quaternion]
    norm = float(np.linalg.norm([x, y, z, w]))
    if norm <= 1.0e-12:
        raise ValueError("TF Quaternion의 크기가 0입니다.")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    matrix = np.eye(4, dtype=np.float64)
    matrix[0, 0] = 1.0 - 2.0 * (y * y + z * z)
    matrix[0, 1] = 2.0 * (x * y - w * z)
    matrix[0, 2] = 2.0 * (x * z + w * y)
    matrix[1, 0] = 2.0 * (x * y + w * z)
    matrix[1, 1] = 1.0 - 2.0 * (x * x + z * z)
    matrix[1, 2] = 2.0 * (y * z - w * x)
    matrix[2, 0] = 2.0 * (x * z - w * y)
    matrix[2, 1] = 2.0 * (y * z + w * x)
    matrix[2, 2] = 1.0 - 2.0 * (x * x + y * y)
    matrix[:3, 3] = np.asarray(translation, dtype=np.float64)
    return matrix


def transform_direction(vectors, matrix):
    vectors = np.asarray(vectors, dtype=np.float64)
    return vectors @ np.asarray(matrix, dtype=np.float64)[:3, :3].T


def pixel_to_camera_ray(u, v, fx, fy, cx, cy):
    """ROS optical frame(+Z 전방)의 픽셀 광선을 단위벡터로 변환한다."""
    u = np.asarray(u, dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    directions = np.stack(
        ((u - cx) / fx, (v - cy) / fy, np.ones_like(u)), axis=-1
    )
    norms = np.linalg.norm(directions, axis=-1, keepdims=True)
    return directions / np.where(norms > 1.0e-12, norms, 1.0)


def pixel_grid_uv(width, height, step_px):
    width = int(width)
    height = int(height)
    step = max(1, int(step_px))
    if width <= 0 or height <= 0:
        return np.empty(0), np.empty(0)
    u = np.arange(0, width, step, dtype=np.float64) + 0.5
    v = np.arange(0, height, step, dtype=np.float64) + 0.5
    grid_u, grid_v = np.meshgrid(u, v)
    return grid_u.ravel(), grid_v.ravel()


def build_raycasting_scene(triangle_positions):
    positions = np.asarray(triangle_positions, dtype=np.float64)
    if positions.ndim != 3 or positions.shape[1:] != (3, 3):
        raise ValueError(f"triangle_positions 형식 오류: {positions.shape}")
    if len(positions) == 0:
        raise ValueError("레이캐스팅 장면에 삼각형이 없습니다.")

    # 정점을 공유하지 않고 삼각형 순서 그대로 펼친다. Open3D가 반환하는
    # primitive_id가 SceneMesh의 전역 삼각형 인덱스와 정확히 일치한다.
    vertices = positions.reshape(-1, 3).astype(np.float32)
    triangles = np.arange(len(vertices), dtype=np.uint32).reshape(-1, 3)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(
        o3c.Tensor(vertices, dtype=o3c.float32),
        o3c.Tensor(triangles, dtype=o3c.uint32),
    )
    return scene


def cast_visibility_rays(
    scene,
    ray_origin,
    ray_directions,
    min_depth_m,
    max_depth_m,
):
    """광선의 첫 hit 지점과 전역 삼각형 인덱스를 반환한다."""
    origin = np.asarray(ray_origin, dtype=np.float64).reshape(3)
    directions = np.asarray(ray_directions, dtype=np.float64).reshape(-1, 3)
    if directions.size == 0:
        return np.empty((0, 3)), np.empty(0, dtype=np.int64)

    norms = np.linalg.norm(directions, axis=1, keepdims=True)
    unit_directions = directions / np.where(norms > 1.0e-12, norms, 1.0)
    origins = np.broadcast_to(origin, (len(unit_directions), 3))
    rays = np.concatenate((origins, unit_directions), axis=1).astype(np.float32)
    result = scene.cast_rays(o3c.Tensor(rays, dtype=o3c.float32))
    t_hit = result["t_hit"].numpy()
    primitive_ids = result["primitive_ids"].numpy()

    minimum = max(0.0, float(min_depth_m))
    maximum = max(minimum, float(max_depth_m))
    valid = np.isfinite(t_hit) & (t_hit >= minimum) & (t_hit <= maximum)
    valid_t = t_hit[valid].astype(np.float64)
    hit_points = origin + unit_directions[valid] * valid_t[:, np.newaxis]
    triangle_indices = primitive_ids[valid].astype(np.int64)
    return hit_points, triangle_indices

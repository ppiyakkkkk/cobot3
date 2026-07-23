#!/usr/bin/env python3

"""환경/지형 mesh npz 파일을 읽고 그룹을 동적으로 스캔한다."""

import numpy as np


def scan_dynamic_groups(npz_keys):
    keys = set(npz_keys)
    groups = []
    for key in sorted(keys):
        if key.endswith("_vertices"):
            name = key[: -len("_vertices")]
            if f"{name}_triangles" in keys:
                groups.append(name)
    return groups


def load_environment_groups(path):
    groups = {}
    with np.load(path, allow_pickle=False) as data:
        for name in scan_dynamic_groups(data.files):
            vertices = np.asarray(data[f"{name}_vertices"], dtype=np.float64)
            triangles = np.asarray(data[f"{name}_triangles"], dtype=np.int64)
            if vertices.size == 0 or triangles.size == 0:
                continue
            groups[name] = (vertices, triangles)
    return groups


def load_terrain_group(path):
    with np.load(path, allow_pickle=False) as data:
        vertices = np.asarray(data["vertices"], dtype=np.float64)
        triangles = np.asarray(data["triangles"], dtype=np.int64)
    if vertices.size == 0 or triangles.size == 0:
        return {}
    return {"terrain": (vertices, triangles)}

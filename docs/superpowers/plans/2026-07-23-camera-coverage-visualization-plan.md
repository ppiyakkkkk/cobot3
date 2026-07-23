# 드론 카메라 커버리지 시각화 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 드론 3대의 카메라가 depth 기반으로 실제로 확인한 지형/식생 삼각형을 누적해서 RViz에 보라 계열 색으로 표시하고, 커버된 총 면적을 1초 주기로 토픽 발행한다.

**Architecture:** 순수 함수 모듈 3개(`coverage_geometry.py`, `coverage_mesh.py`, `coverage_ownership.py`)로 기하 계산·mesh 로딩·소유권 상태를 분리하고, `coverage_visualization_node.py` 하나가 이 모듈들을 조합해 드론 3대의 CameraInfo/Depth/TF를 구독하고 마커·면적을 발행한다.

**Tech Stack:** rclpy, numpy, cv_bridge, tf2_ros, sensor_msgs/visualization_msgs/std_msgs, pytest.

**Spec:** `docs/superpowers/specs/2026-07-23-camera-coverage-visualization-design.md`

## Global Constraints

- 대상 그룹: terrain + `generated_environment_meshes.npz` 안의 모든 `{name}_vertices`/`{name}_triangles` 쌍을 동적 스캔 (코드 수정 없이 신규 그룹 자동 포함).
- 삼각형 판정: centroid 1개 지점만 검사.
- 가시성 판정: `visibility_tolerance_m` 기본 `0.5`, `minimum_depth_m` 기본 `0.20`, `maximum_depth_m` 기본 `30.0` (기존 `victim_localizer_node`와 동일 기본값).
- 소유권: "먼저 본 드론이 유지" — 처리 순서는 `drone_ids` 리스트 순서(기본 `quadrotor_01 → 02 → 03`), 이미 소유된 삼각형은 재검사하지 않음.
- 좌표변환·픽셀투영·depth lookup은 반드시 numpy 벡터화 배치 처리로 구현한다 (삼각형별 파이썬 반복문 금지 — 1Hz 성능 목표의 전제 조건).
- 출력: `/forest_rescue/coverage_markers` (MarkerArray, Depth 1/Transient Local/Reliable/Keep Last), `/forest_rescue/coverage_area_m2` (std_msgs/Float32, 1초 주기, volatile).
- mesh 파일은 시뮬레이션 시작 전 1회성 export이므로 최초 로드 성공 후에는 재로드하지 않는다.
- `refresh_period_sec` 기본 `1.0`, `coverage_z_offset_m` 기본 `0.05`.
- 드론별 색상 파라미터명: `drone_01_color_rgb`(`[0.55,0.0,0.85]`), `drone_02_color_rgb`(`[0.73,0.33,0.83]`), `drone_03_color_rgb`(`[0.60,0.0,0.50]`).
- 이 환경의 `pytest`는 `launch_testing` 플러그인이 pytest 9.x hookspec과 충돌해 맨 `python3 -m pytest`가 즉시 크래시한다. 모든 pytest 실행 명령에 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 환경변수를 반드시 붙인다.

---

### Task 1: `coverage_geometry.py` — mesh 조립 (centroid/면적)

**Files:**
- Create: `src/forest_rescue_system/forest_rescue_system/coverage_geometry.py`
- Test: `src/forest_rescue_system/test/test_coverage_geometry.py`

**Interfaces:**
- Produces:
  - `triangle_vertex_positions(vertices: np.ndarray, triangles: np.ndarray) -> np.ndarray` (shape `(N,3,3)`)
  - `triangle_centroids(triangle_positions: np.ndarray) -> np.ndarray` (shape `(N,3)`)
  - `triangle_areas(triangle_positions: np.ndarray) -> np.ndarray` (shape `(N,)`)
  - `SceneMesh` dataclass: `group_names: list[str]`, `group_slices: dict[str, slice]`, `triangle_positions: np.ndarray`, `centroids: np.ndarray`, `areas: np.ndarray`
  - `assemble_scene(groups: dict[str, tuple[np.ndarray, np.ndarray]]) -> SceneMesh` — `groups`는 삽입 순서대로 이어붙임

- [ ] **Step 1: 실패하는 테스트 작성**

`src/forest_rescue_system/test/test_coverage_geometry.py` 생성:

```python
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
```

- [ ] **Step 2: 테스트 실행 후 실패 확인**

Run: `cd /home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_coverage_geometry.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'forest_rescue_system.coverage_geometry'`

- [ ] **Step 3: 최소 구현 작성**

`src/forest_rescue_system/forest_rescue_system/coverage_geometry.py` 생성:

```python
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
```

- [ ] **Step 4: 테스트 실행 후 통과 확인**

Run: `cd /home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_coverage_geometry.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: 커밋**

```bash
git add src/forest_rescue_system/forest_rescue_system/coverage_geometry.py src/forest_rescue_system/test/test_coverage_geometry.py
git commit -m "feat: coverage mesh 조립 순수 함수 추가"
```

---

### Task 2: `coverage_geometry.py` — 카메라 투영 수학

**Files:**
- Modify: `src/forest_rescue_system/forest_rescue_system/coverage_geometry.py`
- Modify: `src/forest_rescue_system/test/test_coverage_geometry.py`

**Interfaces:**
- Consumes: 없음 (독립 함수)
- Produces:
  - `scaled_intrinsics(k, info_width: int, info_height: int, depth_width: int, depth_height: int) -> tuple[float, float, float, float]` — `(fx, fy, cx, cy)`
  - `transform_matrix_from_tf(translation: tuple[float, float, float], quaternion: tuple[float, float, float, float]) -> np.ndarray` (shape `(4,4)`, quaternion 순서는 `(x,y,z,w)`)
  - `apply_transform(points: np.ndarray, matrix: np.ndarray) -> np.ndarray` (shape `(N,3)`)

- [ ] **Step 1: 실패하는 테스트 추가**

`test_coverage_geometry.py` 끝에 추가:

```python
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
```

- [ ] **Step 2: 테스트 실행 후 실패 확인**

Run: `cd /home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_coverage_geometry.py -v -k "scaled_intrinsics or transform_matrix"`
Expected: FAIL — `AttributeError: module 'forest_rescue_system.coverage_geometry' has no attribute 'scaled_intrinsics'`

- [ ] **Step 3: 최소 구현 추가**

`coverage_geometry.py` 끝에 추가:

```python
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
```

- [ ] **Step 4: 테스트 실행 후 통과 확인**

Run: `cd /home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_coverage_geometry.py -v`
Expected: PASS (7 passed)

- [ ] **Step 5: 커밋**

```bash
git add src/forest_rescue_system/forest_rescue_system/coverage_geometry.py src/forest_rescue_system/test/test_coverage_geometry.py
git commit -m "feat: coverage 카메라 투영/TF 변환 순수 함수 추가"
```

---

### Task 3: `coverage_geometry.py` — depth 기반 가시성 판정

**Files:**
- Modify: `src/forest_rescue_system/forest_rescue_system/coverage_geometry.py`
- Modify: `src/forest_rescue_system/test/test_coverage_geometry.py`

**Interfaces:**
- Consumes: 없음 (독립 함수, `scaled_intrinsics`가 계산한 `fx,fy,cx,cy`를 인자로 받는 용도)
- Produces:
  - `visibility_mask(points_camera: np.ndarray, fx: float, fy: float, cx: float, cy: float, depth_image: np.ndarray, tolerance_m: float, min_depth_m: float, max_depth_m: float) -> np.ndarray` (shape `(N,)`, dtype bool)

- [ ] **Step 1: 실패하는 테스트 추가**

`test_coverage_geometry.py` 끝에 추가:

```python
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
```

- [ ] **Step 2: 테스트 실행 후 실패 확인**

Run: `cd /home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_coverage_geometry.py -v -k visibility_mask`
Expected: FAIL — `AttributeError: module 'forest_rescue_system.coverage_geometry' has no attribute 'visibility_mask'`

- [ ] **Step 3: 최소 구현 추가**

`coverage_geometry.py` 끝에 추가:

```python
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
```

- [ ] **Step 4: 테스트 실행 후 통과 확인**

Run: `cd /home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_coverage_geometry.py -v`
Expected: PASS (12 passed)

- [ ] **Step 5: 커밋**

```bash
git add src/forest_rescue_system/forest_rescue_system/coverage_geometry.py src/forest_rescue_system/test/test_coverage_geometry.py
git commit -m "feat: coverage depth 기반 가시성 판정 함수 추가"
```

---

### Task 4: `coverage_mesh.py` — npz mesh 로딩 및 동적 그룹 스캔

**Files:**
- Create: `src/forest_rescue_system/forest_rescue_system/coverage_mesh.py`
- Test: `src/forest_rescue_system/test/test_coverage_mesh.py`

**Interfaces:**
- Consumes: 없음
- Produces:
  - `scan_dynamic_groups(npz_keys: list[str]) -> list[str]` (정렬된 그룹 이름 목록)
  - `load_environment_groups(path: Path) -> dict[str, tuple[np.ndarray, np.ndarray]]`
  - `load_terrain_group(path: Path) -> dict[str, tuple[np.ndarray, np.ndarray]]` (키는 항상 `"terrain"`)

mtime/size 기반 파일-변경-감지 시그니처 함수는 만들지 않는다. Global Constraints에 명시된 대로 mesh는 최초 1회만 로드하고 재로드하지 않으므로(Task 6 `_load_mesh_if_ready`가 `self.scene is not None`이면 즉시 return), 변경 감지 자체가 불필요하다.

- [ ] **Step 1: 실패하는 테스트 작성**

`src/forest_rescue_system/test/test_coverage_mesh.py` 생성:

```python
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
```

- [ ] **Step 2: 테스트 실행 후 실패 확인**

Run: `cd /home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_coverage_mesh.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'forest_rescue_system.coverage_mesh'`

- [ ] **Step 3: 최소 구현 작성**

`src/forest_rescue_system/forest_rescue_system/coverage_mesh.py` 생성:

```python
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
```

- [ ] **Step 4: 테스트 실행 후 통과 확인**

Run: `cd /home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_coverage_mesh.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: 커밋**

```bash
git add src/forest_rescue_system/forest_rescue_system/coverage_mesh.py src/forest_rescue_system/test/test_coverage_mesh.py
git commit -m "feat: coverage npz mesh 로딩/동적 그룹 스캔 추가"
```

---

### Task 5: `coverage_ownership.py` — 소유권 상태 관리 (먼저 본 드론 유지)

**Files:**
- Create: `src/forest_rescue_system/forest_rescue_system/coverage_ownership.py`
- Test: `src/forest_rescue_system/test/test_coverage_ownership.py`

**Interfaces:**
- Consumes: 없음
- Produces:
  - `TriangleOwnership(triangle_count: int)` 클래스
    - `.owner_ids -> np.ndarray` (dtype int32, 미소유는 `-1`)
    - `.unclaimed_mask() -> np.ndarray` (dtype bool)
    - `.claim(triangle_indices, drone_index: int) -> None` — 이미 소유된 인덱스는 덮어쓰지 않음
    - `.indices_for_drone(drone_index: int) -> np.ndarray`

- [ ] **Step 1: 실패하는 테스트 작성**

`src/forest_rescue_system/test/test_coverage_ownership.py` 생성:

```python
import numpy as np

from forest_rescue_system.coverage_ownership import TriangleOwnership


def test_claim_assigns_owner_to_unclaimed_triangles():
    ownership = TriangleOwnership(3)
    ownership.claim([0, 2], drone_index=1)
    np.testing.assert_array_equal(ownership.owner_ids, [1, -1, 1])


def test_claim_does_not_overwrite_existing_owner():
    ownership = TriangleOwnership(2)
    ownership.claim([0], drone_index=0)
    ownership.claim([0, 1], drone_index=1)
    np.testing.assert_array_equal(ownership.owner_ids, [0, 1])


def test_unclaimed_mask_reflects_current_state():
    ownership = TriangleOwnership(3)
    ownership.claim([1], drone_index=0)
    np.testing.assert_array_equal(
        ownership.unclaimed_mask(), [True, False, True]
    )


def test_indices_for_drone_returns_only_owned_indices():
    ownership = TriangleOwnership(4)
    ownership.claim([0, 3], drone_index=2)
    np.testing.assert_array_equal(ownership.indices_for_drone(2), [0, 3])
    np.testing.assert_array_equal(ownership.indices_for_drone(0), [])
```

- [ ] **Step 2: 테스트 실행 후 실패 확인**

Run: `cd /home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_coverage_ownership.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'forest_rescue_system.coverage_ownership'`

- [ ] **Step 3: 최소 구현 작성**

`src/forest_rescue_system/forest_rescue_system/coverage_ownership.py` 생성:

```python
#!/usr/bin/env python3

"""삼각형별 소유 드론을 관리한다 (먼저 본 드론이 소유권 유지)."""

import numpy as np


class TriangleOwnership:
    def __init__(self, triangle_count):
        self._owner = np.full(triangle_count, -1, dtype=np.int32)

    @property
    def owner_ids(self):
        return self._owner

    def unclaimed_mask(self):
        return self._owner < 0

    def claim(self, triangle_indices, drone_index):
        indices = np.asarray(triangle_indices, dtype=np.int64)
        if indices.size == 0:
            return
        unclaimed = indices[self._owner[indices] < 0]
        self._owner[unclaimed] = drone_index

    def indices_for_drone(self, drone_index):
        return np.where(self._owner == drone_index)[0]
```

- [ ] **Step 4: 테스트 실행 후 통과 확인**

Run: `cd /home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_coverage_ownership.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: 커밋**

```bash
git add src/forest_rescue_system/forest_rescue_system/coverage_ownership.py src/forest_rescue_system/test/test_coverage_ownership.py
git commit -m "feat: coverage 삼각형 소유권 상태 관리 추가"
```

---

### Task 6: `coverage_visualization_node.py` — ROS 2 노드 구현

**Files:**
- Create: `src/forest_rescue_system/forest_rescue_system/coverage_visualization_node.py`
- Test: `src/forest_rescue_system/test/test_coverage_visualization_node.py`

**Interfaces:**
- Consumes:
  - `coverage_geometry.assemble_scene`, `.scaled_intrinsics`, `.transform_matrix_from_tf`, `.apply_transform`, `.visibility_mask`, `.SceneMesh` (Task 1-3)
  - `coverage_mesh.load_terrain_group`, `.load_environment_groups` (Task 4)
  - `coverage_ownership.TriangleOwnership` (Task 5)
- Produces: `CoverageVisualizationNode` 클래스, `main(args=None)`

**주의:** 테스트에서 `parameter_overrides`로 mesh 경로를 임시 파일로 바꿔치기해야 하므로, 생성자가 `**kwargs`를 `Node.__init__`으로 그대로 전달하도록 만든다 (다른 노드들에는 없는 패턴이지만, 테스트 주입을 위해 이 노드에 한해 필요).

- [ ] **Step 1: 실패하는 테스트 작성**

`src/forest_rescue_system/test/test_coverage_visualization_node.py` 생성:

```python
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformException

from forest_rescue_system.coverage_ownership import TriangleOwnership
from forest_rescue_system.coverage_visualization_node import (
    CoverageVisualizationNode,
)
from forest_rescue_system import coverage_geometry, coverage_mesh


class _StubTfBuffer:
    """map→camera 항등 변환만 돌려주는 테스트용 TF 버퍼."""

    def lookup_transform(self, target_frame, source_frame, time, timeout=None):
        stamped = TransformStamped()
        stamped.transform.rotation.w = 1.0
        return stamped


@pytest.fixture
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node(rclpy_context, tmp_path):
    terrain_path = tmp_path / "terrain.npz"
    np.savez(
        terrain_path,
        vertices=np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
        ),
        triangles=np.array([[0, 1, 2]]),
    )
    env_path = tmp_path / "env.npz"
    np.savez(
        env_path,
        rocks_vertices=np.array(
            [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 2.0, 0.0]]
        ),
        rocks_triangles=np.array([[0, 1, 2]]),
    )
    node = CoverageVisualizationNode(
        parameter_overrides=[
            Parameter(
                "terrain_mesh_path",
                Parameter.Type.STRING,
                str(terrain_path),
            ),
            Parameter(
                "environment_mesh_path",
                Parameter.Type.STRING,
                str(env_path),
            ),
        ]
    )
    return node


def test_constructor_loads_mesh_and_builds_scene(rclpy_context, tmp_path):
    node = _make_node(rclpy_context, tmp_path)
    try:
        assert node.scene is not None
        assert sorted(node.scene.group_names) == ["rocks", "terrain"]
        assert len(node.scene.centroids) == 2
        assert node.ownership is not None
    finally:
        node.destroy_node()


def _synthetic_scene_and_ownership():
    vertices = np.array(
        [
            [-0.05, -0.05, 5.0],
            [0.05, -0.05, 5.0],
            [0.0, 0.05, 5.0],
            [-0.05, -0.05, 20.0],
            [0.05, -0.05, 20.0],
            [0.0, 0.05, 20.0],
        ]
    )
    triangles = np.array([[0, 1, 2], [3, 4, 5]])
    scene = coverage_geometry.assemble_scene({"synthetic": (vertices, triangles)})
    ownership = TriangleOwnership(len(scene.centroids))
    return scene, ownership


def _camera_info():
    info = CameraInfo()
    info.k = [100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0]
    info.width = 100
    info.height = 100
    return info


def test_process_drone_claims_visible_triangle_and_rejects_occluded_one(
    rclpy_context, tmp_path
):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        depth_image = np.zeros((100, 100), dtype=np.float32)
        depth_image[50, 50] = 5.0
        node.depth_by_drone["quadrotor_01"] = depth_image

        node._process_drone(0, "quadrotor_01")

        np.testing.assert_array_equal(node.ownership.owner_ids, [0, -1])
    finally:
        node.destroy_node()


def test_process_drone_does_not_overwrite_existing_owner(rclpy_context, tmp_path):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.ownership.claim([0], drone_index=1)
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        depth_image = np.zeros((100, 100), dtype=np.float32)
        depth_image[50, 50] = 5.0
        node.depth_by_drone["quadrotor_01"] = depth_image

        node._process_drone(0, "quadrotor_01")

        np.testing.assert_array_equal(node.ownership.owner_ids, [1, -1])
    finally:
        node.destroy_node()


def test_build_coverage_marker_array_uses_owner_namespaces_and_colors(
    rclpy_context, tmp_path
):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.ownership.claim([0], drone_index=0)

        marker_array = node._build_coverage_marker_array()

        assert [marker.ns for marker in marker_array.markers] == [
            "coverage_drone_01",
            "coverage_drone_02",
            "coverage_drone_03",
        ]
        assert len(marker_array.markers[0].points) == 3
        assert len(marker_array.markers[1].points) == 0

        expected_color = [
            float(value)
            for value in node.get_parameter("drone_01_color_rgb").value
        ]
        marker_color = marker_array.markers[0].color
        assert [marker_color.r, marker_color.g, marker_color.b] == expected_color
    finally:
        node.destroy_node()


def test_compute_total_area_sums_only_owned_triangles(rclpy_context, tmp_path):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()

        assert node._compute_total_area() == pytest.approx(0.0)

        node.ownership.claim([0], drone_index=0)
        expected = float(node.scene.areas[0])
        assert node._compute_total_area() == pytest.approx(expected)
    finally:
        node.destroy_node()


def test_process_drone_skips_when_camera_info_missing(rclpy_context, tmp_path):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.tf_buffer = _StubTfBuffer()
        # camera_info_by_drone에 아무것도 등록하지 않음 (아직 수신 전 상황)

        node._process_drone(0, "quadrotor_01")

        np.testing.assert_array_equal(node.ownership.owner_ids, [-1, -1])
    finally:
        node.destroy_node()


def test_process_drone_skips_when_depth_missing(rclpy_context, tmp_path):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        # depth_by_drone에 아무것도 등록하지 않음 (아직 수신 전 상황)

        node._process_drone(0, "quadrotor_01")

        np.testing.assert_array_equal(node.ownership.owner_ids, [-1, -1])
    finally:
        node.destroy_node()


def test_process_drone_skips_when_tf_lookup_fails(rclpy_context, tmp_path):
    class _FailingTfBuffer:
        def lookup_transform(self, target_frame, source_frame, time, timeout=None):
            raise TransformException("no transform available")

    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.tf_buffer = _FailingTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        depth_image = np.zeros((100, 100), dtype=np.float32)
        depth_image[50, 50] = 5.0
        node.depth_by_drone["quadrotor_01"] = depth_image

        node._process_drone(0, "quadrotor_01")

        np.testing.assert_array_equal(node.ownership.owner_ids, [-1, -1])
    finally:
        node.destroy_node()


def test_constructor_leaves_scene_none_when_mesh_files_are_missing(
    rclpy_context, tmp_path
):
    node = CoverageVisualizationNode(
        parameter_overrides=[
            Parameter(
                "terrain_mesh_path",
                Parameter.Type.STRING,
                str(tmp_path / "missing_terrain.npz"),
            ),
            Parameter(
                "environment_mesh_path",
                Parameter.Type.STRING,
                str(tmp_path / "missing_env.npz"),
            ),
        ]
    )
    try:
        assert node.scene is None
        assert node.ownership is None
    finally:
        node.destroy_node()


def test_load_mesh_if_ready_does_not_reload_after_first_success(
    rclpy_context, tmp_path, monkeypatch
):
    node = _make_node(rclpy_context, tmp_path)
    try:
        assert node.scene is not None

        call_count = 0
        original = coverage_mesh.load_terrain_group

        def _counting_load_terrain_group(path):
            nonlocal call_count
            call_count += 1
            return original(path)

        monkeypatch.setattr(
            coverage_mesh, "load_terrain_group", _counting_load_terrain_group
        )

        node._load_mesh_if_ready()
        node._load_mesh_if_ready()

        assert call_count == 0
    finally:
        node.destroy_node()
```

- [ ] **Step 2: 테스트 실행 후 실패 확인**

Run: `cd /home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_coverage_visualization_node.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'forest_rescue_system.coverage_visualization_node'`

- [ ] **Step 3: 노드 구현 작성**

`src/forest_rescue_system/forest_rescue_system/coverage_visualization_node.py` 생성:

```python
#!/usr/bin/env python3

"""드론 3대가 depth로 실제 확인한 지형/식생을 누적 표시한다."""

from functools import partial
from pathlib import Path
import time

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from forest_rescue_system import coverage_geometry, coverage_mesh
from forest_rescue_system.coverage_ownership import TriangleOwnership
from forest_rescue_system.log_utils import TimestampedNode

_COLOR_PARAMETERS = (
    "drone_01_color_rgb",
    "drone_02_color_rgb",
    "drone_03_color_rgb",
)


class CoverageVisualizationNode(TimestampedNode):
    """카메라 depth 기반 실제 가시성으로 커버리지를 누적 표시한다."""

    def __init__(self, **kwargs):
        super().__init__("coverage_visualization_node", **kwargs)

        self.declare_parameter(
            "drone_ids",
            ["quadrotor_01", "quadrotor_02", "quadrotor_03"],
        )
        self.declare_parameter(
            "terrain_mesh_path",
            "~/b3_cobot3_ws/isaac_sim/generated_terrain_mesh.npz",
        )
        self.declare_parameter(
            "environment_mesh_path",
            "~/b3_cobot3_ws/isaac_sim/generated_environment_meshes.npz",
        )
        self.declare_parameter(
            "coverage_marker_topic", "/forest_rescue/coverage_markers"
        )
        self.declare_parameter(
            "coverage_area_topic", "/forest_rescue/coverage_area_m2"
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("refresh_period_sec", 1.0)
        self.declare_parameter("area_publish_period_sec", 1.0)
        self.declare_parameter("visibility_tolerance_m", 0.5)
        self.declare_parameter("minimum_depth_m", 0.20)
        self.declare_parameter("maximum_depth_m", 30.0)
        self.declare_parameter("coverage_z_offset_m", 0.05)
        self.declare_parameter("drone_01_color_rgb", [0.55, 0.0, 0.85])
        self.declare_parameter("drone_02_color_rgb", [0.73, 0.33, 0.83])
        self.declare_parameter("drone_03_color_rgb", [0.60, 0.0, 0.50])

        self.drone_ids = [
            str(value) for value in self.get_parameter("drone_ids").value
        ]
        self.terrain_mesh_path = Path(
            str(self.get_parameter("terrain_mesh_path").value)
        ).expanduser()
        self.environment_mesh_path = Path(
            str(self.get_parameter("environment_mesh_path").value)
        ).expanduser()
        self.map_frame = str(self.get_parameter("map_frame").value)

        self.scene = None
        self.ownership = None
        self.last_mesh_wait_log_at = float("-inf")

        self.bridge = CvBridge()
        self.camera_info_by_drone = {}
        self.depth_by_drone = {}

        for drone_id in self.drone_ids:
            self.create_subscription(
                CameraInfo,
                f"/{drone_id}/Camera/camera_info",
                partial(self._camera_info_callback, drone_id),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                f"/{drone_id}/Camera/depth",
                partial(self._depth_callback, drone_id),
                qos_profile_sensor_data,
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("coverage_marker_topic").value),
            marker_qos,
        )
        self.area_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter("coverage_area_topic").value),
            10,
        )

        self._load_mesh_if_ready()

        refresh_period = max(
            0.1, float(self.get_parameter("refresh_period_sec").value)
        )
        self.refresh_timer = self.create_timer(
            refresh_period, self._refresh_coverage
        )
        area_period = max(
            0.1, float(self.get_parameter("area_publish_period_sec").value)
        )
        self.area_timer = self.create_timer(area_period, self._publish_area)

        self.get_logger().info("커버리지 시각화 노드 시작")

    def _camera_info_callback(self, drone_id, message):
        self.camera_info_by_drone[drone_id] = message

    def _depth_callback(self, drone_id, message):
        try:
            depth = self.bridge.imgmsg_to_cv2(
                message, desired_encoding="passthrough"
            )
        except CvBridgeError as error:
            self.get_logger().error(f"{drone_id} Depth 변환 실패: {error}")
            return
        self.depth_by_drone[drone_id] = np.asarray(
            depth, dtype=np.float32
        ).copy()

    def _load_mesh_if_ready(self):
        if self.scene is not None:
            return
        if (
            not self.terrain_mesh_path.is_file()
            or not self.environment_mesh_path.is_file()
        ):
            now = time.monotonic()
            if now - self.last_mesh_wait_log_at >= 5.0:
                self.get_logger().info(
                    "지형/환경 Mesh 파일 대기 중: "
                    f"{self.terrain_mesh_path}, {self.environment_mesh_path}"
                )
                self.last_mesh_wait_log_at = now
            return

        try:
            groups = {}
            groups.update(
                coverage_mesh.load_terrain_group(self.terrain_mesh_path)
            )
            groups.update(
                coverage_mesh.load_environment_groups(
                    self.environment_mesh_path
                )
            )
        except (OSError, KeyError, ValueError) as error:
            self.get_logger().warning(
                f"Mesh 읽기 실패, 다음 주기에 재시도: {error}"
            )
            return

        if not groups:
            self.get_logger().warning(
                "Mesh 파일에 표시 가능한 그룹이 없습니다."
            )
            return

        self.scene = coverage_geometry.assemble_scene(groups)
        self.ownership = TriangleOwnership(len(self.scene.centroids))
        self.get_logger().info(
            "Mesh 로드 완료: "
            f"groups={self.scene.group_names}, "
            f"triangles={len(self.scene.centroids)}"
        )

    def _refresh_coverage(self):
        self._load_mesh_if_ready()
        if self.scene is None or self.ownership is None:
            return

        for drone_index, drone_id in enumerate(self.drone_ids):
            self._process_drone(drone_index, drone_id)

        self._publish_markers()

    def _process_drone(self, drone_index, drone_id):
        camera_info = self.camera_info_by_drone.get(drone_id)
        depth_image = self.depth_by_drone.get(drone_id)
        if camera_info is None or depth_image is None:
            return

        candidate_indices = np.where(self.ownership.unclaimed_mask())[0]
        if candidate_indices.size == 0:
            return

        camera_frame = f"{drone_id}/camera_optical_frame"
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                camera_frame,
                self.map_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            return

        matrix = coverage_geometry.transform_matrix_from_tf(
            (
                transform_stamped.transform.translation.x,
                transform_stamped.transform.translation.y,
                transform_stamped.transform.translation.z,
            ),
            (
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z,
                transform_stamped.transform.rotation.w,
            ),
        )
        points_camera = coverage_geometry.apply_transform(
            self.scene.centroids[candidate_indices], matrix
        )

        depth_height, depth_width = depth_image.shape[:2]
        info_width = int(camera_info.width) or depth_width
        info_height = int(camera_info.height) or depth_height
        fx, fy, cx, cy = coverage_geometry.scaled_intrinsics(
            camera_info.k, info_width, info_height, depth_width, depth_height
        )

        visible = coverage_geometry.visibility_mask(
            points_camera,
            fx,
            fy,
            cx,
            cy,
            depth_image,
            float(self.get_parameter("visibility_tolerance_m").value),
            float(self.get_parameter("minimum_depth_m").value),
            float(self.get_parameter("maximum_depth_m").value),
        )
        visible_global_indices = candidate_indices[visible]
        if visible_global_indices.size:
            self.ownership.claim(visible_global_indices, drone_index)

    def _build_coverage_marker_array(self):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        z_offset = float(self.get_parameter("coverage_z_offset_m").value)

        for drone_index in range(len(self.drone_ids)):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = stamp
            marker.ns = f"coverage_drone_{drone_index + 1:02d}"
            marker.id = 0
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.pose.position.z = z_offset
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.frame_locked = True

            color = [
                float(value)
                for value in self.get_parameter(
                    _COLOR_PARAMETERS[drone_index]
                ).value
            ]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0

            marker.points = []
            for triangle_index in self.ownership.indices_for_drone(
                drone_index
            ):
                for vertex in self.scene.triangle_positions[triangle_index]:
                    point = Point()
                    point.x = float(vertex[0])
                    point.y = float(vertex[1])
                    point.z = float(vertex[2])
                    marker.points.append(point)

            marker_array.markers.append(marker)

        return marker_array

    def _publish_markers(self):
        self.marker_publisher.publish(self._build_coverage_marker_array())

    def _compute_total_area(self):
        if self.scene is None or self.ownership is None:
            return 0.0
        owned_mask = self.ownership.owner_ids >= 0
        return float(np.sum(self.scene.areas[owned_mask]))

    def _publish_area(self):
        message = Float32()
        message.data = self._compute_total_area()
        self.area_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageVisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 테스트 실행 후 통과 확인**

Run: `cd /home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_coverage_visualization_node.py -v`
Expected: PASS (10 passed)

- [ ] **Step 5: 전체 pytest 스위트 재확인**

Run: `cd /home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -v`
Expected: PASS (30 passed)

- [ ] **Step 6: 커밋**

```bash
git add src/forest_rescue_system/forest_rescue_system/coverage_visualization_node.py src/forest_rescue_system/test/test_coverage_visualization_node.py
git commit -m "feat: coverage_visualization_node 구현"
```

---

### Task 7: config/launch/setup 연동

**Files:**
- Modify: `src/forest_rescue_system/config/forest_rescue.yaml`
- Modify: `src/forest_rescue_system/launch/forest_rescue_system.launch.py`
- Modify: `src/forest_rescue_system/setup.py`

**Interfaces:**
- Consumes: Task 6의 `coverage_visualization_node.py`가 선언하는 파라미터 이름 전부, `console_scripts` 실행 파일명 `coverage_visualization`

- [ ] **Step 1: `config/forest_rescue.yaml`에 노드 섹션 추가**

`rviz_visualization_node:` 섹션 바로 뒤(`mission_manager_node:` 앞)에 삽입:

```yaml
coverage_visualization_node:
  ros__parameters:
    drone_ids: [quadrotor_01, quadrotor_02, quadrotor_03]
    terrain_mesh_path: ~/b3_cobot3_ws/isaac_sim/generated_terrain_mesh.npz
    environment_mesh_path: ~/b3_cobot3_ws/isaac_sim/generated_environment_meshes.npz
    coverage_marker_topic: /forest_rescue/coverage_markers
    coverage_area_topic: /forest_rescue/coverage_area_m2
    map_frame: map
    refresh_period_sec: 1.0
    area_publish_period_sec: 1.0
    visibility_tolerance_m: 0.5
    minimum_depth_m: 0.20
    maximum_depth_m: 30.0
    coverage_z_offset_m: 0.05
    drone_01_color_rgb: [0.55, 0.0, 0.85]
    drone_02_color_rgb: [0.73, 0.33, 0.83]
    drone_03_color_rgb: [0.60, 0.0, 0.50]

```

- [ ] **Step 2: `launch/forest_rescue_system.launch.py`에 노드 추가**

`rviz_visualization_node` 등록 직후(드론 루프 `for index in range(1, 4):` 이전)에 삽입:

```python
        # 드론 카메라가 실제로 확인한 지형/식생을 보라색으로 누적 표시한다.
        Node(
            package="forest_rescue_system",
            executable="coverage_visualization",
            name="coverage_visualization_node",
            output="screen",
            parameters=[config, {"use_sim_time": use_sim_time}],
        ),
```

- [ ] **Step 3: `setup.py`에 console_scripts 추가**

`"rviz_visualization = forest_rescue_system.rviz_visualization_node:main",` 다음 줄에 추가:

```python
            "coverage_visualization = forest_rescue_system.coverage_visualization_node:main",
```

- [ ] **Step 4: 빌드로 검증**

Run: `cd /home/hwangjeongui/b3_cobot3_ws && colcon build --packages-select forest_rescue_system --symlink-install`
Expected: `Summary: 1 package finished` (에러 없음)

Run: `ros2 pkg executables forest_rescue_system | grep coverage_visualization`
Expected: `forest_rescue_system coverage_visualization` 출력

- [ ] **Step 5: 커밋**

```bash
git add src/forest_rescue_system/config/forest_rescue.yaml src/forest_rescue_system/launch/forest_rescue_system.launch.py src/forest_rescue_system/setup.py
git commit -m "feat: coverage_visualization_node를 launch/config/setup에 연결"
```

---

### Task 8: RViz Display 추가 + 수동 통합 확인

**Files:**
- Modify: `src/forest_rescue_system/config/forest_rescue_multi.rviz`

- [ ] **Step 1: MarkerArray Display 블록 추가**

`Environment Groups` MarkerArray Display 블록(`Class: rviz_default_plugins/MarkerArray`, `Name: Environment Groups`) 바로 뒤, `Enabled: true` (Displays 리스트 닫힘) 앞에 삽입:

```yaml
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: Coverage Markers
      Namespaces:
        coverage_drone_01: true
        coverage_drone_02: true
        coverage_drone_03: true
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /forest_rescue/coverage_markers
      Value: true
```

- [ ] **Step 2: 커밋**

```bash
git add src/forest_rescue_system/config/forest_rescue_multi.rviz
git commit -m "feat: RViz에 Coverage Markers Display 추가"
```

- [ ] **Step 3: 수동 통합 확인 (코드 변경 없음, 확인 절차만)**

Run: `ros2 launch forest_rescue_system forest_rescue_system.launch.py use_rviz:=true`

확인 항목:
1. RViz Displays 패널에 `Coverage Markers`가 나타나고 체크돼 있는지 확인.
2. 드론이 이동하며 실제로 카메라로 지나간 지형/나무가 보라색으로 바뀌는지 육안 확인.
3. 별도 터미널에서 `ros2 topic echo /forest_rescue/coverage_area_m2 --once` 반복 실행하여 값이 시간에 따라 증가하는지 확인.
4. 커버된 영역 위로 드론이 다시 지나가도 색이 사라지지 않는지(다른 위치로 이동한 뒤에도 이전 영역이 계속 보라색인지) 확인.
5. `ros2 topic hz /forest_rescue/coverage_markers`로 약 1Hz 발행을 확인.

이 단계는 시뮬레이터가 실행 중이어야 하므로 사람이 직접 확인한다. 문제가 없으면 계획 완료.

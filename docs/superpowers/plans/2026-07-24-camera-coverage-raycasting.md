# 드론 카메라 커버리지 레이캐스팅 재설계 + 손전등 시각화 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** depth 이미지 리프로젝션 비교 방식의 커버리지 가시성 판정을, 카메라 픽셀 광선을 메쉬와 직접 교차판정하는 방식(섀도우 레이)으로 교체해 그레이징 각도 버그를 근본적으로 없애고, RViz에 드론 카메라를 손전등처럼 보여주는 실시간 시각화를 추가한다.

**Architecture:** `coverage_geometry.py`에 open3d `RaycastingScene` 기반 순수 함수(`pixel_to_camera_ray`, `pixel_grid_uv`, `build_raycasting_scene`, `cast_visibility_rays`)를 추가하고 기존 depth 비교 함수 전체를 제거한다. `coverage_visualization_node.py`는 mesh 로드 시 1회 레이캐스팅 씬을 빌드하고, 매 프레임 카메라 픽셀 격자를 광선으로 쏴 얻은 히트 결과를 (a) `TriangleOwnership` 누적 클레임과 (b) 손전등 마커 양쪽에 재사용한다.

**Tech Stack:** ROS 2 (rclpy), numpy, open3d (`o3d.t.geometry.RaycastingScene`), tf2_ros, visualization_msgs/Marker, pytest.

## Global Constraints

- 스펙 문서: `docs/superpowers/specs/2026-07-24-camera-coverage-raycasting-design.md` (필독, 이 플랜의 근거).
- open3d는 rosdep 표준 키가 없으므로 `requirements.txt`에만 추가하고 `package.xml`은 건드리지 않는다. 개발 환경에서 open3d 0.19.0으로 API 검증됨.
- `o3d.t.geometry.RaycastingScene.add_triangles`는 정점 `Float32 [N,3]`, 삼각형 `UInt32 [M,3]` 텐서를 요구한다. 지오메트리를 하나만 추가하면 `cast_rays` 결과의 `primitive_ids`가 그 삼각형 배열의 0..N-1 인덱스와 그대로 대응한다 (검증됨).
- `cast_rays`의 광선 방향 벡터는 반드시 정규화해야 `t_hit`이 유클리드 거리가 된다 (검증됨). 미스(hit 없음)는 `t_hit=inf`, `primitive_ids=4294967295`.
- tf2 `lookup_transform(target_frame, source_frame, time)`가 반환하는 transform은 `source_frame`의 점을 `target_frame` 좌표로 옮긴다. 카메라 원점을 맵 좌표로, 카메라 방향벡터를 맵 방향벡터로 얻으려면 `lookup_transform(map_frame, camera_frame, time)`을 호출한다 (기존 코드의 `lookup_transform(camera_frame, map_frame, time)`과 인자 순서가 반대, 검증됨).
- 기존 코드 스타일(스네이크케이스, numpy 벡터화, ROS 파라미터 선언 패턴)을 그대로 따른다. 관련 없는 코드는 건드리지 않는다.
- 각 태스크가 끝나면 해당 범위의 pytest가 전부 통과해야 한다. **반드시 아래 형태로 실행할 것** (저장소 루트 `/home/hwangjeongui/b3_cobot3_ws`에서):
  ```bash
  PYTHONPATH="/home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system:$PYTHONPATH" \
    python3 -m pytest -p no:launch_testing -p no:launch_ros src/forest_rescue_system/test/ -v
  ```
  이 환경은 `colcon build` 없이 소스를 바로 테스트하도록 `PYTHONPATH`에 `src/forest_rescue_system`을 직접 추가해야 한다 (그렇지 않으면 `install/`의 오래된 빌드가 임포트되어 최신 소스 변경이 반영 안 된 상태로 테스트됨). 또한 이 환경의 pip `pytest`(9.1.1)가 ROS Humble의 `launch_testing`/`launch_ros` pytest 플러그인(구버전 hookspec 기준)과 호환되지 않아 플러그인을 명시적으로 꺼야 한다(`-p no:launch_testing -p no:launch_ros`) — 이 저장소의 pytest 실행 전반에 있는 기존 환경 문제이며 이 플랜과 무관하다. 개별 테스트를 `-k`로 좁힐 때도 이 prefix는 그대로 유지한다.
  - **알려진 기존 실패 (이 플랜과 무관, 손대지 말 것)**: `test_process_drone_looks_up_transform_at_depth_image_capture_time`가 `TypeError: Can't compare times with different clock types`로 실패한다 (ROS_TIME vs SYSTEM_TIME clock_type 불일치, 최근 커밋 "시간 동기화 문제 해결시도"와 관련된 기존 이슈). 이 태스크들의 변경으로 새로 깨진 게 아니라면 이 실패는 무시하고 넘어간다. 만약 어느 태스크 이후 이 테스트의 실패 메시지가 달라지거나 다른 테스트가 추가로 깨지면 그건 회귀이므로 반드시 고친다.

---

## 실행 순서 정정 (Task 3은 Task 6 다음에 실행)

**플랜 작성 시 놓친 의존성:** Task 3(구 함수/테스트 제거)은 문서상 Task 2 다음에 배치돼 있지만, `coverage_visualization_node.py`의 `_process_drone`이 `triangle_sample_points`/`visibility_mask_multi_sample`을 여전히 호출하는 건 Task 6에서만 바뀐다. 따라서 **Task 3을 Task 6 완료 후로 미뤄서 실행해야 한다.** 실제 실행 순서: Task 1 → 2 → 4 → 5 → 6 → 3 → 7 → 8 → 9. (Task 4는 config.yaml만 건드리므로 어디에 껴도 무방 — 다른 순서 변경과 독립적이다.) 각 태스크의 본문 내용/코드는 이 순서 변경과 무관하게 그대로 유효하다. Task 3의 브리핑에서 "Task 6을 먼저 완료했는지 확인"이라는 문구가 이미 이 의존성을 암시하고 있었다.

---

## Task 1: `pixel_to_camera_ray` + `pixel_grid_uv` (순수 수학, open3d 불필요)

**Files:**
- Modify: `src/forest_rescue_system/forest_rescue_system/coverage_geometry.py`
- Test: `src/forest_rescue_system/test/test_coverage_geometry.py`

**Interfaces:**
- Produces: `pixel_to_camera_ray(u, v, fx, fy, cx, cy) -> np.ndarray` (입력 u,v는 스칼라 또는 1차원 배열, 출력은 정규화된 방향벡터, 입력이 배열이면 shape `(N,3)`, 스칼라면 `(3,)`)
- Produces: `pixel_grid_uv(width, height, step_px) -> tuple[np.ndarray, np.ndarray]` (픽셀 중심 좌표 `(u_flat, v_flat)`, 둘 다 1차원 `float64` 배열, 길이는 `ceil(width/step_px) * ceil(height/step_px)`)

- [ ] **Step 1: 실패하는 테스트 작성**

`test_coverage_geometry.py` 맨 끝에 추가:

```python
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
```

- [ ] **Step 2: 테스트가 실패하는지 확인**

Run: `PYTHONPATH="/home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system:$PYTHONPATH" python3 -m pytest -p no:launch_testing -p no:launch_ros src/forest_rescue_system/test/test_coverage_geometry.py -k "pixel_to_camera_ray or pixel_grid_uv" -v`
Expected: FAIL (`AttributeError: module 'forest_rescue_system.coverage_geometry' has no attribute 'pixel_to_camera_ray'`)

- [ ] **Step 3: 최소 구현 작성**

`coverage_geometry.py`에서 `transform_direction` 함수 정의 바로 뒤(그 다음 줄, `DEFAULT_NEIGHBORHOOD_PX` 상수 정의보다 앞)에 삽입:

```python
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
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `PYTHONPATH="/home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system:$PYTHONPATH" python3 -m pytest -p no:launch_testing -p no:launch_ros src/forest_rescue_system/test/test_coverage_geometry.py -k "pixel_to_camera_ray or pixel_grid_uv" -v`
Expected: PASS (5 passed)

- [ ] **Step 5: 커밋**

```bash
git add src/forest_rescue_system/forest_rescue_system/coverage_geometry.py \
        src/forest_rescue_system/test/test_coverage_geometry.py
git commit -m "feat: 픽셀-카메라광선 변환과 픽셀 격자 생성 순수 함수 추가"
```

---

## Task 2: `build_raycasting_scene` + `cast_visibility_rays` (open3d 의존)

**Files:**
- Modify: `src/forest_rescue_system/forest_rescue_system/coverage_geometry.py`
- Modify: `requirements.txt`
- Test: `src/forest_rescue_system/test/test_coverage_geometry.py`

**Interfaces:**
- Consumes: `pixel_to_camera_ray`, `pixel_grid_uv` (Task 1, 이 태스크에서는 직접 쓰지 않지만 같은 모듈)
- Produces: `build_raycasting_scene(triangle_positions) -> open3d.t.geometry.RaycastingScene`
- Produces: `cast_visibility_rays(scene, ray_origin, ray_directions, min_depth_m, max_depth_m) -> tuple[np.ndarray, np.ndarray]` — `(hit_points_map, triangle_indices)`. `hit_points_map`은 `(K,3) float64`, `triangle_indices`는 `(K,) int64`, `K`는 유효 범위 안에서 실제로 맞은 광선 수(입력 광선 수 이하).

- [ ] **Step 1: requirements.txt에 open3d 추가**

`requirements.txt` 맨 끝에 추가:

```
# 커버리지 레이캐스팅(coverage_visualization_node)에 사용
open3d
```

Run: `pip install -r requirements.txt` (이미 설치돼 있으면 스킵됨)

- [ ] **Step 2: 실패하는 테스트 작성**

`test_coverage_geometry.py` 맨 끝에 추가:

```python
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
    # 가까운 쪽만 히트되어야 한다 (실제 오클루전, tolerance 없음).
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
```

- [ ] **Step 3: 테스트가 실패하는지 확인**

Run: `PYTHONPATH="/home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system:$PYTHONPATH" python3 -m pytest -p no:launch_testing -p no:launch_ros src/forest_rescue_system/test/test_coverage_geometry.py -k cast_visibility_rays -v`
Expected: FAIL (`AttributeError: ... has no attribute 'build_raycasting_scene'`)

- [ ] **Step 4: 최소 구현 작성**

`coverage_geometry.py` 상단 import에 추가 (`import numpy as np` 바로 아래):

```python
import open3d as o3d
import open3d.core as o3c
```

`pixel_grid_uv` 함수 바로 뒤에 추가:

```python
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
```

- [ ] **Step 5: 테스트 통과 확인**

Run: `PYTHONPATH="/home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system:$PYTHONPATH" python3 -m pytest -p no:launch_testing -p no:launch_ros src/forest_rescue_system/test/test_coverage_geometry.py -k "cast_visibility_rays or build_raycasting_scene" -v`
Expected: PASS (7 passed)

- [ ] **Step 6: 커밋**

```bash
git add src/forest_rescue_system/forest_rescue_system/coverage_geometry.py \
        src/forest_rescue_system/test/test_coverage_geometry.py \
        requirements.txt
git commit -m "feat: open3d RaycastingScene 기반 가시성 광선 교차판정 함수 추가"
```

---

## Task 3: 뎁스 리프로젝션 비교 함수/테스트 제거

**Files:**
- Modify: `src/forest_rescue_system/forest_rescue_system/coverage_geometry.py`
- Modify: `src/forest_rescue_system/test/test_coverage_geometry.py`

**Interfaces:**
- Consumes: 없음 (제거만 수행)
- Produces: 없음 (`visibility_mask`, `visibility_mask_multi_sample`, `grazing_angle_tolerance`, `triangle_sample_points`가 더 이상 존재하지 않음을 이후 태스크가 전제로 함)

- [ ] **Step 1: 사전 확인 — 다른 곳에서 안 쓰이는지 재확인**

Run: `grep -rn "visibility_mask\|grazing_angle_tolerance\|triangle_sample_points" src/forest_rescue_system --include=*.py`
Expected: `coverage_geometry.py`(정의부)와 `test_coverage_geometry.py`(제거 대상 테스트)만 나와야 한다. `coverage_visualization_node.py`에 나오면 Task 6을 먼저 완료했는지 확인.

- [ ] **Step 2: `test_coverage_geometry.py`에서 관련 테스트 삭제**

아래 테스트 함수들을 파일에서 전부 삭제한다 (본문 그대로 검색해서 제거):
- `test_visibility_mask_accepts_point_matching_depth_image`
- `test_visibility_mask_rejects_occluded_point_behind_closer_surface`
- `test_visibility_mask_rejects_point_outside_max_depth_range`
- `test_visibility_mask_rejects_point_projecting_outside_image_bounds`
- `test_visibility_mask_samples_depth_image_as_row_v_col_u`
- `test_triangle_sample_points_returns_vertices_plus_centroid`
- `test_visibility_mask_multi_sample_true_if_any_sample_visible`
- `test_visibility_mask_multi_sample_false_if_all_samples_occluded`
- `test_grazing_angle_tolerance_keeps_base_value_when_facing_camera`
- `test_grazing_angle_tolerance_scales_up_near_grazing_incidence`
- `test_visibility_mask_matches_neighboring_pixel_within_window`

(`test_triangle_normals_returns_unit_length_perpendicular_vector`, `test_transform_direction_rotates_without_applying_translation`는 그대로 유지 — 삭제하지 않는다.)

- [ ] **Step 3: `coverage_geometry.py`에서 관련 함수/상수 삭제**

`transform_direction` 함수와 `pixel_to_camera_ray` 함수 사이에 있던(Task 1에서 새 함수를 그 뒤에 삽입했으므로, 새 함수들보다 뒤쪽에 위치하게 된) 아래 블록 전체를 삭제한다:

```python
# 그레이징(스침) 각도에서 depth tolerance를 얼마나/어디까지 넓힐지에 대한 기본값.
DEFAULT_NEIGHBORHOOD_PX = 1
DEFAULT_MIN_GRAZING_COSINE = 0.2
DEFAULT_MAX_TOLERANCE_SCALE = 5.0


def grazing_angle_tolerance(
    ...
):
    ...


def visibility_mask(
    ...
):
    ...


def triangle_sample_points(triangle_positions):
    ...


def visibility_mask_multi_sample(
    ...
):
    ...
```

(생략된 `...` 부분은 이 파일 현재 내용 그대로 — `grazing_angle_tolerance`, `visibility_mask`, `triangle_sample_points`, `visibility_mask_multi_sample` 네 함수 전체와 그 위 세 상수를 통째로 지운다. 파일 끝까지가 이 블록이므로, 지우고 나면 파일은 `cast_visibility_rays` 함수로 끝난다.)

- [ ] **Step 4: 전체 테스트 통과 확인**

Run: `PYTHONPATH="/home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system:$PYTHONPATH" python3 -m pytest -p no:launch_testing -p no:launch_ros src/forest_rescue_system/test/test_coverage_geometry.py -v`
Expected: 모두 PASS, `visibility_mask`/`grazing_angle_tolerance`/`triangle_sample_points` 관련 테스트는 목록에서 사라짐

Run: `python3 -c "from forest_rescue_system import coverage_geometry; print(hasattr(coverage_geometry, 'visibility_mask'))"`
Expected: `False`

- [ ] **Step 5: 커밋**

```bash
git add src/forest_rescue_system/forest_rescue_system/coverage_geometry.py \
        src/forest_rescue_system/test/test_coverage_geometry.py
git commit -m "refactor: 뎁스 리프로젝션 비교 방식의 가시성 판정 함수 제거"
```

---

## Task 4: 파라미터/설정 정리 (`config/forest_rescue.yaml`)

**Files:**
- Modify: `src/forest_rescue_system/config/forest_rescue.yaml`

**Interfaces:**
- Consumes: 없음
- Produces: Task 6이 참조할 `ray_grid_step_px` 파라미터가 config에 존재함

- [ ] **Step 1: `coverage_visualization_node` 섹션 수정**

`config/forest_rescue.yaml`의 `visibility_tolerance_m: 0.5` 줄을 아래로 교체:

```yaml
    ray_grid_step_px: 4
```

(`minimum_depth_m`, `maximum_depth_m`, `refresh_period_sec` 등 나머지 줄은 그대로 둔다. `flashlight_*` 파라미터는 Task 7에서 추가한다 — 지금은 `ray_grid_step_px`만.)

- [ ] **Step 2: yaml 문법 확인**

Run: `python3 -c "import yaml; yaml.safe_load(open('src/forest_rescue_system/config/forest_rescue.yaml'))"`
Expected: 예외 없이 종료

- [ ] **Step 3: 커밋**

```bash
git add src/forest_rescue_system/config/forest_rescue.yaml
git commit -m "config: visibility_tolerance_m을 ray_grid_step_px로 교체"
```

---

## Task 5: 노드 depth 콜백/씬 빌드 배선 (cv_bridge 제거)

**Files:**
- Modify: `src/forest_rescue_system/forest_rescue_system/coverage_visualization_node.py`
- Test: `src/forest_rescue_system/test/test_coverage_visualization_node.py`

**Interfaces:**
- Consumes: `coverage_geometry.build_raycasting_scene` (Task 2)
- Produces: `self.raycasting_scene`(빌드된 씬 또는 `None`), `self.depth_shape_by_drone[drone_id] -> tuple[int, int]`(height, width). Task 6이 이 두 상태를 사용한다.

- [ ] **Step 1: `__init__`에서 상태/파라미터 수정**

`self.scene = None` 다음 줄에 추가:

```python
        self.raycasting_scene = None
```

`self.depth_by_drone = {}` 줄을 다음으로 교체:

```python
        self.depth_shape_by_drone = {}
```

`self.declare_parameter("visibility_tolerance_m", 0.5)` 줄을 다음으로 교체:

```python
        self.declare_parameter("ray_grid_step_px", 4)
```

파일 상단 import에서 다음 줄 삭제:

```python
from cv_bridge import CvBridge, CvBridgeError
```

`self.bridge = CvBridge()` 줄 삭제.

- [ ] **Step 2: `_depth_callback` 재작성**

기존:

```python
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
        self.depth_stamp_by_drone[drone_id] = message.header.stamp
```

교체:

```python
    def _depth_callback(self, drone_id, message):
        self.depth_shape_by_drone[drone_id] = (
            int(message.height), int(message.width)
        )
        self.depth_stamp_by_drone[drone_id] = message.header.stamp
```

- [ ] **Step 3: `_load_mesh_if_ready`에서 레이캐스팅 씬 빌드**

`self.scene = coverage_geometry.assemble_scene(groups)` 바로 다음 줄에 추가:

```python
        self.raycasting_scene = coverage_geometry.build_raycasting_scene(
            self.scene.triangle_positions
        )
```

- [ ] **Step 4: 기존 테스트 중 `depth_by_drone`을 쓰는 부분을 `depth_shape_by_drone`으로 갱신**

`test_coverage_visualization_node.py`에서 아래 패턴을 전부 찾아 교체한다 (총 8곳: `test_process_drone_claims_visible_triangle_and_rejects_occluded_one`, `test_process_drone_claims_triangle_whose_centroid_is_occluded_but_a_vertex_is_visible`, `test_process_drone_does_not_overwrite_existing_owner`, `test_process_drone_skips_when_depth_missing`, `test_process_drone_skips_when_tf_lookup_fails`, `test_process_drone_looks_up_transform_with_correct_frame_order`, `test_process_drone_looks_up_transform_at_depth_image_capture_time`, `test_refresh_coverage_first_seen_wins_across_drones_same_cycle`, `test_refresh_coverage_skips_marker_publish_when_nothing_newly_claimed`, `test_refresh_coverage_publishes_marker_when_something_newly_claimed`):

이 태스크에서는 아직 `_process_drone`을 레이캐스팅으로 바꾸지 않으므로, 이 테스트들은 Task 6에서 통째로 재작성한다. **이 태스크(Task 5)에서는 아래 두 개만 수정한다** (depth 콜백/씬 빌드만 검증하는 신규 테스트):

`test_coverage_visualization_node.py` 맨 끝에 추가:

```python
def test_constructor_builds_raycasting_scene(rclpy_context, tmp_path):
    node = _make_node(rclpy_context, tmp_path)
    try:
        assert node.raycasting_scene is not None
    finally:
        node.destroy_node()


def test_depth_callback_records_shape_and_stamp_without_cv_bridge(
    rclpy_context, tmp_path
):
    from sensor_msgs.msg import Image

    node = _make_node(rclpy_context, tmp_path)
    try:
        message = Image()
        message.height = 48
        message.width = 64
        message.encoding = "32FC1"
        message.header.stamp = TimeMsg(sec=1, nanosec=0)

        node._depth_callback("quadrotor_01", message)

        assert node.depth_shape_by_drone["quadrotor_01"] == (48, 64)
        assert node.depth_stamp_by_drone["quadrotor_01"] == message.header.stamp
    finally:
        node.destroy_node()
```

- [ ] **Step 5: 새 테스트만 우선 통과 확인 (기존 `_process_drone` 테스트는 Task 6까지 실패 상태로 남아있는 게 정상)**

Run: `PYTHONPATH="/home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system:$PYTHONPATH" python3 -m pytest -p no:launch_testing -p no:launch_ros src/forest_rescue_system/test/test_coverage_visualization_node.py -k "builds_raycasting_scene or depth_callback_records" -v`
Expected: PASS (2 passed)

- [ ] **Step 6: 커밋**

```bash
git add src/forest_rescue_system/forest_rescue_system/coverage_visualization_node.py \
        src/forest_rescue_system/test/test_coverage_visualization_node.py
git commit -m "refactor: depth 콜백에서 cv_bridge 제거, mesh 로드 시 레이캐스팅 씬 빌드"
```

Note: 이 커밋 시점에는 `_process_drone` 관련 기존 테스트들이 깨져 있다 (다음 태스크에서 고침). `subagent-driven-development`/`executing-plans`로 실행한다면 이 사실을 리뷰 코멘트에 남길 것.

---

## Task 6: `_process_drone` 레이캐스팅으로 재작성

**Files:**
- Modify: `src/forest_rescue_system/forest_rescue_system/coverage_visualization_node.py`
- Test: `src/forest_rescue_system/test/test_coverage_visualization_node.py`

**Interfaces:**
- Consumes: `coverage_geometry.pixel_grid_uv`, `coverage_geometry.pixel_to_camera_ray`, `coverage_geometry.transform_direction`, `coverage_geometry.cast_visibility_rays`, `coverage_geometry.scaled_intrinsics`, `coverage_geometry.transform_matrix_from_tf` (Task 1, 2, 기존), `self.raycasting_scene`, `self.depth_shape_by_drone` (Task 5)
- Produces: `self.flashlight_state[drone_id] -> {"origin": np.ndarray(3,), "corner_directions": np.ndarray(4,3), "hit_points": np.ndarray(K,3)}`. Task 8이 이 상태를 소비한다. `_process_drone`은 여전히 `bool`(새로 클레임된 삼각형 있었는지)을 반환한다.

- [ ] **Step 1: `__init__`에 `flashlight_state` 추가**

`self.raycasting_scene = None` 다음 줄에 추가:

```python
        self.flashlight_state = {}
```

- [ ] **Step 2: `_process_drone` 전체 재작성**

기존 `_process_drone` 메서드 전체(camera_info/depth_image 조회부터 `return bool(newly_claimed.size)`까지)를 아래로 교체:

```python
    def _process_drone(self, drone_index, drone_id):
        camera_info = self.camera_info_by_drone.get(drone_id)
        depth_shape = self.depth_shape_by_drone.get(drone_id)
        depth_stamp = self.depth_stamp_by_drone.get(drone_id)
        if camera_info is None or depth_shape is None or depth_stamp is None:
            self.flashlight_state.pop(drone_id, None)
            return False
        if self.raycasting_scene is None:
            self.flashlight_state.pop(drone_id, None)
            return False

        camera_frame = f"{drone_id}/camera_optical_frame"
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.map_frame,
                camera_frame,
                Time.from_msg(depth_stamp),
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            self.flashlight_state.pop(drone_id, None)
            return False

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
        camera_origin = matrix[:3, 3]

        depth_height, depth_width = depth_shape
        info_width = int(camera_info.width) or depth_width
        info_height = int(camera_info.height) or depth_height
        fx, fy, cx, cy = coverage_geometry.scaled_intrinsics(
            camera_info.k, info_width, info_height, depth_width, depth_height
        )

        grid_u, grid_v = coverage_geometry.pixel_grid_uv(
            depth_width,
            depth_height,
            int(self.get_parameter("ray_grid_step_px").value),
        )
        ray_directions_camera = coverage_geometry.pixel_to_camera_ray(
            grid_u, grid_v, fx, fy, cx, cy
        )
        ray_directions_map = coverage_geometry.transform_direction(
            ray_directions_camera, matrix
        )

        hit_points, triangle_indices = coverage_geometry.cast_visibility_rays(
            self.raycasting_scene,
            camera_origin,
            ray_directions_map,
            float(self.get_parameter("minimum_depth_m").value),
            float(self.get_parameter("maximum_depth_m").value),
        )

        corner_u = np.array([0.0, float(depth_width), 0.0, float(depth_width)])
        corner_v = np.array([0.0, 0.0, float(depth_height), float(depth_height)])
        corner_directions_camera = coverage_geometry.pixel_to_camera_ray(
            corner_u, corner_v, fx, fy, cx, cy
        )
        corner_directions_map = coverage_geometry.transform_direction(
            corner_directions_camera, matrix
        )
        self.flashlight_state[drone_id] = {
            "origin": camera_origin,
            "corner_directions": corner_directions_map,
            "hit_points": hit_points,
        }

        newly_claimed = np.asarray([], dtype=np.int64)
        if triangle_indices.size:
            newly_claimed = self.ownership.claim(
                np.unique(triangle_indices), drone_index
            )
        return bool(newly_claimed.size)
```

- [ ] **Step 3: 기존 `_process_drone` 관련 테스트를 레이캐스팅 방식으로 재작성**

`test_coverage_visualization_node.py`에서 `_camera_info()` 헬퍼는 그대로 두되(fx=fy=100, cx=cy=50, width=height=100 — 레이캐스팅에서도 그대로 유효), depth 픽셀 배열을 쓰던 부분을 `depth_shape_by_drone`으로 바꾸고, `_StubTfBuffer`/`_RecordingTfBuffer`는 그대로 재사용 가능(항등 변환이므로 카메라=맵 원점, 방향 변환도 항등).

아래 테스트들을 통째로 교체한다:

```python
def test_process_drone_claims_visible_triangle_and_rejects_occluded_one(
    rclpy_context, tmp_path
):
    # 카메라 원점(0,0,0)에서 +z 방향을 보는 항등 변환.
    # near 삼각형(z=5)이 far 삼각형(z=20)과 같은 시선 방향에 겹쳐 있으므로
    # 실제 오클루전으로 far는 가려져야 한다.
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        node.depth_shape_by_drone["quadrotor_01"] = (100, 100)
        node.depth_stamp_by_drone["quadrotor_01"] = TimeMsg()

        node._process_drone(0, "quadrotor_01")

        np.testing.assert_array_equal(node.ownership.owner_ids, [0, -1])
    finally:
        node.destroy_node()


def test_process_drone_does_not_overwrite_existing_owner(rclpy_context, tmp_path):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        node.ownership.claim([0], drone_index=1)
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        node.depth_shape_by_drone["quadrotor_01"] = (100, 100)
        node.depth_stamp_by_drone["quadrotor_01"] = TimeMsg()

        node._process_drone(0, "quadrotor_01")

        np.testing.assert_array_equal(node.ownership.owner_ids, [1, -1])
    finally:
        node.destroy_node()


def test_process_drone_skips_when_camera_info_missing(rclpy_context, tmp_path):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        node.tf_buffer = _StubTfBuffer()

        node._process_drone(0, "quadrotor_01")

        np.testing.assert_array_equal(node.ownership.owner_ids, [-1, -1])
    finally:
        node.destroy_node()


def test_process_drone_skips_when_depth_missing(rclpy_context, tmp_path):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        # depth_shape_by_drone에 아무것도 등록하지 않음

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
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        node.tf_buffer = _FailingTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        node.depth_shape_by_drone["quadrotor_01"] = (100, 100)
        node.depth_stamp_by_drone["quadrotor_01"] = TimeMsg()

        node._process_drone(0, "quadrotor_01")

        np.testing.assert_array_equal(node.ownership.owner_ids, [-1, -1])
    finally:
        node.destroy_node()


def test_process_drone_looks_up_transform_with_correct_frame_order(
    rclpy_context, tmp_path
):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        recording_tf_buffer = _RecordingTfBuffer()
        node.tf_buffer = recording_tf_buffer
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        node.depth_shape_by_drone["quadrotor_01"] = (100, 100)
        node.depth_stamp_by_drone["quadrotor_01"] = TimeMsg()

        node._process_drone(0, "quadrotor_01")

        assert [(call[0], call[1]) for call in recording_tf_buffer.calls] == [
            (node.map_frame, "quadrotor_01/camera_optical_frame")
        ]
    finally:
        node.destroy_node()


def test_process_drone_looks_up_transform_at_depth_image_capture_time(
    rclpy_context, tmp_path
):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        recording_tf_buffer = _RecordingTfBuffer()
        node.tf_buffer = recording_tf_buffer
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        node.depth_shape_by_drone["quadrotor_01"] = (100, 100)
        node.depth_stamp_by_drone["quadrotor_01"] = TimeMsg(
            sec=123, nanosec=456
        )

        node._process_drone(0, "quadrotor_01")

        used_time = recording_tf_buffer.calls[0][2]
        assert used_time == Time(seconds=123, nanoseconds=456)
    finally:
        node.destroy_node()


def test_process_drone_records_flashlight_state_with_hit_points(
    rclpy_context, tmp_path
):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        node.depth_shape_by_drone["quadrotor_01"] = (100, 100)
        node.depth_stamp_by_drone["quadrotor_01"] = TimeMsg()

        node._process_drone(0, "quadrotor_01")

        state = node.flashlight_state["quadrotor_01"]
        np.testing.assert_allclose(state["origin"], [0.0, 0.0, 0.0], atol=1e-9)
        assert state["corner_directions"].shape == (4, 3)
        assert state["hit_points"].shape[1] == 3
        assert state["hit_points"].shape[0] > 0
    finally:
        node.destroy_node()


def test_process_drone_clears_flashlight_state_when_tf_lookup_fails(
    rclpy_context, tmp_path
):
    class _FailingTfBuffer:
        def lookup_transform(self, target_frame, source_frame, time, timeout=None):
            raise TransformException("no transform available")

    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        node.depth_shape_by_drone["quadrotor_01"] = (100, 100)
        node.depth_stamp_by_drone["quadrotor_01"] = TimeMsg()
        node._process_drone(0, "quadrotor_01")
        assert "quadrotor_01" in node.flashlight_state

        node.tf_buffer = _FailingTfBuffer()
        node._process_drone(0, "quadrotor_01")

        assert "quadrotor_01" not in node.flashlight_state
    finally:
        node.destroy_node()
```

`test_refresh_coverage_first_seen_wins_across_drones_same_cycle`, `test_refresh_coverage_skips_marker_publish_when_nothing_newly_claimed`, `test_refresh_coverage_publishes_marker_when_something_newly_claimed`은 각각 `depth_by_drone[...] = depth_image.copy()` / `depth_by_drone["quadrotor_01"] = depth_image` 줄을 `depth_shape_by_drone[drone_id] = (100, 100)` / `depth_shape_by_drone["quadrotor_01"] = (100, 100)`로 바꾸고, `node.raycasting_scene = coverage_geometry.build_raycasting_scene(node.scene.triangle_positions)`를 `node.scene, node.ownership = _synthetic_scene_and_ownership()` 다음 줄에 추가한다 (나머지 로직은 동일).

`test_process_drone_claims_triangle_whose_centroid_is_occluded_but_a_vertex_is_visible`은 삭제한다 — 이 테스트는 "무게중심만 보는 기존 방식 vs 다중 샘플링"을 검증하는 것이었는데, 레이캐스팅 방식은 픽셀 격자 전체를 쏘는 것이지 삼각형 샘플점을 쏘는 게 아니므로 더 이상 의미가 없다.

- [ ] **Step 4: 전체 노드 테스트 통과 확인**

Run: `PYTHONPATH="/home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system:$PYTHONPATH" python3 -m pytest -p no:launch_testing -p no:launch_ros src/forest_rescue_system/test/test_coverage_visualization_node.py -v`
Expected: 전부 PASS

- [ ] **Step 5: 커밋**

```bash
git add src/forest_rescue_system/forest_rescue_system/coverage_visualization_node.py \
        src/forest_rescue_system/test/test_coverage_visualization_node.py
git commit -m "feat: _process_drone을 레이캐스팅 기반 가시성 판정으로 교체"
```

---

## Task 7: 손전등 파라미터/퍼블리셔 추가

**Files:**
- Modify: `src/forest_rescue_system/forest_rescue_system/coverage_visualization_node.py`
- Modify: `src/forest_rescue_system/config/forest_rescue.yaml`

**Interfaces:**
- Produces: `self.flashlight_marker_publisher`(MarkerArray, VOLATILE QoS), 파라미터 `flashlight_marker_topic`/`flashlight_color_rgb`. Task 8이 이 퍼블리셔를 사용한다.

- [ ] **Step 1: config.yaml에 파라미터 추가**

`config/forest_rescue.yaml`의 `ray_grid_step_px: 4` 다음 줄에 추가:

```yaml
    flashlight_marker_topic: /forest_rescue/flashlight_markers
    flashlight_color_rgb: [1.0, 0.95, 0.7]
```

- [ ] **Step 2: 노드에 파라미터 선언 + 퍼블리셔 생성**

`coverage_visualization_node.py`의 `self.declare_parameter("ray_grid_step_px", 4)` 다음 줄에 추가:

```python
        self.declare_parameter(
            "flashlight_marker_topic", "/forest_rescue/flashlight_markers"
        )
        self.declare_parameter("flashlight_color_rgb", [1.0, 0.95, 0.7])
```

`self.area_publisher = self.create_publisher(...)` 블록 바로 다음에 추가:

```python
        flashlight_qos = QoSProfile(depth=1)
        flashlight_qos.reliability = ReliabilityPolicy.RELIABLE
        flashlight_qos.durability = DurabilityPolicy.VOLATILE
        self.flashlight_marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("flashlight_marker_topic").value),
            flashlight_qos,
        )
```

`self._flashlight_published_drones = set()`를 `self.flashlight_state = {}` 다음 줄에 추가.

- [ ] **Step 3: 노드 생성이 여전히 정상 동작하는지 확인**

Run: `PYTHONPATH="/home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system:$PYTHONPATH" python3 -m pytest -p no:launch_testing -p no:launch_ros src/forest_rescue_system/test/test_coverage_visualization_node.py -k constructor -v`
Expected: PASS

- [ ] **Step 4: 커밋**

```bash
git add src/forest_rescue_system/forest_rescue_system/coverage_visualization_node.py \
        src/forest_rescue_system/config/forest_rescue.yaml
git commit -m "feat: 손전등 마커 파라미터와 VOLATILE QoS 퍼블리셔 추가"
```

---

## Task 8: 손전등 마커 빌드/발행 + `_refresh_coverage` 연결

**Files:**
- Modify: `src/forest_rescue_system/forest_rescue_system/coverage_visualization_node.py`
- Test: `src/forest_rescue_system/test/test_coverage_visualization_node.py`

**Interfaces:**
- Consumes: `self.flashlight_state`(Task 6), `self.flashlight_marker_publisher`(Task 7)
- Produces: `_build_flashlight_marker_array() -> MarkerArray`, `_publish_flashlight_markers()`. 외부에서 직접 소비하는 곳 없음(이 태스크가 마지막 소비자).

- [ ] **Step 1: 상수 + 헬퍼 함수 추가**

`_COLOR_PARAMETERS` 튜플 정의 바로 다음에 추가:

```python
_FLASHLIGHT_CONE_ALPHA = 0.15
_FLASHLIGHT_POINT_ALPHA = 0.6
_FLASHLIGHT_LINE_WIDTH_M = 0.02
_FLASHLIGHT_POINT_SIZE_M = 0.1


def _to_point(vector):
    point = Point()
    point.x = float(vector[0])
    point.y = float(vector[1])
    point.z = float(vector[2])
    return point
```

- [ ] **Step 2: `_build_flashlight_marker_array` + `_delete_marker` + `_publish_flashlight_markers` 추가**

`_publish_markers` 메서드 바로 다음에 추가:

```python
    def _delete_marker(self, ns, marker_id, stamp):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def _build_flashlight_marker_array(self):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        color = [
            float(value)
            for value in self.get_parameter("flashlight_color_rgb").value
        ]
        max_depth = float(self.get_parameter("maximum_depth_m").value)

        published_drones = set()
        for drone_index, drone_id in enumerate(self.drone_ids):
            ns = f"flashlight_drone_{drone_index + 1:02d}"
            state = self.flashlight_state.get(drone_id)
            if state is None:
                if drone_id in self._flashlight_published_drones:
                    marker_array.markers.append(
                        self._delete_marker(ns, 0, stamp)
                    )
                    marker_array.markers.append(
                        self._delete_marker(ns, 1, stamp)
                    )
                continue

            published_drones.add(drone_id)
            origin = state["origin"]
            far_points = origin + state["corner_directions"] * max_depth

            cone_marker = Marker()
            cone_marker.header.frame_id = self.map_frame
            cone_marker.header.stamp = stamp
            cone_marker.ns = ns
            cone_marker.id = 0
            cone_marker.type = Marker.LINE_LIST
            cone_marker.action = Marker.ADD
            cone_marker.pose.orientation.w = 1.0
            cone_marker.scale.x = _FLASHLIGHT_LINE_WIDTH_M
            cone_marker.color.r = color[0]
            cone_marker.color.g = color[1]
            cone_marker.color.b = color[2]
            cone_marker.color.a = _FLASHLIGHT_CONE_ALPHA
            cone_marker.frame_locked = True
            cone_marker.points = []
            for far_point in far_points:
                cone_marker.points.append(_to_point(origin))
                cone_marker.points.append(_to_point(far_point))
            for i in range(4):
                cone_marker.points.append(_to_point(far_points[i]))
                cone_marker.points.append(
                    _to_point(far_points[(i + 1) % 4])
                )
            marker_array.markers.append(cone_marker)

            hit_marker = Marker()
            hit_marker.header.frame_id = self.map_frame
            hit_marker.header.stamp = stamp
            hit_marker.ns = ns
            hit_marker.id = 1
            hit_marker.type = Marker.POINTS
            hit_marker.action = Marker.ADD
            hit_marker.pose.orientation.w = 1.0
            hit_marker.scale.x = _FLASHLIGHT_POINT_SIZE_M
            hit_marker.scale.y = _FLASHLIGHT_POINT_SIZE_M
            hit_marker.color.r = color[0]
            hit_marker.color.g = color[1]
            hit_marker.color.b = color[2]
            hit_marker.color.a = _FLASHLIGHT_POINT_ALPHA
            hit_marker.frame_locked = True
            hit_marker.points = [
                _to_point(point) for point in state["hit_points"]
            ]
            marker_array.markers.append(hit_marker)

        self._flashlight_published_drones = published_drones
        return marker_array

    def _publish_flashlight_markers(self):
        self.flashlight_marker_publisher.publish(
            self._build_flashlight_marker_array()
        )
```

- [ ] **Step 3: `_refresh_coverage`에서 매 사이클 손전등 발행**

기존:

```python
    def _refresh_coverage(self):
        self._load_mesh_if_ready()
        if self.scene is None or self.ownership is None:
            return

        any_newly_claimed = False
        for drone_index, drone_id in enumerate(self.drone_ids):
            any_newly_claimed |= self._process_drone(drone_index, drone_id)

        if any_newly_claimed:
            self._publish_markers()
```

교체:

```python
    def _refresh_coverage(self):
        self._load_mesh_if_ready()
        if self.scene is None or self.ownership is None:
            return

        any_newly_claimed = False
        for drone_index, drone_id in enumerate(self.drone_ids):
            any_newly_claimed |= self._process_drone(drone_index, drone_id)

        if any_newly_claimed:
            self._publish_markers()
        self._publish_flashlight_markers()
```

- [ ] **Step 4: 실패하는 테스트 작성**

`test_coverage_visualization_node.py` 맨 끝에 추가:

```python
def test_build_flashlight_marker_array_uses_drone_namespace_and_two_markers(
    rclpy_context, tmp_path
):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        node.depth_shape_by_drone["quadrotor_01"] = (100, 100)
        node.depth_stamp_by_drone["quadrotor_01"] = TimeMsg()
        node._process_drone(0, "quadrotor_01")

        marker_array = node._build_flashlight_marker_array()

        drone_01_markers = [
            marker
            for marker in marker_array.markers
            if marker.ns == "flashlight_drone_01"
        ]
        assert len(drone_01_markers) == 2
        assert {marker.id for marker in drone_01_markers} == {0, 1}
        cone = next(m for m in drone_01_markers if m.id == 0)
        assert cone.type == Marker.LINE_LIST
        assert len(cone.points) == 16  # 모서리 4선 + 먼 사각형 4선, 선당 2점
        hits = next(m for m in drone_01_markers if m.id == 1)
        assert hits.type == Marker.POINTS
        assert len(hits.points) > 0
    finally:
        node.destroy_node()


def test_build_flashlight_marker_array_deletes_marker_when_drone_disappears(
    rclpy_context, tmp_path
):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        node.depth_shape_by_drone["quadrotor_01"] = (100, 100)
        node.depth_stamp_by_drone["quadrotor_01"] = TimeMsg()
        node._process_drone(0, "quadrotor_01")
        node._build_flashlight_marker_array()  # 첫 발행으로 published_drones 기록

        node.flashlight_state.pop("quadrotor_01")
        marker_array = node._build_flashlight_marker_array()

        drone_01_markers = [
            marker
            for marker in marker_array.markers
            if marker.ns == "flashlight_drone_01"
        ]
        assert len(drone_01_markers) == 2
        assert all(m.action == Marker.DELETE for m in drone_01_markers)
    finally:
        node.destroy_node()


def test_refresh_coverage_publishes_flashlight_markers_every_cycle(
    rclpy_context, tmp_path
):
    node = _make_node(rclpy_context, tmp_path)
    try:
        node.scene, node.ownership = _synthetic_scene_and_ownership()
        all_indices = np.arange(len(node.scene.centroids))
        node.ownership.claim(all_indices, drone_index=0)  # 커버리지는 새로 안 늘어남
        node.raycasting_scene = coverage_geometry.build_raycasting_scene(
            node.scene.triangle_positions
        )
        node.tf_buffer = _StubTfBuffer()
        node.camera_info_by_drone["quadrotor_01"] = _camera_info()
        node.depth_shape_by_drone["quadrotor_01"] = (100, 100)
        node.depth_stamp_by_drone["quadrotor_01"] = TimeMsg()

        coverage_calls = []
        flashlight_calls = []
        node.marker_publisher.publish = lambda msg: coverage_calls.append(msg)
        node.flashlight_marker_publisher.publish = (
            lambda msg: flashlight_calls.append(msg)
        )

        node._refresh_coverage()

        assert len(coverage_calls) == 0  # 새로 클레임된 게 없으므로 스킵
        assert len(flashlight_calls) == 1  # 손전등은 항상 발행
    finally:
        node.destroy_node()
```

- [ ] **Step 5: 테스트 통과 확인**

Run: `PYTHONPATH="/home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system:$PYTHONPATH" python3 -m pytest -p no:launch_testing -p no:launch_ros src/forest_rescue_system/test/test_coverage_visualization_node.py -v`
Expected: 전부 PASS

- [ ] **Step 6: 커밋**

```bash
git add src/forest_rescue_system/forest_rescue_system/coverage_visualization_node.py \
        src/forest_rescue_system/test/test_coverage_visualization_node.py
git commit -m "feat: 손전등 원뿔/히트포인트 마커 빌드 및 매 사이클 발행"
```

---

## Task 9: 전체 검증 + RViz 통합 확인 안내

**Files:**
- 없음 (검증만)

**Interfaces:**
- Consumes: Task 1~8의 모든 변경
- Produces: 없음

- [ ] **Step 1: 전체 pytest 실행**

Run: `PYTHONPATH="/home/hwangjeongui/b3_cobot3_ws/src/forest_rescue_system:$PYTHONPATH" python3 -m pytest -p no:launch_testing -p no:launch_ros src/forest_rescue_system/test/ -v`
Expected: 전부 PASS, 실패/스킵 없음

- [ ] **Step 2: lint 확인**

Run: `cd src/forest_rescue_system && python3 -m flake8 forest_rescue_system/coverage_geometry.py forest_rescue_system/coverage_visualization_node.py test/test_coverage_geometry.py test/test_coverage_visualization_node.py`
Expected: 출력 없음 (에러 없음)

- [ ] **Step 3: `visibility_mask`/`grazing_angle_tolerance`/`triangle_sample_points`가 저장소 어디에도 안 남아있는지 최종 확인**

Run: `grep -rn "visibility_mask\|grazing_angle_tolerance\|triangle_sample_points" src/forest_rescue_system --include=*.py`
Expected: 출력 없음

`cv_bridge`는 `victim_localizer_node.py`/`human_detector_node.py`가 여전히 사용하므로 패키지 전체에서 검사하면 안 된다 (package.xml의 `cv_bridge` exec_depend는 그대로 유지). `coverage_visualization_node.py`에서만 제거됐는지 확인:

Run: `grep -n "cv_bridge" src/forest_rescue_system/forest_rescue_system/coverage_visualization_node.py`
Expected: 출력 없음

- [ ] **Step 4: (수동) RViz 통합 확인**

시뮬레이터 + `use_rviz:=true`로 launch 후:
- 이전에 그레이징 각도에서 안 칠해지던 내리막 지형 삼각형이 정상적으로 보라색이 되는지 육안 확인.
- `/forest_rescue/flashlight_markers` MarkerArray Display를 RViz에 추가(Topic Value를 정확히 지정, QoS는 Reliable/Volatile로 발행 QoS와 일치)하고, 드론이 이동할 때 원뿔+히트포인트가 카메라를 따라 자연스럽게 움직이는지, TF가 끊기거나 드론이 사라졌을 때 이전 빔이 화면에 남지 않는지 확인.

이 단계는 자동화된 테스트가 아니므로 결과를 코드로 검증할 수 없음 — 수동 확인 후 다음 단계(finishing-a-development-branch)로 넘어간다.

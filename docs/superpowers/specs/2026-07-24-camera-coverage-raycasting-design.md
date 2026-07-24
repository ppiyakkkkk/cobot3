# 드론 카메라 커버리지: 레이캐스팅 기반 재설계 + 손전등 시각화

이 문서는 `2026-07-23-camera-coverage-visualization-design.md`(이하 "이전 설계")의 "가시성 판정 알고리즘" 섹션을 대체한다. 노드 구성, mesh 로딩, 면적 발행, launch/config 통합 등 이전 설계의 나머지 내용은 그대로 유효하다.

## 배경 / 문제

이전 설계는 삼각형 샘플점을 카메라에 투영해 depth 이미지의 (u,v) 픽셀값과 비교하는 방식(섀도우맵 리프로젝션)이었다. 그레이징(스침) 각도에서 tolerance를 키우는 보정(`grazing_angle_tolerance`, 3x3 이웃 탐색)을 추가했음에도, 내리막 지형의 삼각형이 그레이징 각도에서 안정적으로 칠해지지 않는 문제가 남았다. 픽셀 그리드 근사 비교라는 방식 자체의 구조적 한계로 판단해, 뎁스 리프로젝션 비교를 정확한 ray-mesh 교차판정(섀도우 레이)으로 교체한다.

## 핵심 변경 요약

1. **가시성 판정**: depth 이미지 값 비교 → 카메라 픽셀마다 광선을 쏴 메쉬와 교차판정(첫 히트 삼각형 = 보이는 삼각형). depth 이미지는 더 이상 가시성 판정에 쓰이지 않는다.
2. **신규 기능**: RViz에서 각 드론 카메라가 "손전등"처럼 현재 보고 있는 영역에 빛을 쏘는 것처럼 보이는 실시간(비누적) 시각화 추가.

## 가시성 판정 알고리즘 (레이캐스팅)

### 정적 준비 (mesh 로드 시 1회)

`assemble_scene()` 직후, 전체 씬(지형+환경 전 그룹)의 `triangle_positions`를 평탄화한 정점 배열 + `arange` 삼각형 인덱스로 `open3d.t.geometry.RaycastingScene`을 1회 빌드해 `self.raycasting_scene`에 보관한다. 정점을 공유시키지 않고 그대로 넣으므로, 반환되는 `primitive_id`가 곧 `scene.centroids`/`normals`/`areas`와 동일한 전역 삼각형 인덱스가 된다 (별도 매핑 불필요).

### 매 프레임 (드론별, `refresh_period_sec` 주기)

1. `depth_stamp`가 있는지만 확인한다 (카메라가 실제로 스트리밍 중이라는 생존 신호. 픽셀 값은 사용하지 않는다).
2. TF 조회를 `lookup_transform(map_frame, camera_frame, ...)`로 수행한다 (이전 설계의 `camera_frame, map_frame` 순서와 반대). 이 변환의 translation이 맵 좌표계에서의 카메라 원점, rotation이 카메라축 → 맵 프레임 회전이다.
3. `camera_info` 해상도를 `ray_grid_step_px`(기본 4px) 간격으로 서브샘플링한 픽셀 격자를 만들고, 각 픽셀을 핀홀 카메라 모델로 카메라 프레임 광선 방향으로 변환한 뒤 맵 프레임으로 회전시킨다. `self.raycasting_scene`이 아직 없으면(mesh 미로드) 이전 설계와 동일하게 해당 사이클을 스킵한다.
4. 공통 원점 + 방향 배열을 `RaycastingScene.cast_rays()`에 배치로 투입해 `t_hit`, `primitive_id`를 얻는다.
5. `minimum_depth_m ≤ t_hit ≤ max_depth_m` 범위인 것만 남기고, 유효한 `primitive_id`의 고유값을 이번 프레임에 "보인" 삼각형으로 집계한다.
6. `TriangleOwnership.claim()`에 그대로 전달한다 (이미 점유된 인덱스는 자동 무시되므로 별도 필터링 불필요).

이 방식은 소유권과 무관하게 전체 메쉬를 대상으로 교차판정하므로, 다른 드론이 이미 점유한 나무/바위도 여전히 정확한 오클루더로 작용한다 (이전 설계에서는 depth 이미지가 이 역할을 암묵적으로 담당했다).

### `coverage_geometry.py` 변경

**제거**: `visibility_mask`, `visibility_mask_multi_sample`, `grazing_angle_tolerance`, `triangle_sample_points`, 관련 상수(`DEFAULT_NEIGHBORHOOD_PX`, `DEFAULT_MIN_GRAZING_COSINE`, `DEFAULT_MAX_TOLERANCE_SCALE`) — 전부 뎁스 리프로젝션 비교 전용이라 필요 없어진다.

**유지**: `SceneMesh`, `assemble_scene`, `triangle_vertex_positions`/`triangle_centroids`/`triangle_areas`/`triangle_normals`, `scaled_intrinsics`, `transform_matrix_from_tf`, `apply_transform`, `transform_direction`.

**추가** (open3d 의존):
- `pixel_to_camera_ray(u, v, fx, fy, cx, cy)` — 픽셀 좌표(배열 가능) → 카메라 프레임 단위 방향벡터. 격자 생성과 손전등 프러스텀 모서리 계산에 공용으로 사용.
- `pixel_grid_uv(width, height, step_px)` — 서브샘플링된 픽셀 중심 좌표 (u, v) 배열 생성.
- `build_raycasting_scene(triangle_positions)` — 위에서 설명한 씬 빌드 함수.
- `cast_visibility_rays(scene, ray_origin, ray_directions, min_depth_m, max_depth_m)` — 배치 광선을 쏘고 범위 안에서 실제로 맞은 것만 걸러 `(hit_points_map, triangle_indices)`를 반환. 이 결과를 커버리지 클레임과 손전등 히트 포인트 마커 양쪽에서 그대로 재사용한다.

### `coverage_visualization_node.py` 변경

- `_load_mesh_if_ready()`: `assemble_scene()` 직후 `self.raycasting_scene = coverage_geometry.build_raycasting_scene(...)` 빌드.
- `_depth_callback`: cv_bridge 디코딩 제거. `message.width`, `message.height`, `message.header.stamp`만 기록한다 (픽셀 배열을 저장/변환하지 않으므로 이 노드에서 `cv_bridge` 의존성 자체가 사라진다).
- `_process_drone`: 위 "매 프레임" 알고리즘을 구현. 카메라 4개 모서리 방향, 즉 `(u,v) ∈ {(0,0), (width,0), (0,height), (width,height)}` 4쌍을 `pixel_to_camera_ray`로 변환해 `self.flashlight_state[drone_id]`에 원점 + 모서리 방향 4개 + 히트 포인트를 저장 (손전등 시각화에서 재사용). `pixel_to_camera_ray`는 정수 픽셀 중심이든 이미지 테두리 좌표든 동일한 핀홀 공식으로 처리하므로, 커버리지 격자(픽셀 중심)와 모서리(0/width, 0/height)를 같은 함수로 계산해도 좌표계 불일치는 없다.
- **손전등 상태 소멸 처리**: `depth_stamp`가 없거나 TF 조회가 실패해 해당 사이클에 갱신할 수 없는 드론은 `self.flashlight_state.pop(drone_id, None)`으로 즉시 제거한다 (이전 프레임의 빔 위치가 화면에 고정되는 "고스트 빔" 방지).
- `_refresh_coverage`: 커버리지 마커는 기존과 동일하게 `any_newly_claimed`일 때만 발행. 손전등 마커는 **매 주기 항상** 발행한다 (드론이 이동하면 커버리지가 늘지 않아도 빔은 이동해야 하므로).

## 손전등 시각화 (신규)

각 드론 카메라가 현재 보고 있는 영역에 빛을 쏘는 것처럼, 원뿔(프러스텀) 윤곽선 + 히트 포인트를 실시간(비누적)으로 표시한다.

- **원뿔 윤곽선**: 카메라 원점 → 4개 모서리 방향으로 `maximum_depth_m` 길이만큼 뻗은 4개 선분 + 그 끝점들을 잇는 먼 쪽 사각형 테두리 4개 선분, 총 8선의 LINE_LIST 마커. 실제 지형 접촉 여부와 무관하게 항상 `maximum_depth_m`로 고정된 길이를 사용한다 (일부 모서리가 허공을 향해 아무것도 맞히지 않아도 원뿔 형태가 깨지지 않도록).
- **히트 포인트**: `cast_visibility_rays`가 이미 계산한 `hit_points_map`을 그대로 POINTS 마커로 표시 (추가 연산 없음).
- **갱신 주기**: 커버리지와 같은 `refresh_period_sec` 주기, 같은 프레임의 raycast 결과를 재사용 (별도 타이머/중복 연산 없음).
- **토픽/QoS**: 신규 `flashlight_marker_topic`(기본 `/forest_rescue/flashlight_markers`)에 VOLATILE QoS로 발행. 누적 커버리지 마커(`coverage_marker_topic`, TRANSIENT_LOCAL)와는 분리 — 손전등은 매 프레임 덮어쓰는 순간적 상태라 TRANSIENT_LOCAL 특성이 맞지 않고, 늦게 구독하는 새 RViz가 오래된(이미 지난) 손전등 위치를 받는 것도 방지한다.
- **색상**: 드론별 커버리지 색상과 무관하게 신규 파라미터 `flashlight_color_rgb`(기본 `[1.0, 0.95, 0.7]`, 따뜻한 흰색/노란색)를 모든 드론이 공유. 원뿔은 낮은 알파(0.15), 히트 포인트는 높은 알파(0.6)로 코드 상수 고정 (기존 마커들도 알파를 파라미터화하지 않은 패턴과 동일하게 유지).
- **마커 ns/id 및 scale**: 드론별로 `ns=f"flashlight_drone_{drone_index+1:02d}"` 네임스페이스 아래 원뿔 마커 `id=0`(LINE_LIST, `scale.x=0.02`m 선 굵기), 히트 포인트 마커 `id=1`(POINTS, `scale.x=scale.y=0.1`m 점 크기)로 고정한다.
- **드론별 상태 없음 처리**: `flashlight_state`가 없는(최초 미생성 또는 위에서 pop된) 드론은 해당 프레임 마커 배열에 새로 추가하지 않되, 직전 프레임까지 발행 이력이 있었다면 같은 `ns`/`id` 조합으로 `action=Marker.DELETE` 마커를 발행해 RViz에 남아있는 이전 빔을 명시적으로 지운다 (예: 노드가 `self._flashlight_published_drones: set`로 직전에 실제 발행한 드론 집합을 추적).

## 파라미터 변경

- 제거: `visibility_tolerance_m` (tolerance 개념 자체가 없어짐)
- 추가: `ray_grid_step_px` (기본 4) — 커버리지 판정용 광선 격자 간격(px)
- 추가: `flashlight_marker_topic` (기본 `/forest_rescue/flashlight_markers`)
- 추가: `flashlight_color_rgb` (기본 `[1.0, 0.95, 0.7]`)
- 유지: `minimum_depth_m`, `maximum_depth_m` — 이제 광선 범위 클리핑 + 손전등 원뿔 길이로 재사용

**`config/forest_rescue.yaml`의 `coverage_visualization_node` 섹션 diff** (이전 설계 문서의 "나머지 내용은 유효하다"는 이 파라미터 변경에는 적용되지 않음 — 아래가 우선):
- 제거: `visibility_tolerance_m: 0.5`
- 추가: `ray_grid_step_px: 4`, `flashlight_marker_topic: /forest_rescue/flashlight_markers`, `flashlight_color_rgb: [1.0, 0.95, 0.7]`

## 의존성

- 루트 `requirements.txt`에 `open3d` 추가. (trimesh는 검토했으나 open3d의 `RaycastingScene`으로 충분해 실제로는 쓰지 않으므로 추가하지 않음.)
- `open3d`는 rosdep 표준 키가 없는 pip 패키지이므로 `package.xml`은 건드리지 않고 `requirements.txt` 설치로 충당한다 (기존 `mavsdk`/`ultralytics`와 동일한 패턴). 다른 팀원 환경에 `trimesh`/`open3d`가 없을 수 있으므로, PR에는 `pip install -r requirements.txt` 재실행 안내를 남긴다.
- 이 노드에서 `cv_bridge`/`CvBridgeError` import 및 사용 제거.

## 테스트 계획

- `test_coverage_geometry.py`: `visibility_mask*`/`grazing_angle_tolerance`/`triangle_sample_points` 관련 테스트 전부 제거.
  - `pixel_to_camera_ray`, `pixel_grid_uv` 단위 테스트 추가.
  - `build_raycasting_scene` + `cast_visibility_rays`를 합성 메쉬(내리막 경사를 이루는 인접 삼각형 2개 + 그 앞을 가리는 별도 삼각형 1개)로 검증:
    - 그레이징 각도의 내리막 삼각형이 정상적으로 히트되는지 (원래 버그의 회귀 테스트).
    - 앞을 가리는 삼각형이 있으면 뒤 삼각형은 히트되지 않는지 (오클루전 정확성).
    - `min/max_depth_m` 범위 밖 히트가 걸러지는지.
- `test_coverage_visualization_node.py`: TF 조회 방향 변경, depth 콜백이 더는 배열을 저장하지 않는 점, 손전등 마커 발행 로직(매 주기 발행, 상태 없는 드론 제외)을 반영해 갱신.
- 통합 확인: `use_rviz:=true`로 launch 후, 이전에 안 칠해지던 내리막 그레이징 삼각형이 정상적으로 보라색이 되는지 육안 확인. 손전등 마커가 드론 카메라를 따라 자연스럽게 이동하는지, 원뿔이 지형에 닿는 지점에 히트 포인트가 나타나는지, 드론이 사라지거나 TF가 끊겼을 때 이전 빔이 화면에 남지 않는지 확인.

## 구현 순서 제안

레이캐스팅 교체(가시성 판정 알고리즘 + 회귀 테스트)와 손전등 시각화(신규 기능)는 서로 독립적이므로, 구현 계획은 이 둘을 별도 단계로 나눠 각각 검증하는 것을 권장한다 (레이캐스팅 교체가 실제로 그레이징 버그를 고쳤는지 먼저 확인한 뒤 손전등을 얹는 순서).

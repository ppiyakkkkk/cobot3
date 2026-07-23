# 드론 카메라 커버리지 시각화 설계

## 배경 / 목적

현재 RViz에는 지형(terrain), 식생/바위 그룹(pineforest, broadleafforest, bushes, rocks), 드론 3대의 탐색 경로(Path)가 표시된다. 여기에 추가로, 각 드론의 카메라가 실제로 촬영해서 확인한 지면·나무·수풀 영역을 보라색으로 표시해서 "지금까지 얼마나 수색했는지"를 한눈에 파악할 수 있게 한다.

## 핵심 요구사항

- **누적 커버리지**: 한 번이라도 카메라에 잡혀서 "보였다"고 판정된 영역은 계속 보라색으로 남는다 (fog-of-war 방식). 드론이 다른 곳으로 이동해도 지워지지 않는다.
- **드론별 색상 구분**: drone_01/02/03이 각각 다른 보라 계열 톤으로 표시된다. 두 드론 이상이 같은 영역을 본 경우, 먼저 본 드론의 색이 유지된다 (나중에 본 드론이 덮어쓰지 않음).
- **대상 범위**: terrain(지면) + pineforest + broadleafforest + bushes + rocks 전부. 향후 지형 데이터에 그룹이 추가되더라도 코드 수정 없이 자동으로 포함되어야 한다.
- **가시성 판정**: 단순 카메라 FOV 기하학적 범위가 아니라, depth 이미지 기반의 실제 가시성으로 판정한다. 나무 등에 가려진 뒤쪽 지면/삼각형은 보라색으로 칠해지지 않는다.
- **삼각형 판정 기준**: 삼각형의 무게중심(centroid) 1개 지점만 검사한다. 대표점 1개 = 판정 결과 1개로 기준이 명확하고, mesh 밀도가 충분히 조밀해서 오차가 미미하다.
- **커버리지 면적**: 보라색으로 칠해진 영역의 총 면적(m²)을 1초 주기로 토픽 발행한다. 드론별 세분화 없이 전체 합계 1개 값만.

## 아키텍처

### 노드 구성: 단일 노드가 드론 3대를 모두 관리

`coverage_visualization_node` 하나가 drone_01/02/03의 카메라 데이터를 전부 구독하고 처리한다.

기존 코드베이스에는 두 가지 노드 패턴이 있다: `sensor_tf_01/02/03`처럼 드론별로 독립된 노드 3개를 띄우는 패턴과, `mission_manager_node`처럼 싱글턴으로 드론 3대를 모두 구독/관리하는 패턴. 이 기능은 후자를 따른다.

이유: "먼저 본 드론이 소유권을 유지한다"는 규칙을 지키려면 삼각형별 소유자 정보를 여러 드론이 공유해야 한다. 드론별로 노드를 분리하면 이 소유권 정보를 프로세스 간에 주고받는 추가 통신 수단(토픽, 락 등)이 필요해져 불필요하게 복잡해진다. 노드 하나 안에서는 공유 배열로 간단히 처리된다.

트레이드오프: 단일 노드 내에서 드론별로 순차 처리하므로, 한 드론의 처리(투영/depth lookup)가 느려지면 다른 드론의 처리와 마커 재발행이 함께 지연된다. 또한 같은 refresh 사이클에서 처음 노출되는 영역이 여러 드론에 동시에 잡히는 경우, 실제로는 처리 순서(드론 루프 순서, 기본 01→02→03)가 소유권을 결정한다 — 이는 "소유권 우선순위" 섹션에서 별도로 명시한다.

### 입력

드론별(01/02/03), 기존 config 파라미터 이름과 동일한 관례를 따름:
- `CameraInfo`: `/quadrotor_0N/Camera/camera_info`
- Depth Image: `/quadrotor_0N/Camera/depth` (`passthrough` 인코딩, 픽셀 값 단위는 미터 — `victim_localizer_node`와 동일하게 처리)
- QoS: `qos_profile_sensor_data` (BEST_EFFORT) — 기존 depth/camera_info/rgb 구독 노드들과 동일
- TF: `tf2_ros.Buffer`/`TransformListener`로 `map → quadrotor_0N/camera_optical_frame` 조회. 이 체인은 `drone_controller_node`(동적, map→base_link)와 `sensor_tf_node`(정적, base_link→camera_optical_frame)가 이미 발행하고 있음을 확인했다. 조회 실패 시 해당 드론은 그 사이클만 스킵.

지형 데이터 (파일 경로 파라미터는 `rviz_visualization_node`와 동일한 기본값 재사용):
- `terrain_mesh_path` → `generated_terrain_mesh.npz`, 키 `vertices`/`triangles` (그룹 접두어 없음, 그룹명은 `"terrain"`으로 취급)
- `environment_mesh_path` → `generated_environment_meshes.npz`, 그 안의 `{이름}_vertices` + `{이름}_triangles` 키 쌍을 전부 스캔해서 그룹을 동적으로 인식한다. `{이름}_source_paths`, `{이름}_original_triangle_count` 같은 다른 접미어 키와 겹치지 않음을 확인했으므로 안전하다. 이렇게 하면 pineforest/broadleafforest/bushes/rocks 외에 향후 그룹이 추가돼도 코드 수정 없이 자동 포함된다.
- 두 npz 파일 모두 `map_frame="map"`, `coordinate_convention="world_enu"`이고, mesh는 USD stage의 world transform을 회전/스케일 변환 없이 그대로 저장하므로 축 방향(East-x/North-y/Up-z)과 단위(미터)는 드론 TF와 일치함을 확인했다. 다만 원점(map=world) 일치는 stage에서 직접 쿼리해 검증된 것이 아니라, 드론 TF의 `map` 원점이 `sim_config.py`의 `DRONE_CONFIGS`에 하드코딩된 `home_world_enu` 상수에 의존한다 — **즉 축/단위는 코드로 확인됐지만, 원점 일치는 이 상수가 실제 드론 스폰 위치와 맞는다는 수동 동기화 가정이다. 드론 스폰 위치가 바뀌면 이 값도 함께 갱신해야 하며, 어긋나도 자동으로 감지되지 않는다.**
- 이 파일들은 시뮬레이션 시작 전 1회성으로 export되고 실행 중 재생성되지 않는다는 것을 확인했다. 따라서 한 번 로드한 뒤 vertices/triangles 배열과 인덱스를 그대로 캐싱해서 재사용해도 안전하다 (인덱스 무효화 걱정 없음). `rviz_visualization_node`처럼 mtime/size 시그니처로 파일 등장을 기다리는 로직은 유지하되, 최초 로드 이후에는 재로드하지 않는다.

### 가시성 판정 알고리즘

`refresh_period_sec` 파라미터(기본 1.0초) 주기로 드론별 실행:

1. 아직 소유자가 없는 삼각형들의 centroid만 검사 대상으로 삼는다 (이미 다른 드론이 점유한 삼각형은 재검사하지 않음 → 시간이 지날수록 검사량이 줄어듦).
2. centroid를 `map → camera_optical_frame` 변환으로 카메라 좌표계로 옮기고, 카메라 앞쪽(Z>0)인 것만 통과시킨다.
3. `camera_info.k`로 픽셀 좌표(u,v)를 계산한다: `fx=k[0]*scale_x, fy=k[4]*scale_y, cx=k[2]*scale_x, cy=k[5]*scale_y`. `scale_x/y`는 `camera_info`의 해상도와 실제 depth 이미지 해상도가 다를 경우를 위한 보정(`victim_localizer_node`와 동일 패턴)이며, 이미지 범위 안에 들어오는 픽셀만 통과시킨다.
4. depth 이미지의 (u,v) 값을 읽어서 실제 카메라-프레임 거리 Z와 비교한다. `abs(depth_image[v,u] - Z) < visibility_tolerance_m`(기본 0.5m) 이내면 "보임"으로 판정한다. 가려진 뒷면은 depth 값이 훨씬 작게 나와서 자동으로 tolerance를 벗어나 제외된다.
5. `minimum_depth_m`(0.2) ~ `maximum_depth_m`(30.0) 범위도 적용한다 (기존 victim_localizer 파라미터 값과 동일한 기본값 사용).
6. 통과한 삼각형은 그 드론의 소유로 영구 마킹한다. "먼저 본 드론 색 유지" 규칙은 1번의 "소유자 없는 것만 검사"로 자연히 충족된다.

**알려진 한계 (의도적으로 감수):**
- 영구 마킹이므로 재검사가 없다. depth 노이즈나 tolerance(0.5m) 오차로 한 번 오탐(false positive)되면 그 삼각형은 영구히 고정되고 되돌릴 수 없다. `victim_localizer_node`와 달리 다중 샘플 통계 보정이 없는 단일 centroid 판정이라 이 리스크가 상대적으로 크지만, 시각화 목적(정확한 측량이 아님)에는 허용 가능한 것으로 판단한다.
- "먼저 본 드론"은 시간적으로 가장 먼저 관찰했다는 의미가 아니라, 동일 refresh 사이클 내에서 드론 처리 순서(기본 01→02→03)상 먼저 처리된 드론이라는 의미다. 같은 사이클에 여러 드론이 동시에 새 영역을 노출시키면 항상 낮은 번호 드론이 소유권을 갖는다.

### 출력

**1. `/forest_rescue/coverage_markers` (MarkerArray)**
- QoS: Depth 1, Durability=Transient Local, Reliability=Reliable, History=Keep Last (기존 scene_markers/environment_meshes와 동일)
- 드론별 namespace(`coverage_drone_01`, `coverage_drone_02`, `coverage_drone_03`)로 TRIANGLE_LIST 마커 1개씩(총 3개), 매 사이클 그 드론이 현재 소유한 전체 삼각형으로 재발행한다.
- 색상: 보라 계열 3톤, config 파라미터화 (`drone_01_color_rgb`, `drone_02_color_rgb`, `drone_03_color_rgb` 등, 기본값 예시: 진보라 `[0.55,0.0,0.85]`, 바이올렛 `[0.73,0.33,0.83]`, 자주 `[0.60,0.0,0.50]`).
- z-fighting 방지를 위해 원본 지형 위로 `coverage_z_offset_m`(기본 0.05m)만큼 살짝 띄워서 덮어 그린다. 완벽한 법선 방향 오프셋은 아니고 단순 Z축 상수 오프셋이지만, 시각화 목적에는 충분하다.

**2. `/forest_rescue/coverage_area_m2` (std_msgs/Float32)**
- 1초 주기 타이머로 소유된 모든 삼각형의 면적 합계(m², 벡터 외적으로 계산)를 발행한다. 드론별 세분화 없이 전체 1개 값.
- QoS: 기본(volatile, depth 10).

### 통합 변경 사항

- `config/forest_rescue.yaml`: `coverage_visualization_node:` 섹션 추가 (mesh 경로, tolerance, depth 범위, 색상, refresh_period, z_offset 등 파라미터).
- `launch/forest_rescue_system.launch.py`: `nodes` 리스트에 `coverage_visualization` 노드 1개 추가 (드론 루프 밖, `mission_manager`/`rviz_visualization`과 같은 자리 — `use_rviz` 조건과 무관하게 항상 실행).
- `setup.py`: console_scripts에 `coverage_visualization = forest_rescue_system.coverage_visualization_node:main` 추가.
- `config/forest_rescue_multi.rviz`: 기존 `Scene Markers`/`Environment Groups` Display 블록과 동일한 구조로 MarkerArray Display 1개 추가 (`Name: Coverage Markers`, Topic Value `/forest_rescue/coverage_markers`, QoS는 발행 QoS와 반드시 일치시켜야 함 — 불일치 시 DDS 매칭 실패로 아무것도 안 보임을 확인함).

### 이름 충돌 확인

`/forest_rescue/coverage_markers`, `/forest_rescue/coverage_area_m2`, `coverage_visualization_node`, `coverage_visualization` 모두 기존 코드베이스에 존재하지 않아 충돌 없이 사용 가능함을 확인했다. 기존 `/forest_rescue/` 네임스페이스에는 `scene_markers`, `terrain_mesh`, `environment_meshes` 3개만 있다.

### 성능

환경 mesh 약 7.5만 삼각형 + 지형 약 8천 삼각형, 총 약 8만 삼각형 × 3드론이지만, 이미 점유된 삼각형은 재검사하지 않으므로 시간이 지날수록 연산량이 줄어든다. numpy 벡터화 연산으로 1Hz 주기에서 충분히 여유 있게 처리 가능하다.

**구현 필수 조건**: 이 성능 결론은 좌표변환·픽셀투영·depth lookup을 삼각형별로 배치 처리(numpy 배열 연산)하는 것을 전제로 한다. `victim_localizer_node`처럼 점 하나씩 `tf2_geometry_msgs`로 변환하는 반복문 방식으로 구현하면 최초 사이클(최대 8만×3회)에서 1Hz를 만족하지 못할 수 있다 — TF는 4x4 변환 행렬로 한 번만 조회한 뒤 전체 centroid 배열에 일괄 행렬곱을 적용하고, depth (u,v) lookup도 fancy indexing으로 벡터화해야 한다.

### 예외 처리

- camera_info/depth/TF 중 하나라도 아직 수신되지 않았으면 해당 드론은 그 사이클만 스킵한다 (에러 로그 아님, info 로그로 대기 상태만 남김 — 기존 `rviz_visualization_node`의 "파일 대기 중" 로그 패턴과 동일).
- 지형/환경 mesh 파일이 아직 없으면 전체 처리를 스킵한다 (기존 `rviz_visualization_node`와 동일 패턴).

## 테스트 계획

- 단위 테스트: 픽셀 투영(fx/fy/cx/cy + scale 보정) 계산, depth 비교를 통한 가시성 판정(보임/가려짐/범위 밖), 삼각형 면적 계산 함수를 순수 함수로 분리해서 검증.
- 단위 테스트: 소유권 우선순위 — 일부 삼각형에 이미 owner가 있는 상태를 입력으로 주고, 이후 다른 드론이 같은 삼각형을 다시 "보임"으로 판정해도 기존 owner가 덮어써지지 않는지 확인.
- 단위 테스트: 동적 그룹 스캔 — mock npz에 기존 4개 그룹 외에 임의의 새 `{name}_vertices`/`{name}_triangles` 키 쌍을 추가하고, 코드 수정 없이 해당 그룹이 자동으로 인식/처리되는지 확인.
- 단위 테스트: 예외 경로 — TF lookup 예외, mesh 파일 미존재 상황을 mock해서 크래시 없이 해당 사이클(또는 전체 처리)이 스킵되는지 확인.
- 단위 테스트: npz 캐싱 — 로드 함수를 mock하고 refresh를 여러 사이클 반복 호출해도, 최초 1회 이후로는 로드 함수가 다시 호출되지 않는지 확인.
- 통합 확인: `use_rviz:=true`로 launch 후, 시뮬레이터에서 드론이 이동하며 실제로 지나간 지형/나무가 보라색으로 바뀌는지, `ros2 topic echo /forest_rescue/coverage_area_m2`로 면적 값이 시간에 따라 증가하는지 육안/CLI로 확인.
- 통합 확인(누적성 회귀): 특정 영역을 지나간 드론이 다른 위치로 이동한 뒤에도, 이전에 마킹된 영역이 계속 보라색으로 남아 있는지(사라지지 않는지) 확인.

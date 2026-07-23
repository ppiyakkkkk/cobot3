# 임무 중 3드론 동시 비행 + 사후 순차 지도 생성 설계

## 배경

지금까지는 정지한 드론 한 대씩 `isaac_sim/final_24.py`를 라이브로 띄워놓고
`ring_filler_node.py` → `lio_sam run.launch.py` → `save_map` 서비스를 수동으로
호출해 PCD를 저장하고, `merge_maps.py`로 3대를 합치는 방식을 검증했다
(`world_x/y/z`, `imu_topic` 런치 인자, `Horizon_SCAN` 등 파라미터 튜닝 포함).

이제 실제 victim 탐지 임무(`forest_rescue_system.launch.py`)에서 3드론이
**동시에** 탐색 경로를 비행하는 동안 그 경로로 포인트클라우드 지도를
만들 수 있어야 한다.

## 결정 사항

- **비행은 동시, LIO-SAM 처리는 사후 순차.** 3드론은 지금처럼 동시에
  실제 임무를 수행한다(수색 효율 변화 없음). 임무 중에는 각 드론의
  라이다/IMU 데이터를 rosbag으로 녹화만 하고, 임무 종료 후 bag 3개 분량을
  드론별로 순서대로 재생하며 기존에 검증된 LIO-SAM 파이프라인에 태운다.
  (3개 LIO-SAM 인스턴스를 실시간 동시 실행하는 방식은 GPU/CPU 부담과
  토픽/TF 네임스페이스 충돌 문제로 채택하지 않음.)
- **녹화/처리 완전 자동화.** `/mission/state` 토픽을 지켜보는 스크립트가
  `"SEARCHING"` 진입 시 자동 녹화 시작, 이탈 시 자동 종료 → 곧바로 후처리
  스크립트가 3대를 순서대로 처리하고 마지막에 `merge_maps.py`까지 호출한다.
- **기존 코드는 건드리지 않는다.** `mission_manager_node.py`,
  `forest_rescue_system.launch.py`는 그대로 두고, `/mission/state`를
  구독만 하는 독립 스크립트 2개를 추가한다.

## 구성 요소

### 1. `scripts/record_mission_maps.py`

- `/mission/state`(std_msgs/String) 구독.
- 값이 `"SEARCHING"`이 되면 아래 토픽을 하나의 bag으로 녹화 시작
  (`ros2 bag record` 서브프로세스 실행):
  - `/quadrotor_01/point_cloud`, `/quadrotor_01/imu/data`
  - `/quadrotor_02/point_cloud`, `/quadrotor_02/imu/data`
  - `/quadrotor_03/point_cloud`, `/quadrotor_03/imu/data`
  - `/clock`
- 값이 `"SEARCHING"`이 아닌 다른 상태로 바뀌면(`COMPLETE`,
  `RETURNING_NO_VICTIM` 등) 녹화 프로세스 종료.
- bag 저장 경로: `~/lio_sam_maps/bags/mission_<타임스탬프>/`.
- 녹화 종료 후 bag 경로를 표준출력에 출력하고, 이어서
  `process_mission_maps.py`를 그 bag 경로 인자로 자동 실행.

### 2. `scripts/process_mission_maps.py`

- 인자로 받은 bag 경로에 대해 quadrotor_01 → 02 → 03 순서로 반복:
  1. `ring_filler_node.py` 서브프로세스 시작
     (`input_topic:=/quadrotor_0N/point_cloud`, `output_topic:=/points`)
  2. `imu_filter_node.py` 서브프로세스 시작 — 급기동 시 순간적으로 튀는
     가속도(> 15 m/s²)를 직전 정상값으로 클램핑해서
     `/quadrotor_0N/imu/data_filtered`로 republish
  3. `lio_sam run.launch.py` 서브프로세스 시작
     (`imu_topic:=/quadrotor_0N/imu/data_filtered`, 해당 드론 `world_x/y/z`,
     `use_sim_time:=true`)
  4. `ros2 bag play <bag> --clock` 실행(bag 전체를 끊지 않고 통째로),
     재생 끝날 때까지 대기
  5. `/lio_sam/save_map` 서비스 호출
     (`destination: /lio_sam_maps/quadrotor_0N`)
  6. 위 프로세스들을 프로세스 그룹째로 정리 후 다음 드론으로
     (`ros2 launch`의 SIGINT가 자식까지 확실히 못 죽이는 경우가 있어서
     `start_new_session=True`로 띄우고 `os.killpg`로 정리한다 — 안 그러면
     이전 드론의 `world→map` 오프셋을 계속 발행하는 좀비가 쌓여서 지도가
     "순간이동"하는 것처럼 보였다)

  **급기동 구간을 통째로 건너뛰는 방식은 시도했다가 롤백했다.** 세그먼트
  전환마다(-\-start-offset로 재시작) 궤적이 실제로 순간이동했고,
  `save_map`이 오히려 더 자주 실패했다(빈 키프레임). `imuPreintegration.cpp`의
  진짜 원인(TF frame_id 미설정)을 고친 뒤로는 `imu_filter_node`의 클램핑만으로
  충분한지 다시 검증 중이다.
- 3대 모두 끝나면 `merge_maps.py`를 호출해
  `~/lio_sam_maps/merged.pcd` 생성까지 자동 수행.
- **`ROS_DOMAIN_ID`를 라이브 임무와 다른 값(149, `.bashrc`의 기본값 144/
  크로스머신용 143과 명백히 구분됨)으로 격리해서 실행한다.**
  임무 종료 후에도 Isaac Sim은 계속 떠 있고 자체 `/clock`을 계속 발행하는데,
  이 스크립트가 같은 도메인에서 bag을 `--clock`으로 재생하면 `/clock`
  발행자가 두 개가 되어 시간이 거꾸로 튀고 rviz/TF가 깨진다(실제로 겪은
  문제). bag에 필요한 데이터가 다 있어 라이브 토픽이 필요 없으므로 완전
  격리가 안전하다.
- 드론별 스폰 좌표(`world_x/y/z`)는 지금까지와 동일하게 스크립트 내
  상수로 정의(`sim_config.py`의 `DRONE_CONFIGS`와 동일 값,
  `merge_maps.py`의 `DRONE_OFFSETS`와 동일한 패턴).
- **z만 스폰 좌표 + 이륙 고도(기본 6.0m).** bag 녹화는 스폰 순간이 아니라
  `SEARCHING` 진입 시점(이륙+호버링 완료 후)부터 시작되므로, LIO-SAM
  map 원점은 스폰 위치가 아니라 그만큼 위에 있다. 실제 임무 로그
  (`~/.ros/log`의 `drone_controller_0N` 로그, `output='screen'`이라
  파일로는 우연히 별도 `python_*.log`에 남아있었음)로 확인:
  x,y는 SEARCHING 시작 시점에 스폰과 동일(N=0,E=0, 수평 드리프트 없음),
  bag 첫 IMU 샘플의 yaw도 세 드론 다 0도 근접이라 identity 회전 가정은
  유효했다. z만 어긋나 있었다(세 산이 따로 보이던 원인).

  (서브에이전트로 ENU/NED 축 반전 여부도 별도 조사했음 — Isaac Sim→
  Pegasus→PX4→MAVSDK 전 구간에서 ENU/NED 변환이 일관되게 올바름을
  확인, 축 반전 버그는 없었다.)

### 3. `src/lio_sam/launch/run.launch.py` 변경

- `use_sim_time` 런치 인자 추가, 기본값 `false`
  (기존 수동 실시간 테스트 방식 그대로 유지).
- 이 값을 `static_transform_publisher`(2개), `robot_state_publisher`,
  5개 `lio_sam_*` 노드, `rviz2`에 파라미터로 전달.
- `process_mission_maps.py`는 이 인자를 `true`로 넘겨 bag의 `/clock`을
  따라가도록 한다.

## 에러 처리

- `/mission/state`가 한 번도 `"SEARCHING"`이 안 되고 스크립트가 종료되면
  (Ctrl+C) 녹화 없이 그냥 종료.
- 각 드론 처리 중 `save_map`이 `success: false`를 반환하면(빈 키프레임 등)
  경고만 출력하고 다음 드론으로 계속 진행 — 전체 파이프라인이 한 드론
  실패로 멈추지 않는다.

## 범위 밖

- 3 LIO-SAM 인스턴스 실시간 동시 실행은 이번 설계에 포함하지 않는다.
- `mission_manager_node.py`의 상태 문자열이 바뀌면 이 스크립트들도 같이
  수정해야 한다(하드코딩된 `"SEARCHING"` 문자열 의존).

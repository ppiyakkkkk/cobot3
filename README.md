# Forest Rescue Drone Baseline

Isaac Sim 5.1, Pegasus Simulator 5.1, PX4 SITL, ROS 2 Humble, MAVSDK를 이용한 산림 조난자 탐지 드론 기본 시스템입니다.

현재 기본 시스템은 YOLO11을 이용해 다음 흐름을 연결합니다.

```text
PX4 연결 → 5m 자동 이륙·Hover → 지그재그 수색
→ YOLO11 사람 탐지 → Depth/map 위치 계산 → 현 위치 Hover → 수동 착륙
```

처음 설치하는 과정과 터미널별 실행 명령은
[YOLO11 전체 실행 안내서](docs/YOLO_EXECUTION_GUIDE.md)를 따릅니다.

## 검증된 환경

- Ubuntu 22.04
- Isaac Sim 5.1
- Pegasus Simulator v5.1.0
- PX4-Autopilot v1.14.3
- ROS 2 Humble
- `ROS_DOMAIN_ID=143`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- MAVSDK UDP: `udpin://0.0.0.0:14540`

## 디렉터리

```text
isaac_sim/
  forest_rescue_sim.py
scripts/
  01_mavsdk_takeoff_test.py
  02_mavsdk_motion_test.py
  03_sensor_view_test.py
  04_camera_yaw_test.py
  setup_integration_env.sh
  check_yolo_setup.sh
  build_ros2.sh
src/
  forest_rescue_interfaces/
  forest_rescue_system/
docs/
  system_architecture.md
```

## 1. 기존 ROS 2/MAVSDK 환경 확인

새 터미널에서 사용자가 직접 기존 alias를 실행합니다.

```bash
source ~/.bashrc
ros_setup
mavsdk_on
```

`rclpy`와 MAVSDK를 같은 Python에서 import할 수 있는지 확인합니다.

```bash
cd ~/b3_cobot3_ws
bash scripts/setup_integration_env.sh
```

YOLO11 패키지와 가중치까지 설치합니다.

```bash
bash scripts/setup_integration_env.sh --with-yolo
bash scripts/check_yolo_setup.sh
```

## 2. ROS 2 빌드

```bash
cd ~/b3_cobot3_ws
ros_setup
mavsdk_on
bash scripts/build_ros2.sh
source install/setup.bash
```

빌드 후 인터페이스 확인:

```bash
ros2 interface show forest_rescue_interfaces/msg/VictimDetection
```

## 3. Isaac Sim 실행

첫 번째 터미널:

```bash
source ~/.bashrc
ros_setup
isaac_ros_setup

cd ~/b3_cobot3_ws
isaac_python isaac_sim/forest_rescue_sim.py
```

시뮬레이션 파일은 다음 설정을 코드에서 적용합니다.

- 조난자 위치: `[8.0, 12.0, 0.0]`
- 수색 범위: PX4 local NED 기준 약 `8m × 8m`
- 배치 의도: 초기 카메라 시야 바깥, 수색 지점 5/9 전후부터 시야 진입
- 카메라 focal length: `18.0mm`
- 예상 수평 FOV: 약 `60도`
- 카메라 하향각: `30도`
- LiDAR: `Example_Rotary`, `/point_cloud`, `base_scan`

## 4. 통합 시스템 실행

두 번째 터미널:

```bash
source ~/.bashrc
ros_setup
mavsdk_on
source ~/b3_cobot3_ws/install/setup.bash

ros2 launch forest_rescue_system \
  forest_rescue_system.launch.py
```

PX4 연결 후 드론은 자동으로 5m까지 이륙하고 제자리 Hover를 유지합니다.
Launch 로그에 `READY`가 표시된 후 임무를 시작합니다.
YOLO 탐지 노드가 정상 실행되고 `/victim/annotated_image`가 발행되는지
확인한 다음 임무를 시작합니다.

세 번째 터미널:

```bash
source ~/.bashrc
ros_setup
source ~/b3_cobot3_ws/install/setup.bash

ros2 service call \
  /mission/start \
  std_srvs/srv/Trigger \
  "{}"
```

탐지 후 드론은 복귀하지 않고 조난자 위치에서 Hover를 유지합니다. 확인 후 착륙시킵니다.

```bash
ros2 service call \
  /mission/land \
  std_srvs/srv/Trigger \
  "{}"
```

## 5. 주요 모니터링 명령

```bash
ros2 topic echo /mission/state
ros2 topic echo /drone/status
ros2 topic echo /victim/detection
ros2 topic echo /victim/position_camera
ros2 topic echo /victim/position_map
ros2 topic echo /obstacle/min_distance
```

RViz2에서는 다음 토픽을 추가합니다.

- Image: `/quadrotor/Camera/rgb`
- Image: `/victim/annotated_image`
- PointCloud2: `/point_cloud`

TF 확인:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link Camera
ros2 run tf2_ros tf2_echo base_link base_scan
```

기존에 `/tf --once`가 계속 대기했던 것은 TF publisher가 없었기 때문입니다. 통합 시스템을 실행하면 `drone_controller_node`와 `sensor_tf_node`가 `/tf`, `/tf_static`을 발행합니다.

## 6. YOLO11 또는 팀 학습 모델 사용

기본 YAML은 `yolo11s.pt`를 사용하는 실제 YOLO 모드입니다.

설정 파일:

```text
src/forest_rescue_system/config/forest_rescue.yaml
```

다음처럼 변경합니다.

```yaml
human_detector_node:
  ros__parameters:
    detector_mode: yolo
    model_path: ~/b3_cobot3_ws/models/yolo11s.pt
    person_class_id: 0
    confidence_threshold: 0.25
```

변경 후 다시 빌드하지 않아도 `--symlink-install`에서는 소스 설정이 반영되지만, 설치 설정을 확실히 갱신하려면 다시 빌드합니다.

```bash
bash scripts/build_ros2.sh
```

팀원별 교체 범위와 고정 인터페이스는 [docs/system_architecture.md](docs/system_architecture.md)를 참고하세요.

## 안전 주의

- 임무 시작 전 Isaac Sim의 PX4 연결과 센서 토픽을 확인하세요.
- 첫 통합 실행에서는 RViz와 터미널 로그를 보면서 즉시 착륙할 수 있게 준비하세요.
- `/obstacle/blocked`는 기본 전방 부채꼴 안전 정지 기능이며 완성된 회피 알고리즘이 아닙니다.
- LiDAR 장애물 감시는 이륙 중에는 비활성이고 `READY`, `SEARCHING`에서만 활성화됩니다.
- LiDAR 안전 감시는 카메라 60도와 별개로 360도 전체 수평면을 사용합니다.
- 조난자 위치는 임무당 최초 유효 위치로 고정되며 다음 수색 임무에서 초기화됩니다.
- 각 수색 지점은 수평·고도 오차로 실제 도착을 확인한 후 다음 지점으로 진행합니다.
- 전체 경로에서 사람을 못 찾으면 `SEARCH_COMPLETE_NOT_FOUND`로 종료하고 Hover합니다.
- COCO 사전학습 YOLO11 결과는 Isaac Sim 합성 환경에서 별도 검증해야 합니다.

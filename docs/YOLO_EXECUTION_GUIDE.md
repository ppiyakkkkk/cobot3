# 산림 조난자 탐지 드론 시스템 실행 방법 — YOLO11 기준

## 1. 현재 시스템 동작

```text
Isaac Sim·Pegasus·PX4 실행
→ ROS 2 통합 시스템 실행
→ PX4 연결 후 드론이 자동으로 5m 이륙
→ READY 상태에서 임무 시작 대기
→ /mission/start 호출
→ 지그재그 수색 및 YOLO11 사람 탐지
→ RGB Bounding Box와 Depth로 조난자 위치 계산
→ 사람 3회 연속 탐지 후 현 위치 Hover
→ /mission/land 호출 후 착륙
```

기본 탐지 모델은 COCO 사전학습 `yolo11s.pt`이며 사람 클래스 ID는 `0`입니다.

## 2. 터미널별 역할

| 터미널 | 역할 | 종료 시점 |
| --- | --- | --- |
| 터미널 1 | Isaac Sim, Pegasus, PX4 실행 | 가장 마지막 |
| 터미널 2 | 전체 ROS 2 노드와 YOLO11 실행 | 착륙 확인 후 |
| 터미널 3 | RViz2 센서·탐지 결과 확인 | 착륙 확인 후 |
| 터미널 4 | 상태 확인, 임무 시작, 착륙 | 임무 종료까지 |

## 3. 최초 한 번만 실행하는 준비 과정

다음 경우에 실행합니다.

- 프로젝트를 처음 설치했을 때
- 새로운 컴퓨터나 가상환경을 사용할 때
- YOLO 가중치가 없을 때
- Python 패키지 환경을 다시 구성할 때

인터넷 연결이 필요합니다.

```bash
cd ~/b3_cobot3_ws

ros_setup
mavsdk_on

bash scripts/setup_integration_env.sh --with-yolo
```

이 스크립트는 다음 작업을 수행합니다.

- ROS 2와 MAVSDK 통합 환경 검사
- ROS 인터페이스 생성용 `empy`, `catkin_pkg`, `lark-parser` 설치
- YOLO용 NumPy와 OpenCV 설치
- `ultralytics==8.4.101` 설치
- `~/b3_cobot3_ws/models/yolo11s.pt` 다운로드

설치 결과를 다시 확인합니다.

```bash
bash scripts/check_yolo_setup.sh
```

정상 출력의 마지막 부분:

```text
[OK] YOLO11: /home/rokey/b3_cobot3_ws/models/yolo11s.pt
[OK] YOLO 실행 환경 확인 완료
```

### ROS 2 패키지 최초 빌드

```bash
cd ~/b3_cobot3_ws

ros_setup
mavsdk_on

bash scripts/build_ros2.sh
source install/setup.bash
```

인터페이스 확인:

```bash
ros2 interface show forest_rescue_interfaces/msg/VictimDetection
```

`image_width`, `image_height` 필드까지 나타나면 정상입니다.

## 4. 코드나 설정을 수정했을 때

Python 노드, Launch, YAML, ROS 인터페이스를 수정한 경우 다시 빌드합니다.

```bash
cd ~/b3_cobot3_ws
ros_setup
mavsdk_on
bash scripts/build_ros2.sh
source install/setup.bash
```

컴퓨터를 재부팅했다는 이유만으로 환경 설치나 빌드를 다시 할 필요는 없습니다.

## 5. 평상시 전체 시스템 실행

반드시 터미널 1부터 순서대로 실행합니다.

### 터미널 1 — Isaac Sim, Pegasus, PX4

```bash
cd ~/b3_cobot3_ws

ros_setup
isaac_ros_setup

isaac_python isaac_sim/forest_rescue_sim.py
```

다음 항목을 확인합니다.

- Isaac Sim 창이 열림
- Rough Plane과 Iris 드론이 보임
- 조난자 모델이 배치됨
- PX4 SITL 로그가 나타남
- RGB, Depth, CameraInfo, LiDAR 토픽이 발행됨

센서 토픽 확인:

```bash
ros2 topic list | grep -E "Camera|point_cloud"
```

터미널 1은 임무가 끝날 때까지 종료하지 않습니다.

### 터미널 2 — ROS 2 통합 시스템과 YOLO11

Isaac Sim과 PX4가 준비된 후 새 터미널에서 실행합니다.

```bash
cd ~/b3_cobot3_ws

ros_setup
mavsdk_on
source install/setup.bash

ros2 launch forest_rescue_system \
  forest_rescue_system.launch.py
```

Launch 파일이 탐지 노드와 드론 제어 노드에
`~/venvs/pegasus_control/bin/python`을 자동 적용합니다.

다음 로그를 확인합니다.

```text
YOLO 모델 로드 완료: /home/rokey/b3_cobot3_ws/models/yolo11s.pt
탐지 모드=yolo
PX4 연결 성공
INITIAL_TAKEOFF
INITIAL_HOVER
READY
LiDAR 장애물 감시 활성화: mission_state=READY
```

Launch 실행과 동시에 PX4 연결 후 5m 자동 이륙을 시작합니다. Isaac Sim과 PX4가 준비되기 전에 터미널 2를 실행하지 않습니다.

`human_detector_node`가 오류로 종료했거나 `READY`가 나오지 않았으면 임무를 시작하지 않습니다.

### 터미널 3 — RViz2

```bash
cd ~/b3_cobot3_ws

ros_setup
source install/setup.bash

rviz2
```

RViz2 Display:

| Display | Topic |
| --- | --- |
| Image | `/quadrotor/Camera/rgb` |
| Image | `/victim/annotated_image` |
| PointCloud2 | `/point_cloud` |

Fixed Frame은 `map`으로 설정합니다.

먼저 `/victim/annotated_image`에서 실제 사람에게 Bounding Box가 표시되는지 확인합니다.

### 터미널 4 — 상태 확인과 임무 제어

```bash
cd ~/b3_cobot3_ws

ros_setup
source install/setup.bash
```

핵심 노드 확인:

```bash
ros2 node list | grep -E \
  "human_detector|victim_localizer|mission_manager|drone_controller"
```

드론 상태 확인:

```bash
ros2 topic echo /drone/status
```

별도 탭에서 임무 상태 확인:

```bash
ros2 topic echo /mission/state
```

`READY`이고 YOLO 탐지 노드가 살아 있을 때만 임무를 시작합니다.

```bash
ros2 service call \
  /mission/start \
  std_srvs/srv/Trigger \
  "{}"
```

예상 상태:

```text
READY
→ SEARCHING
→ VICTIM_DETECTED
→ VICTIM_LOCATED
→ COMPLETE
```

전체 수색 경로에서 사람을 찾지 못한 경우:

```text
READY
→ SEARCHING
→ SEARCH_COMPLETE_NOT_FOUND
→ 현 위치 Hover
```

탐지 결과를 직접 확인하려면 다음 명령을 사용합니다.

```bash
ros2 topic echo /victim/detection
```

계산된 위치:

```bash
ros2 topic echo /victim/position_camera
ros2 topic echo /victim/position_map
```

## 6. 착륙과 프로그램 종료

조난자 탐지 여부와 관계없이 프로그램을 종료하기 전에 먼저 착륙시킵니다.

```bash
ros2 service call \
  /mission/land \
  std_srvs/srv/Trigger \
  "{}"
```

종료 순서:

1. `/mission/land` 호출
2. Isaac Sim 화면에서 실제 착륙 확인
3. 터미널 2 Launch 종료: `Ctrl+C`
4. 터미널 3 RViz2 종료: `Ctrl+C`
5. 터미널 1 Isaac Sim 종료

## 7. 매일 실행할 명령만 요약

### 터미널 1

```bash
cd ~/b3_cobot3_ws
ros_setup
isaac_ros_setup
isaac_python isaac_sim/forest_rescue_sim.py
```

### 터미널 2

```bash
cd ~/b3_cobot3_ws
ros_setup
mavsdk_on
source install/setup.bash
ros2 launch forest_rescue_system forest_rescue_system.launch.py
```

### 터미널 3

```bash
cd ~/b3_cobot3_ws
ros_setup
source install/setup.bash
rviz2
```

### 터미널 4

```bash
cd ~/b3_cobot3_ws
ros_setup
source install/setup.bash

ros2 service call /mission/start std_srvs/srv/Trigger "{}"
```

### 임무 종료

```bash
ros2 service call /mission/land std_srvs/srv/Trigger "{}"
```

## 8. YOLO11 설정

설정 파일:

```text
~/b3_cobot3_ws/src/forest_rescue_system/config/forest_rescue.yaml
```

기본값:

```yaml
detector_mode: yolo
model_path: ~/b3_cobot3_ws/models/yolo11s.pt
person_class_id: 0
confidence_threshold: 0.25
inference_period_sec: 0.20
```

기본 모델은 정확도를 우선해 `yolo11s.pt`를 사용합니다. Isaac Sim과 동시에 실행할 때 추론 부하가 크면 `yolo11n.pt`로 낮출 수 있습니다. 팀 학습 모델을 받으면 `model_path`를 해당 `.pt` 파일로 변경합니다.

## 9. 중요 주의사항

- 현재 YOLO11은 COCO 사전학습 모델이므로 Isaac Sim 합성 사람을 항상 탐지한다는 보장은 없습니다.
- 탐지 노드가 죽어도 현재 버전에서는 드론 제어 노드가 계속 동작할 수 있으므로 Launch 로그를 반드시 확인합니다.
- LiDAR 감시는 이륙 중 비활성이고 `READY`, `SEARCHING`에서만 활성화됩니다.
- 카메라 수평 FOV는 약 60도이지만 LiDAR는 충돌 안전을 위해 360도 전체 수평면을 감시합니다.
- 각 Offboard 수색 지점은 수평오차 0.5m, 고도오차 0.5m 이내 도착을 확인하고 4초간 관측한 뒤 다음 지점으로 이동합니다.
- 한 지점에 15초 안에 도착하지 못하면 `ERROR_WAYPOINT_TIMEOUT`으로 전환하고 Hover합니다.
- 조난자는 `[8.0, 12.0, 0.0]`에 배치되어 초기 시야 바깥에 있고 8m×8m 수색 경로의 5번째 지점 전후부터 카메라 시야에 들어오도록 설계했습니다.
- `/mission/start`를 `COMPLETE` 또는 `SEARCH_COMPLETE_NOT_FOUND` 상태에서 다시 호출하면 새로운 수색 임무가 시작됩니다.

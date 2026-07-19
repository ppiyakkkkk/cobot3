# 산림 조난자 탐지 드론 기본 시스템 구조

## 전체 데이터 흐름

```text
Isaac Sim / Pegasus
 ├─ RGB ───────────────→ human_detector_node
 ├─ Depth/CameraInfo ──→ victim_localizer_node
 └─ LiDAR ─────────────→ obstacle_monitor_node

human_detector_node
 ├─ /victim/detection ───────→ mission_manager_node
 └─ /victim/annotated_image ─→ RViz2

victim_localizer_node
 ├─ /victim/position_camera
 └─ /victim/position_map ────→ 위치 정확도 검증 및 임무 관리

mission_manager_node
 └─ /drone/command ──────────→ drone_controller_node → MAVSDK/PX4
```

## 팀원별 고정 인터페이스

### 환경 모델링

- 유지해야 하는 드론 Prim: `/World/quadrotor`
- 유지해야 하는 카메라 Prim: `/World/quadrotor/body/Camera`
- 현재 조난자 객체 이름: `victim_01`
- 최종 산출물 권장 위치: `isaac_sim/worlds/forest_world.usd`

### 사람 탐지 모델

- 입력: `/quadrotor/Camera/rgb` (`sensor_msgs/Image`)
- 출력: `/victim/detection` (`forest_rescue_interfaces/VictimDetection`)
- 시각화: `/victim/annotated_image` (`sensor_msgs/Image`)
- 팀원은 `human_detector_node.py` 내부 추론부나 YAML의 `model_path`만 교체한다.

### 드론 이동 제어

- 명령 입력: `/drone/command` (`std_msgs/String`)
- 상태 출력: `/drone/status` (`std_msgs/String`)
- 기본 명령: `TAKEOFF`, `START_SEARCH`, `HOVER`, `LAND`
- LiDAR 안전 정지: `/obstacle/blocked` (`std_msgs/Bool`)
- LiDAR 감시 상태: `READY`, `SEARCHING`에서만 활성
- 팀원은 `drone_controller_node.py`의 수색·회피 로직을 개선한다.

## 현재 기본값과 한계

- 카메라: focal length 18mm, 약 60도 수평 FOV, 아래쪽 30도
- 이륙 고도: 5m
- 탐색 Yaw: 0도(초기 Hover에서는 현재 PX4 Yaw 유지)
- 탐지 후: 출발점으로 복귀하지 않고 Hover
- Mock 지연 시간은 `SEARCHING` 진입 시점부터 계산한다.
- 최초 유효 조난자 위치는 임무가 끝날 때까지 고정한다.
- Mock 탐지는 연결 검증용이며 실제 사람 인식 성능을 의미하지 않는다.
- 동적 TF는 MAVSDK NED 위치를 ROS ENU로 변환하고, 기본 버전에서는 roll/pitch를 생략한다.

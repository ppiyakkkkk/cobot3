# Forest Rescue Multi-Drone System

Isaac Sim, Pegasus Simulator, PX4 SITL, ROS 2, MAVSDK와 YOLO를 이용한  
**산림 내 조난자 자동 탐지 다중 드론 시뮬레이션 프로젝트**입니다.

3대의 3DR Iris 드론이 산림 지형을 구역별로 나누어 수색하고, 비행 중 LiDAR로 장애물을 감지·회피합니다.  
조난자가 탐지되면 RGB-D 데이터를 이용해 위치를 계산하고 해당 드론은 Hover 상태를 유지합니다.

> 이 저장소에는 일반적으로 YOLO `.pt` 가중치를 포함하지 않습니다.  
> 처음 클론한 사용자는 아래의 **모델 가중치 준비** 절차를 반드시 수행해야 합니다.

---

## 1. 프로젝트 목표

- Isaac Sim 산림 환경에서 조난자 위치 무작위 생성
- 3대의 드론을 이용한 담당 구역 병렬 수색
- 지형 높이를 반영한 지그재그 수색 경로 자동 생성
- RGB 영상과 YOLO를 이용한 사람 탐지
- Depth 영상과 TF를 이용한 조난자 3차원 위치 계산
- 360° LiDAR 기반 실시간 장애물 감지
- 로컬 2D 점유격자와 A*를 이용한 우회 경로 생성
- A* 실패 시 VFH 방향 회피
- 수평 회피 실패 시 단계적 상승 회피
- 특정 드론에 오류가 발생해도 나머지 드론은 수색 계속
- ROS 2 토픽, 서비스, TF 및 RViz를 이용한 상태 확인

---

## 2. 전체 동작 흐름

```text
Isaac Sim 산림 USD 로드
    ↓
드론 3대·구조자·조난자 생성
    ↓
USD 지형 높이 기반 수색 구역 및 지그재그 경로 생성
    ↓
generated_search_plan.json 저장
    ↓
PX4 SITL 3대 연결 및 자동 이륙
    ↓
각 드론이 담당 구역 병렬 수색
    ↓
LiDAR 장애물 감지
    ├─ 로컬 A* 우회
    ├─ A* 실패 시 VFH 우회
    └─ 수평 회피 실패 시 상승 회피
    ↓
YOLO 사람 탐지
    ↓
Depth + CameraInfo + TF로 조난자 위치 계산
    ↓
탐지 드론 Hover 및 위치 보고
    ↓
사용자 착륙 명령
```

---

## 3. 주요 구현 기능

### 3.1 다중 드론 시뮬레이션

- 3DR Iris 드론 3대 사용
- PX4 vehicle ID를 드론별로 분리
- 카메라와 LiDAR 토픽을 드론별 namespace로 분리
- 시작 직후 서로의 LiDAR 점유 영역에 들어가지 않도록 간격을 두고 배치
- 각 드론은 서로 다른 산림 구역을 담당

드론 이름:

```text
quadrotor_01
quadrotor_02
quadrotor_03
```

센서 토픽 예시:

```text
/quadrotor_01/Camera/rgb
/quadrotor_01/Camera/depth
/quadrotor_01/Camera/camera_info
/quadrotor_01/Camera/depth_pcl
/quadrotor_01/point_cloud
```

드론 2, 3도 같은 형식으로 번호만 변경됩니다.

---

### 3.2 전역 수색 경로 생성

현재 프로젝트는 전역 3D 점유맵이나 전역 2D 코스트맵을 사용하지 않습니다.

대신 Isaac Sim의 산림 USD에서 지형 높이를 읽고 다음 순서로 수색 계획을 만듭니다.

```text
산림 USD
→ 지형 높이 샘플링
→ 전체 영역을 3개 담당 구역으로 분할
→ 지형 위 일정 높이를 유지하는 지그재그 경로 생성
→ ENU와 NED 좌표를 JSON으로 저장
```

생성 파일:

```text
isaac_sim/generated_search_plan.json
```

이 파일은 `isaac_sim/final_13.py` 실행 시 현재 USD 지형을 기준으로 자동 생성됩니다.

좌표계:

- `world_enu`: Isaac Sim 및 ROS map 좌표
  - X: East
  - Y: North
  - Z: Up
- `north_m`, `east_m`, `down_m`: PX4 local NED 좌표
  - X: North
  - Y: East
  - Z: Down

---

### 3.3 실시간 장애물 회피

각 드론은 360° LiDAR PointCloud를 이용해 현재 비행 높이 주변의 장애물을 감지합니다.

장애물 회피 우선순위:

```text
로컬 A* → VFH → 상승 회피
```

#### 로컬 A*

1. LiDAR 점군에서 현재 비행 높이 주변 점만 선택
2. 드론 중심의 로컬 2D 점유격자 생성
3. 장애물 주변을 안전 반경만큼 팽창
4. 현재 진행 방향 앞쪽에 로컬 목표 설정
5. A*로 우회 경로 계산
6. 경로 전체가 아니라 가까운 우회점만 드론 컨트롤러에 전달
7. 이동 직전과 이동 중에 LiDAR로 경로를 재검증

현재 맵은 일반적인 Nav2 단계형 코스트맵보다 단순한  
**팽창된 이진 로컬 점유격자**에 가깝습니다.

#### VFH

VFH는 `Vector Field Histogram`의 약자입니다.

A* 경로를 만들 수 없을 때 LiDAR의 여러 각도 후보를 비교하여:

- 장애물 여유거리가 크고
- 원래 진행 방향에서 회전량이 작으며
- 최소 안전거리를 만족하는

방향을 임시 우회 방향으로 선택합니다.

#### 상승 회피

로컬 A*와 VFH 모두 안전한 수평 경로를 찾지 못하면 고도를 단계적으로 높여 재계획합니다.

특정 드론이 최종적으로 회피에 실패하더라도 해당 드론만 Hover 또는 오류 상태로 전환되고,  
다른 드론은 계속 수색하도록 구성되어 있습니다.

---

### 3.4 조난자 탐지와 위치 계산

탐지 흐름:

```text
RGB Image
→ YOLO person Bounding Box
→ Bounding Box 중심 픽셀
→ Depth 영상 거리값
→ CameraInfo 내부 파라미터
→ Camera 좌표계 3D 위치
→ TF 변환
→ map 좌표계 조난자 위치
```

기본 탐지 모델:

```text
YOLO11s COCO pretrained model
```

기본 사람 클래스:

```text
person_class_id: 0
```

시작 지점의 구조자를 조난자로 오탐하지 않도록 수색 시작 후 일정 시간 동안 탐지를 비활성화할 수 있습니다.

---

## 4. 검증 환경

- Ubuntu 22.04
- Python 3.10
- Isaac Sim 5.1
- Pegasus Simulator v5.1.0
- PX4-Autopilot v1.14.3
- ROS 2 Humble
- MAVSDK
- Ultralytics YOLO11
- OpenCV
- NumPy 1.26.x
- `ROS_DOMAIN_ID=143`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`

이 저장소만 클론한다고 Isaac Sim, Pegasus Simulator, PX4와 ROS 2가 자동 설치되지는 않습니다.  
각 프로그램은 먼저 별도로 설치되어 있어야 합니다.

---

## 5. 저장소 구조

```text
b3_cobot3_ws/
├── docs/
│   ├── system_architecture.md
│   └── YOLO_EXECUTION_GUIDE.md
├── isaac_sim/
│   ├── final_13.py
│   ├── generated_search_plan.json  # 실행 시 자동 생성
│   └── worlds/
│       ├── forest_world.usda
│       ├── korean_mountain_100x100_dense_grouped.usd
│       ├── korean_mountain_dense_rugged_grouped.usd
│       └── my_forest.usd
├── models/
│   └── yolo11s.pt
├── scripts/
│   ├── 01_mavsdk_takeoff_test.py
│   ├── 02_mavsdk_motion_test.py
│   ├── 03_sensor_view_test.py
│   ├── 04_camera_yaw_test.py
│   ├── build_ros2.sh
│   ├── check_yolo_setup.sh
│   └── setup_integration_env.sh
├── src/
│   ├── forest_rescue_interfaces/
│   │   └── msg/VictimDetection.msg
│   └── forest_rescue_system/
│       ├── config/
│       │   ├── forest_rescue.yaml
│       │   └── forest_rescue_multi.rviz
│       ├── forest_rescue_system/
│       │   ├── drone_controller_node.py
│       │   ├── human_detector_node.py
│       │   ├── mission_manager_node.py
│       │   ├── obstacle_monitor_node.py
│       │   ├── sensor_tf_node.py
│       │   └── victim_localizer_node.py
│       └── launch/forest_rescue_system.launch.py
├── requirements.txt
└── README.md
```

다음 폴더는 빌드 또는 실행 과정에서 생성되므로 Git에 올리지 않습니다.

```text
build/
install/
log/
models/
```

---

# 설치 및 실행

## 6. 저장소 클론

브랜치 이름이 `feat/multi-drone-forest-rescue`인 경우:

```bash
cd ~
git clone \
  -b feat/multi-drone-forest-rescue \
  https://github.com/ppiyakkkkk/cobot3.git \
  b3_cobot3_ws

cd ~/b3_cobot3_ws
```

이미 저장소를 클론한 경우:

```bash
cd ~/b3_cobot3_ws
git fetch origin
git switch feat/multi-drone-forest-rescue
git pull
```

실제 브랜치 이름이 다르면 위 명령의 브랜치 이름을 변경합니다.

---

## 7. 사전 설치 항목

다음 항목은 저장소 외부에서 먼저 준비되어 있어야 합니다.

1. ROS 2 Humble
2. Isaac Sim 5.1
3. Pegasus Simulator 5.1
4. PX4-Autopilot 1.14.3
5. MAVSDK용 Python 가상환경
6. Isaac Sim ROS 2 Bridge 실행 환경

현재 프로젝트에서는 다음 alias를 사용합니다.

```bash
ros_setup
isaac_ros_setup
mavsdk_on
isaac_python
```

예시 `.bashrc` 환경:

```bash
export ROS_DOMAIN_ID=143
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

alias의 실제 경로는 각 PC의 Isaac Sim, PX4 및 가상환경 설치 위치에 맞게 설정해야 합니다.

---

## 8. 모델 가중치 준비

### 8.1 저장소에 `.pt` 파일이 없는 이유

`.gitignore`에서 `models/` 폴더를 제외하므로 일반적인 Git push에는 `.pt` 모델이 포함되지 않습니다.

이 방식은 다음 문제를 피하기 위한 것입니다.

- 대용량 파일로 인한 Git 저장소 용량 증가
- GitHub 일반 파일 용량 제한
- 팀 학습 모델의 불필요한 중복 업로드
- 모델 라이선스와 배포 범위 문제

따라서 처음 클론한 사용자는 자신의 PC에서 모델을 준비해야 합니다.

---

### 8.2 기본 YOLO11s 자동 다운로드

인터넷에 연결된 환경에서는 제공된 스크립트가 Ultralytics 패키지와 `yolo11s.pt`를 준비합니다.

```bash
cd ~/b3_cobot3_ws

source ~/.bashrc
ros_setup
mavsdk_on

bash scripts/setup_integration_env.sh --with-yolo
bash scripts/check_yolo_setup.sh
```

정상 완료 시 다음 파일이 생성됩니다.

```text
~/b3_cobot3_ws/models/yolo11s.pt
```

스크립트가 수행하는 주요 작업:

- ROS 2 Python 환경 확인
- MAVSDK 가상환경 확인
- `empy==3.3.4`, `catkin_pkg`, `lark-parser` 설치
- `numpy==1.26.4` 설치
- `opencv-python==4.11.0.86` 설치
- `ultralytics==8.4.101` 설치
- `yolo11s.pt` 자동 다운로드
- `rclpy`, MAVSDK, OpenCV, YOLO 모델 로드 검사

첫 다운로드 시 인터넷 연결이 필요합니다.

---

### 8.3 다른 Ultralytics 기본 모델 사용

예를 들어 `yolo11n.pt`를 받고 싶다면:

```bash
cd ~/b3_cobot3_ws

source ~/.bashrc
ros_setup
mavsdk_on

YOLO_MODEL_NAME=yolo11n.pt \
bash scripts/setup_integration_env.sh --with-yolo
```

그 후 `src/forest_rescue_system/config/forest_rescue.yaml`의 `model_path`도 같은 파일명으로 변경합니다.

---

### 8.4 팀 학습 모델 사용

팀에서 학습한 모델을 별도 공유받은 경우:

```bash
cd ~/b3_cobot3_ws
mkdir -p models

cp /모델이/있는/경로/best.pt \
  ~/b3_cobot3_ws/models/forest_rescue_best.pt
```

설정 파일:

```text
src/forest_rescue_system/config/forest_rescue.yaml
```

예시:

```yaml
detector_mode: yolo
model_path: ~/b3_cobot3_ws/models/forest_rescue_best.pt
person_class_id: 0
confidence_threshold: 0.25
```

주의사항:

- 사용자 정의 모델에서 사람 클래스 번호가 `0`이 아닐 수 있습니다.
- 이 경우 `person_class_id`를 학습 데이터의 클래스 순서에 맞게 수정해야 합니다.
- 모델 파일명만 변경하고 YAML 경로를 수정하지 않으면 탐지 노드가 시작되지 않습니다.
- 오프라인 PC에서는 팀원이 `.pt` 파일을 USB, 내부 서버 또는 클라우드 저장소로 별도 전달해야 합니다.

---

### 8.5 모델 없이 연결만 시험

YOLO 모델이 아직 없다면 YAML에서 탐지 모드를 임시로 변경할 수 있습니다.

```yaml
detector_mode: mock
```

Mock 모드는 ROS 2 토픽, 위치 계산, 임무 상태 전환을 시험하기 위한 기능입니다.  
실제 사람 탐지 성능을 의미하지 않습니다.

---

## 9. ROS 2 및 Python 환경 확인

```bash
cd ~/b3_cobot3_ws

source ~/.bashrc
ros_setup
mavsdk_on

bash scripts/setup_integration_env.sh
```

YOLO까지 확인하려면:

```bash
bash scripts/check_yolo_setup.sh
```

다음 오류가 발생하면 먼저 alias 실행 여부를 확인합니다.

```text
[ERROR] 먼저 ros_setup을 실행하세요.
[ERROR] 먼저 mavsdk_on을 실행하세요.
```

---

## 10. ROS 2 빌드

```bash
cd ~/b3_cobot3_ws

source ~/.bashrc
ros_setup
mavsdk_on

bash scripts/build_ros2.sh
source install/setup.bash
```

인터페이스 확인:

```bash
ros2 interface show forest_rescue_interfaces/msg/VictimDetection
```

소스나 YAML을 변경한 뒤에는 다시 빌드하는 것을 권장합니다.

```bash
bash scripts/build_ros2.sh
source install/setup.bash
```

---

## 11. 실행 순서

### 터미널 1: Isaac Sim과 PX4 SITL 3대 실행

```bash
cd ~/b3_cobot3_ws

source ~/.bashrc
ros_setup
isaac_ros_setup

isaac_python isaac_sim/final_13.py
```

이 과정에서:

- 산림 USD 로드
- 드론 3대 생성
- 조난자 무작위 생성
- 구조자 생성
- 카메라와 LiDAR 생성
- PX4 SITL 3대 실행
- 수색 경로 생성
- `isaac_sim/generated_search_plan.json` 저장

이 수행됩니다.

Isaac Sim에서 드론 3대와 PX4 연결이 모두 준비된 후 다음 터미널을 실행합니다.

---

### 터미널 2: ROS 2 통합 노드 실행

```bash
cd ~/b3_cobot3_ws

source ~/.bashrc
ros_setup
mavsdk_on
source install/setup.bash

ros2 launch forest_rescue_system \
  forest_rescue_system.launch.py
```

RViz를 함께 실행하려면:

```bash
ros2 launch forest_rescue_system \
  forest_rescue_system.launch.py \
  use_rviz:=true
```

Launch는 다음 노드를 실행합니다.

```text
mission_manager_node

sensor_tf_01
obstacle_monitor_01
human_detector_01
victim_localizer_01
drone_controller_01

sensor_tf_02
obstacle_monitor_02
human_detector_02
victim_localizer_02
drone_controller_02

sensor_tf_03
obstacle_monitor_03
human_detector_03
victim_localizer_03
drone_controller_03
```

---

### 터미널 3: 임무 시작

```bash
cd ~/b3_cobot3_ws

source ~/.bashrc
ros_setup
source install/setup.bash

ros2 service call \
  /mission/start \
  std_srvs/srv/Trigger \
  "{}"
```

정상 흐름:

```text
IDLE
→ PX4 연결
→ 자동 이륙
→ READY
→ SEARCHING
→ 조난자 탐지 및 위치 계산
→ 탐지 드론 Hover
→ 임무 결과 보고
```

특정 드론이 오류 상태가 되어도 다른 드론은 수색을 계속할 수 있습니다.

---

### 터미널 4: 모니터링

전체 토픽 확인:

```bash
ros2 topic list | sort
```

관련 토픽만 확인:

```bash
ros2 topic list | \
grep -E "mission|quadrotor|drone_|victim|obstacle|point_cloud"
```

임무 상태:

```bash
ros2 topic echo /mission/state
```

드론 1 관련 토픽:

```bash
ros2 topic list | grep drone_01
```

센서 주기 확인:

```bash
ros2 topic hz /quadrotor_01/Camera/rgb
ros2 topic hz /quadrotor_01/point_cloud
```

TF 확인:

```bash
ros2 run tf2_ros tf2_echo \
  map \
  quadrotor_01/base_link
```

실제 frame 이름은 다음 명령으로 먼저 확인할 수 있습니다.

```bash
ros2 topic echo /tf_static --once
```

---

## 12. 임무 종료 및 착륙

조난자를 찾은 뒤 또는 시험을 종료할 때:

```bash
ros2 service call \
  /mission/land \
  std_srvs/srv/Trigger \
  "{}"
```

각 터미널은 `Ctrl+C`로 종료합니다.

Isaac Sim을 먼저 종료하면 PX4 연결과 ROS 2 센서 토픽도 함께 끊길 수 있으므로,  
가능하면 다음 순서로 종료합니다.

```text
1. /mission/land 호출
2. ROS 2 launch 종료
3. RViz 종료
4. Isaac Sim 종료
```

---

## 13. 주요 설정 파일

```text
src/forest_rescue_system/config/forest_rescue.yaml
```

이 파일에서 주로 조절하는 항목:

- 드론별 MAVSDK 연결 주소
- 검색 계획 JSON 경로
- 탐지 모드
- YOLO 모델 경로
- 사람 클래스 번호
- Confidence threshold
- 탐지 시작 유예시간
- LiDAR 장애물 판정 거리
- 로컬 A* 지도 크기와 해상도
- 장애물 팽창 반경
- VFH 후보 각도와 안전거리
- 상승 회피 간격과 최대 고도
- Waypoint 도착 허용 오차
- 재계획 주기

설정 변경 후:

```bash
bash scripts/build_ros2.sh
source install/setup.bash
```

---

## 14. 주요 ROS 2 노드

### `mission_manager_node`

- 전체 임무 상태 관리
- 3대 드론의 상태 수집
- 임무 시작과 착륙 서비스 제공
- 특정 드론 오류 격리
- 전체 성공 또는 실패 조건 판단

### `drone_controller_node`

- MAVSDK를 이용한 PX4 연결
- 이륙, Hover, Waypoint 이동, 착륙
- 수색 계획 JSON 로드
- 장애물 발생 시 감속과 재계획
- A*, VFH 및 상승 회피 결과 실행

### `obstacle_monitor_node`

- LiDAR PointCloud 수신
- 전방·좌측·우측·360° 최소 거리 계산
- 로컬 2D 점유격자 생성
- 장애물 팽창
- 로컬 A* 우회점 생성
- VFH 후보 방향 계산

### `human_detector_node`

- RGB 영상 수신
- YOLO 또는 Mock 사람 탐지
- `VictimDetection` 메시지 발행
- Bounding Box가 표시된 영상 발행
- 시작 지점 오탐 방지를 위한 탐지 유예시간 적용

### `victim_localizer_node`

- RGB Bounding Box와 Depth 영상 결합
- 카메라 좌표계 조난자 위치 계산
- TF를 이용해 map 좌표로 변환
- 최초 유효 조난자 위치 고정

### `sensor_tf_node`

- 카메라와 LiDAR의 정적 TF 발행
- 센서 프레임과 드론 base frame 연결

---

## 15. 현재 구현한 지도와 구현하지 않은 지도

### 사용 중

- Isaac Sim 산림 USD
- USD 지형 기반 2.5D 높이 정보
- LiDAR 기반 로컬 2D 이진 점유격자
- 장애물 팽창 영역
- 로컬 A* 우회 경로

### 현재 미사용

- 전역 3D 점유맵
- 전역 3D 코스트맵
- 전역 2D 코스트맵
- 전역 장애물 정보를 이용한 A*
- OctoMap
- nvblox
- Nav2 costmap

현재 전역 수색 경로는 나무 위치 전체를 미리 고려하지 않습니다.  
USD의 지형 높이로 기본 수색 경로를 생성하고, 실제 나무 회피는 비행 중 LiDAR로 처리합니다.

---

## 16. 문제 해결

### YOLO 가중치 파일을 찾지 못함

오류 예시:

```text
FileNotFoundError: YOLO 가중치 파일을 찾을 수 없습니다
```

해결:

```bash
cd ~/b3_cobot3_ws
ros_setup
mavsdk_on

bash scripts/setup_integration_env.sh --with-yolo
bash scripts/check_yolo_setup.sh
```

---

### `No module named cv2`

가상환경에서 OpenCV가 설치되지 않은 경우입니다.

```bash
ros_setup
mavsdk_on
bash scripts/setup_integration_env.sh --with-yolo
```

---

### `No module named rclpy`

ROS 2 환경을 source하지 않았거나 Isaac Sim과 시스템 Python 환경이 섞인 경우입니다.

```bash
source ~/.bashrc
ros_setup
```

MAVSDK 노드 실행 터미널에서는 이어서:

```bash
mavsdk_on
source ~/b3_cobot3_ws/install/setup.bash
```

---

### 모델 파일명은 맞지만 탐지 노드가 시작되지 않음

YAML의 `model_path`와 실제 파일 경로를 비교합니다.

```bash
ls -lh ~/b3_cobot3_ws/models/
grep -R "model_path" \
  ~/b3_cobot3_ws/src/forest_rescue_system/config/
```

---

### 수색 계획 JSON이 없거나 오래됨

`generated_search_plan.json`은 직접 작성하지 않고 시뮬레이션 실행 시 다시 생성합니다.

```bash
cd ~/b3_cobot3_ws
ros_setup
isaac_ros_setup
isaac_python isaac_sim/final_13.py
```

---

### 세 드론 중 하나가 오류가 난 뒤 나머지도 멈춤

정상 설계에서는 특정 드론 오류가 다른 드론의 `SEARCHING` 상태를 중지시키지 않아야 합니다.

확인 사항:

```bash
ros2 topic echo /mission/state
ros2 topic list | grep drone_
```

로그에서 다음을 구분합니다.

- 특정 드론만 오류: 해당 드론만 Hover, 나머지는 계속 수색
- 모든 드론이 실패: 전체 임무 `MISSION_FAILED`
- 조난자 탐지로 임무 정책상 정지: 오류와 별개의 정상 상태 전환

---

## 17. Git 브랜치 생성 및 업로드

새 브랜치 생성:

```bash
cd ~/b3_cobot3_ws

git switch main
git pull origin main

git switch -c feat/multi-drone-forest-rescue
```

추적 파일 확인:

```bash
git status
git status --ignored
```

빌드 결과와 모델이 제외되는지 확인합니다.

```text
build/
install/
log/
models/
```

`generated_search_plan.json`도 실행 시 다시 생성되는 파일이므로 Git에서 제외하는 것을 권장합니다.

```bash
printf "\n# Generated search plan\nisaac_sim/generated_search_plan.json\n" \
  >> .gitignore

# 과거에 이미 추적한 파일이라면 Git 인덱스에서만 제거
git rm --cached isaac_sim/generated_search_plan.json 2>/dev/null || true
```

변경 사항 추가:

```bash
git add \
  README.md \
  LICENSE \
  requirements.txt \
  scripts \
  docs \
  isaac_sim/final_13.py \
  isaac_sim/worlds \
  src \
  .gitignore \
  .gitattributes
```

커밋:

```bash
git commit -m \
  "feat: add multi-drone forest rescue simulation"
```

원격 브랜치로 push:

```bash
git push -u origin feat/multi-drone-forest-rescue
```

GitHub에서 해당 브랜치를 기준으로 Pull Request를 생성합니다.

> `git add .`을 사용해도 `.gitignore`에 등록된 폴더는 제외되지만,  
> push 전에 `git status`로 대용량 파일과 자동 생성 파일이 포함되지 않았는지 반드시 확인하세요.

---

## 18. 대용량 모델 공유 방법

기본 운영 방식은 `.pt` 파일을 Git에 올리지 않고 각 PC에서 다운로드하거나 복사하는 것입니다.

팀 학습 모델을 공유할 때 권장 방법:

1. Google Drive, OneDrive, 사내 NAS 등에 모델 업로드
2. GitHub Release asset으로 별도 배포
3. 모델 다운로드 URL과 SHA256을 문서에 기록
4. 필요할 때만 Git LFS 사용

SHA256 확인:

```bash
sha256sum models/forest_rescue_best.pt
```

Git LFS를 사용하려면 `.gitignore`에서 해당 모델을 예외 처리해야 합니다.  
현재 `.gitattributes`에 `.pt`, `.pth`, `.onnx`, `.engine`용 LFS 규칙이 있어도  
`models/`가 `.gitignore`에 포함되어 있으면 모델은 자동으로 추적되지 않습니다.

---

## 19. 안전 및 실행 주의사항

- Isaac Sim과 PX4가 준비되기 전에 ROS 2 임무를 시작하지 마세요.
- 첫 실행에서는 RViz와 로그를 확인하면서 즉시 착륙할 수 있도록 준비하세요.
- 카메라 탐지와 LiDAR 장애물 감지는 서로 다른 센서 흐름입니다.
- LiDAR의 `inf` 값은 측정 범위 안에 반사점이 없는 열린 공간일 수 있습니다.
- 로컬 A*가 경로를 만들었다고 해도 이동 직전에 LiDAR로 다시 검사합니다.
- YOLO COCO 사전학습 모델은 Isaac Sim 합성 영상에서 오탐 또는 미탐이 발생할 수 있습니다.
- 현재 시스템은 연구·교육용 시뮬레이션이며 실제 산림 구조 비행에 바로 사용할 수 없습니다.

---

## 20. 향후 개선 사항

- 전역 3D 점유맵 생성
- 전역 3D 또는 2D 코스트맵 구축
- 전역 경로와 로컬 경로의 계층형 계획
- 로컬 점유격자 ROS 메시지 발행 및 RViz 시각화
- 조난자 다중 탐지와 중복 제거
- 여러 조난자에 대한 드론 작업 할당
- 드론 간 충돌 회피
- 배터리와 통신 상태를 고려한 임무 재할당
- 실제 산림 데이터로 학습한 탐지 모델 적용
- 탐지 결과의 시간적 추적과 신뢰도 누적
- 구조대 지상 경로 계획 연동

---

## 21. 라이선스

이 프로젝트는 MIT License를 따릅니다.

자세한 내용은 [LICENSE](LICENSE)를 확인하세요.

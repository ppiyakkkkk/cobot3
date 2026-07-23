# LIO-SAM-ROS2 실습 튜토리얼 (KITTI 데이터)

이 저장소(`LIO-SAM-ROS2-main`)에는 실행형 튜토리얼 스크립트가 없습니다. 있는 건
① `README.md`의 텍스트 절차, ② ROS1 전용 변환 유틸리티 `config/doc/kitti2bag/kitti2bag.py`
뿐입니다. 이 문서는 그 공백을 메우기 위해 KITTI 공식 데이터로 끝까지 실행해보는 절차를
정리한 것입니다.

## 사전 확인된 환경

- Ubuntu 22.04, ROS2 Humble (이미 설치됨)
- ROS1이 설치되어 있지 않음 → 번들된 `kitti2bag.py`(rospy/rosbag 기반)는 그대로 못 씀
- 이 워크스페이스와 별개로 **독립 워크스페이스**(`~/lio_sam_ws`)를 새로 만들어 빌드

## 왜 kitti2bag.py를 그대로 못 쓰는가

`config/doc/kitti2bag/kitti2bag.py`는 `rospy`, `rosbag`, `tf`, `cv_bridge`를 임포트하는
ROS1 전용 스크립트입니다. Ubuntu 22.04에는 ROS1 Noetic이 (공식적으로) 설치되지 않으므로
이 스크립트를 그대로 실행할 수 없습니다.

그래서 같은 폴더에 **`kitti2rosbag2.py`를 새로 작성**했습니다. `rosbags`라는 순수 Python
라이브러리(ROS 설치 불필요)로 CDR 직렬화를 직접 수행하며, kitti2bag.py의 핵심 로직
(velodyne ring 계산 공식, IMU 필드 매핑)을 그대로 재현합니다. 이 세션에서 다음을 직접
검증했습니다:

- 가짜 KITTI 폴더 구조로 스크립트를 실행 → 정상적으로 `.bag` 생성
- `ros2 bag info`로 메타데이터 파싱 확인 (topic/message count 일치)
- `ros2 bag play` 후 `ros2 topic echo`로 실제 DDS 파이프라인에서 PointCloud2 / Imu
  메시지가 정확한 값으로 디코딩되는 것까지 확인

단, **실제 KITTI 데이터로 전체 파이프라인(다운로드 → 변환 → LIO-SAM 실행)을 끝까지
돌려본 것은 아닙니다** — 실제 드라이브 하나가 sync+extract 합쳐 약 3.9GB라 이 세션에서
임의로 받지 않았습니다. 아래 절차대로 사용자가 직접 진행하면 됩니다.

---

## 0. 목표 / 성공 기준

1. `~/lio_sam_ws`에 패키지 빌드 → verify: `colcon build`가 에러 없이 끝남
2. KITTI 드라이브를 ROS2 bag으로 변환 → verify: `ros2 bag info`에 `/points`, `/imu/data` 토픽이 보임
3. `ros2 launch lio_sam run.launch.py` + `ros2 bag play`로 재생 → verify: rviz2에 지도(포인트 클라우드)가 누적되며 그려짐

---

## 1. 의존성 설치

```bash
sudo apt install ros-humble-perception-pcl \
                  ros-humble-pcl-msgs \
                  ros-humble-vision-opencv \
                  ros-humble-xacro

sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

**verify**: `dpkg -l | grep libgtsam-dev` 로 설치 확인

## 2. 독립 워크스페이스 빌드

```bash
mkdir -p ~/lio_sam_ws/src
ln -s /home/rokey/b3_cobot3_ws/LIO-SAM-ROS2-main ~/lio_sam_ws/src/lio_sam
cd ~/lio_sam_ws
colcon build --symlink-install
source install/setup.bash
```

`b3_cobot3_ws`의 원본 폴더를 심볼릭 링크로 연결하므로, 소스를 고치면 두 워크스페이스
어디서 빌드해도 같은 코드를 참조합니다.

**verify**: `ros2 pkg list | grep lio_sam` → `lio_sam` 출력됨

## 3. KITTI 데이터 다운로드

작고 널리 쓰이는 시퀀스인 `2011_09_26` / drive `0084`를 기준으로 설명합니다
(kitti2bag.py의 README에도 나오는 예제입니다). **sync + extract 합쳐 약 3.9GB**이니
디스크/네트워크를 확인하세요.

```bash
mkdir -p ~/kitti_data && cd ~/kitti_data
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0084/2011_09_26_drive_0084_sync.zip
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0084/2011_09_26_drive_0084_extract.zip
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_calib.zip
unzip 2011_09_26_drive_0084_sync.zip
unzip 2011_09_26_drive_0084_extract.zip
unzip 2011_09_26_calib.zip
```

(다른 시퀀스를 쓰고 싶으면 [KITTI raw data 페이지](http://www.cvlibs.net/datasets/kitti/raw_data.php)에서
`date`/`drive` 번호만 바꾸면 됩니다.)

**verify**: `ls ~/kitti_data/2011_09_26` 안에 `2011_09_26_drive_0084_sync`, `_extract` 폴더가 보임

## 4. ROS2 bag으로 변환

```bash
pip3 install --user rosbags
python3 ~/lio_sam_ws/src/lio_sam/config/doc/kitti2bag/kitti2rosbag2.py \
    --kitti_dir ~/kitti_data --date 2011_09_26 --drive 0084 \
    --output ~/kitti_data/kitti_2011_09_26_0084.bag
```

**verify**:
```bash
ros2 bag info ~/kitti_data/kitti_2011_09_26_0084.bag
```
`/points` (`sensor_msgs/msg/PointCloud2`)와 `/imu/data` (`sensor_msgs/msg/Imu`) 두 토픽,
메시지 개수 > 0 이 보이면 성공입니다.

## 5. params.yaml을 KITTI/Velodyne HDL-64E에 맞게 수정

`~/lio_sam_ws/src/lio_sam/config/params.yaml`에서 아래 값을 바꾸세요. 앞의 3개
(`extrinsic*`)와 `N_SCAN`, `downsampleRate`는 원래 README의 KITTI 섹션에 있던 값이고,
**`sensor: velodyne`과 `Horizon_SCAN: 1800`은 이 튜토리얼에서 추가로 필요해서 넣은
값**입니다 (기본값 `sensor: ouster`인 채로 두면 point cloud 필드 구성이 안 맞아서
`imageProjection`이 ring 채널을 못 읽습니다).

```yaml
sensor: velodyne          # 기본값 ouster에서 변경 (필수)
N_SCAN: 64
Horizon_SCAN: 1800        # Velodyne 기본값 (파일 내 주석 참고)
downsampleRate: 2         # 또는 4, 포인트가 너무 많으면 늘리기

extrinsicTrans: [-8.086759e-01, 3.195559e-01, -7.997231e-01]
extrinsicRot: [9.999976e-01, 7.553071e-04, -2.035826e-03, -7.854027e-04, 9.998898e-01, -1.482298e-02, 2.024406e-03, 1.482454e-02, 9.998881e-01]
extrinsicRPY: [9.999976e-01, 7.553071e-04, -2.035826e-03, -7.854027e-04, 9.998898e-01, -1.482298e-02, 2.024406e-03, 1.482454e-02, 9.998881e-01]
```

참고: KITTI 포인트 클라우드에는 포인트별 상대 시간(`time` 필드)이 없어서 deskew(왜곡 보정)가
자동으로 꺼집니다(`imageProjection`이 경고 로그를 띄웁니다). README에도 명시된 KITTI의
알려진 한계이며, 데모용으로는 문제 없이 동작합니다.

## 6. 실행

**주의**: `run.launch.py`의 `params_file` 기본값은 `params.yaml`이 아니라
`params_rs16.yaml`입니다 (토픽명도 `/sensing/lidar/top/rslidar_sdk/rs/points`,
`/chc/imu`로 완전히 다름). 5단계에서 고친 `params.yaml`을 쓰려면 반드시
`params_file`을 명시해야 합니다. 안 하면 bag을 재생해도 아무 노드도 메시지를
받지 못해 rviz가 빈 화면으로 남습니다.

터미널 A:
```bash
cd ~/lio_sam_ws && source install/setup.bash
ros2 launch lio_sam run.launch.py params_file:=$(pwd)/install/lio_sam/share/lio_sam/config/params.yaml
```

터미널 B:
```bash
source /opt/ros/humble/setup.bash
ros2 bag play ~/kitti_data/kitti_2011_09_26_0084.bag --rate 1
```

**verify**: rviz2 창에서 "Map (cloud)"에 포인트 클라우드가 누적되고, 차량 궤적(odometry)이
그려지면 성공입니다.

## 7. 맵 저장 (선택)

```bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2, destination: /Downloads/kitti_map/}"
```

## 8. USD로 변환 (선택)

`config/doc/pcd2usd/pcd_to_usd.py`로 저장된 `.pcd`를 `.usd` 포인트 클라우드로 변환할 수 있습니다.
`usd-core`(pxr)와 PCL CLI 도구(`pcl_convert_pcd_ascii_binary`)가 이미 설치되어 있어야 하며,
바이너리/ASCII PCD 모두 지원합니다.

```bash
python3 ~/lio_sam_ws/src/lio_sam/config/doc/pcd2usd/pcd_to_usd.py \
    ~/Downloads/kitti_map/GlobalMap.pcd ~/Downloads/kitti_map/GlobalMap.usd
```

- intensity 필드가 있으면 흑백 `displayColor`로 매핑되어 usdview/Omniverse 등에서 바로 형태가 보입니다.
- `--point-size`로 점 크기 조절 가능 (기본 0.05).

**verify**: `usdview ~/Downloads/kitti_map/GlobalMap.usd` (또는 Omniverse/Blender의 USD 임포터)로 열어서
지도가 보이면 성공입니다.

---

## 트러블슈팅

- **`mapOptmization` 크래시**: 대부분 GTSAM 버전 문제. 1단계의 PPA로 설치한 버전인지 확인.
- **로그에 "ring channel not available"**: `params.yaml`의 `sensor`가 `velodyne`인지 다시 확인.
- **base_link가 튀어오름**: extrinsic 값이 틀렸을 가능성. 5단계 값을 그대로 복사했는지 확인.

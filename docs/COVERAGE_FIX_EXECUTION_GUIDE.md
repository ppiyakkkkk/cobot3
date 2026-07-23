# 커버리지 시각화 버그 수정 실행 명령

## 터미널 1 — 빌드 + 테스트

```bash
cd ~/b3_cobot3_ws
ros_setup
bash scripts/build_ros2.sh
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/forest_rescue_system/test/ -q
```

## 터미널 2 — terrain mesh 재생성 (Bug 2)

```bash
cd ~/b3_cobot3_ws
ros_setup
isaac_ros_setup
isaac_python isaac_sim/final_24.py
```

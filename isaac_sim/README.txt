final_24.py 리팩터링 파일 구성
================================

실행:
    isaac_python final_24.py

배치:
    아래 파일들을 기존 isaac_sim 디렉터리에 모두 함께 둡니다.
    worlds/my_forest.usd 경로는 기존 구조를 그대로 사용합니다.

파일 역할:
- final_24.py       : SimulationApp 생성, 전체 초기화 순서, 메인 루프
- sim_config.py     : 카메라/드론/사람/경로/파일 경로 설정값
- sim_terrain.py    : Terrain 높이 보간, RViz Terrain 및 환경 Mesh 추출
- sim_utils.py      : 수색 경로 JSON, Ground Truth, 장면 조명/검증
- sim_people.py     : 조난자·구조자 생성, 캡슐 충돌체 동기화
- sim_drone.py      : Iris/PX4/ROS 카메라/RTX LiDAR 구성
- sim_viewports.py  : 센서 Viewport 도킹, 메인 추적 카메라

주의:
- final_24.py만 실행합니다. 나머지 파일은 import되는 보조 모듈입니다.
- sim_config.py 외의 Isaac/Pegasus 관련 모듈은 SimulationApp 생성 이후
  import되도록 final_24.py에서 순서를 유지했습니다.

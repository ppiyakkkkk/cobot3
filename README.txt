수정 내용
- 감속 실패 시 새 수평/수직 회피 명령 중단
- VFH 후보 검사 예산 계산 보강
- A* 시작영역에서 원본 장애물 셀 보존
- 시뮬 시간 되감기 시 회피 방향 선호 초기화
- eval_coverage ray 최대거리 13m
- coverage visualization exact TF 우선 + 최신 TF 0.15초 제한 fallback

적용:
  python3 tools/apply_all_fixes.py --workspace ~/b3_cobot3_ws
  cd ~/b3_cobot3_ws
  colcon build --symlink-install --packages-select forest_rescue_system
  source install/setup.bash

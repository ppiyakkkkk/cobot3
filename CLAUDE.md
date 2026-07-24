# CLAUDE.md

이 저장소는 팀 프로젝트입니다(`Forest Rescue Multi-Drone System`, README.md 참고).
로컬 작업은 항상 팀원과의 통합을 전제로 합니다. 아래 규칙을 따르세요.
이 md는 feat/lio-sam-integration에만 적용된다.

## 1. 저장소 구조 유지

- 폴더 구조는 README.md "5. 저장소 구조"를 기준으로 합니다. 구조를 바꾸기 전에 README를 먼저 확인하세요.
- 새 코드는 기존 구조 안에 추가합니다. 새 최상위 폴더를 만들지 마세요.
  - ROS 2 패키지 → `src/<package_name>/` (예: `forest_rescue_system`, `forest_rescue_interfaces`, `lio_sam`)
  - Isaac Sim 스크립트/월드 → `isaac_sim/`
  - 문서 → `docs/`
  - 테스트/유틸 스크립트 → `scripts/`
- 기존 폴더나 파일을 리네임·이동·삭제하지 마세요. 꼭 필요하면 먼저 사용자에게 확인을 구합니다.
- 최상위에 새 폴더가 필요해 보이면, 먼저 기존 폴더(`isaac_sim/`, `src/`, `scripts/`, `docs/`) 안에 통합할 수 있는지 검토하고, 정말 필요할 때만 사용자와 상의 후 추가하세요. (예: `isaac_sim/` 옆에 별도 폴더를 새로 만들지 말고 `isaac_sim/` 하위에 넣을 방법을 먼저 고려)

## 2. Git / 협업

- `main`에 직접 커밋·push하지 않습니다. 작업은 `feat/*` 브랜치에서 진행합니다(README "17. Git 브랜치 생성 및 업로드" 참고).
- `build/`, `install/`, `log/`, `models/`, `generated_search_plan.json`, `__pycache__/` 등 자동 생성 파일은 커밋하지 않습니다(`.gitignore`에 등록됨. 커밋 전 `git status`로 재확인).
- `git add -A` / `git add .` 대신 필요한 파일만 지정해서 add 합니다.
- 커밋, push, 브랜치 삭제 등 팀과 공유되는 상태에 영향을 주는 작업은 실행 전 사용자에게 확인합니다.
- 대용량 바이너리(`.usd`, `.pt`, `.bag` 등)는 직접 커밋하지 않고 README "18. 대용량 모델 공유 방법"을 따릅니다.

## 3. 코드 스타일

- 각 ROS 2 패키지(`config/`, `launch/`, 노드 파일)의 기존 네이밍과 스타일을 그대로 따릅니다. 새로운 컨벤션을 임의로 도입하지 않습니다.

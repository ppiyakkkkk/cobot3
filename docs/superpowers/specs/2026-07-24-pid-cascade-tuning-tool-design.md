# PX4 캐스케이드 PID 수동 튜닝 스크립트 — 설계

## 배경 / 목적

`PX4_비행제어_캐스케이드_파라미터_정리.md`에서 확인했듯 이 프로젝트는 PX4의 4단 캐스케이드(1.Position→2.Velocity→3.Attitude→4.Rate) 중 1~2단계의 속도/가속도 상한과 위치 P만 오버라이드하고, 3~4단계(Attitude/Rate)는 PX4 기본값을 그대로 쓴다. 드론을 더 빠르게 비행시키기 위해 4단계 전부를 대상으로 튜닝하되, 이론치가 아니라 사용자가 Isaac Sim(Pegasus + PX4 SITL)에서 직접 여러 값을 반복 실험하며 순차적으로(안쪽 4단계→바깥쪽 1단계 순서) 값을 결정한다.

이 문서는 그 반복 실험을 도와줄 도구의 설계다.

## 핵심 아이디어

MAVSDK `Offboard`는 `PositionNedYaw` 외에도 `VelocityBodyYawspeed`/`VelocityNedYaw`, `Attitude`, `AttitudeRate`를 직접 지원한다. 즉 캐스케이드의 바깥 단계를 우회하고 원하는 단계에 스텝 자극을 직접 주입할 수 있다.

| 단계 | 자극(MAVSDK 호출) | 우회하는 바깥 단계 | 측정 텔레메트리 |
|---|---|---|---|
| 4. Rate | `offboard.set_attitude_rate(AttitudeRate(...))` 스텝 | 1~3단계 전부 | `telemetry.attitude_angular_velocity_body()` |
| 3. Attitude | `offboard.set_attitude(Attitude(...))` 스텝 | 1~2단계 (4단계는 거침) | `telemetry.attitude_euler()` |
| 2. Velocity | `offboard.set_velocity_body(VelocityBodyYawspeed(...))` 스텝 | 1단계 (3~4단계는 거침) | `telemetry.velocity_ned()` / `position_velocity_ned()` |
| 1. Position | `offboard.set_position_ned(PositionNedYaw(...))` 스텝 | 없음 (전체 캐스케이드 통과) | `telemetry.position_velocity_ned()` |

기존 `drone_controller_node.py`는 `PositionNedYaw`만 쓰기 때문에 단계별 격리 테스트가 불가능하다. 이 도구는 그 격리를 가능하게 하는 것이 핵심 가치다.

## 실행 환경 전제

- Isaac Sim + Pegasus + PX4 SITL은 튜닝 세션 내내 한 번만 기동한 채로 유지한다 (재기동 시 ~1분 로딩 발생, 세션 중에는 불필요).
- 드론 리셋은 시뮬레이터 재시작이 아니라 매 실행마다 "홈 호버 자세로 오프보드 복귀"로 처리한다 (수 초 단위).
- PX4 파라미터(`set_param_float`)는 실시간 반영되므로 재시작 불필요.
- 대상: 4대 중 1대(기본 `udpin://0.0.0.0:14540`, `--system-address`로 변경 가능).

## 구조

기존 `scripts/` 디렉토리의 독립 MAVSDK 스크립트 컨벤션(`01_mavsdk_takeoff_test.py` 등: ROS 없이 `asyncio.run(main())` 기반 평범한 async 함수)을 따르되, 모듈로 분리한다.

```
scripts/pid_tuning/
├── mavsdk_client.py      # connect / wait_for_connection / wait_for_health / arm+takeoff / offboard 시작 / param set-get
├── stimulus.py           # stage×axis별 스텝 자극 생성 (위 표의 4가지)
├── telemetry_logger.py   # 자극 구간 동안 관련 텔레메트리 비동기 수집 (baseline 0.5초 포함)
├── metrics.py            # 오버슈트%, rise time, settling time 계산 (순수 함수)
├── plot.py               # setpoint vs 실측 PNG 생성
├── run_ledger.py         # 실행 결과 1건을 CSV 한 줄로 append
├── runs/                 # PNG + ledger.csv 저장 위치 (.gitignore에 추가, 실행 결과물이라 커밋 대상 아님)
└── tune_cascade.py       # CLI 진입점, 위 모듈 조합
```

## CLI 인터페이스

```bash
python scripts/pid_tuning/tune_cascade.py \
    --stage rate --axis roll \
    --set MC_ROLLRATE_P=0.18 MC_ROLLRATE_D=0.004 \
    --step-mag 30 \
    --duration 3.0 \
    --system-address udpin://0.0.0.0:14540
```

| 인자 | 값 | 필수 | 기본값 |
|---|---|---|---|
| `--stage` | `rate \| attitude \| velocity \| position` | Y | - |
| `--axis` | `roll\|pitch\|yaw` (rate/attitude) 또는 `horizontal\|vertical` (velocity/position) | Y | - |
| `--set` | `PARAM=VALUE` 공백구분 다중 허용 | N (0개도 허용, 현재값으로만 재측정 가능) | - |
| `--step-mag` | 자극 크기 | N | stage별 기본값(아래) |
| `--duration` | 자극 유지 + 로깅 시간(초) | N | 3.0 |
| `--system-address` | MAVSDK 연결 주소 | N | `udpin://0.0.0.0:14540` |

**stage별 기본 `--step-mag`**: rate=30(deg/s), attitude=10(deg), velocity=1.0(m/s), position=3.0(m)

파라미터 값 자체에 대한 범위 검증(음수/과도한 값 차단)은 넣지 않는다 — 사용자가 직접 판단하며 실험하는 도구이므로 방어 로직으로 실험 자유도를 제한하지 않는다 (사용자 확인 완료).

## 데이터 흐름 (매 실행 공통 시퀀스)

```
1. connect(system_address) + wait_for_connection (timeout 30s)
2. wait_for_health (global/home position ok, timeout 60s)
3. 미비행 상태면 arm + takeoff(canonical altitude, 기본 5m) + offboard 시작
   이미 offboard 비행 중이면 skip
4. 홈 호버 복귀: PositionNedYaw(0, 0, -home_altitude, yaw=0) →
   위치 오차 tolerance(0.15m) 이내 안정될 때까지 대기 (timeout 20s)
   → 이전 실행 상태와 무관하게 항상 동일한 시작 조건 보장
5. --set 파라미터들을 param.set_param_float로 적용, 적용값 콘솔 출력
6. stimulus.py로 stage×axis에 맞는 스텝 자극 시작 + telemetry_logger.py로 해당 채널 구독 시작
   (baseline 0.5초 선행 기록)
7. --duration 동안 자극 유지, 텔레메트리 계속 수집 (asyncio.wait_for로 전체를 감싸 duration 초과 시 강제 종료)
8. 자극 종료 → 4번과 동일하게 홈 호버 복귀 (다음 실행을 위한 리셋 겸용)
9. metrics.py로 지표 계산 → plot.py로 PNG 저장 → run_ledger.py로 CSV append →
   콘솔 요약 출력 → disconnect
```

## 지표 계산 (metrics.py)

- **overshoot %**: `(peak - final_setpoint) / (final_setpoint - initial) * 100`, 0 이하면 오버슈트 없음
- **rise time**: 10%→90% 도달 시간
- **settling time**: 최종값 ±5% 밴드에 재진입 없이 안착한 시점까지 시간
- 발산(밴드 재진입 안 함)하는 경우 `settling_time = None`으로 표시, 억지로 숫자를 만들지 않음

## 플롯 (plot.py)

- setpoint(점선) vs 실측(실선) 그래프, 제목에 이번 실행의 `--set` 요약 표시
- 파일명: `{stage}_{axis}_{YYYYMMDD_HHMMSS}.png`, 저장 위치 `scripts/pid_tuning/runs/`

## 이력 (run_ledger.py)

`scripts/pid_tuning/runs/ledger.csv`에 한 줄 append:

```
timestamp, stage, axis, set_params(JSON), overshoot_pct, rise_time_s, settling_time_s, plot_path
```

특정 파라미터로 검색해서 지금까지 시도한 값과 결과를 비교할 수 있게 하는 것이 목적.

## 에러 처리

`ParamError`(PX4 빌드에 없는 파라미터명), `OffboardError`(오프보드 거부), `asyncio.TimeoutError`(연결/헬스/안정화 타임아웃) 3종을 캐치하여 원인을 콘솔에 출력하고, 가능하면 홈 호버 복귀를 한 번 더 시도한 뒤 종료한다. 이 이상의 자동 복구(재시도 등)는 넣지 않는다 — 실패 시 사용자가 직접 보고 판단.

## 테스트 계획

- `metrics.py`: 순수 함수이므로 pytest 단위 테스트. 합성 시계열(1차 지연 응답, 오버슈트 있는 2차 응답)로 overshoot/rise/settling 계산 검증.
- 나머지 모듈(`mavsdk_client`, `stimulus`, `telemetry_logger`, `plot`, `run_ledger`, `tune_cascade`)은 실제 PX4 SITL 연결이 있어야 의미 있게 검증되므로 자동 테스트 대상에서 제외. 사용자가 Isaac Sim에서 1회 시험 실행으로 직접 검증.

## 스코프 밖 (Out of scope)

- `NAV_MC_ALT_RAD`, `MPC_TILTMAX_AIR`, `MC_YAW_WEIGHT` 등 캐스케이드 게인이 아닌 파라미터의 자동 튜닝
- 파라미터 값에 대한 자동 안전 범위 검증/차단
- 다중 드론 동시 튜닝 (한 번에 1대)
- 자동 재시도/복구 로직

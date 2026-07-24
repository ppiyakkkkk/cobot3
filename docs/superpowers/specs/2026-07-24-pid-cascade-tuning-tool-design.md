# PX4 캐스케이드 PID 수동 튜닝 스크립트 — 설계

## 배경 / 목적

`PX4_비행제어_캐스케이드_파라미터_정리.md`에서 확인했듯 이 프로젝트는 PX4의 4단 캐스케이드(1.Position→2.Velocity→3.Attitude→4.Rate) 중 1~2단계의 속도/가속도 상한과 위치 P만 오버라이드하고, 3~4단계(Attitude/Rate)는 PX4 기본값을 그대로 쓴다. 드론을 더 빠르게 비행시키기 위해 4단계 전부를 대상으로 튜닝하되, 이론치가 아니라 사용자가 Isaac Sim(Pegasus + PX4 SITL)에서 직접 여러 값을 반복 실험하며 순차적으로(안쪽 4단계→바깥쪽 1단계 순서) 값을 결정한다.

이 문서는 그 반복 실험을 도와줄 도구의 설계다.

## 핵심 아이디어

MAVSDK `Offboard`는 `PositionNedYaw` 외에도 `VelocityBodyYawspeed`/`VelocityNedYaw`, `Attitude`, `AttitudeRate`를 직접 지원한다. 즉 캐스케이드의 바깥 단계를 우회하고 원하는 단계에 스텝 자극을 직접 주입할 수 있다.

| 단계 | 자극(MAVSDK 호출) | 우회하는 바깥 단계 | 측정 텔레메트리 |
|---|---|---|---|
| 4. Rate | `offboard.set_attitude_rate(AttitudeRate(...))` 스텝 (단위 **deg/s**) | 1~3단계 전부 | `telemetry.attitude_angular_velocity_body()` (단위 **rad/s** — metrics.py에서 deg/s로 환산 필요) |
| 3. Attitude | `offboard.set_attitude(Attitude(...))` 스텝 (단위 deg) | 1~2단계 (4단계는 거침) | `telemetry.attitude_euler()` (단위 deg) |
| 2. Velocity | `offboard.set_velocity_body(VelocityBodyYawspeed(...))` 스텝 | 1단계 (3~4단계는 거침) | `telemetry.velocity_ned()` / `position_velocity_ned()` |
| 1. Position | `offboard.set_position_ned(PositionNedYaw(...))` 스텝 | 없음 (전체 캐스케이드 통과) | `telemetry.position_velocity_ned()` |

기존 `drone_controller_node.py`는 `PositionNedYaw`만 쓰기 때문에 단계별 격리 테스트가 불가능하다. 이 도구는 그 격리를 가능하게 하는 것이 핵심 가치다.

> **검증 완료 (mavsdk 3.15.3 소스 확인)**: 위 5개 메서드와 dataclass 필드는 실제로 존재한다(`offboard.py` L943/998/1024/1076). 다만 `attitude_angular_velocity_body()`와 `attitude_euler()`는 동일한 MAVLink `ATTITUDE` 메시지를 디코드하는 것이라 **별도 스트림이 아니다** — 텔레메트리 rate 설정도 `set_rate_attitude_euler()` 하나로 Rate/Attitude 두 단계 모두에 적용된다. 아래 "텔레메트리 rate 설정" 절 참고.
>
> **미검증 항목 (근거 약함)**: PositionNedYaw ↔ Attitude/AttitudeRate 간 오프보드 세션 중 setpoint 타입 전환이 PX4에서 실제로 매끄러운지는 mavsdk 파이썬 소스 레벨에서 막는 코드가 없다는 것만 확인했고, PX4 펌웨어 단의 실동작은 확인하지 못했다. 이 도구의 **첫 실행이 사실상의 검증**이므로, 테스트 계획 절에 명시한다.

## 실행 환경 전제

- Isaac Sim + Pegasus + PX4 SITL은 튜닝 세션 내내 한 번만 기동한 채로 유지한다 (재기동 시 ~1분 로딩 발생, 세션 중에는 불필요).
- 드론 리셋은 시뮬레이터 재시작이 아니라 매 실행마다 "홈 호버 자세로 오프보드 복귀"로 처리한다 (수 초 단위).
- PX4 파라미터(`set_param_float`)는 실시간 반영되므로 재시작 불필요. **검증 완료**: PX4 `main` 브랜치 파라미터 정의(yaml) 확인 결과 튜닝 대상 26개 게인(`MC_*RATE_P/I/D`, `MC_ROLL_P/PITCH_P/YAW_P/YAW_WEIGHT`, `MPC_XY_VEL_*_ACC`, `MPC_Z_VEL_*_ACC`, `MPC_XY_P`, `MPC_Z_P`, `MPC_XY_VEL_MAX`, `MPC_Z_VEL_MAX_UP/DN`, `MPC_ACC_HOR`) 전부 `reboot_required` 태그 없음. 단, PX4 공식 문서는 `MPC_XY_VEL_MAX`/`MPC_ACC_HOR`처럼 jerk-limited 궤적 생성기와 연관된 파라미터를 비행 중 변경하면 진행 중이던 궤적과 일시적 불연속이 생길 수 있다고 일반 경고한다 — 데이터 흐름의 "홈 호버 복귀 후 안정화 대기" 단계(4번)가 이를 흡수하므로 별도 조치는 불필요.
- 대상: 4대 중 1대(기본 `udpin://0.0.0.0:14540`, `--system-address`로 변경 가능).
- 기체는 Pegasus `Multirotor` + `ROBOTS["Iris"]` 표준 모델 그대로이며(`isaac_sim/sim_drone.py`의 `spawn_iris()`), 커스텀 질량/추력 오버라이드는 없다. 스텝 크기 판단 기준은 PX4 표준 Iris 기본 튜닝값이다.

## 구조

기존 `scripts/` 디렉토리의 독립 MAVSDK 스크립트 컨벤션(`01_mavsdk_takeoff_test.py` 등: ROS 없이 `asyncio.run(main())` 기반 평범한 async 함수)을 따르되, 모듈로 분리한다.

```
scripts/pid_tuning/
├── mavsdk_client.py      # connect / wait_for_connection / wait_for_health / arm+takeoff / offboard 시작 /
│                         # param set-get / get_hover_thrust() (MPC_THR_HOVER 조회, 실패시 0.5 fallback)
├── stimulus.py           # stage×axis별 스텝 자극 생성 (위 표의 4가지). attitude/rate 스텝의 thrust_value는
│                         # get_hover_thrust() 결과를 사용 (근거: MAVSDK 공식 예제 offboard_attitude.py도
│                         # "시뮬레이션 한정" 경고 아래 근사 thrust 하드코딩 — 확립된 정밀 계산법은 없음,
│                         # SITL 전용 도구이므로 이 근사가 허용 범위)
├── telemetry_logger.py   # 자극 구간 동안 관련 텔레메트리 비동기 수집 (baseline 0.5초 포함).
│                         # 수신 시각은 message-embedded timestamp가 아니라 time.monotonic()으로 통일
│                         # (AngularVelocityBody/VelocityNed에는 timestamp 필드 자체가 없음)
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

**stage별 기본 `--step-mag`**: rate=30(deg/s), attitude=10(deg), velocity=1.5(m/s), position=3.0(m)

(기본값 근거: 기체가 표준 Iris이고 `MPC_TILTMAX_AIR=45°`인 점을 감안하면 rate 30deg/s·attitude 10deg는 전복 위험 없이 뚜렷한 응답을 관찰할 수 있는 크기. velocity는 이 프로젝트가 실제 수색 비행에 쓰는 `search_horizontal_speed_m_s=1.6`과 맞추기 위해 애초 검토했던 1.0에서 1.5로 상향 — 실제 운용 대역을 더 잘 대표함. position 3.0m은 저속 구간에서도 약 2초 내 도달해 캐스케이드 전체 응답을 관찰하기에 합리적.)

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
6. 텔레메트리 rate 설정: stage가 rate/attitude면 set_rate_attitude_euler(100), velocity/position이면
   set_rate_velocity_ned/set_rate_position_velocity_ned(50) 호출 (SITL 단일 드론·로컬 루프백이라
   대역폭 제약 없음 — 기존 프로젝트의 5~10Hz는 4대 동시 운용을 고려한 보수치라 이 도구엔 그대로 안 씀).
   요청 rate가 실제로 반영됐는지 첫 수신 간격을 측정해 어긋나면 콘솔 경고만 출력(차단 아님)
7. stimulus.py로 stage×axis에 맞는 스텝 setpoint를 1회 전송 + telemetry_logger.py로 백그라운드
   asyncio.create_task로 해당 채널 구독 시작 (baseline 0.5초 선행 기록).
   재전송 루프는 두지 않음 — mavsdk-server가 마지막 setpoint를 20Hz로 자동 재전송함이 확인됐고
   (mavsdk offboard.py docstring), 기존 프로젝트의 set_position_ned 호출도 동일하게 1회 호출 후
   폴링만 하는 방식이라 이 컨벤션을 그대로 따름
8. --duration 동안 자극 유지, 텔레메트리 계속 수집 (asyncio.wait_for로 전체를 감싸 duration 초과 시 강제 종료).
   rate/attitude 단계에서는 수직속도 절대값이 임계치(예 3 m/s)를 넘으면 콘솔 경고만 출력 —
   hover thrust 근사치가 부정확해 고도가 발산하는 상황을 사용자가 즉시 알아챌 수 있도록 하되,
   자동 중단은 하지 않음(스코프 밖 항목인 "자동 안전 검증/차단"과 구분되는 정보성 로그일 뿐)
9. 자극 종료 → 4번과 동일하게 홈 호버 복귀 (다음 실행을 위한 리셋 겸용)
10. metrics.py로 지표 계산 → plot.py로 PNG 저장 → run_ledger.py로 CSV append →
    콘솔 요약 출력 → disconnect
```

## 구현 패턴 참고 (동시성)

`drone_controller_node.py`가 이미 쓰는 패턴(백그라운드 `asyncio.create_task`로 텔레메트리 `async for` 루프를 띄우고, 메인 흐름은 그 결과만 읽는 구조)을 그대로 재사용한다:

```python
async def run_stimulus_with_logging(drone, stimulus_fn, telemetry_stream, duration, baseline_s=0.5):
    samples = []
    async def collector():
        async for t in telemetry_stream:
            samples.append((time.monotonic(), t))
    task = asyncio.create_task(collector())
    await asyncio.sleep(baseline_s)
    try:
        await asyncio.wait_for(stimulus_fn(drone), timeout=duration)
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
    return samples
```

`stimulus_fn`은 위 표의 setpoint를 1회 전송(재전송 루프 없음, 이유는 데이터 흐름 7번 참고).

## 지표 계산 (metrics.py)

- **overshoot %**: `(peak - final_setpoint) / (final_setpoint - initial) * 100`, 0 이하면 오버슈트 없음
- **rise time**: 10%→90% 도달 시간
- **settling time**: 최종값 ±5% 밴드에 재진입 없이 안착한 시점까지 시간
- 발산(밴드 재진입 안 함)하는 경우 `settling_time = None`으로 표시, 억지로 숫자를 만들지 않음
- **단위 변환**: rate 단계는 setpoint가 deg/s, 텔레메트리(`AngularVelocityBody`)가 rad/s이므로 비교 전 반드시 한쪽으로 통일(설계상 deg/s로 통일 — PX4 파라미터/사용자 입력 관례와 일치)

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
- **첫 시험 실행에서 특히 확인할 것**: (1) position↔attitude/rate 간 오프보드 setpoint 타입 전환이 매끄러운지(코드 레벨로는 막는 로직이 없다는 것만 확인됨, PX4 펌웨어 실동작은 미검증), (2) `get_hover_thrust()`가 반환하는 `MPC_THR_HOVER` 근사값으로 attitude/rate 스텝 중 고도가 크게 발산하지 않는지, (3) 요청한 텔레메트리 rate(attitude 100Hz 등)가 실제로 반영되는지.

## 스코프 밖 (Out of scope)

- `NAV_MC_ALT_RAD`, `MPC_TILTMAX_AIR`, `MC_YAW_WEIGHT` 등 캐스케이드 게인이 아닌 파라미터의 자동 튜닝
- 파라미터 값에 대한 자동 안전 범위 검증/차단
- 다중 드론 동시 튜닝 (한 번에 1대)
- 자동 재시도/복구 로직

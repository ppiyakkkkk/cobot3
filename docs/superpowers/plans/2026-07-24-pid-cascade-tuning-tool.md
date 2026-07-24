# PX4 캐스케이드 PID 수동 튜닝 스크립트 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** PX4 SITL(Isaac Sim + Pegasus) 드론의 캐스케이드 PID 4단계(Rate/Attitude/Velocity/Position)를 MAVSDK Offboard의 서로 다른 setpoint 타입으로 개별 격리해서 스텝 응답 테스트하는 CLI 도구를 만든다.

**Architecture:** ROS2와 무관한 독립 asyncio 스크립트 모음(`scripts/pid_tuning/`). 각 실행은 연결 → 홈 호버 리셋 → 파라미터 적용 → 단일 stage/axis에 대한 스텝 setpoint 1회 전송 + 텔레메트리 동시 수집 → 지표 계산/플롯/이력 저장 → 홈 호버 복귀 → 종료의 1회성 완결 실행이다.

**Tech Stack:** Python 3.10, `mavsdk` 3.15.3(설치돼있음), `matplotlib`(설치돼있음, Agg 백엔드), 표준 라이브러리(`asyncio`, `argparse`, `csv`, `json`), `pytest`.

**설계 문서:** `docs/superpowers/specs/2026-07-24-pid-cascade-tuning-tool-design.md` (이 계획의 근거 전체가 여기 있음, 서브에이전트 6건 조사로 검증됨)

## Global Constraints

- 파라미터 값 자체에 대한 범위 검증(음수/과도한 값 차단)은 넣지 않는다 — 사용자가 직접 판단하는 도구.
- 다중 드론 동시 튜닝은 지원하지 않는다 (한 번에 1대, `--system-address`로 대상 선택).
- 자동 재시도/복구 로직은 넣지 않는다 — 실패 시 콘솔에 원인만 출력.
- `scripts/pid_tuning/runs/`(PNG + ledger.csv)는 실행 결과물이므로 `.gitignore`에 추가하고 커밋하지 않는다.
- 재전송 루프를 두지 않는다 — mavsdk-server가 마지막 setpoint를 20Hz로 자동 재전송하므로 각 stage의 setpoint는 1회만 전송한다.
- Rate 단계 setpoint는 deg/s, 텔레메트리(`AngularVelocityBody`)는 rad/s이므로 비교 전 deg/s로 통일한다.

---

## 파일 구조

```
scripts/pid_tuning/
├── mavsdk_client.py          # 연결/헬스체크/이륙/오프보드/파라미터 적용/호버 thrust 조회
├── stimulus.py                # stage×axis별 스텝 setpoint 생성 및 1회 전송
├── telemetry_logger.py        # stage별 텔레메트리 스트림/추출기/rate 설정 테이블
├── metrics.py                  # overshoot_pct / rise_time / settling_time (순수 함수)
├── plot.py                     # setpoint vs 실측 PNG 저장
├── run_ledger.py               # 실행 결과 CSV 이력 append
├── tune_cascade.py              # CLI 진입점, 위 모듈을 조합해 데이터 흐름 오케스트레이션
├── test_metrics.py
├── test_plot.py
├── test_run_ledger.py
└── test_tune_cascade_args.py
```

`scripts/` 디렉토리 컨벤션(`01_mavsdk_takeoff_test.py` 등: 패키지 없이 평범한 모듈, `python scripts/pid_tuning/tune_cascade.py`로 직접 실행)을 따른다. `tune_cascade.py`를 직접 실행하면 파이썬이 그 파일이 있는 디렉토리를 `sys.path[0]`에 자동으로 넣어주므로 `import mavsdk_client` 같은 평범한 import로 형제 모듈을 가져올 수 있다 — 별도 패키지 설정 불필요.

---

## Task 1: `.gitignore` 항목 추가 + `metrics.py` (TDD)

**Files:**
- Modify: `.gitignore` (프로젝트 루트)
- Create: `scripts/pid_tuning/metrics.py`
- Test: `scripts/pid_tuning/test_metrics.py`

**Interfaces:**
- Produces:
  - `overshoot_pct(setpoint_initial: float, setpoint_final: float, values: list[float]) -> float`
  - `rise_time(times: list[float], values: list[float], setpoint_initial: float, setpoint_final: float) -> float | None`
  - `settling_time(times: list[float], values: list[float], setpoint_initial: float, setpoint_final: float, band_pct: float = 5.0) -> float | None`
  - 세 함수 모두 `times[0]`이 자극 시작 시각(0초)인 것을 전제로 한다.

- [ ] **Step 1: `.gitignore`에 실행 결과물 디렉토리 추가**

`.gitignore` 파일 맨 아래에 추가:

```
scripts/pid_tuning/runs/
```

- [ ] **Step 2: 실패하는 테스트 작성**

`scripts/pid_tuning/test_metrics.py`:

```python
"""metrics.py 단위 테스트."""

import pytest

import metrics


def test_overshoot_pct_no_overshoot():
    values = [0.0, 5.0, 8.0, 9.5, 10.0]
    assert metrics.overshoot_pct(0.0, 10.0, values) == 0.0


def test_overshoot_pct_with_overshoot():
    values = [0.0, 4.0, 8.0, 10.0, 11.0, 12.0, 11.5, 10.8, 10.3, 10.1, 10.0]
    assert metrics.overshoot_pct(0.0, 10.0, values) == 20.0


def test_overshoot_pct_zero_delta():
    assert metrics.overshoot_pct(5.0, 5.0, [5.0, 5.0, 5.0]) == 0.0


def test_overshoot_pct_negative_step():
    values = [0.0, -4.0, -8.0, -10.0, -12.0, -11.0, -10.0]
    assert metrics.overshoot_pct(0.0, -10.0, values) == 20.0


def test_rise_time_basic():
    times = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
    values = [0.0, 1.0, 5.0, 9.0, 10.0, 10.0]
    assert metrics.rise_time(times, values, 0.0, 10.0) == pytest.approx(0.2)


def test_rise_time_never_reaches_90pct():
    times = [0.0, 0.1, 0.2]
    values = [0.0, 1.0, 2.0]
    assert metrics.rise_time(times, values, 0.0, 10.0) is None


def test_settling_time_settles():
    times = [0.0, 1.0, 2.0, 3.0, 4.0]
    values = [0.0, 12.0, 10.6, 9.9, 10.0]
    result = metrics.settling_time(times, values, 0.0, 10.0, band_pct=5.0)
    assert result == pytest.approx(3.0)


def test_settling_time_never_settles():
    times = [0.0, 0.5, 1.0, 1.5]
    values = [0.0, 15.0, 5.0, 15.0]
    assert metrics.settling_time(times, values, 0.0, 10.0, band_pct=5.0) is None


def test_settling_time_zero_delta():
    assert metrics.settling_time([0.0, 1.0], [5.0, 5.0], 5.0, 5.0) == 0.0
```

- [ ] **Step 3: 테스트 실패 확인**

Run: `cd scripts/pid_tuning && python -m pytest test_metrics.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'metrics'`

- [ ] **Step 4: 최소 구현 작성**

`scripts/pid_tuning/metrics.py`:

```python
"""step response 지표(오버슈트/rise time/settling time) 계산 — 순수 함수."""


def overshoot_pct(setpoint_initial, setpoint_final, values):
    """스텝 응답의 오버슈트를 퍼센트로 반환한다. 오버슈트가 없으면 0.0."""
    delta = setpoint_final - setpoint_initial
    if delta == 0:
        return 0.0
    if delta > 0:
        peak = max(values)
        overshoot = peak - setpoint_final
    else:
        peak = min(values)
        overshoot = setpoint_final - peak
    return max(0.0, overshoot / abs(delta) * 100.0)


def rise_time(times, values, setpoint_initial, setpoint_final):
    """10%->90% 도달 시간(초)을 반환한다. 도달하지 못하면 None."""
    delta = setpoint_final - setpoint_initial
    if delta == 0:
        return 0.0
    low = setpoint_initial + 0.1 * delta
    high = setpoint_initial + 0.9 * delta
    t_low = None
    t_high = None
    for t, v in zip(times, values):
        reached_low = v >= low if delta > 0 else v <= low
        if t_low is None and reached_low:
            t_low = t
        reached_high = v >= high if delta > 0 else v <= high
        if t_low is not None and t_high is None and reached_high:
            t_high = t
    if t_low is None or t_high is None:
        return None
    return t_high - t_low


def settling_time(times, values, setpoint_initial, setpoint_final, band_pct=5.0):
    """최종값 +-band_pct% 밴드에 재진입 없이 안착한 시각(초)을 반환한다.

    끝까지 밴드를 벗어난 채로 남으면(발산) None을 반환한다.
    """
    delta = setpoint_final - setpoint_initial
    if delta == 0:
        return 0.0
    band = abs(delta) * band_pct / 100.0
    lower = setpoint_final - band
    upper = setpoint_final + band
    last_outside_idx = -1
    for i, v in enumerate(values):
        if v < lower or v > upper:
            last_outside_idx = i
    if last_outside_idx == -1:
        return 0.0
    if last_outside_idx == len(values) - 1:
        return None
    return times[last_outside_idx + 1]
```

- [ ] **Step 5: 테스트 통과 확인**

Run: `cd scripts/pid_tuning && python -m pytest test_metrics.py -v`
Expected: 9 passed

- [ ] **Step 6: 커밋**

```bash
git add .gitignore scripts/pid_tuning/metrics.py scripts/pid_tuning/test_metrics.py
git commit -m "feat(pid_tuning): add step-response metrics calculation"
```

---

## Task 2: `mavsdk_client.py`

**Files:**
- Create: `scripts/pid_tuning/mavsdk_client.py`

**Interfaces:**
- Consumes: 없음 (최하위 모듈)
- Produces:
  - `HOME_ALTITUDE_M: float = 5.0` (모듈 상수, Task 3에서 import해서 사용)
  - `async def connect(system_address: str) -> mavsdk.System`
  - `async def wait_for_health(drone, timeout_sec: float = 60.0) -> None`
  - `async def ensure_flying(drone) -> None`
  - `async def ensure_offboard_started(drone) -> None`
  - `async def return_to_home_hover(drone, timeout_sec: float = 20.0) -> None`
  - `async def get_hover_thrust(drone) -> float`
  - `async def apply_params(drone, parameter_values: dict[str, float]) -> None`

이 파일은 실제 PX4 SITL 연결이 있어야 의미 있게 검증되므로 자동 테스트 대상에서 제외한다(설계 문서 결정). 대신 문법 오류만 정적으로 확인한다.

- [ ] **Step 1: 구현 작성**

`scripts/pid_tuning/mavsdk_client.py`:

```python
"""PX4 SITL 연결, 헬스체크, 이륙/오프보드 시작, 파라미터 적용을 담당한다."""

import asyncio

from mavsdk import System
from mavsdk.offboard import PositionNedYaw
from mavsdk.param import ParamError

HOME_ALTITUDE_M = 5.0
HOME_POSITION_TOLERANCE_M = 0.15
FALLBACK_HOVER_THRUST = 0.5


async def connect(system_address):
    drone = System()
    await drone.connect(system_address=system_address)
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[OK] PX4 연결 성공")
            break
    return drone


async def wait_for_health(drone, timeout_sec=60.0):
    async def _wait():
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("[OK] 헬스 체크 통과")
                return

    await asyncio.wait_for(_wait(), timeout=timeout_sec)


async def ensure_flying(drone):
    async for in_air in drone.telemetry.in_air():
        already_flying = in_air
        break

    if already_flying:
        return

    print(f"[STEP] 이륙 (고도 {HOME_ALTITUDE_M}m)")
    await drone.action.set_takeoff_altitude(HOME_ALTITUDE_M)
    await drone.action.arm()
    await drone.action.takeoff()

    async def _wait_airborne():
        async for position in drone.telemetry.position():
            if position.relative_altitude_m >= HOME_ALTITUDE_M - 0.5:
                return

    await asyncio.wait_for(_wait_airborne(), timeout=40.0)


async def ensure_offboard_started(drone):
    if await drone.offboard.is_active():
        return
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, -HOME_ALTITUDE_M, 0.0)
    )
    await drone.offboard.start()


async def return_to_home_hover(drone, timeout_sec=20.0):
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, -HOME_ALTITUDE_M, 0.0)
    )

    async def _wait_stable():
        async for pv in drone.telemetry.position_velocity_ned():
            position = pv.position
            error_m = (
                position.north_m ** 2
                + position.east_m ** 2
                + (position.down_m - (-HOME_ALTITUDE_M)) ** 2
            ) ** 0.5
            if error_m <= HOME_POSITION_TOLERANCE_M:
                return

    await asyncio.wait_for(_wait_stable(), timeout=timeout_sec)


async def get_hover_thrust(drone):
    try:
        value = await drone.param.get_param_float("MPC_THR_HOVER")
        if 0.1 < value < 0.9:
            return value
    except ParamError as error:
        print(f"[WARN] MPC_THR_HOVER 조회 실패, 기본값 사용: {error}")
    return FALLBACK_HOVER_THRUST


async def apply_params(drone, parameter_values):
    applied = []
    for name, value in parameter_values.items():
        await drone.param.set_param_float(name, value)
        applied.append(f"{name}={value}")
    print(f"[OK] 파라미터 적용: {', '.join(applied)}")
```

- [ ] **Step 2: 문법 확인**

Run: `python -m py_compile scripts/pid_tuning/mavsdk_client.py`
Expected: 출력 없음 (에러 없으면 성공)

- [ ] **Step 3: 커밋**

```bash
git add scripts/pid_tuning/mavsdk_client.py
git commit -m "feat(pid_tuning): add mavsdk connection/hover-reset/param helpers"
```

---

## Task 3: `stimulus.py`

**Files:**
- Create: `scripts/pid_tuning/stimulus.py`

**Interfaces:**
- Consumes: `mavsdk_client.HOME_ALTITUDE_M` (Task 2)
- Produces:
  - `DEFAULT_STEP_MAG: dict[str, float]` = `{"rate": 30.0, "attitude": 10.0, "velocity": 1.5, "position": 3.0}`
  - `async def send_step(drone, stage: str, axis: str, step_mag: float, hover_thrust: float) -> tuple[float, float]` — `(setpoint_initial, setpoint_final)` 반환. 반환값 좌표계는 `telemetry_logger.py`(Task 4)의 추출기와 반드시 일치해야 함: rate=deg/s, attitude=deg, velocity=m/s(수평=north 방향, 수직=상승 양수), position=m(수평=north 원점 기준, 수직=고도 절대값 `HOME_ALTITUDE_M` 기준).

- [ ] **Step 1: 구현 작성**

`scripts/pid_tuning/stimulus.py`:

```python
"""stage x axis별 스텝 setpoint를 생성하고 1회 전송한다."""

from mavsdk.offboard import (
    Attitude,
    AttitudeRate,
    PositionNedYaw,
    VelocityBodyYawspeed,
)

from mavsdk_client import HOME_ALTITUDE_M

DEFAULT_STEP_MAG = {
    "rate": 30.0,
    "attitude": 10.0,
    "velocity": 1.5,
    "position": 3.0,
}


async def send_step(drone, stage, axis, step_mag, hover_thrust):
    """스텝 setpoint를 1회 전송하고 (setpoint_initial, setpoint_final)을 반환한다.

    반환값은 telemetry_logger의 추출기와 같은 좌표계/단위를 쓴다.
    """
    if stage == "rate":
        rates = {"roll_deg_s": 0.0, "pitch_deg_s": 0.0, "yaw_deg_s": 0.0}
        rates[f"{axis}_deg_s"] = step_mag
        await drone.offboard.set_attitude_rate(
            AttitudeRate(thrust_value=hover_thrust, **rates)
        )
        return 0.0, step_mag

    if stage == "attitude":
        angles = {"roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0}
        angles[f"{axis}_deg"] = step_mag
        await drone.offboard.set_attitude(
            Attitude(thrust_value=hover_thrust, **angles)
        )
        return 0.0, step_mag

    if stage == "velocity":
        forward = step_mag if axis == "horizontal" else 0.0
        down = -step_mag if axis == "vertical" else 0.0
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(forward, 0.0, down, 0.0)
        )
        return 0.0, step_mag

    if stage == "position":
        if axis == "horizontal":
            north, down = step_mag, -HOME_ALTITUDE_M
            initial, final = 0.0, step_mag
        else:
            north, down = 0.0, -(HOME_ALTITUDE_M + step_mag)
            initial, final = HOME_ALTITUDE_M, HOME_ALTITUDE_M + step_mag
        await drone.offboard.set_position_ned(PositionNedYaw(north, 0.0, down, 0.0))
        return initial, final

    raise ValueError(f"알 수 없는 stage: {stage}")
```

- [ ] **Step 2: 문법 확인**

Run: `python -m py_compile scripts/pid_tuning/stimulus.py`
Expected: 출력 없음

- [ ] **Step 3: 커밋**

```bash
git add scripts/pid_tuning/stimulus.py
git commit -m "feat(pid_tuning): add per-stage step stimulus builder"
```

---

## Task 4: `telemetry_logger.py`

**Files:**
- Create: `scripts/pid_tuning/telemetry_logger.py`

**Interfaces:**
- Consumes: 없음 (drone 객체와 stage/axis 문자열만 받음)
- Produces:
  - `RATE_HZ: dict[str, float]` = `{"rate": 100.0, "attitude": 100.0, "velocity": 50.0, "position": 50.0}`
  - `STREAM_BUILDERS: dict[str, Callable[[drone], AsyncIterator]]`
  - `EXTRACTORS: dict[str, Callable[[msg, axis], float]]`
  - `async def set_telemetry_rate(drone, stage: str) -> None`

extractor 반환값은 Task 3의 `send_step` 반환값과 같은 좌표계여야 한다: rate=deg/s, attitude=deg, velocity(수평)=north_m_s, velocity(수직)=상승 양수(`-down_m_s`), position(수평)=north_m, position(수직)=고도(`-down_m`).

- [ ] **Step 1: 구현 작성**

`scripts/pid_tuning/telemetry_logger.py`:

```python
"""stage별 텔레메트리 스트림/추출기/rate 설정을 제공한다."""

import math

RATE_HZ = {
    "rate": 100.0,
    "attitude": 100.0,
    "velocity": 50.0,
    "position": 50.0,
}


async def set_telemetry_rate(drone, stage):
    hz = RATE_HZ[stage]
    if stage in ("rate", "attitude"):
        # attitude_euler와 attitude_angular_velocity_body는 동일한 MAVLink
        # ATTITUDE 메시지를 디코드하므로 rate 설정도 공유된다.
        await drone.telemetry.set_rate_attitude_euler(hz)
    elif stage == "velocity":
        await drone.telemetry.set_rate_velocity_ned(hz)
    else:
        await drone.telemetry.set_rate_position_velocity_ned(hz)


def _extract_rate(msg, axis):
    rad_s = {"roll": msg.roll_rad_s, "pitch": msg.pitch_rad_s, "yaw": msg.yaw_rad_s}[axis]
    return math.degrees(rad_s)


def _extract_attitude(msg, axis):
    return {"roll": msg.roll_deg, "pitch": msg.pitch_deg, "yaw": msg.yaw_deg}[axis]


def _extract_velocity(msg, axis):
    velocity = msg.velocity
    if axis == "horizontal":
        return velocity.north_m_s
    return -velocity.down_m_s


def _extract_position(msg, axis):
    position = msg.position
    if axis == "horizontal":
        return position.north_m
    return -position.down_m


STREAM_BUILDERS = {
    "rate": lambda drone: drone.telemetry.attitude_angular_velocity_body(),
    "attitude": lambda drone: drone.telemetry.attitude_euler(),
    "velocity": lambda drone: drone.telemetry.position_velocity_ned(),
    "position": lambda drone: drone.telemetry.position_velocity_ned(),
}

EXTRACTORS = {
    "rate": _extract_rate,
    "attitude": _extract_attitude,
    "velocity": _extract_velocity,
    "position": _extract_position,
}
```

- [ ] **Step 2: 문법 확인**

Run: `python -m py_compile scripts/pid_tuning/telemetry_logger.py`
Expected: 출력 없음

- [ ] **Step 3: 커밋**

```bash
git add scripts/pid_tuning/telemetry_logger.py
git commit -m "feat(pid_tuning): add per-stage telemetry stream/extractor tables"
```

---

## Task 5: `plot.py` (TDD)

**Files:**
- Create: `scripts/pid_tuning/plot.py`
- Test: `scripts/pid_tuning/test_plot.py`

**Interfaces:**
- Consumes: 없음
- Produces: `save_plot(stage, axis, times, values, setpoint_initial, setpoint_final, set_params) -> pathlib.Path`

- [ ] **Step 1: 실패하는 테스트 작성**

`scripts/pid_tuning/test_plot.py`:

```python
"""plot.py 단위 테스트."""

import plot


def test_save_plot_creates_png(tmp_path, monkeypatch):
    monkeypatch.setattr(plot, "RUNS_DIR", tmp_path)

    path = plot.save_plot(
        stage="rate",
        axis="roll",
        times=[0.0, 0.5, 1.0],
        values=[0.0, 15.0, 30.0],
        setpoint_initial=0.0,
        setpoint_final=30.0,
        set_params={"MC_ROLLRATE_P": 0.18},
    )

    assert path.exists()
    assert path.suffix == ".png"
    assert path.parent == tmp_path
    assert path.name.startswith("rate_roll_")
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd scripts/pid_tuning && python -m pytest test_plot.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'plot'`

- [ ] **Step 3: 구현 작성**

`scripts/pid_tuning/plot.py`:

```python
"""setpoint vs 실측 스텝 응답 그래프를 PNG로 저장한다."""

import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

RUNS_DIR = Path(__file__).parent / "runs"


def save_plot(stage, axis, times, values, setpoint_initial, setpoint_final, set_params):
    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = RUNS_DIR / f"{stage}_{axis}_{timestamp}.png"

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(times, values, label="실측", color="tab:blue")
    ax.axhline(setpoint_final, linestyle="--", color="tab:orange", label="setpoint")
    ax.axvline(0.0, linestyle=":", color="gray", linewidth=1)
    ax.set_xlabel("시간 (s, 0=자극 시작)")
    ax.set_ylabel(f"{stage}/{axis}")
    set_params_summary = ", ".join(f"{k}={v}" for k, v in set_params.items())
    ax.set_title(f"{stage}/{axis} step response\n{set_params_summary}")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd scripts/pid_tuning && python -m pytest test_plot.py -v`
Expected: 1 passed

- [ ] **Step 5: 커밋**

```bash
git add scripts/pid_tuning/plot.py scripts/pid_tuning/test_plot.py
git commit -m "feat(pid_tuning): add setpoint-vs-actual PNG plotting"
```

---

## Task 6: `run_ledger.py` (TDD)

**Files:**
- Create: `scripts/pid_tuning/run_ledger.py`
- Test: `scripts/pid_tuning/test_run_ledger.py`

**Interfaces:**
- Consumes: 없음
- Produces: `append_run(stage, axis, set_params, overshoot, rise_time, settling_time, plot_path) -> None`

- [ ] **Step 1: 실패하는 테스트 작성**

`scripts/pid_tuning/test_run_ledger.py`:

```python
"""run_ledger.py 단위 테스트."""

import csv

import run_ledger


def test_append_run_creates_header_once(tmp_path, monkeypatch):
    ledger_path = tmp_path / "ledger.csv"
    monkeypatch.setattr(run_ledger, "LEDGER_PATH", ledger_path)

    run_ledger.append_run(
        stage="rate", axis="roll", set_params={"MC_ROLLRATE_P": 0.18},
        overshoot=5.0, rise_time=0.2, settling_time=1.5, plot_path="a.png",
    )
    run_ledger.append_run(
        stage="rate", axis="roll", set_params={"MC_ROLLRATE_P": 0.20},
        overshoot=8.0, rise_time=0.15, settling_time=None, plot_path="b.png",
    )

    with open(ledger_path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    assert len(rows) == 2
    assert rows[0]["stage"] == "rate"
    assert rows[0]["set_params"] == '{"MC_ROLLRATE_P": 0.18}'
    assert rows[1]["settling_time_s"] == ""
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd scripts/pid_tuning && python -m pytest test_run_ledger.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'run_ledger'`

- [ ] **Step 3: 구현 작성**

`scripts/pid_tuning/run_ledger.py`:

```python
"""튜닝 실행 결과를 CSV 한 줄로 누적 기록한다."""

import csv
import datetime
import json
from pathlib import Path

LEDGER_PATH = Path(__file__).parent / "runs" / "ledger.csv"
FIELDNAMES = [
    "timestamp", "stage", "axis", "set_params",
    "overshoot_pct", "rise_time_s", "settling_time_s", "plot_path",
]


def append_run(stage, axis, set_params, overshoot, rise_time, settling_time, plot_path):
    LEDGER_PATH.parent.mkdir(parents=True, exist_ok=True)
    is_new = not LEDGER_PATH.exists()
    with open(LEDGER_PATH, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        if is_new:
            writer.writeheader()
        writer.writerow({
            "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
            "stage": stage,
            "axis": axis,
            "set_params": json.dumps(set_params, ensure_ascii=False),
            "overshoot_pct": overshoot,
            "rise_time_s": rise_time,
            "settling_time_s": settling_time if settling_time is not None else "",
            "plot_path": str(plot_path),
        })
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd scripts/pid_tuning && python -m pytest test_run_ledger.py -v`
Expected: 1 passed

- [ ] **Step 5: 커밋**

```bash
git add scripts/pid_tuning/run_ledger.py scripts/pid_tuning/test_run_ledger.py
git commit -m "feat(pid_tuning): add CSV run history ledger"
```

---

## Task 7: `tune_cascade.py` (CLI 진입점, 인자 파싱은 TDD)

**Files:**
- Create: `scripts/pid_tuning/tune_cascade.py`
- Test: `scripts/pid_tuning/test_tune_cascade_args.py`

**Interfaces:**
- Consumes:
  - `mavsdk_client.{connect, wait_for_health, ensure_flying, ensure_offboard_started, return_to_home_hover, get_hover_thrust, apply_params}` (Task 2)
  - `stimulus.{DEFAULT_STEP_MAG, send_step}` (Task 3)
  - `telemetry_logger.{RATE_HZ, STREAM_BUILDERS, EXTRACTORS, set_telemetry_rate}` (Task 4)
  - `metrics.{overshoot_pct, rise_time, settling_time}` (Task 1)
  - `plot.save_plot` (Task 5)
  - `run_ledger.append_run` (Task 6)
- Produces: `parse_set_params(pairs: list[str]) -> dict[str, float]`, `parse_args() -> argparse.Namespace` (테스트 대상, 다른 모듈 없음 — 최종 조합 지점)

인자 파싱(`parse_set_params`, `parse_args`)은 네트워크 연결 없이 순수하게 테스트 가능하므로 TDD로 작성한다. `run()`/`main()`은 실제 PX4 SITL 연결이 있어야 검증되므로 자동 테스트 대상에서 제외한다(Task 8에서 수동 검증).

- [ ] **Step 1: 실패하는 인자 파싱 테스트 작성**

`scripts/pid_tuning/test_tune_cascade_args.py`:

```python
"""tune_cascade.py 인자 파싱 단위 테스트."""

import pytest

import tune_cascade


def test_parse_set_params_basic():
    result = tune_cascade.parse_set_params(["MC_ROLLRATE_P=0.18", "MC_ROLLRATE_D=0.004"])
    assert result == {"MC_ROLLRATE_P": 0.18, "MC_ROLLRATE_D": 0.004}


def test_parse_set_params_empty():
    assert tune_cascade.parse_set_params([]) == {}


def test_parse_set_params_missing_value():
    with pytest.raises(ValueError):
        tune_cascade.parse_set_params(["MC_ROLLRATE_P"])


def test_parse_args_rejects_invalid_axis(monkeypatch):
    monkeypatch.setattr(
        "sys.argv", ["tune_cascade.py", "--stage", "rate", "--axis", "horizontal"]
    )
    with pytest.raises(SystemExit):
        tune_cascade.parse_args()


def test_parse_args_defaults(monkeypatch):
    monkeypatch.setattr(
        "sys.argv", ["tune_cascade.py", "--stage", "velocity", "--axis", "horizontal"]
    )
    args = tune_cascade.parse_args()
    assert args.step_mag == 1.5
    assert args.duration == 3.0
    assert args.set_params == {}
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd scripts/pid_tuning && python -m pytest test_tune_cascade_args.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'tune_cascade'`

- [ ] **Step 3: 전체 구현 작성**

`scripts/pid_tuning/tune_cascade.py`:

```python
#!/usr/bin/env python3
"""PX4 캐스케이드 PID 단계별 격리 스텝 응답 테스트 CLI."""

import argparse
import asyncio
import time

from mavsdk.offboard import OffboardError
from mavsdk.param import ParamError

import mavsdk_client
import metrics
import plot
import run_ledger
import stimulus
import telemetry_logger

AXIS_CHOICES = {
    "rate": ("roll", "pitch", "yaw"),
    "attitude": ("roll", "pitch", "yaw"),
    "velocity": ("horizontal", "vertical"),
    "position": ("horizontal", "vertical"),
}

BASELINE_SEC = 0.5
SAFETY_MARGIN_SEC = 5.0
ALTITUDE_DIVERGENCE_LIMIT_M_S = 3.0


def parse_set_params(pairs):
    result = {}
    for pair in pairs:
        name, _, value = pair.partition("=")
        if not value:
            raise ValueError(f"--set 값은 PARAM=VALUE 형식이어야 함: {pair}")
        result[name] = float(value)
    return result


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", required=True, choices=list(AXIS_CHOICES))
    parser.add_argument("--axis", required=True)
    parser.add_argument("--set", nargs="*", default=[], dest="set_pairs")
    parser.add_argument("--step-mag", type=float, default=None)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--system-address", default="udpin://0.0.0.0:14540")
    args = parser.parse_args()

    if args.axis not in AXIS_CHOICES[args.stage]:
        parser.error(
            f"--stage {args.stage}에는 --axis {AXIS_CHOICES[args.stage]} 중 하나만 허용됨"
        )
    args.set_params = parse_set_params(args.set_pairs)
    if args.step_mag is None:
        args.step_mag = stimulus.DEFAULT_STEP_MAG[args.stage]
    return args


async def collect_during_stimulus(drone, stage, axis, step_mag, hover_thrust, duration_sec):
    stream = telemetry_logger.STREAM_BUILDERS[stage](drone)
    extractor = telemetry_logger.EXTRACTORS[stage]
    samples = []

    async def _collector():
        async for msg in stream:
            samples.append((time.monotonic(), extractor(msg, axis)))

    task = asyncio.create_task(_collector())
    await asyncio.sleep(BASELINE_SEC)
    t0 = time.monotonic()
    setpoint_initial, setpoint_final = await stimulus.send_step(
        drone, stage, axis, step_mag, hover_thrust
    )
    await asyncio.wait_for(
        asyncio.sleep(duration_sec), timeout=duration_sec + SAFETY_MARGIN_SEC
    )
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass

    times = [t - t0 for t, _ in samples if t >= t0]
    values = [v for t, v in samples if t >= t0]
    return times, values, setpoint_initial, setpoint_final


async def watch_altitude_divergence(drone, duration_sec):
    async def _watch():
        async for pv in drone.telemetry.position_velocity_ned():
            vertical_speed = -pv.velocity.down_m_s
            if abs(vertical_speed) > ALTITUDE_DIVERGENCE_LIMIT_M_S:
                print(
                    f"[WARN] 수직속도 {vertical_speed:.1f} m/s — "
                    "hover thrust 근사가 부정확할 수 있음"
                )

    task = asyncio.create_task(_watch())
    await asyncio.sleep(duration_sec)
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass


async def run(args):
    drone = await mavsdk_client.connect(args.system_address)
    try:
        await mavsdk_client.wait_for_health(drone)
        await mavsdk_client.ensure_flying(drone)
        await mavsdk_client.ensure_offboard_started(drone)
        await mavsdk_client.return_to_home_hover(drone)

        if args.set_params:
            await mavsdk_client.apply_params(drone, args.set_params)

        await telemetry_logger.set_telemetry_rate(drone, args.stage)
        hover_thrust = await mavsdk_client.get_hover_thrust(drone)

        watch_duration = args.duration + BASELINE_SEC
        if args.stage in ("rate", "attitude"):
            _, (times, values, setpoint_initial, setpoint_final) = await asyncio.gather(
                watch_altitude_divergence(drone, watch_duration),
                collect_during_stimulus(
                    drone, args.stage, args.axis, args.step_mag, hover_thrust, args.duration
                ),
            )
        else:
            times, values, setpoint_initial, setpoint_final = await collect_during_stimulus(
                drone, args.stage, args.axis, args.step_mag, hover_thrust, args.duration
            )

        await mavsdk_client.return_to_home_hover(drone)
    except (ParamError, OffboardError, asyncio.TimeoutError) as error:
        print(f"[ERROR] {type(error).__name__}: {error}")
        try:
            await mavsdk_client.return_to_home_hover(drone)
        except (OffboardError, asyncio.TimeoutError):
            pass
        return

    expected_dt = 1.0 / telemetry_logger.RATE_HZ[args.stage]
    if len(times) >= 2:
        actual_dt = (times[-1] - times[0]) / (len(times) - 1)
        if actual_dt > expected_dt * 3:
            print(
                "[WARN] 요청한 텔레메트리 rate가 반영되지 않은 것으로 보임 "
                f"(기대 dt={expected_dt:.3f}s, 실제 dt={actual_dt:.3f}s)"
            )

    overshoot = metrics.overshoot_pct(setpoint_initial, setpoint_final, values)
    rise = metrics.rise_time(times, values, setpoint_initial, setpoint_final)
    settling = metrics.settling_time(times, values, setpoint_initial, setpoint_final)

    plot_path = plot.save_plot(
        args.stage, args.axis, times, values, setpoint_initial, setpoint_final, args.set_params
    )
    run_ledger.append_run(
        args.stage, args.axis, args.set_params, overshoot, rise, settling, plot_path
    )

    print(f"[RESULT] overshoot={overshoot:.1f}% rise_time={rise} settling_time={settling}")
    print(f"[RESULT] plot={plot_path}")


def main():
    args = parse_args()
    asyncio.run(run(args))


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 인자 파싱 테스트 통과 확인**

Run: `cd scripts/pid_tuning && python -m pytest test_tune_cascade_args.py -v`
Expected: 5 passed

- [ ] **Step 5: 전체 단위 테스트 스위트 통과 확인**

Run: `cd scripts/pid_tuning && python -m pytest -v`
Expected: 모든 테스트(metrics 9개 + plot 1개 + run_ledger 1개 + tune_cascade_args 5개 = 16개) passed

- [ ] **Step 6: 커밋**

```bash
git add scripts/pid_tuning/tune_cascade.py scripts/pid_tuning/test_tune_cascade_args.py
git commit -m "feat(pid_tuning): add tune_cascade CLI entry point"
```

---

## Task 8: Isaac Sim SITL 실환경 수동 검증

**Files:** 없음 (코드 변경 없음, 수동 검증 절차)

이 도구의 핵심 미검증 항목(설계 문서 "테스트 계획" 절)은 실제 PX4 SITL 연결 없이는 확인할 수 없다. Isaac Sim + Pegasus + PX4 SITL을 기동한 상태에서 사용자가 직접 아래를 확인한다.

- [ ] **Step 1: Position 단계로 기본 동작 확인 (가장 안전한 단계부터)**

Run: `python scripts/pid_tuning/tune_cascade.py --stage position --axis horizontal --duration 3.0`

확인할 것:
- 드론이 이륙 → 홈 호버 → 북쪽으로 3m 이동 → 다시 홈 호버로 복귀하는지
- `scripts/pid_tuning/runs/`에 PNG와 `ledger.csv`가 생성되는지
- 콘솔에 `[RESULT] overshoot=... rise_time=... settling_time=...`이 출력되는지

- [ ] **Step 2: Velocity 단계 확인**

Run: `python scripts/pid_tuning/tune_cascade.py --stage velocity --axis horizontal --duration 3.0`

확인할 것: 드론이 전진 속도 setpoint에 반응해서 가속하는지, 종료 후 정상적으로 홈 호버 복귀하는지.

- [ ] **Step 3: Attitude 단계 확인 — position↔attitude 전환 안전성 검증**

Run: `python scripts/pid_tuning/tune_cascade.py --stage attitude --axis roll --duration 2.0`

확인할 것 (설계 문서의 미검증 항목):
- PositionNedYaw로 홈 호버하던 상태에서 Attitude setpoint로 전환할 때 급격한 튐이나 오프보드 거부(`OffboardError`)가 발생하지 않는지
- `[WARN] 수직속도 ... m/s` 경고가 뜨는지(뜬다면 `get_hover_thrust()` 근사치가 부정확하다는 뜻이므로 `MC_THR_HOVER` 실제값과 비교 확인)
- attitude 종료 후 다시 position setpoint(홈 호버)로 정상 복귀하는지

- [ ] **Step 4: Rate 단계 확인**

Run: `python scripts/pid_tuning/tune_cascade.py --stage rate --axis roll --duration 1.5`

확인할 것: attitude 단계와 동일 + rate 100Hz 요청이 실제로 반영됐는지(`[WARN] 요청한 텔레메트리 rate가 반영되지 않은...`이 뜨지 않아야 함).

- [ ] **Step 5: 2개 파라미터 동시 지정 케이스 확인**

Run: `python scripts/pid_tuning/tune_cascade.py --stage rate --axis roll --set MC_ROLLRATE_P=0.18 MC_ROLLRATE_D=0.004 --duration 1.5`

확인할 것: `--set` 다중 인자가 실제로 PX4에 반영되는지(콘솔의 `[OK] 파라미터 적용: ...` 로그 확인), `ledger.csv`의 `set_params` 컬럼에 JSON으로 정확히 기록되는지.

- [ ] **Step 6: 위 5개 단계에서 발견된 문제를 설계 문서/코드에 반영**

문제가 있었다면 (예: setpoint 전환 시 불안정, hover thrust 근사 부정확 등) `docs/superpowers/specs/2026-07-24-pid-cascade-tuning-tool-design.md`에 발견 사항을 추가하고 해당 모듈을 수정한다. 문제없었다면 이 단계는 생략.

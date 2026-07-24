# PID 캐스케이드 튜닝 스크립트

PX4 SITL(Isaac Sim + Pegasus)에 붙어서 캐스케이드 4단계(Rate → Attitude → Velocity → Position)를 하나씩 격리해 스텝 응답을 테스트하는 CLI 도구.

- 설계 문서: `docs/superpowers/specs/2026-07-24-pid-cascade-tuning-tool-design.md`
- 구현 계획: `docs/superpowers/plans/2026-07-24-pid-cascade-tuning-tool.md`

## 사용법

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
| `--set` | `PARAM=VALUE` 공백구분 다중 허용 | N | - |
| `--step-mag` | 자극 크기 | N | rate=30, attitude=10, velocity=1.5, position=3.0 |
| `--duration` | 자극 유지 + 로깅 시간(초) | N | 3.0 |
| `--system-address` | MAVSDK 연결 주소 | N | `udpin://0.0.0.0:14540` |

한 번 실행 = 한 번의 완결된 테스트: 연결 → 홈 호버 리셋 → 파라미터 적용(있으면) → 스텝 자극 + 텔레메트리 수집 → 홈 호버 복귀 → 지표 계산/플롯/이력 저장 → 종료. 시뮬레이터는 세션 내내 켜둔 채로 반복 실행하면 됨(재기동 불필요).

결과는 `scripts/pid_tuning/runs/`에 저장됨(PNG + `ledger.csv`, 둘 다 gitignore 대상).

## 실환경 수동 검증 체크리스트

코드 구현과 자동 테스트(16개, `metrics`/`plot`/`run_ledger`/인자 파싱)는 전부 완료·리뷰 승인됐지만, 실제 PX4 SITL 연결이 있어야만 확인 가능한 항목이 남아있음. Isaac Sim + Pegasus + PX4 SITL을 기동한 상태에서 아래 순서대로 실행.

- [ ] **1. Position 단계 — 가장 안전한 단계부터 기본 동작 확인**

  ```bash
  python scripts/pid_tuning/tune_cascade.py --stage position --axis horizontal --duration 3.0
  ```

  확인: 이륙 → 홈 호버 → 북쪽 3m 이동 → 홈 호버 복귀. `runs/`에 PNG + `ledger.csv` 생성. 콘솔에 `[RESULT] overshoot=... rise_time=... settling_time=...` 출력.

- [ ] **2. Velocity 단계**

  ```bash
  python scripts/pid_tuning/tune_cascade.py --stage velocity --axis horizontal --duration 3.0
  ```

  확인: 전진 속도 setpoint에 반응해 가속, 종료 후 정상적으로 홈 호버 복귀.

- [ ] **3. Attitude 단계 — position↔attitude 전환 안전성 (가장 중요한 미검증 항목)**

  ```bash
  python scripts/pid_tuning/tune_cascade.py --stage attitude --axis roll --duration 2.0
  ```

  확인: PositionNedYaw → Attitude setpoint 전환 시 급격한 튐이나 `OffboardError` 없는지. `[WARN] 수직속도 ... m/s` 경고가 뜨는지(뜨면 `get_hover_thrust()` 근사치가 부정확하다는 뜻 — `MPC_THR_HOVER` 실제값과 비교). attitude 종료 후 다시 position(홈 호버)으로 정상 복귀하는지.

- [ ] **4. Rate 단계**

  ```bash
  python scripts/pid_tuning/tune_cascade.py --stage rate --axis roll --duration 1.5
  ```

  확인: attitude 단계와 동일 + `[WARN] 요청한 텔레메트리 rate가 반영되지 않은...` 경고가 뜨지 않는지(100Hz 요청이 실제로 반영됐는지).

- [ ] **5. 파라미터 2개 동시 지정**

  ```bash
  python scripts/pid_tuning/tune_cascade.py --stage rate --axis roll \
      --set MC_ROLLRATE_P=0.18 MC_ROLLRATE_D=0.004 --duration 1.5
  ```

  확인: 콘솔의 `[OK] 파라미터 적용: ...` 로그로 실제 PX4에 반영됐는지. `ledger.csv`의 `set_params` 컬럼에 JSON으로 정확히 기록되는지.

- [ ] **6. 문제 발견 시**

  발견된 문제를 설계 문서(`docs/superpowers/specs/2026-07-24-pid-cascade-tuning-tool-design.md`)에 추가하고 관련 모듈 수정. 문제없으면 생략.

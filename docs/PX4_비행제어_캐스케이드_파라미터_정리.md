# PX4 비행 제어 캐스케이드 구조 및 파라미터 정리

## 1. 개요

- **목적**: 이 프로젝트가 PX4 비행 제어를 어떻게 다루는지(무엇을 오버라이드하고 무엇을 기본값 그대로 두는지)를 사실 기반으로 정리하여, 향후 파라미터 최적화 작업의 기초 자료로 삼는다.
- **범위**: 이 문서는 파라미터를 튜닝하지 않는다. 현재 상태와 PX4 캐스케이드 구조, 그리고 최적화 시 주의할 지점만 정리한다.
- **검증 방법**: 아래 3개 소스를 직접 읽어 확인했다(추측 없음).
  - `src/forest_rescue_system/forest_rescue_system/drone_controller_node.py`
  - `src/forest_rescue_system/config/forest_rescue.yaml`
  - `/home/rokey/PX4-Autopilot` (로컬 PX4 소스 체크아웃) — `mc_pos_control`, `mc_att_control`, `mc_rate_control`, `navigator` 모듈

---

## 2. 제어 구조 개요 (ROS ↔ PX4 역할 분리)

- `drone_controller_node.py`가 PX4에 스트리밍하는 오프보드 명령은 `offboard.set_position_ned(PositionNedYaw)` 단 하나뿐이다 (10곳에서 호출, 예: `drone_controller_node.py:688, 895, 976, 1068...`).
- ROS 쪽에서 속도·가속도·자세를 직접 계산하지 않는다. 계획된 위치(NED)와 yaw만 넘긴다.
- 실제 속도·가속도·자세각·모터 출력 계산은 전부 PX4 내부 캐스케이드가 전담한다.
- 즉 이 프로젝트는 **경로 계획(ROS)과 제어(PX4)가 완전히 분리된 구조**이며, PX4를 블랙박스로 사용하고 소수의 파라미터만 오버라이드하는 방식이다.

---

## 3. PX4 내부 4단 캐스케이드

| 단계 | PX4 모듈 | 제어기 종류 | 입력 → 출력 | 소스 근거 |
|---|---|---|---|---|
| 1. Position | `mc_pos_control` (`PositionControl.cpp`) | **P만 사용** | 위치 오차 → 속도 setpoint | `_positionControl()` 함수, `// P-position controller` 주석 (`PositionControl.cpp:124-126`) |
| 2. Velocity | `mc_pos_control` (`PositionControl.cpp`) | **완전한 PID** | 속도 오차 → 가속도/자세각 setpoint | `_velocityControl()` 함수, `// PID velocity control` 주석 (`PositionControl.cpp:140-145`) |
| 3. Attitude | `mc_att_control` (`AttitudeControl.cpp`) | **P만 사용** (I, D 없음) | 자세 오차 → 각속도 setpoint | `AttitudeControl::update()` 함수 — `_proportional_gain`만 사용, 적분/미분 항 없음 (`AttitudeControl.cpp:55-91`) |
| 4. Rate | `mc_rate_control` (`MulticopterRateControl.cpp`) | **완전한 PID** | 각속도 오차 → 모터 출력(actuator) | `"The controller has a PID loop for angular rate error."` (`MulticopterRateControl.cpp:340`) |

> **참고**: "MPC"는 Model Predictive Control이 아니라 PX4 모듈명 "Multicopter Position Control"의 약자다. 실제로는 P/PID 기반 고전 제어이며 예측 모델을 사용하지 않는다.

캐스케이드는 안쪽 루프(4단계)가 바깥쪽 루프(1단계)보다 훨씬 빠르게 반응해야 안정적으로 동작하는 구조다(대역폭 분리, bandwidth separation). 이 원칙은 5장에서 최적화 시 고려사항으로 다시 다룬다.

---

## 4. 이 프로젝트가 오버라이드하는 파라미터 (총 6개)

`drone_controller_node.py`가 PX4 파라미터를 직접 설정(`set_param_float`)하는 곳은 코드 전체에서 딱 두 지점뿐이다(`_configure_flight_limits`의 5개 + `_takeoff`의 1개). 그 외 어디에서도 PX4 파라미터를 건드리지 않는다(전체 grep으로 확인).

| PX4 파라미터 | 소스 YAML 필드 | 설정값 | PX4 기본값 | 적용 위치 / 시점 |
|---|---|---|---|---|
| `MPC_XY_VEL_MAX` | `search_horizontal_speed_m_s` | 1.6 m/s | 12.0 m/s | `_configure_flight_limits`, `_ensure_offboard` 최초 1회 호출 시 |
| `MPC_ACC_HOR` | `search_horizontal_acceleration_m_s2` | 1.5 m/s² | 3.0 m/s² | 〃 |
| `MPC_XY_P` | `search_horizontal_position_gain` | 0.8 | 0.95 | 〃 (캐스케이드 1단계 게인) |
| `MPC_Z_VEL_MAX_UP` | `search_vertical_speed_up_m_s` | 1.5 m/s | 3.0 m/s | 〃 |
| `MPC_Z_VEL_MAX_DN` | `search_vertical_speed_down_m_s` | 1.5 m/s | 1.5 m/s (기본값과 동일) | 〃 |
| `NAV_MC_ALT_RAD` | `altitude_acceptance_radius_m` | 0.1 m | 0.8 m | `_takeoff`에서 이륙 직전 매번 설정 (`drone_controller_node.py:656-663`) |

(`_configure_flight_limits`는 `self.flight_limits_applied` 플래그로 보호되어 있어 **비행당 단 한 번만** 적용되고 이후 재적용되지 않는다.)

전 항목이 PX4 기본값보다 보수적(느리고 안전한 쪽)이다. 장애물이 많은 숲 환경에서 수색 비행하는 특성이 반영된 것으로 보인다.

### 4.1 `NAV_MC_ALT_RAD`는 다른 5개와 성격이 다르다 (확인된 사실)

- 나머지 5개는 전부 `mc_pos_control` 모듈(캐스케이드 1~2단계) 파라미터인 반면, `NAV_MC_ALT_RAD`는 **`navigator` 모듈** 소속이다 (`navigator_params.c`).
- PX4 소스에서 이 값을 실제로 읽는 곳은 `Navigator::get_altitude_acceptance_radius()` 하나뿐이고, 이 함수를 호출하는 곳은 `mission.cpp`, `mission_block.cpp` — 즉 **PX4 AUTO.MISSION 모드에서 미션 웨이포인트 "도달 판정"에만 쓰인다.**
- 이 프로젝트는 미션 업로드(`mission.upload` 등)를 한 곳도 사용하지 않는다(grep으로 확인). 비행 중 사용하는 PX4 액션은 `action.arm()/takeoff()/land()/disarm()`과 오프보드 위치 스트리밍뿐이며, 실제 웨이포인트 도달 판정은 ROS 쪽 자체 파라미터(`waypoint_acceptance_radius_m`, `waypoint_altitude_tolerance_m`)가 담당한다.
- 따라서 **`NAV_MC_ALT_RAD`는 이 프로젝트의 실제 비행 경로(오프보드 서치/리턴 구간)에는 적용되지 않을 가능성이 높다.** 실제로 영향을 줄 수 있는 구간은 `action.takeoff()`/`action.land()`가 트리거하는 PX4 자체 AUTO_TAKEOFF/AUTO_LAND 모드뿐인데, 그 두 모드가 이 값을 참조하는지는 이번 조사에서 확인하지 못했다(참조 코드가 `mission.cpp` 계열에서만 발견됨). → 5.3절 참고.

---

## 5. 건드리지 않는 캐스케이드 내부 게인 (PX4 기본 튜닝 그대로)

`_configure_flight_limits`가 설정하는 것은 오직 위 표의 5개 키뿐이다. 아래 게인들은 이 프로젝트 코드 어디에서도 `set_param`으로 설정된 적이 없고, 전부 PX4 기본값이 그대로 적용된다.

### 5.1 Position 단계(1단계)의 나머지 축 — `MPC_Z_P`

| 파라미터 | 의미 | 기본값 |
|---|---|---|
| `MPC_Z_P` | 수직 위치 오차 비례 게인 | **1.0** (미오버라이드) |

수평 위치 게인(`MPC_XY_P`)은 0.8로 낮췄지만, 수직 위치 게인(`MPC_Z_P`)은 기본값 그대로다. 수평/수직 위치 응답 특성이 비대칭이라는 뜻이며, 의도된 것인지는 코드/설정만으로는 알 수 없다.

### 5.2 Velocity 단계(2단계)의 완전한 PID 게인

이 프로젝트가 건드리는 것은 속도의 **상한(MPC_XY_VEL_MAX, MPC_Z_VEL_MAX_UP/DN)과 가속도 상한(MPC_ACC_HOR)**뿐이다. 속도 오차를 실제로 얼마나 공격적으로 좇을지 결정하는 PID 게인 자체는 전부 기본값이다.

| 파라미터 | 의미 | 기본값 |
|---|---|---|
| `MPC_XY_VEL_P_ACC` | 수평 속도 오차 P 게인 (m/s² per m/s) | 1.8 |
| `MPC_XY_VEL_I_ACC` | 수평 속도 오차 I 게인 — 바람 등 외란에 의한 정상상태 오차 제거용 | 0.4 |
| `MPC_XY_VEL_D_ACC` | 수평 속도 오차 D 게인 — 너무 크면 다시 진동 발생 | 0.2 |
| `MPC_Z_VEL_P_ACC` | 수직 속도 오차 P 게인 | 4.0 |
| `MPC_Z_VEL_I_ACC` | 수직 속도 오차 I 게인 — 이착륙 시 호버링 추력 추정 허용 | 2.0 |
| `MPC_Z_VEL_D_ACC` | 수직 속도 오차 D 게인 | 0.0 (기본값 자체가 미사용) |

### 5.3 Attitude 단계(3단계) — P만 사용

| 파라미터 | 의미 | 기본값 |
|---|---|---|
| `MC_ROLL_P` | Roll 자세 오차 P 게인 (1 rad 오차 → 목표 각속도 rad/s) | 6.5 |
| `MC_PITCH_P` | Pitch 자세 오차 P 게인 | 6.5 |
| `MC_YAW_P` | Yaw 자세 오차 P 게인 | 2.8 |
| `MC_YAW_WEIGHT` | 비선형 자세 제어에서 Yaw를 Roll/Pitch 대비 얼마나 덜 우선할지(0~1) | 0.4 |

멀티콥터는 구조상 Yaw 방향 제어 권한이 Roll/Pitch보다 훨씬 작기 때문에 `MC_YAW_P`가 다른 두 축보다 작게 잡혀 있고, `MC_YAW_WEIGHT`로 다시 한번 우선순위를 낮춘다. 이 프로젝트는 이 4개 값 전부 손대지 않는다.

### 5.4 Rate 단계(4단계) — 완전한 PID (+ 피드포워드)

| 파라미터 | 의미 | 기본값 |
|---|---|---|
| `MC_ROLLRATE_P` | Roll 각속도 오차 P 게인 | 0.15 |
| `MC_ROLLRATE_I` | Roll 각속도 오차 I 게인 (무게중심 오프셋/정적 추력차 보정) | 0.2 |
| `MC_ROLLRATE_D` | Roll 각속도 오차 D 게인 | 0.003 |
| `MC_ROLLRATE_FF` | Roll 각속도 피드포워드 (추종 성능 개선) | 0.0 (미사용) |
| `MC_PITCHRATE_P` | Pitch 각속도 오차 P 게인 | 0.15 |
| `MC_PITCHRATE_I` | Pitch 각속도 오차 I 게인 | 0.2 |
| `MC_PITCHRATE_D` | Pitch 각속도 오차 D 게인 | 0.003 |
| `MC_YAWRATE_P` | Yaw 각속도 오차 P 게인 | 0.2 |
| `MC_YAWRATE_I` | Yaw 각속도 오차 I 게인 | 0.1 |
| `MC_YAWRATE_D` | Yaw 각속도 오차 D 게인 | 0.0 (미사용) |

이 8개(+FF 2개)가 실제로 모터 출력을 결정하는 최종 단계 게인이다. 기체(질량, 관성, 프레임, 프로펠러)에 맞춘 PX4 기본 튜닝(전형적으로 X500류 쿼드로터 기준)을 그대로 신뢰하고 있다는 뜻이다.

### 5.5 그 외 참고: 틸트 한계 (`MPC_TILTMAX_AIR`)

| 파라미터 | 의미 | 기본값 |
|---|---|---|
| `MPC_TILTMAX_AIR` | 비행 중 최대 허용 기울기 각도 | 45° |

바깥 루프(속도/가속도 상한)를 아무리 낮춰도, 이 한계 자체는 그대로이므로 순간적으로는 여전히 45°까지 기울 수 있는 하드웨어적 여유가 있다. 다만 `MPC_ACC_HOR`를 1.5 m/s²로 낮춰뒀기 때문에 실제로 이 한계까지 갈 일은 거의 없다.

---

## 6. 요약

- 경로 계획(ROS)과 제어(PX4)가 완전히 분리된 구조이며, 알고리즘 자체(캐스케이드 P/PID)는 단순하다. 실전 난이도는 튜닝과 상태 추정이며 이는 전부 PX4가 처리한다.
- 이 프로젝트는 PX4를 블랙박스로 쓰고, 캐스케이드 1~2단계(Position/Velocity)의 **속도·가속도 상한 및 위치 P 게인 5개** + **미션 전용 고도 도달 판정 1개(`NAV_MC_ALT_RAD`, 사실상 현재 비행 경로엔 미적용 가능성 높음)**만 오버라이드한다.
- 캐스케이드 3~4단계(Attitude/Rate)의 8개 게인과, 2단계의 속도 PID 게인 6개, 1단계의 수직 위치 게인, 틸트 한계는 전부 PX4 기본값 그대로다.

---

## 7. 향후 최적화 시 고려할 점

- **대역폭 분리 원칙**: 지금 오버라이드된 5개는 전부 바깥쪽 2개 루프(Position/Velocity)의 setpoint 상한·게인이고, 안쪽 2개 루프(Attitude/Rate)는 기본 튜닝 그대로다. 안쪽 루프가 바깥 루프보다 충분히 빠르게 반응하는 한(현재 PX4 기본 튜닝은 표준 쿼드로터 기준으로 이 조건을 만족하도록 설계됨) 바깥 루프만 낮춰도 안정적으로 동작한다. 단, 향후 바깥 루프 게인(특히 `MPC_XY_VEL_MAX`, `MPC_ACC_HOR`)을 지금보다 공격적으로 올릴 경우, 병목이 안쪽 루프(Attitude/Rate 게인, `MPC_TILTMAX_AIR`)로 옮겨갈 수 있으므로 그 시점부터는 3~5장의 값들도 함께 검토해야 한다.
- **비대칭 튜닝**: 수평 위치 게인(`MPC_XY_P`=0.8)만 낮추고 수직 위치 게인(`MPC_Z_P`=1.0)은 그대로다. 수평/수직 응답 속도 차이가 의도된 것인지 확인이 필요하다.
- **`NAV_MC_ALT_RAD`의 실효성**: 4.1절에서 확인했듯 이 값은 PX4 AUTO.MISSION 모드 전용으로 보이며, 이 프로젝트는 미션 모드를 쓰지 않는다. 실제 비행 로그(파라미터 적용 성공 여부, 비행 모드 전환 시점)로 이 값이 정말 아무 효과가 없는지 재확인한 뒤, 최적화 우선순위에서 제외하거나 제거를 검토할 수 있다.
- **속도 PID(2단계) 미조정 상태**: `MPC_XY_VEL_P/I/D_ACC`, `MPC_Z_VEL_P/I/D_ACC`는 기본값이다. 상한(`MPC_XY_VEL_MAX` 등)을 낮췄다고 해서 응답 특성(오버슈트, 정착 시간)까지 자동으로 보수적으로 바뀌는 것은 아니므로, 실비행에서 진동/오버슈트가 보인다면 이 6개가 다음 조정 대상이다.

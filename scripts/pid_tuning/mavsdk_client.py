"""PX4 SITL 연결, 헬스체크, 이륙/오프보드 시작, 파라미터 적용을 담당한다."""

import asyncio

from mavsdk import System
from mavsdk.offboard import PositionNedYaw
from mavsdk.param import ParamError

HOME_ALTITUDE_M = 5.0
HOME_POSITION_TOLERANCE_M = 0.15
FALLBACK_HOVER_THRUST = 0.5


async def connect(system_address, timeout_sec=30.0):
    drone = System()
    await drone.connect(system_address=system_address)

    async def _wait_connected():
        async for state in drone.core.connection_state():
            if state.is_connected:
                print("[OK] PX4 연결 성공")
                return

    await asyncio.wait_for(_wait_connected(), timeout=timeout_sec)
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

#!/usr/bin/env python3

"""
MAVSDK Offboard 제어를 이용한 카메라 방향 확인 테스트

비행 순서:
1. PX4 연결
2. 2m 이륙
3. 제자리에서 Yaw를 90도씩 변경
4. 각 방향에서 RGB 및 Depth 영상 확인
5. 착륙
"""

import asyncio

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, PositionNedYaw
from mavsdk.param import ParamError


# 비행 설정값
TAKEOFF_ALTITUDE_M = 2.0
ALTITUDE_ACCEPTANCE_RADIUS_M = 0.1
TAKEOFF_REACHED_TOLERANCE_M = 0.15
WAYPOINT_WAIT_SECONDS = 5


# 사각형 경로의 각 지점이다.
# North와 East는 PX4 local NED 원점 기준 절대 좌표다.
SQUARE_PATH = (
    ("북쪽 지점", 2.0, 0.0, 0.0),
    ("북동쪽 지점", 2.0, 2.0, 90.0),
    ("동쪽 지점", 0.0, 2.0, 180.0),
    ("출발점 상공", 0.0, 0.0, 270.0),
)


# PX4와 MAVSDK의 UDP 연결이 완료될 때까지 기다린다.
async def wait_for_connection(drone):
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[OK] PX4 연결 성공")
            return


# GPS, Home Position, Local Position이 준비될 때까지 기다린다.
async def wait_for_health(drone):
    async for health in drone.telemetry.health():
        print(
            "[WAIT] "
            f"global={health.is_global_position_ok}, "
            f"home={health.is_home_position_ok}, "
            f"local={health.is_local_position_ok}"
        )

        ready = (
            health.is_global_position_ok
            and health.is_home_position_ok
            and health.is_local_position_ok
        )

        if ready:
            print("[OK] Offboard 비행 준비 완료")
            return


# PX4의 고도 수용 반경과 이륙 목표 고도를 설정한다.
async def configure_takeoff(drone):
    previous_radius = await drone.param.get_param_float(
        "NAV_MC_ALT_RAD"
    )

    await drone.param.set_param_float(
        "NAV_MC_ALT_RAD",
        ALTITUDE_ACCEPTANCE_RADIUS_M,
    )

    configured_radius = await drone.param.get_param_float(
        "NAV_MC_ALT_RAD"
    )

    await drone.action.set_takeoff_altitude(
        TAKEOFF_ALTITUDE_M
    )

    configured_altitude = (
        await drone.action.get_takeoff_altitude()
    )

    print(
        "[정보] NAV_MC_ALT_RAD: "
        f"{previous_radius:.2f}m → {configured_radius:.2f}m"
    )
    print(
        "[정보] PX4 이륙 고도: "
        f"{configured_altitude:.2f}m"
    )


# 목표 이륙 고도 근처에 도달할 때까지 기다린다.
async def wait_for_takeoff_altitude(drone):
    minimum_altitude = (
        TAKEOFF_ALTITUDE_M
        - TAKEOFF_REACHED_TOLERANCE_M
    )

    async for position in drone.telemetry.position():
        current_altitude = position.relative_altitude_m

        print(
            f"[WAIT] 현재 고도: {current_altitude:.2f}m / "
            f"목표 고도: {TAKEOFF_ALTITUDE_M:.2f}m"
        )

        if current_altitude >= minimum_altitude:
            print("[OK] 목표 이륙 고도 도달")
            return


# 착륙이 완료될 때까지 기다린다.
async def wait_until_landed(drone):
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("[OK] 착륙 확인")
            return


# 지정한 local NED 좌표로 이동한다.
async def move_to_position(
    drone,
    waypoint_name,
    north_m,
    east_m,
    yaw_deg,
):
    # NED 좌표계에서는 아래쪽이 양수이므로 고도는 음수로 변환한다.
    down_m = -TAKEOFF_ALTITUDE_M

    print(
        f"[이동] {waypoint_name}: "
        f"North={north_m:.1f}m, "
        f"East={east_m:.1f}m, "
        f"Altitude={TAKEOFF_ALTITUDE_M:.1f}m, "
        f"Yaw={yaw_deg:.1f}deg"
    )

    setpoint = PositionNedYaw(
        north_m,
        east_m,
        down_m,
        yaw_deg,
    )

    await drone.offboard.set_position_ned(setpoint)
    await asyncio.sleep(WAYPOINT_WAIT_SECONDS)


# 비행 오류 발생 시 Offboard를 종료하고 착륙을 시도한다.
async def emergency_land(drone, offboard_started):
    print("[안전] 비상 착륙을 시도합니다.")

    if offboard_started:
        try:
            await drone.offboard.stop()
        except OffboardError:
            pass

    try:
        await drone.action.land()

        await asyncio.wait_for(
            wait_until_landed(drone),
            timeout=30,
        )
    except (ActionError, asyncio.TimeoutError):
        print("[ERROR] 자동 착륙 완료 여부를 확인할 수 없습니다.")


async def main():
    drone = System()
    offboard_started = False

    print("[1] PX4 연결 대기")
    await drone.connect(
        system_address="udpin://0.0.0.0:14540"
    )

    try:
        await asyncio.wait_for(
            wait_for_connection(drone),
            timeout=30,
        )

        print("[2] GPS 및 Local Position 대기")
        await asyncio.wait_for(
            wait_for_health(drone),
            timeout=60,
        )

        print("[3] 이륙 설정")
        await configure_takeoff(drone)

        print("[4] Arm")
        await drone.action.arm()

        print("[5] Takeoff")
        await drone.action.takeoff()

        await asyncio.wait_for(
            wait_for_takeoff_altitude(drone),
            timeout=30,
        )

        # PX4는 Offboard 시작 전에 최소 하나의 setpoint가 필요하다.
        print("[6] 초기 Offboard setpoint 설정")
        initial_setpoint = PositionNedYaw(
            0.0,
            0.0,
            -TAKEOFF_ALTITUDE_M,
            0.0,
        )
        await drone.offboard.set_position_ned(
            initial_setpoint
        )

        print("[7] Offboard 시작")
        await drone.offboard.start()
        offboard_started = True

        print("[8] 사각형 경로 비행 시작")
        for waypoint in SQUARE_PATH:
            await move_to_position(
                drone,
                *waypoint,
            )

        print("[9] Offboard 종료")
        await drone.offboard.stop()
        offboard_started = False

        print("[10] 착륙")
        await drone.action.land()

        await asyncio.wait_for(
            wait_until_landed(drone),
            timeout=30,
        )

    except (
        ActionError,
        OffboardError,
        ParamError,
        asyncio.TimeoutError,
    ) as error:
        print(f"[ERROR] 비행 테스트 실패: {error}")
        await emergency_land(
            drone,
            offboard_started,
        )


if __name__ == "__main__":
    asyncio.run(main())
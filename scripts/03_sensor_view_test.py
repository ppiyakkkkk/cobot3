# mavsdk_takeoff_test.py

import asyncio

from mavsdk import System
from mavsdk.action import ActionError


async def wait_for_connection(drone):
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[OK] PX4 연결 성공")
            return


async def wait_for_health(drone):
    async for health in drone.telemetry.health():
        print(
            "[WAIT] "
            f"global={health.is_global_position_ok}, "
            f"home={health.is_home_position_ok}"
        )

        if (
            health.is_global_position_ok
            and health.is_home_position_ok
        ):
            print("[OK] 이륙 준비 완료")
            return


async def wait_until_landed(drone):
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("[OK] 착륙 확인")
            return


async def main():
    drone = System()

    print("[1] PX4 연결 대기")
    await drone.connect(
        system_address="udpin://0.0.0.0:14540"
    )

    await asyncio.wait_for(
        wait_for_connection(drone),
        timeout=30,
    )

    print("[2] GPS 및 Home Position 대기")
    await asyncio.wait_for(
        wait_for_health(drone),
        timeout=60,
    )

    try:
        print("[3] 이륙 고도 5m 설정")
        await drone.action.set_takeoff_altitude(5.0)

        print("[4] Arm")
        await drone.action.arm()

        print("[5] Takeoff")
        await drone.action.takeoff()

        print("[6] 센서 확인을 위해 60초간 제자리 비행")
        await asyncio.sleep(60)

        print("[7] Land")
        await drone.action.land()

        await asyncio.wait_for(
            wait_until_landed(drone),
            timeout=30,
        )

    except ActionError as error:
        print(f"[ERROR] PX4 명령 거부: {error}")

        try:
            await drone.action.land()
        except ActionError:
            pass


if __name__ == "__main__":
    asyncio.run(main())

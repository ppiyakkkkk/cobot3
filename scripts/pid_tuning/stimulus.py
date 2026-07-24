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

    # velocity 단계는 body-frame forward를 보내지만, telemetry_logger의 추출기는
    # NED north_m_s를 읽는다. return_to_home_hover가 매 테스트 전 yaw=0으로
    # 복귀시키므로 body-forward == NED-north가 성립한다 (그 전제가 깨지면 불일치).
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

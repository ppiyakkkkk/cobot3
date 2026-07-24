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
    else:
        # velocity/position 두 stage 모두 STREAM_BUILDERS에서
        # position_velocity_ned()를 구독하므로 rate 설정도 이걸로 통일한다.
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

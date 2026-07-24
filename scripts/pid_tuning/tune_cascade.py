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
    try:
        args.set_params = parse_set_params(args.set_pairs)
    except ValueError as error:
        parser.error(str(error))
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
    await asyncio.sleep(duration_sec)
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

    if not values:
        print("[ERROR] 텔레메트리 샘플을 하나도 수집하지 못함 — 지표/플롯/이력 생략")
        return

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

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

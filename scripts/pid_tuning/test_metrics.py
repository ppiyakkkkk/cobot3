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

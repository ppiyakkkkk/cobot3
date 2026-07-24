"""tune_cascade.py 인자 파싱 단위 테스트."""

import pytest

import tune_cascade


def test_parse_set_params_basic():
    result = tune_cascade.parse_set_params(["MC_ROLLRATE_P=0.18", "MC_ROLLRATE_D=0.004"])
    assert result == {"MC_ROLLRATE_P": 0.18, "MC_ROLLRATE_D": 0.004}


def test_parse_set_params_empty():
    assert tune_cascade.parse_set_params([]) == {}


def test_parse_set_params_missing_value():
    with pytest.raises(ValueError):
        tune_cascade.parse_set_params(["MC_ROLLRATE_P"])


def test_parse_args_rejects_invalid_axis(monkeypatch):
    monkeypatch.setattr(
        "sys.argv", ["tune_cascade.py", "--stage", "rate", "--axis", "horizontal"]
    )
    with pytest.raises(SystemExit):
        tune_cascade.parse_args()


def test_parse_args_defaults(monkeypatch):
    monkeypatch.setattr(
        "sys.argv", ["tune_cascade.py", "--stage", "velocity", "--axis", "horizontal"]
    )
    args = tune_cascade.parse_args()
    assert args.step_mag == 1.5
    assert args.duration == 3.0
    assert args.set_params == {}

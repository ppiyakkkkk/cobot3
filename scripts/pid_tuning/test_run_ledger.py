"""run_ledger.py 단위 테스트."""

import csv

import run_ledger


def test_append_run_creates_header_once(tmp_path, monkeypatch):
    ledger_path = tmp_path / "ledger.csv"
    monkeypatch.setattr(run_ledger, "LEDGER_PATH", ledger_path)

    run_ledger.append_run(
        stage="rate", axis="roll", set_params={"MC_ROLLRATE_P": 0.18},
        overshoot=5.0, rise_time=0.2, settling_time=1.5, plot_path="a.png",
    )
    run_ledger.append_run(
        stage="rate", axis="roll", set_params={"MC_ROLLRATE_P": 0.20},
        overshoot=8.0, rise_time=0.15, settling_time=None, plot_path="b.png",
    )

    with open(ledger_path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    assert len(rows) == 2
    assert rows[0]["stage"] == "rate"
    assert rows[0]["set_params"] == '{"MC_ROLLRATE_P": 0.18}'
    assert rows[1]["settling_time_s"] == ""

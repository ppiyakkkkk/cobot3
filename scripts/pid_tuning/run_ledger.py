"""튜닝 실행 결과를 CSV 한 줄로 누적 기록한다."""

import csv
import datetime
import json
from pathlib import Path

LEDGER_PATH = Path(__file__).parent / "runs" / "ledger.csv"
FIELDNAMES = [
    "timestamp", "stage", "axis", "set_params",
    "overshoot_pct", "rise_time_s", "settling_time_s", "plot_path",
]


def append_run(stage, axis, set_params, overshoot, rise_time, settling_time, plot_path):
    LEDGER_PATH.parent.mkdir(parents=True, exist_ok=True)
    is_new = not LEDGER_PATH.exists()
    with open(LEDGER_PATH, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        if is_new:
            writer.writeheader()
        writer.writerow({
            "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
            "stage": stage,
            "axis": axis,
            "set_params": json.dumps(set_params, ensure_ascii=False),
            "overshoot_pct": overshoot,
            "rise_time_s": rise_time,
            "settling_time_s": settling_time if settling_time is not None else "",
            "plot_path": str(plot_path),
        })

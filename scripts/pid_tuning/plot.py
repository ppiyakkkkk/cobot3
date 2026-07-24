"""setpoint vs 실측 스텝 응답 그래프를 PNG로 저장한다."""

import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = "Noto Sans CJK JP"
plt.rcParams["axes.unicode_minus"] = False

RUNS_DIR = Path(__file__).parent / "runs"


def save_plot(stage, axis, times, values, setpoint_initial, setpoint_final, set_params):
    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = RUNS_DIR / f"{stage}_{axis}_{timestamp}.png"

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(times, values, label="실측", color="tab:blue")
    ax.axhline(setpoint_final, linestyle="--", color="tab:orange", label="setpoint")
    ax.axvline(0.0, linestyle=":", color="gray", linewidth=1)
    ax.set_xlabel("시간 (s, 0=자극 시작)")
    ax.set_ylabel(f"{stage}/{axis}")
    set_params_summary = ", ".join(f"{k}={v}" for k, v in set_params.items())
    ax.set_title(f"{stage}/{axis} step response\n{set_params_summary}")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path

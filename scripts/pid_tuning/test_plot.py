"""plot.py 단위 테스트."""

import plot


def test_save_plot_creates_png(tmp_path, monkeypatch):
    monkeypatch.setattr(plot, "RUNS_DIR", tmp_path)

    path = plot.save_plot(
        stage="rate",
        axis="roll",
        times=[0.0, 0.5, 1.0],
        values=[0.0, 15.0, 30.0],
        setpoint_initial=0.0,
        setpoint_final=30.0,
        set_params={"MC_ROLLRATE_P": 0.18},
    )

    assert path.exists()
    assert path.suffix == ".png"
    assert path.parent == tmp_path
    assert path.name.startswith("rate_roll_")

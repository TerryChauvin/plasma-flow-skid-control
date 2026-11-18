"""Static dashboard: renders variables.csv (+ events.csv) of a run to a PNG."""
from __future__ import annotations

import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def _col(rows: list[dict], key: str) -> list:
    return [float(r[key]) if key != "state" else r[key] for r in rows]


def render(run_dir: str | Path) -> Path:
    d = Path(run_dir)
    rows = list(csv.DictReader(open(d / "variables.csv")))
    events = list(csv.DictReader(open(d / "events.csv")))
    t = _col(rows, "t")
    fig, ax = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    ax[0].plot(t, _col(rows, "gas_flow"), label="gas (sccm)")
    ax[0].plot(t, _col(rows, "gas_sp"), "--", label="gas sp")
    ax[0].plot(t, [10 * v for v in _col(rows, "liquid_flow")], label="liquid x10 (mL/min)")
    ax[0].plot(t, [10 * v for v in _col(rows, "liq_sp")], "--", label="liquid sp x10")
    ax[1].plot(t, _col(rows, "temp"), label="temperature (degC)")
    ax[1].plot(t, _col(rows, "temp_sp"), "--", label="temp sp")
    ax[1].plot(t, _col(rows, "cool"), label="cooling (%)", alpha=0.6)
    ax[2].plot(t, _col(rows, "pressure"), label="pressure (bar abs)")
    ax[3].plot(t, _col(rows, "power"), label="plasma power (W)")
    ax[3].plot(t, [50 * v for v in _col(rows, "alarms")], "r", label="alarm word x50")
    for e in events:
        for a in ax:
            a.axvline(float(e["t"]), color="red" if e["event"].startswith("ALARM") else "grey",
                      lw=0.6, ls=":")
        ax[0].text(float(e["t"]), ax[0].get_ylim()[1], e["detail"], rotation=90, va="top", fontsize=7)
    for a in ax:
        a.legend(loc="upper left", fontsize=7)
        a.grid(alpha=0.3)
    ax[3].set_xlabel("simulated time (s)")
    fig.tight_layout()
    out = d / "dashboard.png"
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out

"""Regenerate every image of the README from real runs of the simulator.

    python docs/make_figures.py
"""
from __future__ import annotations

import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402

from scada.campaign import campaign  # noqa: E402
from scada.demo import FAULTS, run as run_demo  # noqa: E402
from sim.skid import Alarm, Skid, State  # noqa: E402

IMG = ROOT / "docs" / "img"
FAULT, FAULT_AT = "cooling_loss", 45.0


def dashboard_png() -> None:
    with tempfile.TemporaryDirectory() as d:
        shutil.copy(run_demo("none", out_dir=d), IMG / "dashboard_nominal.png")


def fault_gif() -> None:
    skid, recs = Skid(seed=0), []
    for k in range(3000):
        if k == int(FAULT_AT * 10):
            FAULTS[FAULT](skid.plant)
        rec = skid.step(start=(k == 5))
        recs.append(rec)
        if k > 50 and rec["state"] == State.IDLE.name:
            break
    recs = recs[::10]  # 1 simulated second per frame
    t = [r["t"] for r in recs]
    trip = next(r["t"] for r in recs if r["alarms"])
    panels = [("temp", "temperature (degC)", "tab:red"), ("power", "plasma power (W)", "tab:purple"),
              ("liquid_flow", "liquid flow (mL/min)", "tab:blue"), ("gas_flow", "gas flow (sccm)", "tab:green")]
    fig, axes = plt.subplots(4, 1, figsize=(7, 6), sharex=True)
    lines = []
    for ax, (key, label, c) in zip(axes, panels):
        ax.set_xlim(0, t[-1])
        ax.set_ylim(min(r[key] for r in recs) * 0.95 - 1, max(r[key] for r in recs) * 1.08 + 1)
        ax.set_ylabel(label, fontsize=8)
        ax.grid(alpha=0.3)
        (ln,) = ax.plot([], [], c=c)
        lines.append((ln, key))
    axes[0].axhline(70.0, color="k", ls="--", lw=0.8)
    axes[0].text(1, 71, "over-temperature limit", fontsize=7)
    axes[-1].set_xlabel("simulated time (s)")
    title = fig.suptitle("", fontsize=10, family="monospace")
    fig.tight_layout(rect=(0, 0, 1, 0.94))

    def draw(i: int):
        for ln, key in lines:
            ln.set_data(t[: i + 1], [r[key] for r in recs[: i + 1]])
        r = recs[i]
        alarms = Alarm(r["alarms"]).name if r["alarms"] else "none"
        title.set_text(f"t={r['t']:5.0f}s  state={r['state']:<11} alarm={alarms}  (fault: {FAULT} at {FAULT_AT:.0f}s)")
        title.set_color("red" if r["alarms"] else "black")
        if r["t"] == trip:
            for ax in axes:
                ax.axvline(trip, color="red", lw=1)
        return [ln for ln, _ in lines] + [title]

    frames = list(range(len(recs))) + [len(recs) - 1] * 15  # hold the last frame
    FuncAnimation(fig, draw, frames=frames, blit=False).save(IMG / "fault_demo.gif", writer=PillowWriter(fps=10), dpi=70)
    plt.close(fig)


def convergence_png() -> None:
    hist = campaign(n_init=4, n_iter=12, seed=0)
    best, cur = [], 0.0
    for h in hist:
        cur = max(cur, h["yield"])
        best.append(cur)
    fig, ax = plt.subplots(figsize=(7, 3.8))
    for phase, c in (("random", "tab:grey"), ("bayes", "tab:blue")):
        pts = [h for h in hist if h["phase"] == phase]
        ax.scatter([h["n"] + 1 for h in pts], [h["yield"] for h in pts], c=c, label=f"experiment ({phase})")
    ax.step([h["n"] + 1 for h in hist], best, where="post", c="k", label="best so far")
    ax.axvline(4.5, ls=":", c="grey")
    ax.set_xlabel("experiment #")
    ax.set_ylabel("simulated yield")
    ax.set_title("Campaign: 4 random starts, then GP + expected improvement (seed 0)", fontsize=9)
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(IMG / "campaign_convergence.png", dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    IMG.mkdir(parents=True, exist_ok=True)
    dashboard_png()
    fault_gif()
    convergence_png()
    for p in sorted(IMG.iterdir()):
        print(f"{p.name}: {p.stat().st_size / 1024:.0f} KiB")

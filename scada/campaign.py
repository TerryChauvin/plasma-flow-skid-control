"""Autonomous campaign: Bayesian optimisation (GP + expected improvement) over recipe parameters.

Each experiment is a full batch on the simulated skid (start -> purge -> ... -> RUN -> safe stop);
the simulated yield is computed from the measured steady state of the last 20 s of RUN.

    python -m scada.campaign --n-iter 12 --seed 0
"""
from __future__ import annotations

import argparse
import csv
import math
import random
from pathlib import Path

import numpy as np
from scipy.stats import norm
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import ConstantKernel, Matern, WhiteKernel

from sim.skid import Recipe, Skid, State

BOUNDS = {"power": (50.0, 200.0), "liquid_sp": (2.0, 10.0)}  # W, mL/min
P_OPT, Q_OPT = 130.0, 6.0  # hidden optimum of the simulated chemistry


def run_experiment(power: float, liquid_sp: float, seed: int = 0) -> float:
    """Run one batch and return the simulated yield (0 if any alarm tripped)."""
    skid = Skid(Recipe(power=power, liquid_sp=liquid_sp, run_time=60.0), seed=seed)
    run = []
    for k in range(4000):
        rec = skid.step(start=(k == 5))
        if rec["state"] == State.RUN.name:
            run.append(rec)
        if k > 50 and rec["state"] == State.IDLE.name:
            break
    if not run or any(r["alarms"] for r in run) or skid.ctl.alarms:
        return 0.0
    tail = run[-200:]  # last 20 s of RUN
    q = float(np.mean([r["liquid_flow"] for r in tail]))
    shape = math.exp(-(((power - P_OPT) / 45.0) ** 2) - ((q - Q_OPT) / 2.0) ** 2)
    return shape + random.Random(seed).gauss(0, 0.02)


def _to_unit(x: np.ndarray) -> np.ndarray:
    lo, hi = np.array([b[0] for b in BOUNDS.values()]), np.array([b[1] for b in BOUNDS.values()])
    return (x - lo) / (hi - lo)


def _from_unit(u: np.ndarray) -> np.ndarray:
    lo, hi = np.array([b[0] for b in BOUNDS.values()]), np.array([b[1] for b in BOUNDS.values()])
    return lo + u * (hi - lo)


def next_point(x: np.ndarray, y: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """Fit a GP on the history, return the candidate with the highest expected improvement."""
    kernel = ConstantKernel(1.0) * Matern([0.3, 0.3], (1e-2, 1e2), nu=2.5) + WhiteKernel(1e-3, (1e-5, 1e-1))
    gp = GaussianProcessRegressor(kernel, normalize_y=True, n_restarts_optimizer=3, random_state=0)
    gp.fit(_to_unit(x), y)
    cand = rng.random((2000, x.shape[1]))
    mu, sd = gp.predict(cand, return_std=True)
    z = (mu - y.max()) / np.maximum(sd, 1e-9)
    ei = (mu - y.max()) * norm.cdf(z) + sd * norm.pdf(z)
    return _from_unit(cand[int(np.argmax(ei))])


def campaign(n_init: int = 4, n_iter: int = 12, seed: int = 0) -> list[dict]:
    rng = np.random.default_rng(seed)
    xs = _from_unit(rng.random((n_init, 2)))
    ys = np.array([run_experiment(*x, seed=seed + i) for i, x in enumerate(xs)])
    hist = [{"n": i, "phase": "random", "power": x[0], "liquid_sp": x[1], "yield": y}
            for i, (x, y) in enumerate(zip(xs, ys))]
    for i in range(n_init, n_init + n_iter):
        x = next_point(xs, ys, rng)
        y = run_experiment(*x, seed=seed + i)
        xs, ys = np.vstack([xs, x]), np.append(ys, y)
        hist.append({"n": i, "phase": "bayes", "power": x[0], "liquid_sp": x[1], "yield": y})
    return hist


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-iter", type=int, default=12)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="out/campaign")
    a = ap.parse_args()
    hist = campaign(n_iter=a.n_iter, seed=a.seed)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "campaign.csv", "w", newline="") as f:
        w = csv.DictWriter(f, list(hist[0]))
        w.writeheader()
        w.writerows(hist)
    best = max(hist, key=lambda h: h["yield"])
    print(f"best yield {best['yield']:.3f} at power={best['power']:.0f} W, liquid={best['liquid_sp']:.1f} mL/min "
          f"(hidden optimum {P_OPT:.0f} W, {Q_OPT:.1f} mL/min) -> {out / 'campaign.csv'}")

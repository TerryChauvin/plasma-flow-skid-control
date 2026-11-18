"""Run one batch on the simulated skid, with an optional fault injected, and write the journal + dashboard.

    python -m scada.demo --fault cooling_loss
"""
from __future__ import annotations

import argparse

from scada.dashboard import render
from scada.logger import Journal
from sim.skid import Skid, State

FAULTS = {
    "none": lambda p: None,
    "cooling_loss": lambda p: setattr(p, "cooling", 0.0),
    "gas_loss": lambda p: setattr(p, "gas_supply", 0.0),
    "blockage": lambda p: setattr(p, "blockage", 1.0),
    "door_open": lambda p: setattr(p, "door_closed", False),
}


def run(fault: str = "none", fault_at: float = 45.0, out_dir: str = "out/demo", seed: int = 0) -> str:
    skid, journal = Skid(seed=seed), Journal(out_dir)
    for k in range(3000):  # 300 s max
        t = k * 0.1
        if t >= fault_at and t < fault_at + 0.1:
            FAULTS[fault](skid.plant)
        rec = skid.step(start=(k == 5))
        journal.log(rec)
        if k > 50 and rec["state"] == State.IDLE.name:
            break
    journal.close()
    return str(render(out_dir))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--fault", choices=FAULTS, default="none")
    ap.add_argument("--out", default="out/demo")
    a = ap.parse_args()
    print("dashboard:", run(a.fault, out_dir=a.out))

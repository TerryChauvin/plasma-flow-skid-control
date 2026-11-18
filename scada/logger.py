"""Supervision journal: timestamped variables (CSV) and an alarm/state event log (CSV)."""
from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path

from sim.skid import Alarm

VARS = ["timestamp", "t", "state", "gas_flow", "gas_sp", "liquid_flow", "liq_sp", "temp", "temp_sp",
        "pressure", "pump", "cool", "power", "plasma_on", "door_closed", "alarms"]


class Journal:
    def __init__(self, out_dir: str | Path, every: int = 5) -> None:
        self.dir = Path(out_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.every, self.n = every, 0  # log one row per `every` scans (0.5 s by default)
        self.prev_state, self.prev_alarms = None, 0
        self._v = open(self.dir / "variables.csv", "w", newline="")
        self._e = open(self.dir / "events.csv", "w", newline="")
        self.vars = csv.DictWriter(self._v, VARS, extrasaction="ignore")
        self.events = csv.writer(self._e)
        self.vars.writeheader()
        self.events.writerow(["timestamp", "t", "event", "detail"])

    def log(self, rec: dict) -> None:
        now = datetime.now().isoformat(timespec="milliseconds")
        if rec["state"] != self.prev_state:
            self.events.writerow([now, rec["t"], "STATE", rec["state"]])
        for a in Alarm:
            was, is_ = self.prev_alarms & a, rec["alarms"] & a
            if is_ and not was:
                self.events.writerow([now, rec["t"], "ALARM_SET", a.name])
            elif was and not is_:
                self.events.writerow([now, rec["t"], "ALARM_ACKED", a.name])
        self.prev_state, self.prev_alarms = rec["state"], rec["alarms"]
        if self.n % self.every == 0:
            self.vars.writerow({**rec, "timestamp": now})
        self.n += 1

    def close(self) -> None:
        self._v.close()
        self._e.close()

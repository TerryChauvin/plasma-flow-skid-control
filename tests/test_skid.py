import csv

import pytest

from scada.demo import run as run_demo
from sim.skid import PID, Alarm, Recipe, Skid, State


def run_until(skid, state, max_s=300.0, **kw):
    for k in range(int(max_s / 0.1)):
        rec = skid.step(**kw)
        if rec["state"] == state.name:
            return rec
    raise AssertionError(f"never reached {state.name}, stuck in {rec['state']}")


def started(**kw):
    skid = Skid(**kw)
    skid.step(start=True)
    return skid


def test_nominal_sequence():
    skid = started()
    seen = [State.PURGE.name]
    run_until(skid, State.RUN)
    for k in range(580):  # 58 s into the 60 s RUN
        rec = skid.step()
        if rec["state"] != seen[-1]:
            seen.append(rec["state"])
    assert abs(rec["liquid_flow"] - 6.0) < 0.3
    assert abs(rec["temp"] - 45.0) < 2.0
    assert rec["alarms"] == 0 and rec["plasma_on"] == 1
    assert seen == ["PURGE", "RUN"]
    run_until(skid, State.IDLE)
    assert skid.ctl.alarms == 0


def test_full_state_order():
    skid, order = started(), []
    for _ in range(3000):
        s = skid.step()["state"]
        if not order or order[-1] != s:
            order.append(s)
        if s == "IDLE":
            break
    assert order == ["PURGE", "LIQUID_RAMP", "IGNITE", "RUN", "SAFE_STOP", "IDLE"]


@pytest.mark.parametrize("fault,value,alarm", [
    ("blockage", 1.0, Alarm.OVERPRESSURE),
    ("gas_supply", 0.0, Alarm.GAS_LOSS),
    ("cooling", 0.0, Alarm.OVERTEMP),
    ("door_closed", False, Alarm.DOOR_OPEN),
])
def test_interlock_trips_plasma_off(fault, value, alarm):
    skid = started()
    run_until(skid, State.RUN)
    for _ in range(100):
        skid.step()
    setattr(skid.plant, fault, value)
    rec = run_until(skid, State.SAFE_STOP, max_s=60)
    assert rec["alarms"] & alarm
    assert rec["plasma_on"] == 0 and rec["power"] == 0.0 and rec["pump"] == 0.0


def test_ignition_failure():
    skid = started()
    skid.plant.no_ignition = True
    rec = run_until(skid, State.SAFE_STOP)
    assert rec["alarms"] & Alarm.IGNITION_FAIL and rec["plasma_on"] == 0


def test_stop_button_and_ack_cycle():
    skid = started()
    run_until(skid, State.RUN)
    skid.plant.blockage = 1.0
    run_until(skid, State.SAFE_STOP)
    skid.plant.blockage = 0.0
    run_until(skid, State.IDLE)
    assert skid.ctl.alarms & Alarm.OVERPRESSURE  # latched although the cause is gone
    for _ in range(5):
        skid.step(start=True)  # refused while alarm latched
        skid.step()
    assert skid.ctl.state == State.IDLE
    skid.step(ack=True)
    assert skid.ctl.alarms == 0
    skid.step(start=True)
    assert skid.ctl.state == State.PURGE
    skid.step(stop=True)
    assert skid.ctl.state == State.SAFE_STOP


def test_ack_does_not_clear_active_condition():
    skid = Skid()
    skid.plant.blockage = 1.5  # idle pressure 1 bar + 1.5 > 2 bar limit
    skid.step()
    assert skid.ctl.alarms & Alarm.OVERPRESSURE
    skid.step(ack=True)
    assert skid.ctl.alarms & Alarm.OVERPRESSURE


def test_start_refused_with_door_open():
    skid = Skid()
    skid.plant.door_closed = False
    skid.step(start=True)
    assert skid.ctl.state == State.IDLE


def test_recipe_only_loaded_in_idle():
    skid = started()
    assert not skid.ctl.load_recipe(Recipe(power=10))
    assert skid.ctl.rec.power == 120.0


def test_pid_converges_on_first_order_plant():
    pid, y = PID(2.0, 1.0), 0.0
    for _ in range(600):  # 60 s, plant gain 0.5, tau 3 s
        u = pid.step(10.0, y)
        y += 0.1 / 3.0 * (0.5 * u - y)
    assert abs(y - 10.0) < 0.1


def test_reverse_pid_cools_when_hot():
    pid = PID(8.0, 0.5, reverse=True)
    assert pid.step(45.0, 60.0) > 0.0
    pid.reset()
    assert pid.step(45.0, 30.0) == 0.0


def test_demo_writes_journal_and_dashboard(tmp_path):
    png = run_demo("blockage", out_dir=str(tmp_path))
    assert (tmp_path / "variables.csv").exists() and png.endswith("dashboard.png")
    events = list(csv.DictReader(open(tmp_path / "events.csv")))
    assert any(e["event"] == "ALARM_SET" and e["detail"] == "OVERPRESSURE" for e in events)

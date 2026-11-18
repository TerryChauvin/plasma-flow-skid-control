"""Simulated plasma flow-reactor skid: controller (mirror of plc/plasma_skid.st) + first-order plant.

The Controller is a line-by-line Python twin of the Structured Text program, so the
logic (state machine, PID, interlocks, alarms, recipes) can be tested without a PLC.
"""
from __future__ import annotations

import random
from dataclasses import dataclass
from enum import IntEnum, IntFlag

DT = 0.1  # scan period, s (PLC task interval T#100ms)


class State(IntEnum):
    IDLE = 0
    PURGE = 1
    LIQUID_RAMP = 2
    IGNITE = 3
    RUN = 4
    SAFE_STOP = 5


ACTIVE = (State.PURGE, State.LIQUID_RAMP, State.IGNITE, State.RUN)


class Alarm(IntFlag):
    OVERPRESSURE = 1
    GAS_LOSS = 2
    OVERTEMP = 4
    DOOR_OPEN = 8
    IGNITION_FAIL = 16


@dataclass
class Recipe:
    purge_gas: float = 300.0  # sccm
    purge_time: float = 10.0  # s
    process_gas: float = 200.0  # sccm
    liquid_sp: float = 6.0  # mL/min
    liquid_ramp: float = 1.0  # mL/min per s
    power: float = 120.0  # W
    temp_sp: float = 45.0  # degC
    run_time: float = 60.0  # s


@dataclass
class Limits:
    p_max: float = 2.0  # bar abs
    t_max: float = 70.0  # degC
    gas_loss_frac: float = 0.5  # trip if flow < frac * setpoint ...
    gas_loss_delay: float = 3.0  # ... for this long, s
    ignite_timeout: float = 10.0  # s
    safe_stop_time: float = 10.0  # s of gas purge after plasma off


@dataclass
class Inputs:
    gas_flow: float
    liquid_flow: float
    temp: float
    pressure: float
    door_closed: bool
    plasma_ok: bool
    start: bool = False
    stop: bool = False
    ack: bool = False


@dataclass
class Outputs:
    gas_sp: float = 0.0
    liq_sp: float = 0.0
    temp_sp: float = 0.0
    pump: float = 0.0  # %
    cool: float = 0.0  # %
    power: float = 0.0  # W
    plasma_on: bool = False
    state: State = State.IDLE
    alarms: Alarm = Alarm(0)


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class PID:
    """Parallel PID, derivative on measurement, clamped integrator (anti-windup)."""

    def __init__(self, kp: float, ki: float, kd: float = 0.0, out_min: float = 0.0,
                 out_max: float = 100.0, reverse: bool = False) -> None:
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max, self.reverse = out_min, out_max, reverse
        self.reset()

    def reset(self) -> None:
        self.integ = 0.0
        self.prev_pv: float | None = None

    def step(self, sp: float, pv: float, dt: float = DT) -> float:
        s = -1.0 if self.reverse else 1.0  # reverse-acting: cooling opens when pv > sp
        err = s * (sp - pv)
        dpv = 0.0 if self.prev_pv is None else (pv - self.prev_pv) / dt
        self.prev_pv = pv
        self.integ = clamp(self.integ + self.ki * err * dt, self.out_min, self.out_max)
        return clamp(self.kp * err + self.integ - self.kd * s * dpv, self.out_min, self.out_max)


class Controller:
    def __init__(self, recipe: Recipe | None = None, limits: Limits | None = None) -> None:
        self.rec = recipe or Recipe()
        self.lim = limits or Limits()
        self.state = State.IDLE
        self.t_state = 0.0
        self.alarms = Alarm(0)
        self.cond = Alarm(0)  # conditions currently present (an alarm can be acked only once clear)
        self.prev_start = self.prev_ack = False
        self.gas_loss_t = self.settle_t = self.liq_ramp = self.gas_sp = 0.0
        self.pid_liq = PID(5.0, 2.0)
        self.pid_temp = PID(8.0, 0.5, reverse=True)

    def load_recipe(self, recipe: Recipe) -> bool:
        """Recipes are only accepted in IDLE, like the PLC's recipe_load command."""
        if self.state != State.IDLE:
            return False
        self.rec = recipe
        return True

    def _goto(self, s: State) -> None:
        self.state, self.t_state = s, 0.0

    def step(self, i: Inputs, dt: float = DT) -> Outputs:
        start_r, ack_r = i.start and not self.prev_start, i.ack and not self.prev_ack
        self.prev_start, self.prev_ack = i.start, i.ack
        self.t_state += dt
        rec, lim = self.rec, self.lim
        active = self.state in ACTIVE

        # --- interlock conditions
        cond = Alarm(0)
        if i.pressure > lim.p_max:
            cond |= Alarm.OVERPRESSURE
        if i.temp > lim.t_max:
            cond |= Alarm.OVERTEMP
        if active and not i.door_closed:
            cond |= Alarm.DOOR_OPEN
        low_gas = active and i.gas_flow < lim.gas_loss_frac * self.gas_sp
        self.gas_loss_t = self.gas_loss_t + dt if low_gas else 0.0
        if self.gas_loss_t >= lim.gas_loss_delay:
            cond |= Alarm.GAS_LOSS
        self.cond = cond

        # --- latch alarms; any new one in an active state stops the plasma
        new = cond & ~self.alarms
        self.alarms |= cond
        if ack_r:
            self.alarms &= cond
        if (new and active) or (i.stop and active):
            self._goto(State.SAFE_STOP)

        # --- sequence
        s = self.state
        if s == State.IDLE:
            if start_r and self.alarms == 0 and i.door_closed:
                self.liq_ramp = self.settle_t = 0.0
                self._goto(State.PURGE)
        elif s == State.PURGE:
            if self.t_state >= rec.purge_time:
                self._goto(State.LIQUID_RAMP)
        elif s == State.LIQUID_RAMP:
            self.liq_ramp = min(rec.liquid_sp, self.liq_ramp + rec.liquid_ramp * dt)
            stable = (self.liq_ramp >= rec.liquid_sp
                      and abs(i.liquid_flow - rec.liquid_sp) <= 0.05 * rec.liquid_sp)
            self.settle_t = self.settle_t + dt if stable else 0.0
            if self.settle_t >= 3.0:
                self._goto(State.IGNITE)
        elif s == State.IGNITE:
            if i.plasma_ok:
                self._goto(State.RUN)
            elif self.t_state >= lim.ignite_timeout:
                self.alarms |= Alarm.IGNITION_FAIL
                self._goto(State.SAFE_STOP)
        elif s == State.RUN:
            if self.t_state >= rec.run_time:
                self._goto(State.SAFE_STOP)
        elif s == State.SAFE_STOP:
            if self.t_state >= lim.safe_stop_time:
                self._goto(State.IDLE)

        # --- outputs
        s = self.state
        o = Outputs(state=s, alarms=self.alarms)
        if s in (State.PURGE, State.LIQUID_RAMP, State.SAFE_STOP):
            o.gas_sp = rec.purge_gas
        elif s in (State.IGNITE, State.RUN):
            o.gas_sp = rec.process_gas
        self.gas_sp = o.gas_sp

        if s in (State.LIQUID_RAMP, State.IGNITE, State.RUN):
            o.liq_sp = self.liq_ramp if s == State.LIQUID_RAMP else rec.liquid_sp
            o.pump = self.pid_liq.step(o.liq_sp, i.liquid_flow, dt)
        else:
            self.pid_liq.reset()
        if s in (State.IGNITE, State.RUN):
            o.plasma_on, o.power, o.temp_sp = True, rec.power, rec.temp_sp
            o.cool = self.pid_temp.step(rec.temp_sp, i.temp, dt)
        else:
            self.pid_temp.reset()
            if s == State.SAFE_STOP and i.temp > 30.0:
                o.cool = 100.0
        return o


class Plant:
    """First-order process model. Fault knobs are plain attributes, set by tests or the demo."""

    def __init__(self) -> None:
        self.gas = self.liq = 0.0
        self.temp = 20.0
        self.plasma_t = 0.0
        self.plasma_ok = False
        self.gas_supply = 1.0  # 0 = supply lost
        self.cooling = 1.0  # 0 = coolant lost
        self.blockage = 0.0  # extra back-pressure, bar
        self.no_ignition = False
        self.door_closed = True

    @property
    def pressure(self) -> float:
        return 1.0 + 0.001 * self.gas + 0.02 * self.liq + self.blockage

    def step(self, o: Outputs, dt: float = DT) -> None:
        self.gas += dt / 1.5 * (o.gas_sp * self.gas_supply - self.gas)
        self.liq += dt / 3.0 * (0.12 * o.pump - self.liq)
        self.plasma_t = self.plasma_t + dt if o.plasma_on else 0.0
        self.plasma_ok = self.plasma_t >= 1.0 and self.gas > 50.0 and not self.no_ignition
        heat = 0.01 * o.power if self.plasma_ok else 0.0
        cool = 0.1 * self.cooling * o.cool / 100.0 * max(self.temp - 15.0, 0.0)
        self.temp += dt * ((20.0 - self.temp) / 300.0 + heat - cool)


class Skid:
    """Controller + plant + noisy measurements, advanced one scan at a time."""

    def __init__(self, recipe: Recipe | None = None, limits: Limits | None = None,
                 seed: int = 0, noise: float = 0.005) -> None:
        self.ctl = Controller(recipe, limits)
        self.plant = Plant()
        self.rng = random.Random(seed)
        self.noise = noise
        self.t = 0.0

    def _n(self, x: float) -> float:
        return x * (1.0 + self.noise * self.rng.gauss(0, 1))

    def step(self, start: bool = False, stop: bool = False, ack: bool = False) -> dict:
        p = self.plant
        i = Inputs(gas_flow=self._n(p.gas), liquid_flow=self._n(p.liq),
                   temp=p.temp + 20.0 * self.noise * self.rng.gauss(0, 1),
                   pressure=self._n(p.pressure), door_closed=p.door_closed,
                   plasma_ok=p.plasma_ok, start=start, stop=stop, ack=ack)
        o = self.ctl.step(i)
        p.step(o)
        self.t += DT
        return {"t": round(self.t, 1), "state": o.state.name, "gas_flow": i.gas_flow,
                "liquid_flow": i.liquid_flow, "temp": i.temp, "pressure": i.pressure,
                "gas_sp": o.gas_sp, "liq_sp": o.liq_sp, "temp_sp": o.temp_sp,
                "pump": o.pump, "cool": o.cool, "power": o.power,
                "plasma_on": int(o.plasma_on), "alarms": int(o.alarms),
                "door_closed": int(i.door_closed)}

(* Plasma flow-reactor skid - IEC 61131-3 Structured Text.
   Sequence: IDLE > PURGE > LIQUID_RAMP > IGNITE > RUN > SAFE_STOP > IDLE
   Python twin: sim/skid.py (Controller). Keep both in sync.
   Written for OpenPLC (matiec): analog I/O as scaled INT/UINT words, recipe in %MW.
   NOT compiled or run on a PLC runtime yet - see README. *)

FUNCTION_BLOCK PID_CTRL
  VAR_INPUT
    enable  : BOOL;
    reverse : BOOL;   (* TRUE: output rises when pv > sp (cooling) *)
    sp, pv  : REAL;
    kp, ki, kd : REAL;
    out_min, out_max : REAL;
    dt      : REAL;   (* scan period, s *)
  END_VAR
  VAR_OUTPUT
    out : REAL;
  END_VAR
  VAR
    integ, prev_pv, err, dpv, sgn : REAL;
    first : BOOL := TRUE;
  END_VAR

  IF enable THEN
    sgn := 1.0;
    IF reverse THEN sgn := -1.0; END_IF;
    err := sgn * (sp - pv);
    dpv := 0.0;
    IF NOT first THEN dpv := (pv - prev_pv) / dt; END_IF;
    first := FALSE;
    prev_pv := pv;
    (* clamped integrator = anti-windup; derivative on measurement *)
    integ := LIMIT(out_min, integ + ki * err * dt, out_max);
    out := LIMIT(out_min, kp * err + integ - kd * sgn * dpv, out_max);
  ELSE
    integ := 0.0;
    first := TRUE;
    out := 0.0;
  END_IF;
END_FUNCTION_BLOCK


PROGRAM PlasmaSkid
  VAR CONSTANT
    DT_S : REAL := 0.1;
    (* states *)
    ST_IDLE : INT := 0;  ST_PURGE : INT := 1;  ST_RAMP : INT := 2;
    ST_IGNITE : INT := 3;  ST_RUN : INT := 4;  ST_SAFE : INT := 5;
    (* alarm bits *)
    AL_PRESS : WORD := 16#0001;  AL_GAS : WORD := 16#0002;  AL_TEMP : WORD := 16#0004;
    AL_DOOR : WORD := 16#0008;  AL_IGN : WORD := 16#0010;
    (* interlock limits *)
    P_MAX : REAL := 2.0;  T_MAX : REAL := 70.0;
    GAS_LOSS_FRAC : REAL := 0.5;  GAS_LOSS_DELAY : REAL := 3.0;
    IGNITE_TIMEOUT : REAL := 10.0;  SAFE_STOP_TIME : REAL := 10.0;
  END_VAR

  VAR  (* field I/O *)
    gas_raw   AT %IW0 : INT;    (* sccm *)
    liq_raw   AT %IW1 : INT;    (* 0.01 mL/min *)
    temp_raw  AT %IW2 : INT;    (* 0.1 degC *)
    press_raw AT %IW3 : INT;    (* 0.001 bar abs *)
    door_closed AT %IX0.0 : BOOL;
    start_btn AT %IX0.1 : BOOL;
    stop_btn  AT %IX0.2 : BOOL;
    ack_btn   AT %IX0.3 : BOOL;
    plasma_ok AT %IX0.4 : BOOL;   (* plasma detection feedback *)
    gas_sp_out AT %QW0 : UINT;    (* sccm *)
    pump_out   AT %QW1 : UINT;    (* 0.1 % *)
    cool_out   AT %QW2 : UINT;    (* 0.1 % *)
    power_out  AT %QW3 : UINT;    (* W *)
    state_out  AT %QW4 : UINT;
    alarm_out  AT %QW5 : UINT;    (* alarm word, bits above *)
    plasma_on  AT %QX0.0 : BOOL;
  END_VAR

  VAR  (* recipe, written by the supervisor over Modbus holding registers; latched on recipe_load in IDLE *)
    rec_purge_gas   AT %MW0 : INT;   (* sccm *)
    rec_purge_time  AT %MW1 : INT;   (* s *)
    rec_process_gas AT %MW2 : INT;   (* sccm *)
    rec_liquid      AT %MW3 : INT;   (* 0.01 mL/min *)
    rec_ramp        AT %MW4 : INT;   (* 0.01 mL/min per s *)
    rec_power       AT %MW5 : INT;   (* W *)
    rec_temp        AT %MW6 : INT;   (* 0.1 degC *)
    rec_run_time    AT %MW7 : INT;   (* s *)
    recipe_load     AT %MX0.0 : BOOL;
  END_VAR

  VAR
    (* active recipe, engineering units *)
    (* defaults = sim.skid.Recipe, used until the first recipe_load *)
    purge_gas : REAL := 300.0;  process_gas : REAL := 200.0;
    liquid_sp : REAL := 6.0;  liquid_ramp_rate : REAL := 1.0;
    power_sp : REAL := 120.0;  temp_sp : REAL := 45.0;
    purge_time : REAL := 10.0;  run_time : REAL := 60.0;
    gas_flow, liq_flow, temp, press : REAL;
    gas_sp, liq_ramp, liq_sp, pump, cool, power : REAL;
    state : INT := 0;
    t_state, gas_loss_t, settle_t : REAL;
    alarms, cond : WORD;
    active, low_gas, stable : BOOL;
    start_trig, ack_trig, load_trig : R_TRIG;
    pid_liq, pid_temp : PID_CTRL;
  END_VAR

  (* --- scaling and edges *)
  gas_flow := INT_TO_REAL(gas_raw);
  liq_flow := INT_TO_REAL(liq_raw) * 0.01;
  temp := INT_TO_REAL(temp_raw) * 0.1;
  press := INT_TO_REAL(press_raw) * 0.001;
  start_trig(CLK := start_btn);
  ack_trig(CLK := ack_btn);
  load_trig(CLK := recipe_load);
  t_state := t_state + DT_S;
  active := (state >= ST_PURGE) AND (state <= ST_RUN);

  IF load_trig.Q AND state = ST_IDLE THEN
    purge_gas := INT_TO_REAL(rec_purge_gas);
    purge_time := INT_TO_REAL(rec_purge_time);
    process_gas := INT_TO_REAL(rec_process_gas);
    liquid_sp := INT_TO_REAL(rec_liquid) * 0.01;
    liquid_ramp_rate := INT_TO_REAL(rec_ramp) * 0.01;
    power_sp := INT_TO_REAL(rec_power);
    temp_sp := INT_TO_REAL(rec_temp) * 0.1;
    run_time := INT_TO_REAL(rec_run_time);
  END_IF;

  (* --- interlock conditions *)
  cond := 16#0000;
  IF press > P_MAX THEN cond := cond OR AL_PRESS; END_IF;
  IF temp > T_MAX THEN cond := cond OR AL_TEMP; END_IF;
  IF active AND NOT door_closed THEN cond := cond OR AL_DOOR; END_IF;
  low_gas := active AND (gas_flow < GAS_LOSS_FRAC * gas_sp);
  IF low_gas THEN gas_loss_t := gas_loss_t + DT_S; ELSE gas_loss_t := 0.0; END_IF;
  IF gas_loss_t >= GAS_LOSS_DELAY THEN cond := cond OR AL_GAS; END_IF;

  (* --- latch; a new alarm or the stop button sends an active sequence to SAFE_STOP.
         Acknowledge only clears alarms whose condition is gone. *)
  IF active AND ((cond AND NOT alarms) <> 16#0000 OR stop_btn) THEN
    state := ST_SAFE; t_state := 0.0;
  END_IF;
  alarms := alarms OR cond;
  IF ack_trig.Q THEN alarms := alarms AND cond; END_IF;

  (* --- sequence *)
  IF state = ST_IDLE THEN
    IF start_trig.Q AND alarms = 16#0000 AND door_closed THEN
      liq_ramp := 0.0; settle_t := 0.0;
      state := ST_PURGE; t_state := 0.0;
    END_IF;
  ELSIF state = ST_PURGE THEN
    IF t_state >= purge_time THEN state := ST_RAMP; t_state := 0.0; END_IF;
  ELSIF state = ST_RAMP THEN
    liq_ramp := MIN(liquid_sp, liq_ramp + liquid_ramp_rate * DT_S);
    stable := (liq_ramp >= liquid_sp) AND (ABS(liq_flow - liquid_sp) <= 0.05 * liquid_sp);
    IF stable THEN settle_t := settle_t + DT_S; ELSE settle_t := 0.0; END_IF;
    IF settle_t >= 3.0 THEN state := ST_IGNITE; t_state := 0.0; END_IF;
  ELSIF state = ST_IGNITE THEN
    IF plasma_ok THEN
      state := ST_RUN; t_state := 0.0;
    ELSIF t_state >= IGNITE_TIMEOUT THEN
      alarms := alarms OR AL_IGN;
      state := ST_SAFE; t_state := 0.0;
    END_IF;
  ELSIF state = ST_RUN THEN
    IF t_state >= run_time THEN state := ST_SAFE; t_state := 0.0; END_IF;
  ELSIF state = ST_SAFE THEN
    IF t_state >= SAFE_STOP_TIME THEN state := ST_IDLE; t_state := 0.0; END_IF;
  END_IF;

  (* --- outputs per state *)
  gas_sp := 0.0; liq_sp := 0.0; power := 0.0; plasma_on := FALSE; cool := 0.0;
  IF state = ST_PURGE OR state = ST_RAMP OR state = ST_SAFE THEN gas_sp := purge_gas; END_IF;
  IF state = ST_IGNITE OR state = ST_RUN THEN
    gas_sp := process_gas; plasma_on := TRUE; power := power_sp;
  END_IF;
  IF state = ST_RAMP THEN liq_sp := liq_ramp; END_IF;
  IF state = ST_IGNITE OR state = ST_RUN THEN liq_sp := liquid_sp; END_IF;

  (* flow PID runs from RAMP to RUN; temperature PID (reverse acting, drives cooling) in IGNITE/RUN *)
  pid_liq(enable := state >= ST_RAMP AND state <= ST_RUN, reverse := FALSE,
          sp := liq_sp, pv := liq_flow, kp := 5.0, ki := 2.0, kd := 0.0,
          out_min := 0.0, out_max := 100.0, dt := DT_S);
  pump := pid_liq.out;
  pid_temp(enable := state = ST_IGNITE OR state = ST_RUN, reverse := TRUE,
           sp := temp_sp, pv := temp, kp := 8.0, ki := 0.5, kd := 0.0,
           out_min := 0.0, out_max := 100.0, dt := DT_S);
  cool := pid_temp.out;
  IF state = ST_SAFE AND temp > 30.0 THEN cool := 100.0; END_IF;

  gas_sp_out := REAL_TO_UINT(gas_sp);
  pump_out := REAL_TO_UINT(pump * 10.0);
  cool_out := REAL_TO_UINT(cool * 10.0);
  power_out := REAL_TO_UINT(power);
  state_out := INT_TO_UINT(state);
  alarm_out := WORD_TO_UINT(alarms);
END_PROGRAM


CONFIGURATION Config0
  RESOURCE Res0 ON PLC
    TASK Main(INTERVAL := T#100ms, PRIORITY := 0);
    PROGRAM Inst0 WITH Main : PlasmaSkid;
  END_RESOURCE
END_CONFIGURATION

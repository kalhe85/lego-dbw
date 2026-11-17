# v1.1 on-car test plan

The v1.1 features are switched on in **stages**. Test one stage at a time, in order, and only move on when every check of the current stage passes. Each stage adds one feature to the previous one.

| Stage | Adds | Risk | Where |
|---|---|---|---|
| 0 | Nothing new: v1.0 behaviour with the new code layout (rollback build) | none | stand |
| 1 | PIO-based PWM capture | low | stand |
| 2 | MAVLink telemetry to ArduPilot | none (read-only) | stand |
| 3 | Motor position feedback (steering angle, drive speed) | low (BLE load) | stand, then ground |
| 4 | Automatic steering calibration + drive interlock | **medium**: steering moves by itself | stand |
| 5 | Curvature-linear steering mapping | low | ground, slow |
| 6 | Closed-loop drive speed | **medium**: more torque under load | stand, then ground |
| 7 | Speed-dependent steering gain | low | ground |

## How to build and flash a stage

```bash
cmake -S . -B build -DDBW_STAGE=N
cmake --build build -j
picotool load -fx build/lego_dbw.uf2
```

The boot log confirms what is running:

```
=== LEGO DBW Pico W v1.1-dev (stage N) ===
features: pio=1 telem=1 feedback=1 autocal=0 curvature=0 speedloop=0 speedgain=0
```

**Rollback** at any time: build stage 0, or flash the `main` branch (v1.0).

## Before every stage

- Car on a stand, wheels off the ground (unless the stage says "ground").
- Fresh batteries in the hub.
- Hub on → Pico on → wait for `[BLE] READY` → then flip the enable switch on.
- Keep the serial log open (or let Claude Code read it) and save it if anything looks wrong.

---

## Stage 0: baseline

Confirms the restructured code still behaves like v1.0.

- [ ] Boot log shows `stage 0` and `[PWM] capture: GPIO IRQ`.
- [ ] Drive and steering work exactly as in v1.0.
- [ ] Enable off → motors stop, steering centres.

## Stage 1: PIO PWM capture

PWM pulse widths are now measured by PIO state machines instead of GPIO interrupts.

- [ ] Boot log shows `[PWM] capture: PIO` (if it says `GPIO IRQ`, PIO resources were not available. Report it).
- [ ] `[DBG]` shows `thr`, `str`, `en` near 1500 µs at neutral and about 1000/2000 µs at the ends. Values match what ArduPilot outputs (Mission Planner → Servo Output) within ±3 µs.
- [ ] Values are steady at rest (jitter ≤ 2 µs).
- [ ] Unplug the S1 wire → `thr` shows `(stale)` and `armed=0` within 250 ms. Plug it back → recovers.
- [ ] Drive and steering work as in stage 0.

## Stage 2: telemetry

**Wiring:** Pico **GP0** (physical pin 1) → a free UART **RX** pad on the flight controller. Ground is already shared.

**ArduPilot:** for the serial port n that the pad belongs to: `SERIALn_PROTOCOL = 2` (MAVLink2), `SERIALn_BAUD = 115`. Reboot the flight controller. `TELEM_SYSID` in `src/config.h` must equal `SYSID_THISMAV` (default 1).

Skip this stage if you don't want telemetry; it does not affect driving.

- [ ] Mission Planner → Ctrl+F → **MAVLink Inspector**: a component **25** appears with `HEARTBEAT` and `NAMED_VALUE_FLOAT`.
- [ ] Values `DBW_LINK`, `DBW_ARM`, `DBW_THR`, `DBW_STRCMD`, `DBW_BATT`, `DBW_DROP` update about twice a second and match the `[DBG]` line.
- [ ] Messages tab shows `DBW: hub connected` after the hub connects.
- [ ] Switch the hub off → `DBW: hub disconnected 0x..` appears.

## Stage 3: motor feedback

The hub now reports the steering motor position (every degree) and drive motor A position (every 30°). This is read-only, but it adds BLE traffic, so this stage is mainly a **link stability** check.

- [ ] `[DBG]` shows `pos=` and `spd=` fields. `[HUB] port 0x03 mode 2 notify=1` and `[HUB] port 0x00 mode 2 notify=1` appear after READY.
- [ ] Move the steering stick: `pos` follows `steer` within a few degrees once the motor settles.
- [ ] Throttle forward: `spd` is positive and roughly proportional to throttle; reverse gives a negative value. **Write down `spd` at full throttle** (needed for stage 7).
- [ ] Drive for 5 minutes on the stand with frequent stick movement: **no disconnects**, and `drop` does not climb steadily. If it does, increase `DRIVE_POS_DELTA_DEG` (e.g. 60) and repeat.
- [ ] Short drive on the ground: same, no disconnects.

## Stage 4: steering auto-calibration ⚠

On the first enable after the hub connects, the steering sweeps to both end-stops at low power (`STEER_CAL_POWER`, 35 %), then centres on the midpoint. The usable range is set to 90 % of the measured half-span, so normal steering never pushes into the stops. **The drive is locked** during calibration, and after it until the throttle has been seen at neutral.

Keep fingers away from the steering linkage.

- [ ] Turn the wheels to one side by hand, then switch the hub on. Wait for READY.
- [ ] Throttle at neutral, flip enable on → log shows `[CAL] starting`, `sweeping`, then
      `[CAL] done: end-stops A / B, centre C, usable +/-D motor deg` within about 5 s.
      **Report A, B, C, D.**
- [ ] After calibration the wheels point straight ahead, even though they were turned when the hub was switched on.
- [ ] Full stick left/right reaches (nearly) full lock **without the motor buzzing or straining**.
- [ ] No `HIGH CURRENT` alert during the sweep.
- [ ] During the sweep, push the throttle: wheels do not turn (`[DBG] ... cal=sweep± lock`).
- [ ] After `done`, with the throttle still held: drive stays at 0 until you return the stick to neutral.
- [ ] Flip enable off during a sweep → `[CAL] aborted`. Flip on again → calibration restarts.
- [ ] Hold the throttle while flipping enable on → no calibration until the throttle returns to neutral.
- [ ] Switch the hub off and on with enable still on → after READY, `[DBG]` shows `cal=idle lock (flip enable off/on to calibrate)` and the drive stays locked. Flip enable off → on → calibration runs.

If calibration fails, the log names the reason, and the gateway falls back to v1.0 steering (centre = power-on position):
- `range too small`: the motor could not move the linkage. Raise `STEER_CAL_POWER` by 5–10.
- `no end-stop found before timeout`: the linkage slips or has no hard stop. Report it.
- `no position feedback`: stage 3 checks failed. Report the log.

## Stage 5: curvature steering map

The stick now commands path curvature instead of wheel angle (bicycle model). Full stick is still full lock. Half stick gives about 57 % of full lock instead of 50 %, so small corrections are a bit livelier and the turning radius changes evenly along the stick.

- [ ] On the stand: half stick → `steer` in `[DBG]` ≈ 0.57 × D (D from stage 4).
- [ ] On the ground, slowly: steering feels natural; straight-line driving is not twitchy.

## Stage 6: closed-loop drive speed ⚠

Throttle now sets a **speed** (in % of the motor's maximum). The hub's own speed controller adds power under load to hold that speed. Acceleration is ramped over `DRIVE_ACCEL_MS` (300 ms); stopping is not ramped. With the stick at neutral the motors **actively hold** (brake). Enable off → motors float, as before.

On the stand first:
- [ ] 30 % throttle → wheels spin steadily; `spd` stays roughly constant.
- [ ] Gently slow one wheel with a cloth: `spd` recovers (the motor pushes harder). In stage 5 it would just slow down.
- [ ] Stick back to neutral → wheels stop quickly.
- [ ] Enable off while spinning → motors stop (float).
- [ ] No `LOW VOLTAGE` / `HIGH CURRENT` alerts and no hub resets at full throttle. If the hub resets under load, lower `DRIVE_MAX_POWER` (e.g. 80).

Then on the ground:
- [ ] Very low speeds are now possible and steady (crawl on carpet).
- [ ] Driving feels as before or better; no jerks when starting.

ArduPilot note: in autonomous modes, retune the speed controller (`CRUISE_SPEED`, `CRUISE_THROTTLE`, `ATC_SPEED_*`), because throttle now means speed rather than power.

## Stage 7: speed-dependent steering gain

Steering authority is reduced at speed: 100 % below `STEER_GAIN_FADE_START_DPS` (300 °/s motor speed), down to `STEER_GAIN_MIN_PCT` (55 %) at `STEER_GAIN_FADE_END_DPS` (1200 °/s).

- [ ] Set `STEER_GAIN_FADE_END_DPS` to about 80 % of the full-throttle `spd` you noted in stage 3, and `FADE_START` to about 25 %.
- [ ] On the ground: at walking speed, full stick gives full lock. At high speed, full stick gives a wider, controllable turn and no spin-outs.
- [ ] `[DBG]`: at full stick and `spd` above FADE_END, `steer` ≈ 0.62 × D (stick scaled to 55 %, then through the curvature map of stage 5).

ArduPilot note: in autonomous modes, ArduPilot's steering controller (`ATC_STR_RAT_*`) sees this as a change in steering gain at speed. If you tune those, consider building with `-DFEAT_STEER_SPEED_GAIN=0`.

---

## What to send back after each stage

- The checklist with pass/fail.
- The boot log and a minute of `[DBG]` lines.
- Stage 3: `spd` at full throttle. Stage 4: the `[CAL] done` line.
- Any `[BLE] disconnected`, `[HUB] alert` or `WATCHDOG` lines.

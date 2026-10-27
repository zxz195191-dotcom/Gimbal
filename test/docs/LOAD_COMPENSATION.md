# Pitch/Yaw load compensation bench procedure

All new controls default OFF at MCU boot. Existing Pitch calibration remains 42
bidirectional nodes over 0..41 degrees. No Flash writes, new homing motion, or
360-degree scan is added. All angles below use the current first-enable encoder
zero, not an absolute world heading. Calibration results live in RAM.

## Preconditions: do these before ycalarm

1. Identify the actual motor model/firmware. Read the driver mode and P_MAX,
   V_MAX, T_MAX: this firmware assumes MIT, 12.5 rad, 30 rad/s, 10 N.m.
2. In a de-energized, supported test setup, compare a small measured mechanical
   displacement to the change of `dm_motor[Yaw].p_int` and `y_enc`. Confirm both
   sign and magnitude; do not assume the internal reducer needs an extra ratio.
3. Repeat at two positions and after reboot. Software zero is recaptured on the
   first enable. Confirm the physical interval corresponding to y_enc=10..40 is
   clear of stops and cable hazards. `ycalarm` is the operator's confirmation of
   these checks, NOT automatic proof that the historical multi-turn bug is fixed.
4. Confirm positive logical motion has positive logical velocity. Torque samples
   are `GIMBAL_YAW_DIRECTION * dm_motor[Yaw].t_int`. The latter is the driver's
   decoded torque estimate in nominal output N.m, not an independent shaft torque
   sensor. Its scale depends on driver T_MAX and calibration. A known low load and
   the driver monitor should agree in sign before identification is trusted.
5. Use normal encoder mode. Do not arm if mapping, raw-position continuity, driver
   configuration or torque units are unresolved. Hardware motor-power stop must
   be reachable independently of Bluetooth.

## Commands (newline terminated)

| Command | Behavior |
|---|---|
| `ye` | Existing Yaw enable; new Yaw compensation is reset/off |
| `ymode 0` | Existing encoder control; clears new Yaw compensation |
| `y 12 3` | Existing absolute target, degrees and degrees/sec; verify actual position |
| `ycalarm` | Arm once, only if y_enc is 11..13 degrees, nearly stopped, no repeat/FF/I |
| `ycal` | One calibration transaction: three 12->38->12 degree laps at 5 deg/s |
| `ycalstop` | Cancel calibration and DISABLE Yaw; does not hold position |
| `yd` / `gd` | Existing Yaw/all gimbal disable; also clears calibration and Yaw FF/I |
| `yff 1` / `yff 0` | Enable/disable identified Yaw friction; ON requires a valid result |
| `yi 1` / `yi 0` | Enable/disable Yaw holding integral |
| `pi 1` / `pi 0` | Enable/disable Pitch holding integral |
| `pff 1` / `pff 0` | Existing Pitch physical table enable/disable |
| `ystat` / `ydump` | Snapshot/export last calibration and current enable flags |

Check `#YCAL cmd=... ok=1` after each new command. Rejection reports `ok=0`.
Do not stream commands automatically. During calibration, external Yaw target,
gain, speed, repeat and mode changes are rejected; stop commands remain available.
New compensation is for encoder mode only. `pcal`/`pscan` suspend the new Pitch
integral; its state is cleared so samples do not include this new correction.

Calibration targets are finite endpoints 12/38, leaving margin inside 10/40.
The supervisor aborts and disables Yaw for actual position outside 10..40,
feedback older than 50 ms, impossible position jumps, absolute speed >12 deg/s,
absolute feedback torque >1.5 N.m, command/actual error >3 deg, stalled progress,
phase timeout >12 s, total timeout >75 s, or a raw p_des outside +/-12.4 rad.
These are conservative software checks, NOT certified physical travel guarantees.
Do not enlarge them to make a failed calibration pass before finding the cause.

## Sampling and model

Only 15,20,25,30,35 degree nodes are identified, each with a +/-0.75 degree bin.
The 10/40 boundary values are deliberately not claimed as measured constant-speed
nodes. Each direction requires >=9 fresh feedback samples per node per lap,
limited to 31 samples/bin, collected at most once per 10 ms. Sampling requires
signed speed 4..6 deg/s for 250 ms, >=500 ms after phase start, and remaining
planned travel >1 degree. Each bin uses a median, not a single crossing sample.

For each of three laps, B=(Tplus+Tminus)/2 and F=(Tplus-Tminus)/2. Negative F beyond
0.03 N.m invalidates the result. Repeat F must stay within 0.05+0.2*F N.m of the
three-lap mean; B spread must stay within 0.10 N.m. A table is selected only when
its angle span exceeds max(0.08 N.m,25% mean F) AND three times repeat spread.
Otherwise one mean F is used. These are engineering screening thresholds, not
statistical proof of model validity; repeat the run to confirm.

Yaw applies F only: B is exported as a diagnostic, not called gravity or applied.
F is an effective friction estimate at 5 deg/s and may include viscous friction.
A single speed cannot separately identify Coulomb friction, viscous coefficient,
and inertia J. No J*alpha feedforward is applied (J=0 in this first stage).
Table interpolation is linear; within 10..40 but outside 15..35 it uses nearest
measured F, and outside 10..40 it fades to zero. Above 6 deg/s the Yaw friction
target is zero because that speed was not identified. Direction comes from the
planned command increment, with a 1 deg/s zero-speed blend and 1 N.m/s slew.

## Holding integral and limits

One `LoadComp_Update` is shared by both axes. Separate configurations:

| Item | Yaw | Pitch |
|---|---:|---:|
| Ki (N.m/(rad*s)) | 2 | 1 |
| integral clamp (N.m) | +/-0.35 | +/-0.15 |
| new additive torque clamp (N.m) | +/-1 | +/-0.15 |
| physical+new torque clamp (N.m) | +/-1 | +/-10 (existing MIT range) |

Accumulate only after the planned target is reached, measured speed <=1 deg/s,
error <=3 degrees, moving-friction term decayed below 0.01 N.m, and 300 ms dwell.
No accumulation inside a 0.1-degree deadband. Conditional integration checks both
additive limits and estimated MIT PD+FF saturation. Motion or disabling integral
unwinds at 1 N.m/s; fault/disable clears immediately. Scheduling gaps >20 ms reset
the new compensation. Pitch's physical table is kept in a separate base variable
so integral cannot be fed back into the legacy hold value and accumulate twice.
The old Pitch output is untouched when new compensation is off.

## Reports / ordinary telemetry

Ordinary numeric telemetry remains seven fields, ending in tx_dt_ms. A report
temporarily substitutes seven short text lines (header, five nodes, terminator),
sent by the same DMA buffer, one per telemetry slot; it never waits for UART BUSY.
At 50 Hz this normally takes 140 ms. The next numeric tx_dt_ms includes this gap.
Save raw received text; FireWater graphs ignore the `#YCAL`/`#YNODE` text. Cube has
no extra channels. Reports are snapshots and are not re-read while DMA is active.

State: 0 idle,1 settle-low,2 forward,3 settle-high,4 reverse,5 done,6 abort.
Error: 0 OK,1 cancel,2 bounds,3 feedback,4 jump,5 speed,6 torque,7 tracking,
8 timeout,9 stall,10 insufficient samples,11 repeatability,12 sign,13 mode/axis fault.
`ready=1,state=5,error=0` is necessary before `yff 1` succeeds. `table=0` means
constant F, not missing data. Inspect nonzero B before relying on the model.

## First run and acceptance

1. Complete the mapping/precondition checks. Stop old `ytest`, confirm encoder
   mode, keep Kp/Kd fixed and `yff 0`, `yi 0`, `pi 0`. Stabilize actual y_enc within
   11..13; `y 12 3` may require a small adjustment because of uncompensated error.
2. Send `ycalarm`, inspect ok=1/armed=1, then `ycal`. Observe all three laps.
   Any unexpected movement: `ycalstop` or hardware stop. Do not auto-retry faults.
3. Save the automatic result or `ydump`. Check all 30 bin counts (3*2*5), positive
   F, repeatability and B. Repeat once with compensation still off and compare F.
4. Compare old PD, then `yff 1`, then `yi 1` as three separate experiments. Pitch
   `pi 1` is a separate optional experiment after Yaw acceptance, not required to
   run Yaw calibration. Never enable it automatically at boot.
5. Within verified travel, approach 20 degrees from 15 and 25; approach 40 from
   below and above ONLY IF an external mechanical check allows travel above 40.
   The calibration's 10..40 authorization alone does not allow an above-40 test.
   Otherwise use 35 as the reversible interior point and mark the reverse 40 test
   blocked. Test absolute targets with `y <angle> 5`; no automatic scan is needed.
6. Test 20->21->20->19->20 degree steps at 3 deg/s, one command at a time. Wait
   up to 30 s for holding integral; record final 3 s after settling, repeat each
   direction at least 5 times. Do not change Kp, Kd and compensation together.
7. Proposed pass: all repeated trials have |mean steady error|<=0.3 degree,
   final 3 s peak-to-peak<=0.2 degree, no sustained oscillation/creep, each 1-degree
   step moves in the intended direction without skipped steps, and return repeat
   error<=0.3 degree. Verify actual mechanical motion with an independent mark,
   not only the same encoder used for control. No faults, no saturation plateau.

Compilation and simulations cannot establish hardware mapping or these motion
criteria. Leave new features off if the preconditions or recorded tests fail.

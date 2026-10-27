# Yaw single-round friction calibration

Implemented 2026-10-10. Host simulation and firmware compilation are not hardware acceptance.

## Operation

With Yaw enabled in encoder mode, `ycal` captures current encoder-relative angle C.
The existing trajectory is C -> C-25 -> C+25 -> C-25 degrees, 5 deg/s,
one bidirectional pass with existing 500 ms endpoint dwells (about 26.5 s).
`ycalstop` remains the explicit disable/stop command. No new commands.

Nine nodes C-20 through C+20, spaced 5 deg, are opportunities for actual
encoder crossing interpolation, not mandatory waypoints. Record each actual
direction once. Keep collecting in endpoint dwells; a commanded reversal is
not evidence of an actual reversal. Duplicate CAN frames cannot add records.

Require at least 3 distinct paired nodes. Compute signed Fi=(Tplus-Tminus)/2
and Bi=(Tplus+Tminus)/2; average paired nodes with equal weight. Require finite
means and nonnegative overall F. Do not clamp individual Fi to zero. No repeated
scan, velocity-stability gate, sample-window quota, or exact-arrival gate.

Committed F is separate from candidate node values. Only valid results replace
it. Success enables Yaw FF; sample/sign failure holds current position, keeps
the committed F and restores the previous FF switch. Integral remains off.
Hardware failures still disable via the existing mechanisms.

Constant F has no calibration-center angle cutoff or 6 deg/s switch. Output
direction follows planned velocity and existing near-zero smoothing. Existing
1 Nm command cap and 1 Nm/s slew remain; MIT 12-bit quantization adds its usual
rounding error. Identification at 5 deg/s does not establish accuracy at other
speeds or remove static PD holding error.

## Output

Ordinary nine-column telemetry is unchanged. Completion prints #YCAL, nine
#YNODE lines and #YCAL_END. Summary includes candidate F, paired/9, active_F,
ff_limit, yff/yi/pi. Missing candidate node values print nan, never an old table
value disguised as a current sample. Counts are 0/1 actual crossing records.

Rejected data prints YCAL FAILED, error, paired/minimum, center, old_kept, HOLD,
and angle:positive_count/negative_count for every node. Repeats 3 times at 1 s
spacing; ordinary telemetry resumes after the existing 5 s reading pause.

## Host verification

Run tests/pitch_continuous/run.ps1. The integration test extracts the actual
production float conversion and dm4310_send_mit implementations at test time,
captures CAN bytes and decodes them to drive a PD + inertia + dry/viscous friction
plant. It does not inject completed calibration arrays or force calibration
states. Both GIMBAL_YAW_DIRECTION signs are tested.

Observed results before firmware installation:

| Case | Result |
|---|---|
| Fc=0.4 Nm, damping=0.1, J=0.02, 2 ms feedback | 9 pairs; F=0.41016 Nm; 26502 ms |
| MIT at +5 deg/s | +0.40781 Nm decoded |
| MIT at -20 deg/s | -0.41270 Nm decoded |
| Recalibrate Fc=0.6 | 9 pairs; F=0.60927 Nm |
| Freeze shaft on next trial | 0 pairs; fail; ACTIVE hold; old F and enabled FF retained |
| Resume after failed trial | +0.60806 Nm decoded at +20 deg/s |
| Kp=4, J=0.08, 17 ms feedback | 7 pairs; F=0.40876 Nm |
| Fc=1.4, normal plant | F=1.40844; command capped to 1; +0.99878 Nm decoded |
| Axis direction -1 | Matching F; CAN torque signs reverse |

Separate model tests cover 3-pair adoption, 1/0-pair rejection retaining old F,
5-pair update, negative overall F rejection, crossing after command arrival,
continued old-direction motion after command reversal, duplicate frames,
invalid feedback and total deadline. Integration also asserts stale feedback,
motor error, overtemperature, CAN TX error, manual stop, stopped FF decay,
default disabled integrals, and fault-report repetition/pause recovery.
Existing Pitch continuous and Yaw inertial normal/reversed tests remain passing.

## Hardware still required

Confirm driver torque feedback scale/sign, mechanical transmission ratio,
real CAN timing, actual friction/load/temperature variation and stopping behavior.
No flashing or physical motor test was performed by the agent.

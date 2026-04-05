# Sensor Contract And Datasets

Updated: 2026-04-05

## Sign Contract

This firmware fixes one physical sign convention end-to-end.

| Physical motion | Steering axis | Motor axis | Encoder count | DIR pin | Pulse enum |
|---|---:|---:|---:|---:|---|
| Clockwise (CW) | `+steering_deg` | `+motor_deg` | `+encoder_count` | `1` | `DIR_CW` |
| Counter-clockwise (CCW) | `-steering_deg` | `-motor_deg` | `-encoder_count` | `0` | `DIR_CCW` |

Code owners:
- `project_params.h`
- `encoder_reader.c`
- `adc_potentiometer.c`
- `app_runtime.c`

Related compile-time knobs:
- `ENCODER_COUNT_POLARITY`
- `ADC_POT_STEERING_POLARITY`
- `DIR_ACTIVE_HIGH_FOR_CW`
- `SENSOR_POSITIVE_STEERING_IS_CW`
- `SENSOR_POSITIVE_MOTOR_IS_CW`

Bench acceptance:
- Hand CW/CCW rotation must make `[ENCDBG] signed_delta`, CSV `enc_deg`, and keyboard snapshot `C/ADC/XERR` follow the table above.
- `+1 deg` and `-1 deg` commands must keep `target/current/error/DIR` sign-consistent.

## Runtime Sensor Contract

Sampling:
- `EncoderReader_Service()` updates the encoder cache every fast tick.
- `ADC_Pot_Service()` updates the ADC cache with a non-blocking conversion path.
- `sample_tick_ms` means cache update time.
- `age_ms` means `now - sample_tick_ms`.

Fault linkage:
- Encoder stale: warning at `20 ms`, fault at `50 ms`.
- ADC stale: warning at `20 ms`, fault at `50 ms`.
- Runtime cross-check: warning above `2.0 deg`, fault above `5.0 deg`.
- Homing cross-check: fault above `2.0 deg`.
- Encoder plausibility:
  - velocity warning at `60 deg/s`, fault at `120 deg/s`
  - acceleration warning at `50000 deg/s^2`, fault at `150000 deg/s^2`
- Direction mismatch is evaluated only when applied pulse output is large enough to expect real encoder motion.

Safe action:
- Any sensor fault path emits a `[SENSOR][FAULT]` log with reason, flags, ages, validity bits, and calibration metadata.
- New sensor faults call `PositionControl_EmergencyStop()`.
- Control enable is blocked when homing is incomplete or sensor faults are active.

## ADC Calibration Contract

Storage format:
- medium: backup SRAM
- format version: `ADC_POT_CALIBRATION_VERSION`
- fields: `min_raw`, `max_raw`, `min_angle_deg`, `max_angle_deg`, `checksum`
- integrity: FNV-style checksum over the calibration blob payload

Restore policy:
- if `magic/version/checksum/range` are valid, restore calibration on boot
- otherwise fall back to default range `0..4095` and `-45..+45 deg`
- logs show whether calibration came from restore, config, or default fallback

Operational notes:
- `ADC_Pot_Calibrate()` is now a two-step capture:
  - first call captures the low point
  - second call captures the high point and saves the blob
- runtime sampling stays non-blocking; only calibration workflow is operator-driven

## Required Datasets

### `encoder_longrun`

Purpose:
- verify unwrap continuity
- verify sign contract over long CW/CCW motion
- check stale/plausibility false positives during steady operation

Minimum record:
- run id
- build / git sha
- direction pattern
- sample count or duration
- max `enc_age_ms`
- count discontinuity count
- sensor warn/fault summary

Pass:
- no unwrap discontinuity
- no unexpected stale fault
- sign of `signed_delta` matches physical direction

### `adc_noise`

Purpose:
- measure raw noise and angle stability at standstill
- verify calibration restore metadata

Minimum record:
- run id
- restored calibration version/checksum
- raw min/max/mean
- angle min/max/mean
- sample count
- invalid bit occurrences

Pass:
- no disconnect/range/timeout fault
- low false `JUMP` / `STUCK` rate at standstill

### `sensor_crosscheck`

Purpose:
- compare encoder steering angle vs ADC steering angle on the same axis
- verify runtime mismatch warning/fault thresholds

Minimum record:
- run id
- command profile
- `enc_deg`
- `adc_deg`
- `xerr_deg`
- `sen_warn`
- `sen_fault`
- `sen_reason`

Pass:
- normal runs stay within warning threshold
- injected bias or polarity inversion produces `[SENSOR][WARN]` then `[SENSOR][FAULT]`

## Triage Order

1. Check `[SENCFG]` once after boot and confirm the physical sign contract used by the build.
2. Check `[ENCDBG]` and keyboard snapshot for `DIR`, `signed_delta`, `enc_deg`, and `adc_deg`.
3. If `DIR` is correct but encoder sign is opposite, inspect `ENCODER_COUNT_POLARITY` and encoder wiring.
4. If encoder and ADC both move with the same sign but `xerr_deg` grows, inspect homing/calibration and mechanical coupling.
5. If `adc_valid` shows `DISCONNECT`, `RANGE`, or `TIMEOUT`, fix ADC path before controller tuning.
6. If only `direction_mismatch` faults appear, inspect pulse/direction output polarity, drive acceptance, and motor motion.
7. If only velocity/accel plausibility faults appear, inspect encoder glitches, timer capture noise, and jump injection setup.

## Expected Evidence Bundle

- one boot log with `[SENCFG]`
- one CW/CCW manual sign check log
- one `+1 deg / -1 deg` snapshot set
- one `encoder_longrun` CSV
- one `adc_noise` CSV
- one `sensor_crosscheck` CSV

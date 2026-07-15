# M2006 Hybrid Commissioning

This procedure applies to the ZeroOne X6 V2 hybrid airframe with two DJI
M2006/C610 controllers on CAN1, left ID 1, right ID 2, and the transformation
servo on M8. Software build and host-test results do not constitute hardware
validation.

## Safety prerequisites

- Suspend both wheels and unload the transformation linkage.
- Provide a physical emergency stop that removes C610 power. A flight-controller
  or CAN power loss cannot transmit a final zero-current frame.
- Connect a CAN analyzer and an oscilloscope to M8 before enabling motion.
- Confirm CAN1 is terminated, uses 1 Mbit/s, and has no DroneCAN or Cyphal
  device attached. This firmware does not compile the Cyphal backend.
- Keep `M2K_SPD_P`, `M2K_SPD_I`, and `M2K_SPD_FF` at zero until the checks below
  pass. Commander intentionally rejects uncalibrated driving.

## CAN wheel bench test

1. Verify `UAVCAN_ENABLE=0`, `M2K_EN=1`, `M2K_L_ID=1`, `M2K_R_ID=2`, and
   `M2K_MAX_RPM=500`, then reboot after changing an ownership or ID parameter.
   If `m2006_can` is stopped, reboot before starting it or DroneCAN again; the
   low-level CAN driver cannot be safely reinitialized in the same boot.
2. Limit the first test to 1000 current-command units:

   ```sh
   param set M2K_CUR_LIM 1000
   listener m2006_motor_status 5
   m2006_can status
   ```

3. On the analyzer, confirm one standard `0x200` zero-current command every
   2 ms, left current in bytes 0-1, right current in bytes 2-3, and zero bytes
   4-7. Confirm feedback only at `0x201` and `0x202`. Bytes 6-7 are ignored.
4. Complete the transformation-servo acceptance below before requesting wheel
   motion. Then, with wheels still suspended, temporarily use a small
   proportional gain and keep the other gains zero:

   ```sh
   param set M2K_SPD_P 1
   param set M2K_SPD_I 0
   param set M2K_SPD_FF 0
   ```

   Enter the verified Rover shape, arm, and command a low positive and negative
   Rover demand. Verify left/right identity and sign before using `M2K_L_REV`
   or `M2K_R_REV`; do not swap logical wheel mixing in the driver.
5. Power down or disconnect the CAN branch of one C610 without breaking bus
   termination. Both current commands must become zero,
   the relevant online flag must clear, and the fault must remain latched while
   armed. Reconnect, disarm, keep both feedback streams healthy for at least
   100 ms, then re-arm to verify recovery.
6. On the suspended-wheel bench, stop `hybrid_vehicle_control` while a low
   command is active. Verify that exceeding `M2K_CMD_TO` makes both commands
   zero and reports `DRIVE_FAULT_COMMAND`. Disarm before restarting
   `hybrid_vehicle_control`.
7. Force TX saturation/error conditions if the bench setup supports them.
   Record `tx_full_count`, `tx_error_count`, and aggregate `can_error_count`.
   The aggregate includes bus-off events; the low-level interface does not
   expose a separate trustworthy bus-off counter.
8. Remove physical CAN and then flight-controller power separately. Record the
   C610 behavior and stopping time. Do not accept software zero-frame behavior
   as coverage for total power loss.

Restore or deliberately increase `M2K_CUR_LIM` only after reviewing the log.
The software maximum is 10000.

## Transformation servo calibration

1. Disconnect or unload the linkage. Set conservative `PWM_MAIN_MIN8` and
   `PWM_MAIN_MAX8`, and verify the pulse width with an oscilloscope. M8 must be
   `PWM_MAIN_FUNC8=201`, `PWM_MAIN_DIS8=0`, and `PWM_MAIN_FAIL8=0`.
2. Assign `HYBRID_MAN_CH` to AUX1-AUX6 (`1`-`6`) on a channel different from
   `RC_MAP_TRANS_SW`, then enter the disarmed, prearmed state. Move the assigned
   AUX input by more than `0.5` from its previous value to activate direct M8
   commissioning and find safe normalized endpoints. A stale or non-finite RC
   input, loss of prearm, arming, or a transformation-switch movement must
   cancel commissioning. Set separated values in `HYB_SV_QUD` and
   `HYB_SV_ROV`; both must remain in `[-1, 1]`.
3. With `HYB_SENS_EN=1`, verify AS5600 reaches both targets within
   `HYB_ANG_TOL` and remains valid for `HYB_DBNC_T`. AS5600 has priority.
4. Make AS5600 unavailable and verify the target TMAG5273 device provides the
   fallback endpoint. Check `HYB_MAG_ID_QUD`, `HYB_MAG_ID_ROV`, and their
   thresholds against actual device IDs and measured fields.
5. Assert both TMAG endpoints or make AS5600 and TMAG disagree. Verify a
   conflict fault, `TRANSITION_FAULT`, no propulsion, and no valid M8 pulses.
6. Remove all enabled position feedback both during a transition and after a
   stable endpoint was confirmed. Verify propulsion is inhibited as soon as no
   fresh endpoint confirms the shape, followed by a sensor/transition fault and
   suppressed M8 output. Faults must not automatically degrade to time-only
   mode.
7. Disarm and confirm M8 has no valid pulse. A latched fault must also suppress
   M8 even during RC commissioning.
8. Set `HYB_SENS_EN=0` only for the explicit sensorless test. Verify completion
   occurs at `HYBRID_TRANS_T`, never early, and the stable target PWM remains
   applied while armed/prearmed. Re-enable sensing for the intended hardware.

Parameter snapshots apply atomically only while neither armed nor prearmed.
Reboot or fully leave prearm before judging a changed configuration.

## Tuning and evidence

Tune from the inside out in this mandatory order:

1. M2006 wheel speed (`M2K_SPD_P`, `M2K_SPD_I`, `M2K_SPD_D`,
   `M2K_SPD_FF`, and `M2K_RPM_SLEW`).
2. Rover yaw-rate loop.
3. Rover yaw-attitude loop.
4. Vehicle-speed loop.
5. Position, Offboard, and mission loops.

For every accepted test interval record:

- `m2006_motor_status` target RPM, measured RPM, speed error, current command,
  torque-current feedback, online flags, and fault/counter fields;
- `hybrid_vehicle_status` current/target state, sensor source, validity, elapsed
  transition time, and fault reason;
- the Rover loop setpoint, measurement, error, final actuator output, battery
  state, wheel load, surface, and proof that both wheels were outside unreliable
  static-friction start regions.

Use `rover_ulog_plot.py` for the Rover rate, attitude, and speed loops. Do not
tune an outer loop until its inner loop is accepted.

The current `RO_MAX_THR_SPEED=2.47` m/s is theoretical, derived from 500 rpm,
340 mm wheel diameter, and 3.6:1 reduction. Replace it with a loaded measured
maximum before final vehicle-speed or position tuning.

## Acceptance record

Record firmware commit, parameter export, analyzer capture, oscilloscope
capture, ULog, current limit, load condition, and pass/fail result for every
step. Real-vehicle validation remains pending until these artifacts exist.

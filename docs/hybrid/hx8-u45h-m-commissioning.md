# HX8-U45H-M Transformation Servo Commissioning

This guide applies only to the `zeroone_x6_hybrid` airframe and covers both
the legacy M8 PWM transformation-servo path and the FashionStar HX8-U45H-M
UART path. Host tests and a firmware build verify software behavior only. The
hardware described here has not been physically validated; every PWM and HX8
bench item below remains pending until its measurement artifacts are recorded.

## Safety boundary and wiring

- Unload the transformation linkage and provide a physical emergency stop that
  removes servo power before commissioning motion.
- Power the HX8 from an independent regulated 9.0--12.6 V supply rated for at
  least its 5.5 A peak current. Join the servo-supply and flight-controller
  common ground.
- Connect the flight-controller TX and RX pins to the corresponding full-duplex
  sides of an external automatic-direction half-duplex TTL adapter. Connect the
  adapter's half-duplex side to the servo with one bus wire. The flight
  controller uses normal TX/RX UART mode; there is no DIR/OE GPIO.
- The default port is EXT2 (`/dev/ttyS3`), 115200 baud, 8N1. `HX8_SER_CFG` may
  select another conflict-free serial port; `rc.serial` is the sole driver
  startup owner. Do not hard-code EXT2 or start a second driver instance from
  `rc.hybrid_apps`.
- There is no flight-controller-controlled servo power switch. A complete UART
  failure prevents PX4 from commanding release or removing HX8 power. The
  previously verified internal protection is then the only safety mechanism;
  this is an accepted residual risk, not a software release guarantee.
- The public UART protocol exposes neither model nor firmware identity. Record
  the installed label and serial number as physical proof that the unit is an
  HX8-U45H-M; software can verify only ID, protocol behavior, telemetry, and
  protection readback.

## Select one actuator backend

`HYB_ACT_TYPE=0` selects PWM and `HYB_ACT_TYPE=1` selects HX8. The selection is
reboot-required. PWM mode does not open the HX8 UART. HX8 mode keeps M8 output
disabled and uses only the servo's own position feedback, even when
`HYB_SENS_EN=0`.

There is no runtime fallback between backends. A UART or HX8 fault must remain
an HX8 fault and must never enable M8. Roll back from HX8 to PWM, or from PWM to
HX8, only by changing `HYB_ACT_TYPE`, verifying the selected hardware wiring,
and rebooting.

## Calibrate motion and protection parameters

Perform calibration with the linkage unloaded, propulsion inhibited, and the
vehicle fully disarmed and not prearmed. Save the parameter export used for
each run.

Use these parameter units exactly:

- `HYB_ANG_TOL`: radians; the HX8 adapter converts the tolerance for its
  degree-based endpoint comparison;
- `HX8_ANG_QUD` and `HX8_ANG_ROV`: degrees;
- `HX8_MOVE_T`, `HX8_ACC_T`, and `HX8_DEC_T`: milliseconds;
- `HYBRID_TRANS_T`: seconds;
- `HX8_PWR_LIM`, `HX8_CFG_SPWR`, and `HX8_CFG_PWR`: mW;
- `HX8_CFG_CUR`: mA;
- `HX8_CFG_VMIN` and `HX8_CFG_VMAX`: mV;
- `HX8_CFG_TADC`: raw ADC value, not degrees Celsius.

1. Verify the responding device ID and set `HX8_ID`. Confirm that only the
   intended servo responds.
2. Measure the two mechanical endpoints, then set `HX8_ANG_QUD` and
   `HX8_ANG_ROV`. Keep both angles within the supported single-turn range and
   separated far enough that their `HYB_ANG_TOL` envelopes do not overlap.
   Test endpoints that cross -180/180 degrees using the wrapped angular span;
   do not calibrate an intermediate runtime target.
3. Set `HX8_MOVE_T`, `HX8_ACC_T`, and `HX8_DEC_T` in milliseconds. Require
   `HX8_MOVE_T > HX8_ACC_T + HX8_DEC_T` and
   `HX8_MOVE_T < HYBRID_TRANS_T * 1000`. Start unloaded and increase timing
   margins only from measured travel.
4. Calibrate a nonzero `HX8_PWR_LIM` for runtime motion from the vendor limits
   and measured load. Do not guess a value merely to make preflight pass.
5. Calibrate and record nonzero internal stall/current/power/temperature
   protection values: `HX8_CFG_STL=1`, nonzero `HX8_CFG_SPWR`, nonzero
   `HX8_CFG_CUR`, nonzero `HX8_CFG_PWR`, and nonzero `HX8_CFG_TADC`. Set
   `HX8_CFG_RSP=1`, and deliberately choose `HX8_CFG_BOOT` from the required
   power-on behavior. Verify `HX8_CFG_VMIN` and `HX8_CFG_VMAX` against the
   9.0--12.6 V supply range. These expected values are safety-calibration
   inputs; derive them from HX8-U45H-M documentation and validate them under
   the actual bench load.

Zero placeholders for run power, stall power, current, power, or temperature
protection are uncommissioned and must not be accepted for operation.

## Check and write persistent HX8 configuration

All HX8_* parameters are reboot-required. The driver reads them only during
`init()`; an old running driver instance caches its startup values and does not
reload them after `param set` or `param save`. Use this order:

1. Fully disarm and leave prearm. Unload or mechanically secure the linkage,
   inhibit propulsion, and either remove servo power or otherwise stop motion
   with the verified physical emergency stop.
2. Set and save all HX8 parameters, including `HX8_SER_CFG`, endpoint, timing,
   power, and expected protection values. Recheck the saved parameter export.
   Do not run `hx8_uart_servo config write` before this reboot: the old instance
   would write its cached values rather than the newly saved parameter set.
3. Restore the safe bench wiring and reboot the complete vehicle. Confirm the
   boot log shows `rc.serial` starting one new driver instance on the port
   selected by `HX8_SER_CFG`; do not start another instance manually. Confirm
   `hx8_uart_servo status` now reports that new instance online with a completed
   configuration check before continuing.
4. Run `hx8_uart_servo config check`. This command returns only an overall
   pass/fail result. `hx8_uart_servo status` and `hx8_servo_status` expose the
   overall online, healthy, check-complete, and config-verified state; they do
   not provide the values themselves. PX4 status/uORB does not expose per-item
   configuration values.
5. If the overall check fails, first use `hx8_uart_servo status` and
   `listener hx8_servo_status 5` to separate communication/ID/freshness errors
   from a completed configuration mismatch. Check wiring, supply, selected
   serial port, baud, ID, saved PX4 parameters, and boot log. For the exact
   mismatching servo item, use the vendor configuration tool, a protocol
   capture from a logic analyzer, or the installation record. Do not infer a
   specific item from the aggregate PX4 result.
6. Only after the new driver instance has loaded the reviewed, nonzero expected
   values may fully disarmed explicit commissioning use
   `hx8_uart_servo config write`. Do this only when the external evidence shows
   that programming those exact values is intentional. The driver performs
   internal per-item readback, but the production CLI reports only the overall
   operation result. Follow with `config check` and `status`:

   ```sh
   hx8_uart_servo config write
   hx8_uart_servo config check
   hx8_uart_servo status
   ```

Normal driver startup is read-only: it pings `HX8_ID`, reads every expected
protection item, and reports healthy/configured only after a complete match.
It never repairs a mismatch or writes servo nonvolatile memory automatically.
Arming, prearming, lockdown, failsafe, timeout, protocol failure, or rejected
write must deny/fail commissioning. Never repeatedly write configuration at
normal startup.

### Stopping the UART driver

Use `hx8_uart_servo stop` only while the vehicle is fully disarmed and not
prearmed, the mechanism is safely unloaded, and absence of motion has been
confirmed; if necessary, first remove servo power with the external physical
disconnect. Never stop the driver while armed, transitioning, holding an
endpoint, or commissioning. In particular, `config write` and driver stop must
not run concurrently.

Driver stop does not send release. It closes the UART, after which PX4 cannot
command release or observe the servo. Depending on its persistent/internal
state the HX8 may continue producing torque, so stopping the driver is not an
emergency-stop or torque-release operation.

## Fault handling and diagnosis

On any transformation fault, expect latched `TRANSITION_FAULT`, inhibited
propulsion, cleared RC commissioning, and rejection of new transformation
targets. PWM faults command M8 NaN. HX8 faults make bounded release attempts,
then prohibit motion while retaining low-rate monitoring. A failed UART
release is secondary diagnostic evidence and must not replace the initiating
stall fault.

Inspect both status layers:

```sh
listener hybrid_vehicle_status 5
listener hx8_servo_status 5
hx8_uart_servo status
```

Record backend, position source and normalized position, online/healthy/
config-verified flags, no-progress time, protection summary, raw protection
flags, angle, voltage, current, power, temperature, command result/sequence,
communication counters, and freshness timestamps.

Commander owns the red LED. Verify: healthy is off; generic transformation
fault is a 1 Hz slow flash; stall is two 150 ms flashes followed by a 1 s
pause; overload plus transformation fault is three flashes followed by a
pause. Also verify the existing approximately 10 Hz overload indication and
that green power and blue armed indications are unchanged. Stale hybrid status
must produce the generic fault pattern and prevent arming.

Clear a latched fault only while fully disarmed:

```sh
hybrid_vehicle_control clear_fault
```

The selected configuration must be valid and its required feedback fresh. HX8
also requires restored communication and cleared recoverable protection flags.
A voltage protection that requires a power cycle cannot be cleared in software
while active. A confirmed endpoint restores the stable shape; an intermediate
position remains `UNKNOWN` and unarmable. Never resume the failed target or
retry a transformation automatically.

## Host-only negative command evidence

Wrong servo ID, intermediate angle, stale/out-of-order sequence, and invalid
timing or power envelope rejection are covered by the focused
`Hx8BackendPolicy` and `Hx8Controller` host tests. The production CLI cannot
inject these arbitrary motion commands. They are not mandatory physical bench
rows and must not be attempted by changing a flight configuration to invalid
values. Any optional protocol-level negative hardware experiment requires a
temporary, separately reviewed bench uORB publisher or protocol injector that
is never included in flight firmware and is not exposed through production
CLI.

## Pending PWM physical acceptance matrix

Run this complete matrix separately for AS5600 and TMAG5273; do not combine
their evidence. For TMAG5273 also retain raw XYZ, saturation, monotonicity, and
blind-region data over full travel.

1. Record full travel, wrap/repeatability, normalized monotonicity, endpoint
   tolerance, and source validity.
2. Complete at least 20 Quad-to-Rover and 20 Rover-to-Quad transitions with no
   false stall for each sensor path.
3. Induce immobility, reverse motion, and oscillation stalls at multiple early,
   middle, and late travel positions. Record detection no later than
   `HYB_STALL_T + 100 ms` and preserve the initiating fault.
4. Use an oscilloscope on M8 to prove valid pulses disappear within one hybrid
   control cycle after fault. Verify disarmed/failsafe output is zero, not the
   servo midpoint.
5. Measure servo current before and after M8 pulse loss at every deliberate
   stall. Pulse loss alone does not prove torque release or thermal safety. If
   current remains unsafe, record PWM thermal protection as unproven.
6. Verify healthy/off, slow fault, double-flash stall, overload, and combined
   red LED patterns, including publication-to-indication timing.

## Pending HX8 physical acceptance matrix

1. Capture FC TX, FC RX, and the one-wire bus with a logic analyzer. Verify
   adapter turnaround, optional echo handling, TX drain, 115200 8N1 timing,
   request spacing, response deadlines, and recovery after corrupt/partial
   traffic.
2. Record `config check`, an intentional disarmed `config write`, mandatory
   protection readback, and a subsequent read-only boot. Prove every internal
   stall/current/power/temperature/voltage setting matches the acceptance
   record.
3. Correlate angle, voltage, current, power, temperature, protection flags,
   and command telemetry with independent instruments.
4. With the vehicle in the permitted bench arming/prearming state, use
   QGroundControl or another controlled MAVLink ground-station interface to
   send `VEHICLE_CMD_DO_VTOL_TRANSITION`: `param1=3` requests the configured
   Quad endpoint and `param1=4` requests the configured Rover endpoint. Do not
   invent a shell command or publish arbitrary angles. Exercise both QUD/ROV
   transitions with the calibrated ID, timing, and power; record the accepted
   command/result sequence, endpoint angle, telemetry, and stable-state hold
   monitoring.
5. Induce stalls at several positions and prove stall release plus current
   reduction. Record detection time, release command/result, onboard
   protection timing, and the latched primary fault.
6. Inject checksum/timeout faults and disconnect communication during normal
   motion. Confirm bounded communication-fault detection, prohibited motion,
   no M8 fallback, status counters, and red LED indication.
7. Remove UART during a deliberate stall. Confirm onboard stall release with
   an independent current measurement before accepting the no-power-switch
   residual risk; PX4 cannot actively release through a disconnected UART.
8. Exercise under-voltage and over-voltage protection, document whether a
   power cycle is required, and verify software clear is refused while the
   voltage fault remains active. Restore valid power and communication, fully
   disarm, then verify explicit `clear_fault` behavior.
9. Remove and restore servo power separately from flight-controller power.
   Record boot configuration check, known/unknown endpoint result, arming gate,
   and the absence of automatic movement or backend fallback.

## Acceptance record

For software evidence, record the exact commit, each focused host-test command
and result, the `zeroone_x6_hybrid` firmware artifact, byte size, and linker
FLASH usage. Software results must be reported separately from physical bench
evidence.

For every physical row, record parameter export, wiring revision, supply
voltage/current limit, load and linkage condition, instrument setup, ULog,
logic-analyzer or oscilloscope capture, current trace, timing, observed LED
pattern, and pass/fail result. Hardware acceptance remains pending until all
artifacts exist; do not describe an unexecuted matrix row as tested.

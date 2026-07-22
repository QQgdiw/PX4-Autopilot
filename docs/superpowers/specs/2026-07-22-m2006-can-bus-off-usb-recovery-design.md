# M2006 CAN Bus-Off and USB Startup Recovery Design

## Context

`zeroone_x6_hybrid` starts `m2006_can` before USB CDC. When neither C610 node can
acknowledge CAN traffic, the module currently sends a zero-current group command
every 2 ms. The first unacknowledged transmission can enter the STM32H7 FDCAN
bus-off interrupt/recovery path and prevent PX4 USB CDC from enumerating.

Hardware boundary tests established the following causal chain:

- USB enumerates when `sendCommand()` is disabled.
- One complete `sendCommand()` reproduces the failure.
- Filling FDCAN message RAM without writing `TXBAR` does not reproduce it.
- Writing `TXBAR` with FDCAN interrupt delivery disabled does not reproduce it.
- Writing `TXBAR` with only the bus-off interrupt disabled does not reproduce it.

The failure therefore requires the FDCAN bus-off interrupt path. The M2006 module
also calls `ICanIface` directly instead of servicing `CanDriver::select()`, so the
driver's existing deadline and abort-on-error maintenance does not run.

## Goals

- USB CDC must enumerate when the flight controller starts without powered CAN
  peers.
- An unacknowledged or interrupted CAN transmission must not leave a pending frame
  cycling through bus-off recovery.
- Runtime loss of both C610 nodes must stop transmission, report the existing CAN
  and feedback faults, and preserve the current drive-fault semantics.
- A remaining online C610 must still receive the zero-current group command when
  drive output is inhibited.
- Bus-off interrupts, error counting, and normal controller recovery must remain
  enabled.

## Non-Goals

- Changing the C610 wire protocol, CAN bitrate, motor IDs, or control gains.
- Adding automatic fault-latch clearing or changing `M2006DriveGate` semantics.
- Enabling this module on `zeroone_x6_default` or other boards.
- Replacing the PX4 STM32H7 UAVCAN driver.

## Design

### M2006 transmission policy

`M2006Can::Run()` will service the owning `CanDriver` non-blockingly on every 2 ms
cycle so expired TX deadlines and abort-on-error requests are processed. It will
send the group command only while at least one configured motor has fresh feedback.

This policy prevents traffic before either C610 is known to be present. If one
motor remains online, the group command is still sent and acknowledged, allowing
the online motor to receive a zero-current command while the drive gate inhibits
propulsion. Once both feedback streams are stale, no further frame is submitted.

Shutdown zero commands remain best-effort, but are attempted only when at least one
motor was online at the most recent evaluation. No new parameter is introduced.

### STM32H7 bus-off handling

`CanIface::handleBusOff()` will cancel every software-pending TX mailbox that is
also pending in `TXBRP`, mark the corresponding software item no longer pending,
and signal the driver's update event. Only after the pending requests have been
terminated will it clear `CCCR.INIT` to begin the existing hardware recovery
sequence.

Cancellation is bounded by the fixed mailbox count and runs inside the existing
interrupt context. It performs no logging, allocation, waiting, or uORB work.
Bus-off interrupt enablement and error accounting remain unchanged.

### Fault behavior

At unarmed startup with no feedback, the status remains offline but the module does
not transmit or create a drive-fault latch. During `armed && DRIVING`, stale motor
feedback or a rising CAN error count continues through the existing drive gate and
fault reporting. No runtime fallback or hidden suppression of CAN errors is added.

## Testing

Implementation will follow red-green-refactor:

1. Add host tests for the M2006 transmit decision:
   - neither motor online: do not transmit;
   - left, right, or both online: transmit;
   - an inhibited drive command may still transmit zero while a motor is online.
2. Add a host-testable bus-off cancellation helper test:
   - cancel only mailboxes that are both software-pending and present in `TXBRP`;
   - clear the matching software-pending states;
   - leave unrelated mailboxes unchanged.
3. Run the focused tests, then the relevant hybrid-control test set.
4. Build `make zeroone_x6_hybrid` with output redirected to a log.
5. Hardware acceptance:
   - USB-only startup with no powered C610 nodes enumerates PX4 CDC;
   - startup with both C610 nodes powered retains CAN feedback and QGC USB;
   - removing both nodes during operation produces the existing safe fault without
     starving USB;
   - reconnecting nodes after disarm allows normal CAN communication after the
     controller recovery sequence.

### Bus-off policy host-test placement

The default `make tests` configuration is `px4_sitl_test`; it does not configure
the UAVCAN CMake subtree, so a test registered in `src/drivers/uavcan` is not
built or discovered. The pure `makeBusOffCleanup()` policy therefore keeps its
production header in the STM32H7 driver include directory, but its GoogleTest is
registered in `src/lib/hybrid_control`, which is already part of the default SITL
test graph.

The test target receives only the STM32H7 driver include directory. It does not
link UAVCAN, enable UAVCAN in SITL, or change any board configuration. This keeps
the pure bit-mask policy host-testable while the later ISR adapter remains verified
by the policy test, target compilation, and hardware acceptance.

## Risks and Constraints

The shared STM32H7 driver change affects other boards using this backend, so the
bus-off change is intentionally limited to terminating already-pending TX items
before the pre-existing recovery sequence. The M2006 startup and output changes
remain compiled and started only for `zeroone_x6_hybrid`.

Build success and hardware acceptance will be reported separately. A successful
build alone will not be treated as proof that USB or CAN recovery is fixed.

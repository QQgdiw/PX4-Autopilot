# M2006 CAN2 Isolation Design

**Status:** Approved for implementation on 2026-07-23

## Goal

Run the ZeroOne PMU's DroneCAN service on physical CAN1 and the two private
DJI M2006/C610 controllers on physical CAN2 in `zeroone_x6_hybrid`, without
allowing either protocol to transmit or consume frames on the other bus.

## Confirmed Constraints

- The target is the ZeroOne X6 hybrid board only. The PMU remains physically
  connected to CAN1; the C610 controllers are moved to CAN2.
- CAN1 is FDCAN1 and CAN2 is FDCAN2. The board configures PB12/PB13 for CAN2
  and reports both physical interfaces as available when the hardware manifest
  declares CAN2 present.
- The existing H7 UAVCAN driver was written for one multi-interface driver
  instance. Its process-global interface table currently serves both IRQ
  dispatch and the public driver API. A second CAN2-only instance therefore
  cannot safely coexist with a CAN1-only DroneCAN instance.
- `zeroone_x6_hybrid` does not compile the Cyphal backend. A missing
  `CYPHAL_ENABLE` parameter means disabled, not an M2006 startup failure.
- M2006 must keep using the FDCAN initial filter configuration and software
  C610 ID filtering. It must not re-enter FDCAN INIT at runtime to configure
  filters because that path previously prevented USB CDC enumeration.
- The final topology is intentionally fixed: DroneCAN is restricted to CAN1
  even when `M2K_EN` is disabled. This prevents private C610 traffic from ever
  sharing a DroneCAN interface by accident.

## Alternatives Considered

1. Change M2006 from `_can{1u}` to `_can{2u}`. Rejected: the current driver
   returns a null or foreign `getIface(0)` and `select()` dereferences global
   interface slots that do not belong to the instance.
2. Use one shared multi-interface UAVCAN driver for both protocols. Rejected:
   libuavcan exposes every active interface to DroneCAN, so its node could
   transmit PMU traffic on CAN2. A protocol broker or filtered adapter would be
   materially larger than the required isolation change.
3. Make each driver instance map selected physical interfaces to its own
   contiguous logical interfaces. Selected: it preserves the existing
   `CanInitHelper` API while making CAN1-only DroneCAN and CAN2-only M2006
   independent.

## Design

### Physical Interface Ownership

`CanOwnership` will represent a claim as an owner plus a physical-interface
bit mask. CAN1 is bit 0 and CAN2 is bit 1. Claims are atomic across every bit
in the requested mask: an owner may claim an idle distinct bus, but a second
owner cannot claim any occupied bus. Query and release operations use the same
mask and owner identity.

DroneCAN claims CAN1 before constructing its helper. M2006 claims CAN2 before
touching FDCAN hardware. The current global `UAVCAN_ENABLE`/`CYPHAL_ENABLE`
rejection in M2006 is removed because it rejects valid independent buses. The
ownership claim remains the authoritative startup-order-safe conflict check.

An allocation failure before hardware initialization releases the reservation.
After a helper has attempted low-level FDCAN initialization, its physical bus
remains claimed until reboot. The H7 hardware has no supported deinitialization
path here, so retrying the bus with another owner would be unsafe.

### H7 Driver Mapping

The file-scope pointer table becomes a physical-interface IRQ registry only.
FDCAN1 IRQs use physical slot 0 and FDCAN2 IRQs use physical slot 1. Binding a
physical slot is protected against a second instance claiming the same slot;
an initialization failure clears any partial binding so IRQ handlers have no
dangling object pointer.

Each `CanDriver` stores a local ordered list of its active `CanIface` objects.
`init(0x1)` maps logical interface 0 to the instance's FDCAN1 object;
`init(0x2)` maps logical interface 0 to its FDCAN2 object; `init(0x3)` maps
logical interfaces 0 and 1 to FDCAN1 and FDCAN2 respectively. The physical
index retained by each `CanIface` continues to select the correct FDCAN
registers and message-RAM partition.

`getIface()`, `getNumIfaces()`, `makeSelectMasks()`,
`hasReadableInterfaces()`, `select()`, and activity polling operate only on
that local list. This prevents either instance from examining, transmitting on,
or dereferencing a bus it did not initialize. The shared peripheral clock/IRQ
setup remains one-time global initialization.

### Application Routing

The generic DroneCAN node receives the CAN1 physical mask rather than the
board-wide availability mask. It therefore exposes one logical CAN interface
to libuavcan and cannot broadcast on CAN2.

M2006 changes its helper mask from CAN1 to CAN2. Its existing
`getIface(0)` and periodic `select()` calls remain valid because the new local
mapping makes CAN2 its logical interface 0. Startup reports and event text use
CAN2. The board availability mask is checked before either protocol reserves
its required physical bus.

Commander/hybrid validation changes from a global UAVCAN/M2006 conflict to a
physical-mask overlap test, so enabled DroneCAN/CAN1 plus M2006/CAN2 is valid,
while a same-bus configuration remains an explicit safety failure.

## Error Handling

- Unavailable CAN1 or CAN2 fails before FDCAN initialization with a precise
  error naming the physical bus.
- A claim conflict fails before touching hardware and identifies the occupied
  bus; reboot remains required after a low-level initialization attempt.
- Invalid, empty, or unsupported interface masks are rejected rather than
  constructing a driver with no valid logical interfaces.
- The target's absent Cyphal parameter is read as disabled without calling
  `param_get()` on `PARAM_INVALID`.
- M2006 continues to software-filter C610 IDs and retains the current USB-safe
  FDCAN filter behavior.

## Test Strategy

1. Extend the host `CanOwnership` tests first to prove independent CAN1 and
   CAN2 claims coexist, overlapping claims fail, release validates owner and
   mask, and a multi-bit claim is all-or-nothing.
2. Add a focused host test for the physical-to-logical mapping policy: CAN1
   only and CAN2 only each expose one logical interface 0, while both buses
   expose logical interfaces 0 and 1 in physical order. This test is isolated
   from FDCAN registers.
3. Add hybrid validation tests for the valid PMU/CAN1 plus M2006/CAN2 topology
   and a rejected overlapping-bus topology.
4. Build only `make zeroone_x6_hybrid`, after focused tests pass. Do not build
   `zeroone_x6_default` in parallel.
5. Perform target acceptance in both start orders: confirm USB/QGC remains
   connected, PMU DroneCAN communication remains healthy on CAN1, M2006 status
   receives feedback from IDs 1 and 2 on CAN2, and each protocol has no traffic
   or error counter attributable to the other bus.

## Non-Goals

- Do not enable or refactor Cyphal for `zeroone_x6_hybrid`.
- Do not create a generic protocol multiplexer for shared CAN buses.
- Do not reintroduce runtime FDCAN filter reconfiguration.
- Do not alter other boards or build `zeroone_x6_default`.

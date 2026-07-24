# DroneCAN Pre-init Interface Count Fix

## Problem

The STM32H7 multi-instance CAN change initializes `CanDriver::num_ifaces_` to
zero and only sets it in `CanDriver::init()`. PX4 constructs `UavcanNode` before
the scheduled worker calls `CanDriver::init()`.

During `UavcanNode` construction, libuavcan's `CanIOManager` caches
`ICanDriver::getNumIfaces()`. A value below one invokes
`handleFatalError("Num ifaces")`, terminating the `uavcan start` command before
`UavcanNode::_instance` can be assigned.

Target diagnostics confirmed the exact boundary: helper allocation and
`SystemClock` construction completed, but the log immediately after
`new UavcanNode(...)` was never reached.

## Design

`CanInitHelper` will pass its enabled physical-interface mask into the
`CanDriver` constructor. The driver constructor will use the existing
`CanInterfaceMap` policy to establish its local logical-interface view before
any libuavcan object is constructed:

- physical mask `0x1` exposes one logical interface mapped to CAN1;
- physical mask `0x2` exposes one logical interface mapped to CAN2;
- physical mask `0x3` exposes two logical interfaces in physical order;
- an invalid or empty mask exposes zero interfaces and remains a fatal
  configuration error rather than inventing an interface.

This constructor step only selects pointers and reports interface topology. It
must not bind global IRQ dispatch, touch FDCAN registers, claim ownership, or
initialize hardware. Those operations remain in `CanDriver::init()`.

`CanDriver::init()` will validate that its requested mask matches the
constructor-configured topology, bind the selected physical interfaces, and
perform hardware initialization as before. `num_ifaces_` continues to describe
the configured logical topology throughout the driver's lifetime; failed
hardware initialization is reported by the existing negative return value.

## Error and Lifecycle Semantics

- Invalid topology is distinguishable from hardware initialization failure.
- A valid configured topology is visible to libuavcan before hardware init.
- Physical-interface binding and teardown remain paired through
  `bound_ifaces_`.
- DroneCAN remains fixed to CAN1 and M2006 remains fixed to CAN2.
- CAN initialization stays on the scheduled UAVCAN work queue.

## Verification

Before production changes, a focused host test will demonstrate the regression:
a valid single-interface configuration must report one logical interface before
hardware initialization. It must fail against the current zero-count behavior.

GREEN verification will cover CAN1-only, CAN2-only, dual-interface, and invalid
mask topology. Existing interface-map, ownership, hybrid safety tests, and
`make zeroone_x6_hybrid` must also pass.

The final acceptance criterion is a target power-cycle test with PMU DroneCAN on
CAN1 and M2006 on CAN2. `uavcan start` must return zero, `uavcan status` must
remain running, PMU data must update, M2006 feedback must remain online, and USB
must remain stable.

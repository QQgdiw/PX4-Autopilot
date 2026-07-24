# DroneCAN Pre-init Interface Count Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep the configured logical CAN interface topology visible before hardware initialization so libuavcan can construct the DroneCAN node without a fatal zero-interface error.

**Architecture:** Add a small, hardware-independent topology value object based on the existing `CanInterfaceMap`. Construct the STM32H7 `CanDriver` with the enabled physical mask, populate its logical interface pointers and count immediately, and retain FDCAN register/IRQ binding and initialization in `CanDriver::init()`.

**Tech Stack:** PX4 v1.16.1, C++17, libuavcan/DroneCAN, STM32H7 FDCAN, GoogleTest, CMake/Ninja, NuttX NSH.

## Global Constraints

- Work only in `/home/crocodile/PX4-Autopilot-debug-testc1-v1.16.1` on `debug/testc1-v1.16.1`.
- Build only `make zeroone_x6_hybrid`; do not modify the shared checkout.
- PMU/DroneCAN remains on physical CAN1; M2006/C610 remains on physical CAN2.
- Constructor topology setup must not bind IRQ dispatch, access FDCAN registers, claim ownership, or initialize hardware.
- Hardware initialization remains in the scheduled UAVCAN worker through `CanDriver::init()`.
- Invalid or empty masks report zero configured interfaces; no interface may be fabricated.
- Do not restore runtime `configureFilters()` for M2006.
- Redirect build/test output to files and inspect at most 50 lines.
- Do not commit `state/` or temporary diagnostic instrumentation.

## File Structure

- Create `src/drivers/uavcan/uavcan_drivers/stm32h7/driver/include/uavcan_stm32h7/can_driver_topology.hpp`: pure configured-mask-to-logical-topology value object.
- Modify `src/lib/hybrid_control/CanInterfaceMapTest.cpp`: host regression tests for topology visibility before hardware initialization.
- Modify `src/drivers/uavcan/uavcan_drivers/stm32h7/driver/include/uavcan_stm32h7/can.hpp`: pass the enabled mask into `CanDriver`, retain configured topology, and expose its count before init.
- Modify `src/drivers/uavcan/uavcan_drivers/stm32h7/driver/src/uc_stm32h7_can.cpp`: validate and bind the constructor-configured topology during hardware init.
- Restore `src/drivers/uavcan/uavcan_main.cpp`: remove temporary `diag` markers after host verification.

---

### Task 1: Reproduce and Fix the Pre-init Topology Contract

**Files:**
- Create: `src/drivers/uavcan/uavcan_drivers/stm32h7/driver/include/uavcan_stm32h7/can_driver_topology.hpp`
- Modify: `src/lib/hybrid_control/CanInterfaceMapTest.cpp`
- Modify: `src/drivers/uavcan/uavcan_drivers/stm32h7/driver/include/uavcan_stm32h7/can.hpp`
- Modify: `src/drivers/uavcan/uavcan_drivers/stm32h7/driver/src/uc_stm32h7_can.cpp`

**Interfaces:**
- Consumes: `CanInterfaceMap makeCanInterfaceMap(uint16_t, uint8_t)`.
- Produces: `CanDriverTopology(uint16_t enabled_interfaces, uint8_t physical_interface_count)`.
- Produces: `bool CanDriverTopology::valid() const`, `uint8_t count() const`, and `uint8_t physicalIndex(uint8_t logical_index) const`.
- Changes: `CanDriver` constructor accepts the configured enabled-interface mask and `getNumIfaces()` reports its logical count before `init()`.

- [ ] **Step 1: Write the failing topology regression test**

Add the new include and tests to `CanInterfaceMapTest.cpp`:

```cpp
#include <uavcan_stm32h7/can_driver_topology.hpp>

TEST(CanDriverTopology, ExposesConfiguredInterfacesBeforeHardwareInit)
{
	const uavcan_stm32h7::CanDriverTopology can1(PhysicalCan1, 2);
	ASSERT_TRUE(can1.valid());
	ASSERT_EQ(can1.count(), 1);
	EXPECT_EQ(can1.physicalIndex(0), 0);

	const uavcan_stm32h7::CanDriverTopology can2(PhysicalCan2, 2);
	ASSERT_TRUE(can2.valid());
	ASSERT_EQ(can2.count(), 1);
	EXPECT_EQ(can2.physicalIndex(0), 1);

	const uavcan_stm32h7::CanDriverTopology both(PhysicalCan1 | PhysicalCan2, 2);
	ASSERT_TRUE(both.valid());
	ASSERT_EQ(both.count(), 2);
	EXPECT_EQ(both.physicalIndex(0), 0);
	EXPECT_EQ(both.physicalIndex(1), 1);
}

TEST(CanDriverTopology, RejectsInvalidConfiguredMasks)
{
	EXPECT_FALSE(uavcan_stm32h7::CanDriverTopology(0, 2).valid());
	EXPECT_EQ(uavcan_stm32h7::CanDriverTopology(0, 2).count(), 0);
	EXPECT_FALSE(uavcan_stm32h7::CanDriverTopology(1U << 2, 2).valid());
}
```

- [ ] **Step 2: Run RED and verify the expected failure**

Run:

```bash
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin make tests TESTFILTER=CanInterfaceMap > state/test_can_topology_red.log 2>&1
```

Expected: nonzero exit while compiling `CanInterfaceMapTest.cpp`, specifically because `uavcan_stm32h7/can_driver_topology.hpp` does not exist. Reject unrelated configuration or dependency failures as an invalid RED.

- [ ] **Step 3: Add the minimal pure topology object**

Create `can_driver_topology.hpp`:

```cpp
#pragma once

#include <cstdint>
#include <uavcan_stm32h7/can_interface_map.hpp>

namespace uavcan_stm32h7
{
class CanDriverTopology
{
public:
	constexpr CanDriverTopology(uint16_t enabled_interfaces, uint8_t physical_interface_count) :
		_map(makeCanInterfaceMap(enabled_interfaces, physical_interface_count)) {}

	constexpr bool valid() const { return _map.valid; }
	constexpr uint8_t count() const { return _map.count; }
	constexpr uint8_t physicalIndex(uint8_t logical_index) const
	{
		return logical_index < _map.count ? _map.physical_index[logical_index] : UINT8_MAX;
	}

private:
	CanInterfaceMap _map;
};
} // namespace uavcan_stm32h7
```

- [ ] **Step 4: Integrate topology into the actual STM32H7 driver constructor**

In `can.hpp`, include `can_driver_topology.hpp`, add a `CanDriverTopology topology_` member, and change the templated constructor to accept `enabled_interfaces`:

```cpp
template <unsigned RxQueueCapacity>
CanDriver(CanRxItem (&rx_queue_storage)[UAVCAN_STM32H7_NUM_IFACES][RxQueueCapacity],
	  uint32_t enabled_interfaces)
	: update_event_(*this)
	, if0_(fdcan::Can[0], update_event_, 0, rx_queue_storage[0], RxQueueCapacity)
#if UAVCAN_STM32H7_NUM_IFACES > 1
	, if1_(fdcan::Can[1], update_event_, 1, rx_queue_storage[1], RxQueueCapacity)
#endif
	, active_ifaces_()
	, active_physical_indices_()
	, topology_(enabled_interfaces, UAVCAN_STM32H7_NUM_IFACES)
	, num_ifaces_(topology_.count())
	, bound_ifaces_(0)
	, enabledInterfaces_(enabled_interfaces)
{
	uavcan::StaticAssert<(RxQueueCapacity <= CanIface::MaxRxQueueCapacity)>::check();
	for (uint8_t logical = 0; logical < num_ifaces_; ++logical) {
		active_physical_indices_[logical] = topology_.physicalIndex(logical);
		active_ifaces_[logical] = ifaceForPhysicalIndex(active_physical_indices_[logical]);
	}
}
```

Change `CanInitHelper` to construct the driver with the same mask:

```cpp
CanInitHelper(const uavcan::uint32_t enabled_interfaces =
	      (1U << UAVCAN_STM32H7_NUM_IFACES) - 1U) :
	driver(queue_storage_, enabled_interfaces),
	enabledInterfaces_(enabled_interfaces)
{ }
```

In `CanDriver::init()`, construct a requested topology and reject a mismatch before binding. Do not reset `num_ifaces_` to zero or rebuild the logical view:

```cpp
const CanDriverTopology requested(enabledInterfaces, UAVCAN_STM32H7_NUM_IFACES);

if (!requested.valid() || requested.count() != topology_.count()) {
	return -ErrLogic;
}

for (uint8_t logical = 0; logical < requested.count(); ++logical) {
	if (requested.physicalIndex(logical) != topology_.physicalIndex(logical)) {
		return -ErrLogic;
	}
}
```

Use `topology_.count()` and `topology_.physicalIndex(logical)` for physical binding and initialization. `bound_ifaces_` remains zero until binding succeeds and becomes `topology_.count()` afterward.

- [ ] **Step 5: Run GREEN host tests**

Run the Step 2 command again.

Expected: exit zero and `unit-CanInterfaceMap` passes, including both new `CanDriverTopology` tests.

- [ ] **Step 6: Compile the real firmware integration**

Run:

```bash
make zeroone_x6_hybrid > state/build_can_topology_green.log 2>&1
```

Expected: exit zero, with `zeroone_x6_hybrid.px4` generated. This catches template constructor, STM32H7 driver, DroneCAN, and M2006 call-site integration errors that the pure host test cannot compile.

- [ ] **Step 7: Review the diff and commit the production fix**

Run `git diff --check` and confirm that `uavcan_main.cpp` diagnostic changes and `state/` are not staged. Then commit only the topology test and driver files:

```bash
git add src/lib/hybrid_control/CanInterfaceMapTest.cpp \
  src/drivers/uavcan/uavcan_drivers/stm32h7/driver/include/uavcan_stm32h7/can_driver_topology.hpp \
  src/drivers/uavcan/uavcan_drivers/stm32h7/driver/include/uavcan_stm32h7/can.hpp \
  src/drivers/uavcan/uavcan_drivers/stm32h7/driver/src/uc_stm32h7_can.cpp
git commit -m "fix[uavcan]: expose CAN topology before node construction"
```

No GitHub issue number has been provided; do not invent a `Fixes #...` footer. If the owner supplies an issue number before this step, append the corresponding `Fixes #N` footer.

### Task 2: Remove Diagnostics and Perform Final Host Verification

**Files:**
- Restore: `src/drivers/uavcan/uavcan_main.cpp`
- Update, uncommitted: `state/README.md`, `state/TODO.md`, `state/LOG.md`

**Interfaces:**
- Consumes: corrected `CanDriver` pre-init topology contract from Task 1.
- Produces: production firmware with no temporary `diag` logging.

- [ ] **Step 1: Remove every temporary diagnostic-only change**

Use `apply_patch` to restore the original `UavcanNode::start()`, `Run()`, and CLI start code. Verify:

```bash
rg -n 'diag ' src/drivers/uavcan/uavcan_main.cpp
```

Expected: no output. `git diff -- src/drivers/uavcan/uavcan_main.cpp` must also produce no output.

- [ ] **Step 2: Run focused regression suites**

Run each command separately and redirect each log:

```bash
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin make tests TESTFILTER=CanInterfaceMap > state/test_can_map_final.log 2>&1
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin make tests TESTFILTER=CanOwnership > state/test_can_ownership_final.log 2>&1
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin make tests TESTFILTER=hybridCheck > state/test_hybrid_final.log 2>&1
```

Expected: each exits zero and selects at least one named test. An empty CTest selection is not a pass.

- [ ] **Step 3: Build the final production firmware**

Run:

```bash
make zeroone_x6_hybrid > state/build_dronecan_topology_final.log 2>&1
```

Expected: exit zero. Record artifact size, timestamp, and SHA-256, then copy it to `zeroone_x6_hybrid_dronecan_topology_fix.px4`.

- [ ] **Step 4: Verify repository scope**

Run `git status --short --branch` and `git diff --check`. Expected: production files clean after committed fixes; only untracked `state/` remains. Confirm no shared-checkout changes occurred.

- [ ] **Step 5: Update state records**

Record the RED failure, GREEN test counts, final build evidence, artifact checksum, commit, and remaining target acceptance in the three `state/` files. Do not commit them.

### Task 3: Target Acceptance

**Files:**
- Flash: `build/zeroone_x6_hybrid/zeroone_x6_hybrid_dronecan_topology_fix.px4`
- Update, uncommitted: `state/README.md`, `state/TODO.md`, `state/LOG.md`

**Interfaces:**
- Validates: DroneCAN CAN1, M2006 CAN2, PMU telemetry, and USB coexistence.

- [ ] **Step 1: Validate DroneCAN alone after a full power cycle**

With `M2K_EN=0`, set `UAVCAN_ENABLE=3`, save parameters, remove USB and external power for ten seconds, reconnect, then run:

```sh
uavcan status
echo $?
listener battery_status 3
```

Expected: UAVCAN remains running, shell status is zero, and PMU-backed battery data updates.

- [ ] **Step 2: Validate separated buses together**

Set `M2K_EN=1`, save, fully power-cycle, then run:

```sh
uavcan status
m2006_can status
listener m2006_motor_status 3
```

Expected: DroneCAN remains running on CAN1; M2006 reports CAN2 at 1 Mbps, both IDs online, increasing RX/TX counts, zero CAN errors/timeouts, and no fault flags.

- [ ] **Step 3: Validate connectivity stability**

Keep PMU and both C610 nodes powered while observing QGC for at least one minute and during one C610 disconnect/reconnect cycle.

Expected: PX4 USB CDC remains enumerated, QGC remains connected, PMU communication stays active, and M2006 recovers without a USB failure.

- [ ] **Step 4: Close target state only with evidence**

Record the exact NSH output and acceptance outcome in `state/LOG.md`. Mark the target acceptance tasks complete only if all expected observations occur; otherwise return to systematic diagnosis with the first failed boundary.

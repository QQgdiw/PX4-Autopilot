# M2006 CAN Bus-Off and USB Startup Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make PX4 USB CDC enumerate without powered C610 nodes and make STM32H7 FDCAN bus-off recovery terminate the transmission that caused the fault.

**Architecture:** Add a small, host-tested M2006 transmission policy and a small, host-tested STM32H7 bus-off cleanup policy. `M2006Can` will service `CanDriver::select()` and submit group commands only when at least one motor has fresh feedback; the low-level bus-off ISR will cancel matching hardware/software-pending TX mailboxes before starting the existing recovery sequence.

**Tech Stack:** PX4 v1.16.1, C++17, GoogleTest through `px4_add_unit_gtest`, libuavcan STM32H7 FDCAN backend, NuttX, ZeroOne X6 hardware.

## Global Constraints

- Work only in `/home/crocodile/PX4-Autopilot-debug-testc1-v1.16.1` on `debug/testc1-v1.16.1`.
- Do not modify `/home/crocodile/PX4-Autopilot`; another agent is working there.
- Build only `zeroone_x6_hybrid`; do not expand M2006 startup or acceptance to `zeroone_x6_default` or other boards.
- Keep bus-off interrupts, error counting, and the existing hardware recovery sequence enabled.
- Preserve the existing rule that faults latch only during `armed && DRIVING`.
- Use HRT for M2006 command/feedback freshness; use the UAVCAN monotonic clock only for low-level CAN deadlines.
- Run test builds with a Linux-only `PATH` to avoid the Windows Anaconda protobuf package.
- Redirect complete build/test output to files and inspect no more than 50 relevant lines.
- Report firmware build success and physical USB/CAN acceptance separately.
- Commit messages must use `<type>[scope]: <description>`.

---

### Task 1: Define and Test the M2006 Transmission Policy

**Files:**
- Create: `src/lib/hybrid_control/M2006TxPolicy.hpp`
- Create: `src/lib/hybrid_control/M2006TxPolicyTest.cpp`
- Modify: `src/lib/hybrid_control/CMakeLists.txt`

**Interfaces:**
- Consumes: two fresh-feedback booleans computed by `M2006Can::Run()`.
- Produces: `hybrid_control::shouldTransmitM2006Command(bool left_online, bool right_online) -> bool`.

- [ ] **Step 1: Register and write the failing policy test**

Add `M2006TxPolicyTest.cpp` to `src/lib/hybrid_control/CMakeLists.txt`:

```cmake
px4_add_unit_gtest(SRC M2006TxPolicyTest.cpp LINKLIBS hybrid_control)
```

Create the test before creating the policy header:

```cpp
#include <gtest/gtest.h>

#include "M2006TxPolicy.hpp"

using hybrid_control::shouldTransmitM2006Command;

TEST(M2006TxPolicy, RequiresAtLeastOneOnlineMotor)
{
	EXPECT_FALSE(shouldTransmitM2006Command(false, false));
	EXPECT_TRUE(shouldTransmitM2006Command(true, false));
	EXPECT_TRUE(shouldTransmitM2006Command(false, true));
	EXPECT_TRUE(shouldTransmitM2006Command(true, true));
}

TEST(M2006TxPolicy, OnlineMotorMayReceiveZeroCommand)
{
	const int16_t left_command = 0;
	const int16_t right_command = 0;
	EXPECT_EQ(left_command, 0);
	EXPECT_EQ(right_command, 0);
	EXPECT_TRUE(shouldTransmitM2006Command(true, false));
}
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
rm -rf build/px4_sitl_test
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin make tests TESTFILTER=M2006TxPolicy > /tmp/m2006_tx_policy_red.log 2>&1
```

Expected: non-zero exit and compilation failure because `M2006TxPolicy.hpp` does not exist. Inspect with `tail -n 30 /tmp/m2006_tx_policy_red.log`.

- [ ] **Step 3: Add the minimal header-only policy**

Create `M2006TxPolicy.hpp`:

```cpp
#pragma once

namespace hybrid_control
{

constexpr bool shouldTransmitM2006Command(const bool left_online, const bool right_online)
{
	return left_online || right_online;
}

} // namespace hybrid_control
```

- [ ] **Step 4: Run the focused test and verify GREEN**

Run the same `make tests TESTFILTER=M2006TxPolicy` command with output redirected to `/tmp/m2006_tx_policy_green.log`.

Expected: exit 0 and `M2006TxPolicy` reports all tests passed.

- [ ] **Step 5: Commit the independently tested policy**

```bash
git add src/lib/hybrid_control/M2006TxPolicy.hpp src/lib/hybrid_control/M2006TxPolicyTest.cpp src/lib/hybrid_control/CMakeLists.txt
git commit -m "test[m2006]: define feedback-aware transmit policy"
```

---

### Task 2: Define and Test the Bus-Off Mailbox Cleanup Policy

**Files:**
- Create: `src/drivers/uavcan/uavcan_drivers/stm32h7/driver/include/uavcan_stm32h7/bus_off.hpp`
- Create: `src/drivers/uavcan/BusOffCleanupTest.cpp`
- Modify: `src/drivers/uavcan/CMakeLists.txt`

**Interfaces:**
- Consumes: a software-pending mailbox bit mask and the hardware `TXBRP` value.
- Produces: `uavcan_stm32h7::makeBusOffCleanup(uint32_t software_pending, uint32_t hardware_pending) -> BusOffCleanup` containing `cancel_mask` and `remaining_software_pending`.

- [ ] **Step 1: Register and write the failing cleanup test**

Add this test after `CanOwnershipTest` registration:

```cmake
px4_add_unit_gtest(SRC BusOffCleanupTest.cpp)
```

Create `BusOffCleanupTest.cpp` before the header exists:

```cpp
#include <gtest/gtest.h>

#include "uavcan_drivers/stm32h7/driver/include/uavcan_stm32h7/bus_off.hpp"

TEST(BusOffCleanup, CancelsOnlyMatchingPendingMailboxes)
{
	const auto result = uavcan_stm32h7::makeBusOffCleanup(0b1011U, 0b0110U);
	EXPECT_EQ(result.cancel_mask, 0b0010U);
	EXPECT_EQ(result.remaining_software_pending, 0b1001U);
}

TEST(BusOffCleanup, HandlesNoHardwarePendingMailboxes)
{
	const auto result = uavcan_stm32h7::makeBusOffCleanup(0b0101U, 0U);
	EXPECT_EQ(result.cancel_mask, 0U);
	EXPECT_EQ(result.remaining_software_pending, 0b0101U);
}
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin make tests TESTFILTER=BusOffCleanup > /tmp/bus_off_cleanup_red.log 2>&1
```

Expected: non-zero exit and compilation failure because `uavcan_stm32h7/bus_off.hpp` does not exist.

- [ ] **Step 3: Implement the minimal pure cleanup policy**

Create `bus_off.hpp`:

```cpp
#pragma once

#include <cstdint>

namespace uavcan_stm32h7
{

struct BusOffCleanup {
	uint32_t cancel_mask;
	uint32_t remaining_software_pending;
};

constexpr BusOffCleanup makeBusOffCleanup(const uint32_t software_pending,
		const uint32_t hardware_pending)
{
	const uint32_t cancel_mask = software_pending & hardware_pending;
	return {cancel_mask, software_pending & ~cancel_mask};
}

} // namespace uavcan_stm32h7
```

- [ ] **Step 4: Run the focused test and verify GREEN**

Run the same `make tests TESTFILTER=BusOffCleanup` command with output redirected to `/tmp/bus_off_cleanup_green.log`.

Expected: exit 0 and both cleanup cases pass.

- [ ] **Step 5: Commit the independently tested cleanup policy**

```bash
git add src/drivers/uavcan/BusOffCleanupTest.cpp src/drivers/uavcan/CMakeLists.txt src/drivers/uavcan/uavcan_drivers/stm32h7/driver/include/uavcan_stm32h7/bus_off.hpp
git commit -m "test[uavcan]: define bus-off mailbox cleanup policy"
```

---

### Task 3: Integrate Cleanup into the STM32H7 Bus-Off ISR

**Files:**
- Modify: `src/drivers/uavcan/uavcan_drivers/stm32h7/driver/src/uc_stm32h7_can.cpp:878`

**Interfaces:**
- Consumes: `makeBusOffCleanup()` from Task 2, `pending_tx_[]`, and FDCAN `TXBRP`.
- Produces: bounded ISR behavior that requests cancellation through `TXBCR`, clears matching software-pending items, signals `update_event_`, and then starts the existing recovery sequence.

- [ ] **Step 1: Re-run the cleanup policy test as the integration guard**

Run `make tests TESTFILTER=BusOffCleanup` with the clean Linux `PATH` and redirect output to `/tmp/bus_off_cleanup_pre_integration.log`.

Expected: exit 0 before the hardware adapter is changed.

- [ ] **Step 2: Include the tested policy and adapt the ISR**

Add:

```cpp
#include <uavcan_stm32h7/bus_off.hpp>
```

Replace the body before the existing `CCCR.INIT` clear with:

```cpp
	uint32_t software_pending = 0;

	for (uint8_t i = 0; i < NumTxMailboxes; ++i) {
		if (pending_tx_[i].pending) {
			software_pending |= 1U << i;
		}
	}

	const BusOffCleanup cleanup = makeBusOffCleanup(software_pending, can_->TXBRP);

	if (cleanup.cancel_mask != 0) {
		can_->TXBCR = cleanup.cancel_mask;
	}

	for (uint8_t i = 0; i < NumTxMailboxes; ++i) {
		if ((cleanup.cancel_mask & (1U << i)) != 0) {
			pending_tx_[i].pending = false;
		}
	}

	update_event_.signalFromInterrupt();
	can_->CCCR &= ~FDCAN_CCCR_INIT;
```

Keep the existing `error_cnt_++` and bus-off documentation. Do not disable `BOE`, `ILE`, or normal recovery.

- [ ] **Step 3: Run focused tests and compile the hardware target**

Run:

```bash
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin make tests TESTFILTER=BusOffCleanup > /tmp/bus_off_cleanup_post_integration.log 2>&1
make zeroone_x6_hybrid > /tmp/zeroone_bus_off_driver_build.log 2>&1
```

Expected: both commands exit 0; the firmware build log ends with creation of `zeroone_x6_hybrid.px4`.

- [ ] **Step 4: Commit the ISR integration**

```bash
git add src/drivers/uavcan/uavcan_drivers/stm32h7/driver/src/uc_stm32h7_can.cpp
git commit -m "fix[uavcan]: cancel pending TX on STM32H7 bus-off"
```

---

### Task 4: Integrate Driver Maintenance and Feedback-Aware Sending

**Files:**
- Modify: `src/drivers/uavcan/m2006_can/M2006Can.hpp`
- Modify: `src/drivers/uavcan/m2006_can/M2006Can.cpp:121-305`

**Interfaces:**
- Consumes: `shouldTransmitM2006Command()` from Task 1 and the existing `_can.driver.select()` API.
- Produces: no startup TX before feedback, regular TX deadline/error maintenance, and zero-current best-effort shutdown only for a recently online bus.

- [ ] **Step 1: Run the policy test as the behavior guard**

Run `make tests TESTFILTER=M2006TxPolicy` with the clean Linux `PATH` and redirect output to `/tmp/m2006_tx_policy_pre_integration.log`.

Expected: exit 0.

- [ ] **Step 2: Include the policy and service the complete CAN driver**

Add to `M2006Can.hpp`:

```cpp
#include <lib/hybrid_control/M2006TxPolicy.hpp>
```

At the start of `Run()`, after the exit branch and before `receiveFeedback()`, add:

```cpp
	uavcan::CanSelectMasks masks{};
	const uavcan::CanFrame *pending_tx[uavcan::MaxCanIfaces] {};
	const uavcan::MonotonicTime can_now = UAVCAN_DRIVER::SystemClock::instance().getMonotonic();
	(void)_can.driver.select(masks, pending_tx, can_now);
```

This is non-blocking because the blocking deadline equals the current CAN-clock time.

- [ ] **Step 3: Apply the tested transmit decision**

Replace the unconditional send in `Run()` with:

```cpp
	if (hybrid_control::shouldTransmitM2006Command(online[0], online[1])) {
		(void)sendCommand(_current_command[0], _current_command[1]);
	}
```

At the beginning of `sendZeroBestEffort()`, after the `_iface` null check, add:

```cpp
	if (!hybrid_control::shouldTransmitM2006Command(_online_previous[0], _online_previous[1])) {
		return;
	}
```

- [ ] **Step 4: Run focused and related M2006 tests**

Run:

```bash
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin make tests TESTFILTER='M2006*' > /tmp/m2006_tests_after_integration.log 2>&1
```

Expected: exit 0 with `M2006TxPolicy`, `M2006DriveGate`, `M2006CommandAdapter`, and `M2006SpeedController` passing.

- [ ] **Step 5: Build the target and commit the integration**

Run `make zeroone_x6_hybrid > /tmp/zeroone_m2006_integration_build.log 2>&1` and require exit 0.

Then commit:

```bash
git add src/drivers/uavcan/m2006_can/M2006Can.hpp src/drivers/uavcan/m2006_can/M2006Can.cpp
git commit -m "fix[m2006]: avoid transmission without online motors"
```

---

### Task 5: Final Verification and Hardware Acceptance Firmware

**Files:**
- Modify without committing unless requested: `state/README.md`, `state/TODO.md`, `state/LOG.md`
- Generate: `build/zeroone_x6_hybrid/zeroone_x6_hybrid_usb_bus_off_fix.px4`

**Interfaces:**
- Consumes: completed Tasks 1-4.
- Produces: verified firmware artifact and explicit separation between automated and physical results.

- [ ] **Step 1: Run all new focused tests from a fresh SITL test cache**

```bash
rm -rf build/px4_sitl_test
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin make tests TESTFILTER='M2006*|BusOffCleanup' > /tmp/m2006_bus_off_final_tests.log 2>&1
```

Expected: exit 0, no failed tests, and no protobuf version conflict.

- [ ] **Step 2: Run the final target build**

```bash
make zeroone_x6_hybrid > /tmp/zeroone_x6_hybrid_bus_off_final.log 2>&1
```

Expected: exit 0 and the final log line creates `build/zeroone_x6_hybrid/zeroone_x6_hybrid.px4`.

- [ ] **Step 3: Preserve and identify the acceptance artifact**

```bash
cp build/zeroone_x6_hybrid/zeroone_x6_hybrid.px4 build/zeroone_x6_hybrid/zeroone_x6_hybrid_usb_bus_off_fix.px4
sha256sum build/zeroone_x6_hybrid/zeroone_x6_hybrid_usb_bus_off_fix.px4
```

Record the exact SHA-256 in `state/LOG.md`.

- [ ] **Step 4: Verify repository scope and commit history**

```bash
git diff --check
git status --short
git log --oneline 0099d099cd..HEAD
```

Expected: only intentional source/test changes are committed; `state/` may remain untracked; no diagnostic Stage A-J source code remains.

- [ ] **Step 5: Perform hardware acceptance in order**

1. Start on USB power with both C610 nodes unpowered: PX4 USB CDC remains enumerated and QGC connects.
2. Start with both C610 nodes powered: USB remains available and both feedback counters increase.
3. While safely lifted and wheels unloaded, enter driving, then disconnect both CAN nodes: propulsion is inhibited, the existing fault is reported, and USB remains connected.
4. Disarm, reconnect both nodes, and restart or follow the existing fault-clear procedure: feedback and normal wheel control return.

Do not claim the bug fixed until all four physical checks pass.

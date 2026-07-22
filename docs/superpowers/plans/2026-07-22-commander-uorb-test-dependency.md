# Commander uORB Test Dependency Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `unit-CommanderHybridStatus` reliably wait for generated uORB topic headers in a clean parallel test build.

**Architecture:** Declare the missing build-order edge directly on the single test target. Preserve all source, link, runtime, and shared GoogleTest behavior.

**Tech Stack:** PX4 CMake, Ninja, GoogleTest, generated uORB headers.

## Global Constraints

- Work only in `/home/crocodile/PX4-Autopilot-debug-testc1-v1.16.1` on `debug/testc1-v1.16.1`.
- Do not modify `/home/crocodile/PX4-Autopilot`.
- Change only `src/modules/commander/CMakeLists.txt` for the implementation.
- Do not serialize the build, modify `px4_add_unit_gtest`, or link an unused library.
- Use a Linux-only `PATH` and redirect complete output to local log files.
- A successful incremental rerun is insufficient; GREEN must begin with no `build/px4_sitl_test` directory.
- Commit messages must use `<type>[scope]: <description>`.

---

### Task 1: Add the Missing uORB Header Dependency

**Files:**
- Modify: `src/modules/commander/CMakeLists.txt:83`

**Interfaces:**
- Consumes: existing targets `unit-CommanderHybridStatus` and `uorb_headers`.
- Produces: an explicit CMake target dependency; no C++ or link interface changes.

- [ ] **Step 1: Confirm the existing RED evidence**

Inspect `/tmp/m2006_tx_policy_green_attempt1.log` and
`/tmp/m2006_tx_policy_green_attempt2.log`.

Expected in both logs: clean parallel builds fail while compiling
`CommanderHybridStatusTest.cpp` with
`uORB/topics/hybrid_vehicle_status.h: No such file or directory`, before uORB
topic header generation completes.

- [ ] **Step 2: Add the minimal build dependency**

Immediately after the existing test declaration, add exactly:

```cmake
px4_add_unit_gtest(SRC CommanderHybridStatusTest.cpp)
add_dependencies(unit-CommanderHybridStatus uorb_headers)
```

- [ ] **Step 3: Verify clean GREEN**

Run:

```bash
rm -rf build/px4_sitl_test
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin make tests TESTFILTER=M2006TxPolicy > /tmp/commander_uorb_dependency_green.log 2>&1
```

Expected: exit 0; `unit-CommanderHybridStatus` compiles after generated uORB
headers and `unit-M2006TxPolicy` passes 1/1.

- [ ] **Step 4: Verify scope and commit**

Run:

```bash
git diff --check
git diff -- src/modules/commander/CMakeLists.txt
```

Expected: the implementation diff contains only the one explicit dependency.

Commit:

```bash
git add src/modules/commander/CMakeLists.txt
git commit -m "fix[commander]: order hybrid status test after uORB headers"
```

# Task 7 Report: HX8 Protected Request Scheduler

## Status and Scope

- Status: complete.
- Branch: `change1_v1.16.1`.
- Base HEAD: `b8ad1315ab32369f30d6b14b1bc0f9284441c67b`.
- Commit: `b68d2081c98affe7555744bb34bf20ecf31809ea` (`feat[hx8]: add protected request scheduler`).
- Scope is limited to a deterministic, host-tested HX8 controller/scheduler library and its unit tests.
- No UART file descriptor, uORB, PX4 parameter, HRT call, board/Kconfig/ROMFS change, heap allocation, Task 8/9 integration, or `/state` change was added.

## Design and State-Machine Choices

- A single fixed-size `PendingRequest` slot enforces one outstanding transaction. Every request is separated by at least 20 ms.
- Requests use fixed arrays only. Time enters through `ControllerInput::now_us` in deterministic microseconds.
- The scheduler uses the required 30 ms response timeout and two bounded retries. Retry exhaustion increments `timeout_count`, marks the servo offline/unhealthy/unverified, prohibits motion, and still permits an emergency release request.
- Emergency release is selected before retry, commissioning, target, boot/config, and monitoring work. A changed valid target is sent only once and is rejected when its sequence is repeated/out of order or its timestamp is future/expired. `CommandExpiryUs` is 500000 us.
- Motion requires verified calibration/configuration, online/healthy status, armed or prearmed state, and no lockdown/failsafe. Zero stall-power, temperature ADC, power-limit, or current-limit values are uncalibrated sentinels.
- Boot is read-only: ping the selected servo ID, then read parameter IDs 33, 34, 36, 37-43, and 46. ID is checked against the selected servo and baud code against 115200 (vendor code 5). Every expected value must match before `config_verified` becomes true.
- Persistent writes are latched but start only with fully disarmed, not prearmed, explicit commissioning and no lockdown/failsafe. The fixed nine-item protection table is processed as `write one -> successful command response -> read same item -> compare`; any mismatch, command failure, timeout, or loss of the commissioning gate aborts and leaves configuration unverified.
- Monitoring uses the vendor status command at 20 Hz while the moving flag is set and 5 Hz otherwise. Voltage/current/power/angle/status fields are decoded in vendor little-endian units; temperature uses the vendor thermistor equation.

## Files Changed

- `src/lib/hx8_servo/Hx8Controller.hpp` (new)
- `src/lib/hx8_servo/Hx8Controller.cpp` (new)
- `src/lib/hx8_servo/Hx8ControllerTest.cpp` (new)
- `src/lib/hx8_servo/CMakeLists.txt` (controller library and test target)

## TDD RED

Exact command (Linux-only PATH, output redirected):

```bash
env PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin make tests TESTFILTER=Hx8Controller > /tmp/hx8-t7-red.log 2>&1
```

- Exit code: 2, as expected.
- Intended failure: `Hx8ControllerTest.cpp:6:10: fatal error: Hx8Controller.hpp: No such file or directory`.
- No production controller existed during this run.

## GREEN Verification

The brief's aggregate spelling was attempted first:

```bash
env PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin make tests TESTFILTER='Hx8(Protocol|Controller)' > /tmp/hx8-t7-green.log 2>&1
```

- Exit code: 2 before test execution. PX4 expands `TESTFILTER` into an unquoted shell command, so `/bin/sh` parses the parentheses and reports `Syntax error: "(" unexpected`.
- This is a filter/command-generation limitation, not a compile or test failure.

Equivalent valid focused commands were then run strictly serially:

```bash
env PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin make tests TESTFILTER=Hx8Controller > /tmp/hx8-t7-green-controller.log 2>&1
env PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin make tests TESTFILTER=Hx8Protocol > /tmp/hx8-t7-green-protocol.log 2>&1
```

- Controller exit code: 0; CTest 1/1 executable passed. Direct gtest run passed 13/13 tests.
- Protocol exit code: 0; CTest 1/1 executable passed. Direct gtest run passed 10/10 tests.
- Direct count commands: `build/px4_sitl_test/unit-Hx8Controller` and `build/px4_sitl_test/unit-Hx8Protocol`, both exit 0.
- Astyle checks on the three new files exit 0 with empty logs.
- `git diff --check` exit code: 0, no output.

## Warnings and Investigation Notes

- One earlier attempt incorrectly launched the Controller and Protocol `make tests` commands concurrently against the same `build/px4_sitl_test`. Their simultaneous CMake regeneration caused missing generated uORB/gtest files. No source change was made for those environmental failures; all final evidence above is from strict serial runs after the build recovered.
- The first clean serial rebuild exposed an existing generated-uORB dependency ordering issue in `CommanderHybridStatusTest`; the uORB headers were generated later in that same run, and an unchanged rerun proceeded to the HX8 tests.
- CMake emitted existing developer warnings for legacy `FindPythonInterp` policy and `DOWNLOAD_EXTRACT_TIMESTAMP`; there were no Task 7 compiler warnings.
- A real first Controller run passed 12/13 tests and showed the commissioning-gate fixture had left its servo-ID selection target runnable. The fixture now uses a future timestamp so the target is rejected after boot; the production commissioning gate was not weakened. Final Controller evidence is 13/13.

## Concerns and Residual Risks

- This task is host-only. No physical HX8-U45H-M, UART bus, protection trip, persistent EEPROM behavior, or flight-controller timing was tested.
- The public vendor protocol has no model or firmware-version readback. Software verifies servo ID, 115200 baud behavior, protocol responses, and protection values; physical model identity remains a commissioning-record responsibility.
- `CommandExpiryUs` is fixed at 500 ms in this library because the brief requires expiry rejection but does not provide a separate runtime configuration. Task 8 integration must preserve the producer/consumer timing assumption.
- Persistent-write safety still ultimately depends on the servo honoring successful write responses and readback values; power loss during the nine-item sequence leaves `config_verified=false` and requires a complete subsequent boot verification.
- No board firmware build was run because Task 7 explicitly requires a pure host-tested library and focused host tests.

## Review Fixes

- Root cause 1: `Controller::update()` returned immediately for an outstanding request before evaluating the commissioning gate, so an armed, prearmed, lockdown, or failsafe input could leave a persistent write transaction active. The fix evaluates the gate first and clears outstanding/retry state before `abortPersistentWrite()`.
- Root cause 2: `Controller::acceptResponse()` counted command/ID/shape mismatches but retained an active write transaction, allowing a later valid response to advance it. The fix clears outstanding/retry state and aborts persistent writes on any mismatch while `_write_state != Idle`; ordinary RX error counting and emergency-release pending state are preserved.
- RED: `env PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin make tests TESTFILTER=Hx8Controller > /tmp/hx8-t7-fix-red.log 2>&1` exited 1 with 2 failing tests (`AbortsOutstandingWriteWhenCommissioningGateIsLost`, `AbortsWriteOnUnexpectedResponseAndIgnoresLaterValidResponse`).
- GREEN: the required serial Controller command in `/tmp/hx8-t7-fix-green.log` exited 0 (1/1 CTest passed); the subsequent serial Protocol command in `/tmp/hx8-t7-fix-protocol.log` exited 0 (1/1 CTest passed).
- Files: `src/lib/hx8_servo/Hx8Controller.cpp`, `src/lib/hx8_servo/Hx8ControllerTest.cpp`, `.superpowers/sdd/task-7-report.md`.
- Baseline commit SHA: `b68d2081c98affe7555744bb34bf20ecf31809ea`.
- `git diff --check`: clean (exit 0).

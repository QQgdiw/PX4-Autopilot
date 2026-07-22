# HX8 final-review remediation report

## Findings closed

- Motion command health is now tied to the current MOVE sequence. Before a
  MOVE exists, for stale/mismatched status sequences, and while the current
  sequence is `RESULT_NONE`, transition health remains permitted. Only a
  terminal non-accepted result for the current MOVE rejects the transition;
  only current-sequence `RESULT_ACCEPTED` makes motion effective.
- HX8 status protection masking now follows the public protocol contract:
  bit 1 remains raw command-error status and bits 2--7 (`0xfc`) are protection.
- The driver destructor remains release-free. The commissioning guide now
  documents the safe preconditions and limitations of `hx8_uart_servo stop`,
  including its prohibition during config write.

## TDD evidence

RED:

- `make tests TESTFILTER=Hx8BackendPolicy` failed to compile because
  `motionCommandHealthy` did not exist. Log:
  `/tmp/hx8-final-red-policy.log`.
- Direct `unit-Hx8Controller` build/CTest ran 22 tests and failed the new
  bit1-only status assertion (21 passed, 1 failed). Logs:
  `/tmp/hx8-final-red-controller-build.log` and
  `/tmp/hx8-final-red-controller.log`.
- `python3 .superpowers/sdd/task-10-hx8-doc-contract.py` failed only the new
  driver-stop safety boundary (20/21). Log:
  `/tmp/hx8-final-red-doc.log`.

GREEN:

- Serial `make tests TESTFILTER=<target>` passed 1/1 for
  `Hx8BackendPolicy`, `TransformationStateMachine`, `Hx8Controller`,
  `Hx8Protocol`, and `CommanderHybridStatus`. Logs:
  `/tmp/hx8-final-regression-<target>.log`.
- Documentation contract passed 21/21:
  `/tmp/hx8-final-doc-contract.log`.
- Source contract passed sequence-gated module integration, exact `0xfc`
  mask, and release-free destructor:
  `/tmp/hx8-final-source-contract.log`.
- `make zeroone_x6_hybrid` passed and created
  `build/zeroone_x6_hybrid/zeroone_x6_hybrid.px4`. FLASH is
  1,865,856 / 1,966,080 bytes (94.90%). Log:
  `/tmp/hx8-final-review-fix-build.log`.

## Commits

- `54b0869c59` `fix[hybrid]: gate HX8 results by motion sequence`
- `9a98d3fef8` `fix[hx8]: correct protection status mask`
- `d86bdd6a4a` `docs[hybrid]: clarify HX8 driver stop safety`

## Residual evidence gap

No HX8 UART hardware, servo, automatic-direction adapter, or loaded mechanism
was exercised. Host tests and firmware build do not establish physical release,
stall-current reduction, or thermal protection behavior.

---

# Whole-branch final-review remediation (2026-07-22)

## Scope and root causes

This remediation closes only the three findings in
`final-review-fix-brief.md`.

1. Quad-Rover Commander applied the dedicated Rover policy only when
   `rover_velocity` was already selected. Legacy `velocity`, `body_rate`, and
   other selections therefore bypassed the shape/status gate, while the two
   differential controllers could fall through to their legacy branches.
2. The hybrid status stream factory existed, but no normal default MAVLink
   mode configured it.
3. Mission feasibility whitelisted command 50000 without validating its
   target or reserved parameters.

## TDD evidence

### Critical: independent Rover Offboard

RED command:

`cmake --build build/px4_sitl_test --target unit-RoverVelocityOffboardPolicy`

Expected failure: the new exact-mode/vehicle-identity tests did not compile
because `roverOffboardModeAvailable` was absent. Exit 1 is recorded in
`final-review-fix-critical-red.log`.

GREEN evidence:

- `unit-RoverVelocityOffboardPolicy` passed after adding the shared pure
  vehicle-aware availability and controller-ownership predicates.
- `unit-CommanderHybridStatus` covers legacy velocity/body-rate rejection,
  exact dedicated selection, healthy driving status, stale status, and fault.
- Existing policy coverage retains pre-transition and same-timestamp setpoint
  rejection, nonfinite rejection, and exact timeout behavior.
- Both `DifferentialVelControl` and `DifferentialRateControl` now subscribe to
  `vehicle_status`, claim every armed Quad-Rover Offboard selection before the
  legacy branches, and publish controller-owned zero throttle/steering when
  the exact dedicated input/status checks fail. The executable external
  contract asserts both integration sites. Non-hybrid Rover retains the old
  flag-derived legacy branch decision.

Logs: `final-review-fix-critical-green.log`,
`final-review-fix-critical-modules.log`, and
`final-review-fix-focused-tests.log`.

The focused re-review then exposed that the initial three-argument ownership
helper did not explicitly distinguish non-hybrid generic velocity from
non-hybrid dedicated `rover_velocity`. A real uORB functional test was added
for both differential controllers. With the old ambiguous decision restored,
`functional-DifferentialOffboardControl` was RED: 2/3 tests passed and
`NonHybridLegacyVelocityAndBodyRateRemainActive` failed because throttle was
zero. After adding explicit `rover_velocity_selected` ownership, it was GREEN
3/3. The same binary verifies Quad-Rover legacy velocity and body-rate publish
zero throttle/steering; pre-transition, fault, and stale status publish zeros;
valid dedicated input publishes nonzero commands; and non-hybrid legacy
velocity/body-rate remain active. Logs: `final-review-fix-controller-red.log`
and `final-review-fix-controller-green.log`.

### Important: default MAVLink status stream

RED command:

`bash test/hybrid_quad_rover_contract.sh`

Expected failures: zero guarded default configurations were found, and the
document did not state a default rate/mode boundary. Exit 1 is recorded in
`final-review-fix-mavlink-red.log`.

GREEN evidence: the same contract passed after adding macro-guarded 1 Hz
configuration in MAVLink Normal and Onboard modes and documenting that exact
default. Other modes remain disabled by default. The existing stream still
returns size zero and exits before its uORB update on MAVLink1. Log:
`final-review-fix-mavlink-green.log`.

### Important: mission feasibility parameters

RED command:

`cmake --build build/px4_sitl_test --target functional-FeasibilityChecker && ctest --test-dir build/px4_sitl_test --output-on-failure -R '^functional-FeasibilityChecker$'`

Expected failure: the new functional test reported false-expected assertions
for fractional/nonfinite targets and nonzero/NaN/infinite reserved values; 8/9
tests passed and the new test failed. Exit 8 is recorded in
`final-review-fix-feasibility-red.log`.

GREEN evidence: `functional-FeasibilityChecker` passed after reusing the exact
mission target predicate and adding a pure finite-zero reserved-parameter
predicate. Command 50000 now accepts only target 1 or 2 and finite zero in
params 2 through 7. Logs: `final-review-fix-feasibility-green.log` and
`final-review-fix-post-nuttx-focused.log`.

The first hybrid build exposed a NuttX macro collision with
`std::fpclassify`; replacing it with the equivalent finite plus exact-zero
test fixed the target-specific compile failure. No fourth speculative fix was
attempted.

## Files changed

- Rover policy/tests, Commander Offboard check/test, and both differential
  velocity/rate controllers, plus
  `src/modules/rover_differential/DifferentialOffboardControlTest.cpp` and its
  functional-test registration in `src/modules/rover_differential/CMakeLists.txt`.
- MAVLink default stream configuration, hybrid external contract document,
  and executable contract script.
- Hybrid mission pure predicates plus feasibility implementation/test.

No direct left/right wheel mapping was added, no default-board build was run,
and `/state` plus the progress ledger were not modified.

## Final verification

- Six focused CTest targets passed: Rover policy, both-controller functional
  Offboard behavior, ModeManagement, Commander hybrid status,
  HybridTransitionMission, and FeasibilityChecker.
- `test/hybrid_quad_rover_contract.sh` passed.
- `git diff --check` passed.
- `make zeroone_x6_hybrid` passed and produced
  `build/zeroone_x6_hybrid/zeroone_x6_hybrid.px4`.
- FLASH is 1,872,928 / 1,966,080 bytes (95.26%). The preceding Task 9 record
  was 1,872,176 bytes (95.22%), so this remediation adds approximately 752
  linked bytes and remains close to the physical limit.

## Commits

- `0c41c2e46f` `fix[hybrid]: close final review safety gaps`
- report commit: the subsequent documentation-only commit containing this
  appended evidence.

## Concerns

- No live MAVLink2 Normal/Onboard link, QGC/companion, real Offboard input, or
  physical Rover output was exercised; software tests do not replace Task 10
  bench validation.
- FLASH is at 95.26%, leaving limited growth margin.

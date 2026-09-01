# Current Task

- [x] Create an isolated worktree from `origin/testc1_v1.16.1`.
- [x] Verify branch, base commit, and clean initial worktree state.
- [x] Build `zeroone_x6_hybrid` and capture the result.
- [x] Confirm the build completed without a reproducible compile or link failure.
- [x] Identify the exact known-good firmware revision from the shared checkout artifact.
- [x] Compare USB/NuttX board configuration and startup behavior against `d86bdd6a4a`.
- [x] Flash the no-M2006 diagnostic firmware and observe whether main-firmware USB enumerates.
- [x] Flash Stage A (stop before UAVCAN `SystemClock`) and record USB enumeration.
- [x] Flash Stage B (initialize `SystemClock` only) and record USB enumeration.
- [x] Flash Stage C (run `_can.init(1000000)` only) and record USB enumeration.
- [x] Flash Stage D (configure CAN feedback filters only) and record USB enumeration.
- [x] Flash Stage E (full initialization with an empty scheduled `Run()`) and record USB enumeration.
- [x] Flash Stage F (full `Run()` with CAN transmit disabled) and record USB enumeration.
- [x] Flash Stage G (full `Run()` with exactly one CAN transmit attempt) and record USB enumeration.
- [x] Flash Stage H (one message-RAM fill, return before `TXBAR`) and record USB enumeration.
- [x] Flash Stage I (one `TXBAR` request with FDCAN interrupt lines disabled) and record USB enumeration.
- [x] Flash Stage J (one TX request with only bus-off interrupt disabled) and record USB enumeration.
- [x] Determine whether USB CDC is absent from the binary, fails during initialization, or is blocked after startup.
- [x] Confirm a single root-cause hypothesis before proposing a source fix.
- [x] Design the production fix for startup without CAN peers and runtime bus-off.
- [x] Repair the Commander unit-test/uORB generated-header dependency gate and
      validate it from clean test and normal firmware build directories.
- [x] Implement and independently verify M2006's feedback-aware transmit policy.
- [x] Correct the Task 2 host-test registration: the default SITL test board does
      not configure the UAVCAN CMake subtree, so its mandatory RED/GREEN command
      now compiles and runs the pure cleanup policy test.
- [x] Integrate and independently review STM32H7 bus-off pending-mailbox cleanup.
- [x] Integrate and independently review M2006 CAN maintenance and feedback-aware TX.
- [x] Implement the approved production fix with test-first verification and final
      clean focused tests/firmware build.
- [x] Perform the remaining target-hardware USB/CAN acceptance checks with C610
      nodes powered and during live disconnect/reconnect.
- [x] Verify USB-only startup with both C610 nodes unpowered after the filter
      configuration fix.
- [x] Isolate and fix post-init STM32H7 FDCAN filter configuration failure.
- [x] Confirm target topology: PMU DroneCAN on CAN1 and C610/M2006 on CAN2.
- [x] Verify CAN2 board routing and identify the existing H7 multi-instance
      mapping and global ownership blockers.
- [x] Obtain approval for the CAN2 isolation design.
- [x] Obtain approval for the written CAN2 isolation specification.
- [x] Review and execute the test-first CAN2 isolation implementation plan.
- [x] Define and host-test the physical-to-logical CAN interface mapping.
- [x] Implement and host-test atomic ownership per physical CAN bus.
- [x] Write focused failing tests for per-physical-bus ownership and local
      physical-to-logical FDCAN interface mapping.
- [x] Close Task 3 review findings: provide a real NuttX completion barrier for
      global FDCAN/IRQ initialization and shut down per-bus interrupt sources
      before detaching a destroyed driver instance.
- [x] Implement CAN1 DroneCAN and CAN2 M2006 isolation without reintroducing
      runtime FDCAN filter configuration.
- [x] Update stale M2006 Kconfig help from CAN1/DroneCAN mutual exclusion to the
      fixed CAN2 topology during final review cleanup.
- [x] Build `zeroone_x6_hybrid` and record the separated-bus firmware checksum.
- [x] Perform PMU/C610/USB target acceptance on the separated CAN buses.
- [ ] Diagnose why the target boot did not leave the DroneCAN application
      running; capture the direct `uavcan start` error before changing code.
- [x] Run a clean DroneCAN-only power-cycle test with `M2K_EN=0` and
      `UAVCAN_ENABLE=0` at boot, then set UAVCAN_ENABLE=3 and start manually.
- [x] Flash the staged DroneCAN startup diagnostic and locate the exact boundary
      at which `uavcan start` exits or the node instance disappears.
- [x] Confirm the DroneCAN startup root cause and approve the pre-init logical
      interface topology design.
- [x] Add a failing host regression test for the driver's pre-init interface
      count, then implement the approved constructor contract.
- [x] Write and self-review the implementation plan for the approved fix.
- [x] Remove temporary diagnostics, run final focused host tests, and build the
      named production firmware artifact.
- [x] Flash `zeroone_x6_hybrid_dronecan_topology_fix.px4` and validate
      DroneCAN/PMU alone after a full power cycle.
- [x] Confirm CAN1 error counters are not increasing before enabling
      simultaneous M2006 traffic.
- [x] Re-enable M2006 and validate simultaneous CAN1 DroneCAN, CAN2 M2006, and
      stable USB/QGC connectivity.
- [x] Confirm the simultaneous CAN2 hardware-error count remains static.
- [x] Perform the disarmed C610 disconnect/reconnect recovery check.
- [x] Complete fresh final tests/build and whole-change independent review.
- [x] Isolate the PMU comparison: PMU A fails on both controllers; PMU B
      publishes on both, so PMU A is faulty/misconfigured and controller A
      DroneCAN is healthy.
- [x] Read and record the numeric HX8 manufacturer-default protection values.
- [x] Confirm commissioning overrides: response=1, stall protection=1, and
      minimum voltage=9000 mV instead of incompatible factory values.
- [x] Decide whether to retain the 20 W factory internal power limit and lower
      runtime motion power to <=20 W, or commission a higher internal limit for
      the previously requested 36 W motion.
- [x] Fix register 41 expected configuration units from raw ADC to degrees C
      while retaining raw-ADC conversion for live status telemetry.
- [x] Fix HX8 backend validation so it does not require fake PWM endpoint
      parameters, and make absent-driver `config check` return failure.
- [ ] Flash the new firmware, set the reviewed HX8 parameter profile, fully
      power-cycle, confirm the automatically started driver, then run the
      intentional disarmed `config write`.
- [ ] After configuration write, verify readback/status and bench-test
      provisional 90/180 degree endpoint motion plus induced stall release.
- [ ] Push commit `ece4655271` only after explicit owner request or completed
      hardware acceptance.
- [ ] Flash the bounded HX8 TX diagnostic build and record `hx8_uart_servo
      status` after a full power cycle to distinguish serial write failure from
      an incorrect physical observation point.
- [ ] Flash the HX8 serial-work-queue FD fix, then record the new TX counters
      and first UART response before any further protocol or board-mapping work.
- [ ] Flash the echoed-parameter-response parser fix and confirm all 11 boot
      configuration reads complete before testing persistent writes.
- [ ] Flash the temperature Celsius-to-ADC encoding fix and confirm
      `config_verified`, `healthy`, and `config check` all pass.
- [x] Replace the invalid `RC_MAP_TRANS_SW` channel-to-AUX reinterpretation with
      the canonical `manual_control_switches.transition_switch` input.
- [x] Stop periodic stable-state HX8 timed-move/HOLD retransmission and cover
      the motion-enable edge behavior with a focused unit test.
- [x] Run the HX8, transformation-state-machine, and ManualControl tests and
      build `zeroone_x6_hybrid`.
- [ ] Flash the RC/HX8 fix and verify Channel 7 changes
      `manual_control_switches.transition_switch`, starts both transformation
      directions, and no longer reports `HX8 actuator command rejected` while
      remaining armed at a stable endpoint.
- [x] Reproduce the HX8 fault with a stable Quad HOLD and correlate
      `command_result=2` with the servo's `status_flags=1` executing state.
- [x] Treat HX8 command response code 1 as accepted/executing and retain
      response codes 2+ as rejection/error; add controller regression coverage.
- [ ] Flash the response-code fix and verify Quad HOLD, Rover MOVE, and arm
      preflight on hardware after a full power cycle.
- [x] Confirm on hardware that response code 1 is accepted for the secondary
      RELEASE after a Rover fault.
- [ ] Temporarily set `HX8_ANG_ROV=179`, reboot, and test the Rover MOVE to
      determine whether exact +180 degrees is a strict vendor command boundary.
- [ ] If 179 degrees is also rejected, capture the first TimedMove request and
      response (`12 4c 0b ...` / `05 1c 0b ...`) before RELEASE overwrites the
      aggregate command result.
- [ ] Flash the diagnostic artifact and capture `local motion accept` or
      `local motion reject`/`local sequence reject` for the first Rover request.
- [ ] Flash the corrected sequence/result diagnostic and capture `motion queued`
      or `controller motion reject`, plus `HX8 motion unhealthy` if fault 11 occurs.

## 2026-08-25 multicopter idle-output investigation

- [x] Audit the MC setpoint and final actuator-output path after the reported
      loss of quadrotor idle.
- [x] Build the corrected cached-status spool-up fix; artifact SHA-256 is
      `c97433197a51f651bc778b8447600c071fa4feb795991366814a1857fa3fc471`.
- [ ] Flash the current spool-up fix and capture `actuator_motors_mc`, final
      `actuator_motors`, `actuator_outputs`, `actuator_armed`, and `dshot
      status` at 0.5 s, 1 s, and 2 s after arming.
- [ ] Distinguish intentional `COM_SPOOLUP_TIME` zero-thrust from a missing
      DShot/PWM idle output before changing safety-critical output code.

## 2026-08-20 audit and implementation

- [x] Audit `testv3`/`change_v1.16.1` history and compare commit `325a9d07ba`.
- [x] Obtain approval for the proposed HX8 transformation/arming safety policy.
- [x] Selectively port Rover auto-disarm and ManualControl gesture fixes;
      do not cherry-pick `325a9d07ba` wholesale.
- [x] Implement disarmed transition MOVE gating while retaining lockdown/failsafe
      blocking and stable HOLD restrictions.
- [x] Prevent the RC re-arm grace period from bypassing fresh HX8 endpoint checks.
- [x] Add airborne-Quad degradation and Rover runtime endpoint fault gating,
      then add focused tests and build `zeroone_x6_hybrid`.

## 2026-08-20 M2006 gate diagnosis

- [x] Add one-shot `m2006_can` gate-fault diagnostics. The warning reports newly
      latched fault bits, armed/driving/inhibit state, feedback online state,
      command freshness/finiteness, controller configuration validity, CAN error
      count delta, TX failures, command age, and actuator controls without changing
      the safety gate behavior.
- [ ] Flash the diagnostic firmware and capture the first warning after Rover arm.
- [x] Change disarmed M2006 fault recovery to ignore normal invalid/NaN wheel
      commands while retaining strict command checks for armed Rover driving.
- [x] Flash the recovery-fix firmware and confirm `fault_flags` clears after
      disarm without a full reboot; cumulative `hw errors` behavior matches
      expectation.

## 2026-08-21 M2006 no-motion software audit

## 2026-08-21 M2006 speed-domain correction and startup preflight

- [x] Confirm hardware speed semantics: C610 feedback is rotor RPM before the
      36:1 gearbox; total mechanical reduction is 144:1 and wheel diameter is
      345 mm.
- [x] Change `M2K_MAX_RPM` and `M2K_RPM_SLEW` to rotor RPM units with defaults
      of 18000 and update the status-message comments, airframe defaults, and
      commissioning documentation.
- [ ] Run focused M2006 speed-controller tests and `make zeroone_x6_hybrid`.
- [ ] Flash and verify target `target_rpm`/`measured_rpm` semantics and loaded
      wheel-speed calibration.
- [ ] Reproduce an actual startup arming failure and capture the correctly
      named `hybrid_vehicle_status` plus health topics before changing startup
      fault policy.

- [x] Verify the live Rover chain reaches both C610 command slots: final motor
      outputs, target RPM, closed-loop current, and feedback all agree.
- [x] Audit the C610 frame encoder and STM32H7 FDCAN standard-frame path; no
      ID, byte-order, CAN-instance, or competing-sender defect was found.
- [x] Run the focused `unit-C610Protocol` regression suite (4/4 passed).
- [ ] Obtain approval for a bounded raw-current commissioning diagnostic only if
      the owner wants to distinguish the C610/motor power stage from the normal
      speed loop without a CAN analyzer.
- [ ] Measure C610 input voltage/current during a bounded raw-current test and
      capture the actual transmitted 0x200 payload before changing control code.
- [x] Add `m2006_can status` diagnostics for the most recent C610 TX ID, DLC,
      payload, and low-level send result without changing control behavior.
- [ ] Flash the TX-diagnostic firmware and capture the payload at the tested
      near-5000-current operating point.
- [x] Verify on hardware that the TX payload tracks the commanded current and
      remains error-free on CAN2.
- [ ] Capture `m2006_motor_status` beside a high-current TX sample, including
      RPM, torque_current, encoder, and both motor online flags.
- [x] Reach and verify the 5000/5000 TX command on hardware.
- [x] Capture a bounded oscilloscope/CAN comparison showing that the bus-side
      `0x200` current slots track PX4 and both C610 feedback current fields rise
      while RPM remains zero; input power increases by about 10.76 W.
- [ ] With the system disarmed, rotate each physical M2006 separately and map
      it to exactly one changing `0x201`/`0x202` encoder and RPM stream; verify
      each motor's phase and feedback cables terminate at the same C610.
- [ ] At a bounded 500--1000 command, use isolated/differential probing to
      determine whether each C610 produces valid three-phase PWM/commutation.
- [ ] If cable pairing and phase PWM are both valid, add an armed-driving,
      watchdog-limited one-motor-at-a-time raw-current commissioning command
      only after owner approval, to isolate left and right without the speed
      loop.
- [ ] Repeat at a lower sustained current (suggest 1500--2000) and determine
      whether the right feedback timeout/beep is current-dependent.
- [ ] If the timeout repeats, add one-shot per-motor feedback-age diagnostics
      to the gate warning before changing timeout policy.
- [ ] Run clean repeated Rover arm trials and capture the complete first CAN
      diagnostic warning before moving to oscilloscope analysis.
- [x] Identify arm-edge ordering: the first fault is `DriveFaultCommand` with a
      fresh but non-finite final wheel sample, before `DriveFaultCan`.
- [x] Add armed-command qualification: do not latch a command fault before the
      first fresh finite Rover sample; retain strict faulting after qualification.
- [ ] Flash the command-qualification firmware and repeat the neutral Rover arm
      test to determine whether the remaining first fault is CAN-only.
- [x] Hardware result after command qualification: neutral Rover arm produces
      only `fault_flags=0x04`, with `current_command=0` and both feedback streams
      online; the remaining fault is isolated to the CAN layer.
- [ ] Capture CAN2 physical-layer waveform and ACK/error behavior at the first
      100 ms after Rover arm.
- [x] Add a first-error STM32H7 FDCAN register snapshot for CEL protocol errors
      and Bus-Off, exposed by `m2006_can status` without changing gate behavior.
- [ ] Flash the register-snapshot artifact and capture the first snapshot after
      a clean disarm baseline followed by one neutral Rover arm.
- [x] Extend H7 diagnostics to split aggregate errors into internal, RX queue
      overflow, CEL, Bus-Off, TX timeout, RX FIFO lost, and voluntary aborts;
      print an explicit `first snapshot=none` when no snapshot trigger occurred.
- [ ] Flash diagnostic v2 and compare the source counters before and after one
      neutral Rover arm, with special attention to `rx_overflow`.
- [x] Confirm diagnostic v2: the entire arm-edge error increment was software
      RX queue overflow (`9 -> 73`), with CEL/Bus-Off/TX-timeout/RX-FIFO-loss all
      zero and both motors online.
- [ ] Flash the 128-entry M2006 RX queue build and repeat clean neutral Rover arm
      trials; verify `rx_overflow` and `fault_flags` remain zero.
- [x] Add an HX8 first-protection snapshot: preserve the first observed raw
      status/protection bits, command sequence, position, voltage, current,
      power, temperature and command result for the driver lifetime.
- [ ] Flash the HX8 protection-snapshot build, reproduce one installed
      transformation protection stop, then capture `hx8_uart_servo status`.
- [x] Capture installed-mechanism HX8 protection: raw `0x41` during sequence 3
      maps to executing plus the `0x40` power-protection bit at 24.718 W.
- [ ] Inspect the mechanism and motion profile around the Quad-direction
      9.4-degree servo position; only then run a lower-power/slower-motion
      confirmation trial without modifying stored HX8 protection thresholds.
- [x] Fix directed HX8/AS5600 angle normalization for a near-180-degree span;
      add regression coverage for `-85 -> 95` with feedback `95.1` and retain
      the existing exact `-90 -> 90` direction tests.
- [ ] Flash the angle-normalization artifact and verify Rover feedback near
      `95.1` reports `position_normalized` near `1.0` and clears the latched
      NoSensor fault only after the normal disarmed clear procedure.

## 2026-08-25 Rover manual steering direction

- [x] Confirm the installed mapping: left wheel is C610 ID 1, right wheel is
      ID 2; `M2K_L_REV=Enabled` and `M2K_R_REV=Disabled` preserve correct
      forward/reverse motion, but manual left steering produces a right turn.
- [x] Reverse only the manual Rover steering input at the shared Rover
      control boundary for Manual, Acro, Stability, and Position modes.
- [x] Build `zeroone_x6_hybrid`; the resulting artifact uses 95.25% FLASH and
      has SHA-256 `d49c3df7ac5818b1c9a91f671b345227e84432fb5b79870c737507eab779d6b0`.
- [ ] Flash the artifact and verify left/right steering on the installed
      vehicle while confirming forward/reverse remains unchanged.

## 2026-08-25 Multicopter zero-thrust fix

- [x] Trace zero rotor output from `manual_control_setpoint.throttle` through
      `vehicle_attitude_setpoint`, `vehicle_thrust_setpoint`, and the allocator.
- [x] Fix `mc_att_control` spool-up state refresh after the top-level
      `vehicle_status` subscription sample is consumed.
- [x] Build `zeroone_x6_hybrid`; artifact SHA-256 is
      `d5f657dd21fd0d0aa1a684cdd95ed7a0a23a6e4e6d55aa796ab7a16103910cb6`.
      FLASH usage is 1,872,832 / 1,966,080 B (95.26%).
- [ ] Flash and verify that manual throttle produces nonzero thrust after the
      configured `COM_SPOOLUP_TIME`, while Rover mode still suppresses MC output.

## 2026-08-26 Rover PID documentation and ULog tooling

- [x] Audit Manual, Acro, Stabilized, Position, Auto/Mission, and Offboard
      control paths in the current `debug/testc1-v1.16.1` source.
- [x] Document every Rover feedback loop with its setpoint, measurement,
      output, gains, cascade/parallel relationship, and applicable modes.
- [x] Document Rover-only and M2006 drive parameters, including airframe
      default overrides and non-PID motion/path limits.
- [x] Reorder the Rover parameter table by practical tuning sequence and add
      per-parameter test methods and high/low-value symptoms.
- [x] Replace the old plotting script with a current-worktree enhanced version
      that loads only a fixed Rover topic whitelist and writes structured plots
      and metrics without exporting raw samples.
- [x] Smoke-test the script on an existing Rover ULog and visually inspect the
      yaw-rate and mode-overview plots.
- [ ] Analyze the next owner-provided ULog using a bounded `--start/--end`
      interval selected from the test procedure.

## 2026-09-01 testc3 checkpoint and testc4 integration

- [x] Commit-ready audit and hybrid target build completed for the validated
      `debug/testc1-v1.16.1` working state on a
      new `testc3_v1.16.1` checkpoint branch and push it without modifying the
      original remote `testc1_v1.16.1`.
- [ ] Create an independent `debug/testc4_v1.16.1` worktree from the identical
      checkpoint and publish remote `testc4_v1.16.1`.
- [ ] Semantically merge `origin/testc2_v1.16.1` at fixed commit
      `82478dbdf26540c7b8719f44cd07dcff1fb66b83`, preserving testc1 hardware
      behavior and testc2 independent Hybrid protocol/control behavior.
- [ ] In a second independent worktree, migrate the Differential Rover tuning
      uORB/MAVLink streams and bounded stream-configuration lifecycle from
      `fix/mini-rover-mavlink-stream-config` without importing the mini board.
- [ ] Run relevant unit tests, isolated-PATH `make tests`, `git diff --check`,
      and serial `make zeroone_x6_hybrid`; record artifacts and residual risks.

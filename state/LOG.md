# Test Log

## 2026-09-04 persistent HX8 protection diagnosis

- A full cold start with the manual gear switch centered produced
  `command_sequence=0`, `status_flags=0`, `protection_flags=0`, and
  `healthy=true`. Moving the switch in either direction immediately reproduces
  power protection, proving that the first PX4 multi-turn motion command is the
  trigger rather than an idle servo condition or a particular endpoint.
- Comparison against FashionStar's official SDK at commit
  `772c181ec683252b9aaa360d29cecf2f76022051` found a concrete `0x0e` packing
  defect: the multi-turn interval field is unsigned 32-bit, but
  `Hx8Controller` currently emits it as 16-bit. The resulting 12-byte payload
  is two bytes short; acceleration, deceleration, and power are shifted from
  the servo's expected offsets. The protocol encoder test contains a correct
  14-byte example, while the controller test incorrectly requires 12 bytes.
- `Hx8Controller` now writes the multi-turn interval with `write32`, moves the
  three 16-bit fields to offsets 8/10/12, and emits all 14 payload bytes. All
  seven HX unit tests passed in `state/test_hx_all_interval32.log`; target build
  passed in `state/build_hx8_interval32.log` with FLASH 1,889,984 / 1,966,080
  bytes. The resulting 1,773,676-byte firmware SHA-256 is
  `21571592d52922060a727335e74f0a754825e94b210e9cdbcda286fc58d25e87`.
- Widening the internal angle limits to +500/-500 degrees did not change the
  `0x40` behavior. This falsifies the earlier leading angle-limit trigger
  hypothesis; the former -180/+180 mismatch was real but not the cause of this
  persistent protection report.
- Hardware returned the bounded first snapshot at 11.841338 s:
  `flags=0x41`, `protection=0x40`, `seq=1`, angle -399.7 degrees, 11.968 V,
  0.014 A, and 0.16 W. Sequence 1 plus moving bit 0 shows that protection was
  first observed during the first post-boot gear command; the low sampled
  power is after the servo had entered protection and is not the trigger peak.
- Servo parameters 51/52 use 0.1-degree units. Values +1800/-1800 therefore
  mean +180/-180 degrees, not +1800/-1800 degrees, and do not cover the
  observed -399.7-degree position or the configured -400/+200-degree travel.
- Live HX8 status `0x41` contains moving bit 0 and protection bit 6 (`0x40`).
  The current U45H-M datasheet defines bit 6 as power protection. PX4 replaces
  the live flag on every status response and does not latch it, so a persistent
  live `protection_flags=0x40` is being repeatedly reported by the servo.
- The prior servo configuration had angle limiting enabled at -180 to +180
  degrees while the configured travel was -400 to +200 degrees. Although this
  was a concrete configuration conflict, the owner widened it to -500/+500 and
  reproduced the same protection, excluding it as the active trigger.
- Current boot verification does not read parameters 48, 51, or 52, so
  `config check=0` cannot detect this internal angle-limit conflict.
- `command_result=2` is downstream: motion scheduling requires live healthy
  status, so a command queued while bit 6 remains active cannot transmit and
  expires as rejected. The low power in a later status sample does not reveal
  the power at the initial protection transition; use the bounded first
  protection snapshot from `hx8_uart_servo status`.

## 2026-09-04 manual landing-gear sequence audit

- Read-only code audit confirms that `LG_AUTO_EN=0` disables only automatic
  landing-gear targets. HX8 online/healthy checks and the gear-down,
  gear-clear, and gear-stowed sequence interlocks remain active.
- A Rover-to-Quad request therefore enters `SEQUENCE_R2Q_PREPARE` and does not
  command the HX-65HM pair until the vehicle is disarmed and the manually
  controlled gear reaches `LG_ANG_DN`. Requests are accepted only from stable
  Quad or stable Rover states; an in-progress preparation cannot be reversed
  by another switch edge.
- `LG_MAN_CH` selects `manual_control_setpoint.aux1` through `aux6`; the actual
  receiver channel is selected separately by the matching `RC_MAP_AUXn`.
- Owner decision supersedes the initial retained-angle-interlock policy:
  `LG_AUTO_EN=0` now skips down/clear/stowed sequence stages and permits
  concurrent gear/shape motion, while retaining HX8 online/config/protection
  health, Quad-to-Rover landing/disarm, and Rover-to-Quad disarm gates.
- The reported rejected HX8 command had `healthy=false`, status flags `0x41`,
  protection flags `0x40`, and command result 2. The U45H-M datasheet maps
  status bit 6 (`0x40`) to power protection. Why it triggered at this physical
  installation remains unidentified; the condition remains intentionally
  blocking after the sequence change.
- Tests passed: `unit-HybridSequenceCoordinator`, `unit-CommanderHybridStatus`,
  `functional-hybridCheck`, and all seven Hx tests. `git diff --check` passed.
- `make zeroone_x6_hybrid` passed with FLASH 1,889,984 / 1,966,080 bytes
  (96.13%). The 1,773,656-byte PX4 artifact SHA-256 is
  `7c99c8242e24d2acd7001404b26aaaf5ac369e792d2ed85bc574d4c5a9ffbdea`.

## 2026-09-04 asymmetric HX8-to-H65 recovery guard

- Implemented a fixed 40 ms H65-not-before deadline after every HX8 TX and each
  HX8 RX chunk. Repeated HX8 traffic restarts the deadline. Once it expires,
  pending H65 work is attempted before ordinary HX8 work so the latter cannot
  immediately destroy the recovery window.
- HX8 emergency release remains allowed to preempt the H65 preference; its TX
  restarts the 40 ms deadline. The existing 5 ms general bus quiet interval is
  retained after H65 responses and response-less broadcasts.
- Trace metadata and raw data now print on separate lines, with raw bytes split
  into 16-byte chunks. This remains below the PX4 console line limit and avoids
  the truncated 21-byte responses observed in the previous target output.
- Focused Hx tests passed 7/7 and `make zeroone_x6_hybrid` passed. FLASH is
  1,890,024 bytes (96.13%). The 1,773,612-byte artifact SHA-256 is
  `ae2adcd53983d87dd927809ea89a5cf01c16a088d9424eea301fc60bbba1b6e4`.
- Hardware validation is pending. The guard is accepted only if both H65
  timeout/retry counters have zero growth over at least 60 seconds.

## 2026-09-04 cross-protocol pre-trigger result

- The ring froze on an ID-2 Monitor timeout with no RX bytes. The retained
  history provides a controlled comparison: an H65 request 34,660 us after a
  valid HX8 response succeeded, while an otherwise normal ID-2 Monitor sent
  only 9,629 us after the next valid HX8 response received no bytes and timed
  out 30,394 us later.
- All intervening H65-only Monitor transactions succeeded. The preceding HX8
  status response contains consecutive `ff ff` bytes near its tail, which can
  be misrecognized as an H65 header by the actuator's protocol parser. The
  timing comparison strongly supports an H65 internal parser recovery timeout
  between approximately 10 and 35 ms after foreign HX8 traffic; the existing
  5 ms cross-protocol quiet interval is insufficient.
- The observed target state remained healthy and every timeout still scheduled
  a retry: left timeout/retry 21/21, right 24/24, aggregate 45/45, and protocol
  error count 0. Raising response timeout or retry count would mask rather than
  correct the protocol-switch defect.
- PX4 console line length truncated the tail of 21-byte RX hex strings even
  though the ring retained their full length. Future trace output must print
  event metadata separately and split raw bytes into bounded chunks.
- This evidence led to the implemented asymmetric fixed 40 ms H65-not-before
  guard after HX8 traffic while retaining the 5 ms general bus gap in the other
  direction. The guard is a protocol-interoperability invariant, not a user
  tuning parameter.

## 2026-09-04 cross-protocol Monitor pre-trigger trace

- Replaced the per-Monitor linear trace with a 16-entry circular history. Each
  entry stores an absolute HRT timestamp, protocol/direction, servo ID, command
  code, parser result, and up to 32 raw bytes; total retained raw data is capped
  at 512 bytes. Frozen output is reordered oldest-to-newest and timestamps are
  relative to the oldest retained event.
- The ring records HX8 TX/RX/timeout and HX-65HM TX/RX/parser/timeout events.
  It begins after successful boot verification and overwrites only the oldest
  history until the first H65 Monitor timeout freezes outcome 4. The retry can
  no longer overwrite the causally relevant pre-trigger window.
- Focused Hx tests passed 7/7 and `make zeroone_x6_hybrid` passed. FLASH is
  1,889,800 bytes (96.12%). The 1,773,492-byte artifact SHA-256 is
  `e4ad65f5db48a10cf44198ed5fcd09a50856d6f632dfe5fe2644454d992dba17`.
- Hardware capture remains pending; no claim is yet made that a preceding HX8
  frame causes the H65 no-response transaction.

## 2026-09-04 first steady-state Monitor timeout

- The no-Ping firmware passed target configuration verification (`config
  check` returned 0); both HX-65HM devices were online, healthy, verified, and
  reporting valid positions. This confirms targeted identity-register Reads
  replaced Ping successfully.
- The frozen first failed Monitor transaction contains only the exact ID-1 Read
  request `ff ff 01 04 02 38 0f b1` followed by a timeout at +34,567 us. No RX
  byte and no parser error occurred during that transaction.
- Later counters were left timeout/retry 236/236 and right 236/236; the uORB
  aggregate reached timeout/retry 484/484 with protocol_error_count 0 while both
  devices remained healthy. Every observed miss therefore recovered on retry,
  and the defect is symmetric, repeatable no-response on an initial Monitor
  attempt rather than damaged responses or a particular actuator branch.
- Boot uses consecutive HX-65HM Reads and succeeds, while steady operation
  interleaves HX8 status traffic. A plausible but unproven cause is that foreign
  HX8 traffic leaves the HX-65HM device parser in a partial state, causing the
  first following Read to be discarded; its retry then runs without intervening
  HX8 traffic and succeeds. The current trace starts at Monitor TX and cannot
  prove this because it excludes the immediately preceding HX8 TX/RX.
- The next diagnostic should retain a circular pre-trigger history of both
  protocols and freeze the last HX8 transaction, switch gap, failed Monitor,
  and timeout together. Do not hide the issue by raising retry count or timeout.

## 2026-09-04 remove multidrop Ping and capture Monitor timeout

- Removed Ping from the HX-65HM pair controller's boot state machine. Each side
  now starts with a targeted Read of address 5 length 4, which validates the
  response-frame ID, stored ID, response level, and adjacent identity fields.
  The low-level protocol encoder retains Ping only as protocol completeness;
  no production controller path references it.
- Added a regression requiring the first boot request to be the ID-1 identity
  Read and every boot request to differ from Ping. Boot retry and post-emergency-
  release tests now also require Identity rather than Ping.
- A successful boot trace is discarded and the same 512-byte buffer is armed
  for runtime Monitor diagnosis. Every valid Monitor response discards its
  transaction and rearms the buffer; the first Monitor timeout freezes its TX,
  raw RX, parser results, and timeout marker with outcome 4. HX8 traffic is not
  captured because the buffer is active only while a Monitor transaction is
  outstanding.
- Focused Hx tests passed 7/7 and `make zeroone_x6_hybrid` passed. FLASH is
  1,889,592 bytes (96.11%). The 1,773,364-byte artifact SHA-256 is
  `7bb077f56b4f3624b59c8c2d2ef6614b4a3c475cb16fe85203f6719f8e2c9ea8`.
- Hardware verification of zero startup contention and the first runtime
  Monitor timeout trace remains pending.

## 2026-09-04 HX-65HM targeted-Ping bus collision confirmed

- Oscilloscope evidence is stored at
  `docs/picture/HX串口调试波形图0904.jpg`. The captured request is the ID-1
  Ping `ff ff 01 02 01 fb`; the decoder reports the same corrupt response
  `ff ff ff ff 01 fb 00 fc` previously captured by PX4.
- With both HX-65HM servos attached, the response contains intermediate analog
  voltage levels. With either servo attached alone, the same ID-1 Ping causes
  that servo to respond using its own configured ID: ID 1 returns
  `ff ff 01 02 00 fc`, while ID 2 returns `ff ff 02 02 00 fb`.
- The two devices begin responding at the same approximately 43.2 us delay.
  Their common header/length/error bits agree, while differing ID/checksum bits
  drive opposing levels and create the observed contention and invalid UART
  bytes. This confirms that HX-65HM firmware treats Ping as bus-wide despite
  the targeted ID field.
- The vendor SDK constructs a targeted Ping and assumes only a matching-ID
  response, but the observed HX-65HM behavior contradicts that assumption on a
  multidrop bus. Ping must therefore not be used for discovery or health checks
  with both actuators connected. A targeted identity-register Read already
  validates presence, returned ID, baud-code field, and response level without
  the collision seen in the boot trace.
- Runtime Monitor retries remain a separate unresolved issue: boot Ping can
  account for only one timeout per side, not the later 21/21 and 22/22 totals.
  A steady-state targeted-Read capture is still required before attributing
  those retries to the same cause.

## 2026-09-04 frozen HX-65HM boot bus trace

- Target capture succeeded without truncation: outcome 1, 31 events and 146
  data bytes. Both HX-65HM devices completed identity, protection, and mode
  verification, proving the exclusive boot scheduler can recover them.
- The first Ping attempt to each side was corrupt before parsing. ID 1 expected
  `ff ff 01 02 00 fc` but `read()` returned `ff ff ff 02 02 fc`; ID 2 expected
  `ff ff 02 02 00 fb` but returned `ff ff ff ff 01 fb 00 fc`. Neither byte
  sequence contains a complete valid response. Both second Ping attempts were
  exact and parsed successfully about 5 ms after transmission.
- Runtime status later showed left timeout/retry 21/21 and right 22/22 while
  both remained online, healthy, and verified. Therefore all counted misses
  recovered on retry; the defect is intermittent first-attempt RX corruption,
  not persistent addressing failure. Since corruption also occurs before any
  HX8 traffic, mixed-protocol boot interleaving is excluded as its cause.
- Software cannot distinguish bus-level contention, converter turnaround, or
  UART framing from byte values alone. The next decisive evidence is a logic-
  analyzer capture at flight-controller TX, RX, and the one-wire servo bus,
  triggered on an HX-65HM request and including its response.
- The exclusive-boot target still failed left ID 1 Ping with the bounded first
  parser error `ff ff 02 ff`. A diagnostic capture was therefore added before
  making any further protocol or timing assumptions.
- Capture starts after UART configuration/flush but before the driver's first
  read. It records exact HX-65HM boot TX and all UART RX chunks, relative
  microsecond timestamps, parser terminal/error results, and timeout events.
  It freezes on verification success, terminal boot failure, or 512 captured
  data bytes. `hx8_uart_servo trace` is the only output path, so capture does not
  perform synchronous logging in the receive scheduler.
- The trace is a one-shot boot diagnostic and adds approximately 1.2 KiB to the
  dynamically allocated driver instance. It does not change parameters or uORB
  messages.
- Existing focused Hx tests passed 7/7 and `make zeroone_x6_hybrid` passed.
  FLASH is 1,889,392 bytes (96.10%). The 1,773,152-byte artifact SHA-256 is
  `78fcc3313ea386adff6db45aca16c7846cf65e05b71a26421f4b96cdbd75dbb0`.
- Boot byte-level diagnosis is complete; electrical waveform diagnosis and a
  steady-state monitor-failure capture remain pending.

## 2026-09-04 HX-65HM-exclusive mixed-bus startup

- After repeated-header recovery, the target captured `result=2 expected_id=1
  kind=1 bytes=ffff02ff`. This is not an ID-1 Ping response; parsed literally it
  starts with ID 2 and an impossible length 255. A valid ID-1 Ping response is
  `ff ff 01 02 00 fc`, while a clean ID-2 Ping response would be
  `ff ff 02 02 00 fb`.
- Scheduler inspection found that HX8 boot-configuration reads were allowed
  between HX-65HM boot Ping attempts. The HX-65HM discovery/configuration boot
  phase is now exclusive: it completes or exhausts its attempts before the
  driver sends any HX8 request. Steady-state mixed-protocol arbitration remains
  unchanged.
- Focused Hx tests passed 7/7 and `make zeroone_x6_hybrid` passed. FLASH is
  1,888,168 bytes (96.04%). The 1,772,244-byte artifact SHA-256 is
  `4cc986fa6cad70c09b8e0eb8af5eaef360e558397732d9db567e8738cbad6fcf`.
- Hardware verification remains pending. If the first H65 boot capture remains
  corrupt in this build, HX8 interleaving cannot be its cause because no HX8
  bytes have yet been transmitted; inspect the flight-controller UART waveform,
  SN74LVC125 direction timing, and wiring with a logic analyzer.

## 2026-09-04 HX-65HM repeated-header resynchronization

- The bounded target capture reported `result=2 expected_id=1 kind=1
  bytes=ffffffff`: the left Ping RX stream begins with at least four 0xff
  bytes, and the parser rejected the fourth byte as an impossible length 255.
- The old parser consumed the first two bytes as the header and the next two as
  ID/length. If the valid tail `01 02 00 fc` followed, it could no longer be
  recognized because its two header bytes had already been consumed.
- HX-65HM response IDs are limited to 0..253; 0xff can only be another header
  byte. The parser now holds the last two bytes of any 0xff run as its header
  and waits for the first non-0xff ID.
- Added the captured-equivalent vector `ff ff ff ff 01 02 00 fc` as a unit
  regression. The bounded snapshot also records the first ordinary timeout as
  diagnostic result 5, so a stream containing only repeated headers remains
  observable after parser recovery. All focused Hx tests passed 7/7 and
  `make zeroone_x6_hybrid` passed. FLASH is 1,888,136 bytes (96.04%). The
  1,772,224-byte artifact SHA-256 is
  `0faf05a738d83d980e72f28064de2d08b653f92f7a43b6eded0003b798c2fdea`.
- The extra preamble's electrical origin is not yet proven; parser acceptance
  is nevertheless protocol-safe because 0xff is not a valid response ID.

## 2026-09-04 bounded HX-65HM raw RX diagnosis

- The RX-resynchronization target still failed at the left-ID-1 boot Ping.
  Exactly three protocol errors accompanied three timed-out attempts; right
  remained unqueried. This strongly indicates one repeatable invalid RX byte
  sequence per Ping rather than general bus silence.
- Added a first-error-only diagnostic snapshot containing at most 16 RX bytes
  in wire order, the parser result, expected ID, and outstanding request kind.
  It is exposed only by `hx8_uart_servo status`, avoiding log spam and uORB ABI
  changes.
- Parser-result values are 2=bad length, 3=bad checksum, and 4=wrong ID;
  request kind 1 is Ping. A valid ID-1 Ping response should be
  `ff ff 01 02 00 fc`.
- All focused Hx tests passed 7/7 and `make zeroone_x6_hybrid` passed. FLASH is
  1,888,024 bytes (96.03%). The 1,772,100-byte artifact SHA-256 is
  `0ea834a7f248aa6d430543a718c3c3b4cf692c9091609d6b278273a6dae5de7e`.

## 2026-09-04 HX-65HM mixed-protocol RX resynchronization

- Target boot failed exclusively at the left-ID-1 ping: left timeout=3,
  retry=2, right timeout=0, both valid-response timestamps zero, and 49
  protocol errors. HX8 remained healthy on the same physical bus.
- The owner confirmed that all three servos and the SN74LVC125-based converter
  are unchanged and stable with either vendor PC application. The key
  difference is that each PC application generates only one wire protocol,
  while PX4 interleaves both.
- Code audit found that any bad-length, bad-checksum, or wrong-ID H65 frame
  aborted the outstanding transaction and disabled H65 parsing for the rest of
  that receive batch. A valid expected response following stale/unrelated
  bytes was therefore discarded, causing repeated ping churn and eventual
  timeout failure.
- RX protocol errors now remain non-terminal: the parser resynchronizes and
  continues waiting for the expected response until the existing 30 ms
  timeout/retry policy expires. Actual transmit failures still abort.
- All focused Hx tests passed 7/7, including a regression that a protocol error
  cannot discard an outstanding Ping. `make zeroone_x6_hybrid` passed; FLASH
  is 1,887,504 bytes (96.00%). The 1,771,724-byte artifact SHA-256 is
  `17303dfadf6f34878f8e79fbe6763e02fc216866ee272f6538ce1ac1525aae29`.
- Hardware validation remains pending; persistent protocol errors after this
  revision require a bounded raw-byte capture rather than further timing
  changes.

## 2026-09-04 per-side HX-65HM timeout diagnosis

- After adding 5 ms shared-bus spacing, a new 60-second target sample remained
  Ready but timeout/retry counters rose from 25 to 98: 73 recovered misses, or
  1.22/s. This is about 32% lower than the previous 1.8/s, so cross-protocol
  spacing helped but did not eliminate the underlying issue.
- Protocol errors stayed at 2, confirming the new failures were absent replies
  rather than malformed frames. Since the vehicle was stationary after boot,
  the interval's new misses were HX-65HM monitor reads.
- The supplied HX-65HM V3.7 register workbook has no address-7 configuration;
  address 6 is baud and address 8 is response level. There is no documented
  persistent response-delay setting to tune in ServoStudio.
- Added per-servo timeout/retry counters internally and thread-safe atomic CLI
  snapshots. `hx8_uart_servo status` now prints left and right counts without
  changing the uORB message ABI.
- All focused Hx tests passed 7/7 and `make zeroone_x6_hybrid` passed. FLASH is
  1,887,496 bytes (96.00%). The 1,771,632-byte artifact SHA-256 is
  `5fad73521c346d7996ef75fd641a81f474b36acc7a6a8ff8aa654b9b6d390dba`.

## 2026-09-04 persistent recovered HX-65HM first-packet loss

- A 60-second target sample remained Ready and fault-free, but HX-65HM timeout
  and retry counters both rose from 231 to 339: 108 recovered first-attempt
  misses, or 1.8/s. Protocol errors stayed at 47, so the new data indicates no
  response rather than malformed responses.
- Inspection found only per-controller request spacing. After receiving one
  protocol's response, the driver could send the other protocol's request in
  the same 5 ms scheduler cycle, leaving no shared-bus turnaround interval.
- Added a 5 ms bus-wide quiet interval after every valid response and after
  response-less HX-65HM broadcasts. This addresses the cross-protocol timing
  defect without lengthening the 30 ms response timeout or suppressing error
  counters.
- All focused Hx tests passed 7/7 and `make zeroone_x6_hybrid` passed. FLASH is
  1,887,264 bytes (95.99%). The 1,771,480-byte artifact SHA-256 is
  `50806038a2c59ed076186c088c31214c7746e2de60834433dfeffc551447c02c`.
- The timing hypothesis still requires target verification. If timeouts keep
  growing after this firmware, logic-analyzer capture and per-side/request
  diagnostics are required before changing timeout thresholds.

## 2026-09-04 HX-65HM retryable-timeout fault latch

- Hardware evidence showed all three servos currently online, healthy, and
  verified, with HX8 at -399.7 deg and HX-65HM at 0/2050 steps. Nevertheless,
  transformation fault 8 and sequence fault 3 remained latched.
- HX-65HM counters were timeout=336 and retry=336 while both servos were still
  online. Each initial miss therefore recovered on its first retry; there was
  no exhausted retry sequence in the captured interval.
- The pair controller incorrectly set a servo offline on every first 30 ms
  timeout before scheduling its retry. The transformation state machine could
  observe that transient false value and immediately latch actuator
  communication fault 8.
- The controller now preserves the last verified online/healthy state during
  its two allowed retries and marks offline only when the third attempt also
  times out. The existing 500 ms status freshness gate remains unchanged.
- Hx65 focused tests passed 3/3, including first-timeout and exhausted-retry
  assertions. `make zeroone_x6_hybrid` passed; FLASH is 1,887,168 bytes
  (95.99%). The 1,771,464-byte artifact SHA-256 is
  `e49b7a5113354dca9bee565a8deb877760722a83ccf15855cf72fa7ee4652e46`.
- Hardware verification of the revised retry behavior and explicit disarmed
  fault clear remains pending.

## 2026-09-03 target Not Ready diagnosis and HX8 baud-check fix

- Target evidence showed HX8 online at -399.7 deg and correctly inside the
  configured down endpoint (-400 +/- 5 deg), but configuration verification
  still failed although the reported persistent protection values matched.
- Root cause was a leftover HX8 parameter-36 check hard-coded to baud code 5.
  The target correctly reports HX8 code 8 for 1 Mbps, so the check made every
  otherwise-correct 1 Mbps setup fail closed. Parameter 36 is now excluded
  from boot verification, matching the already-approved shared-bus design.
- `TESTFILTER=Hx8` passed 4/4, including an assertion that parameter 36 is not
  read. `make zeroone_x6_hybrid` passed; FLASH is 1,887,168 bytes (95.99%) and
  the 1,771,468-byte artifact SHA-256 is
  `b4c86dcc5a04387f6056ad8285f8538c298ef3ce71060177bc08b214c9a158a6`.
- Both HX-65HM status timestamps remained zero and the left-ID-1 boot ping
  exhausted its retries. This is loss of protocol response, not an endpoint,
  skew, or tolerance failure. Hardware revalidation remains required.

## 2026-09-03 shared HX bus baud parameter

- Replaced the former HX8-only, guard-only baud parameter and hard-coded UART
  rate with one `HX_BAUD` parameter in the `Hybrid Control` group.
- `HX_BAUD` defaults to 1 Mbps and exposes the standard PX4 serial-rate enum
  from 50 through 3,000,000 baud, excluding `Auto` because neither wire
  protocol implements baud discovery.
- The driver maps the selected parameter to termios when it opens the one UART.
  HX-65HM boot verification no longer compares its readable baud-code byte;
  absence of replies already detects a device operating at another rate.
- Hx8 regression tests passed 4/4 and Hx65 tests passed 3/3. The final target
  build completed without compiler warnings; FLASH usage is 1,887,176 bytes
  (95.99%). The 1,771,472-byte PX4 artifact has SHA-256
  `aa5855b510bcd335a86d75ab049c9cef06d94e4802539367705c03feed03b500`.
- Added `docs/hybrid/hx-shared-bus-commissioning.zh-CN.md` as the concise field
  procedure. It does not invent the seven mechanical endpoints or landing-gear
  power limit; those remain explicit measured-value placeholders.
- Documentation validation matched all 46 referenced PX4 parameters against
  generated metadata/source, confirmed balanced code fences and consistent
  Markdown table columns, and passed `git diff --check`. No firmware rebuild
  was required because this change only adds documentation and state records.

## 2026-09-02 mixed UART servo protocol audit

- Created `/home/crocodile/PX4-Autopilot-hx8-hx65hm` on
  `feat/hybrid-landing-gear-servos` from the verified remote commit
  `285a9d5716e6f2935545532350645044a52ad11b`.
- The existing HX8 implementation owns one UART and one FashionStar controller;
  it accepts only 115200 baud, one ID, single-turn advanced time-based command
  `0x0b`, single-turn angle read `0x0a`, monitor `0x16`, stop, ping, and selected
  configuration reads/writes. Its monitor response already decodes a signed
  32-bit multi-turn angle.
- FashionStar's public protocol defines multi-turn commands `0x0d`, `0x0e`, and
  `0x0f`, multi-turn read `0x10`, and reset-turn-count `0x11`; advanced
  time-based `0x0e` is the direct multi-turn counterpart needed by the landing
  gear.
- HX-65HM uses an incompatible `0xff 0xff` packet protocol. Local protocol v1.0
  states a 1 Mbps default, while the user manual also says 115200 and later tells
  the PC tool to use 1 Mbps. Register 0x06 confirms selectable rates including
  115200. Both units must be commissioned individually to one shared rate and
  unique IDs before they join the HX8 bus.
- HX-65HM position mode supports absolute targets approximately +/-7.5 turns,
  speed, one symmetric acceleration-rate parameter, synchronous/asynchronous
  multi-servo start, and position/speed/load/voltage/temperature/current/error
  feedback. It does not expose FashionStar's separate acceleration/deceleration
  times or time-based position trajectory semantics.
- The requested sequence needs separate mechanism, landing-gear, propulsion
  ownership, and readiness state. The existing five-state hybrid status cannot
  distinguish airborne Quad control during gear deployment from a propulsion-
  disabled Quad/Rover Not Ready phase.
- The untouched baseline `make zeroone_x6_hybrid` completed successfully. The
  artifact is 1,757,312 bytes with SHA-256
  `06a74494864148b250dc277f03f269012d7036ddafbd04344ab0f1791b72fcf7`.
  The ELF linker reported FLASH at 1,872,640 bytes of 1,920 KiB (95.25%).

## 2026-08-25 multicopter idle-output investigation

- The supplied quad sample showed `vehicle_control_mode.flag_armed=True`,
  `actuator_armed.armed=True`, but `vehicle_attitude_setpoint.thrust_body` and
  `vehicle_rates_setpoint.thrust_body` at zero, with all four
  `actuator_motors_mc.control` values zero. This is upstream of the mixer.
- PX4 intentionally clamps MC manual thrust to zero until
  `COM_SPOOLUP_TIME` (default 1.0 s) has elapsed. The current branch adds a
  status-consumption-safe update of `_spooled_up`; the rebuilt artifact is not
  yet hardware-validated.
- Physical armed idle is a downstream DShot/PWM minimum-output concern. The
      final `actuator_motors` value of zero is remapped by `FunctionMotors` to the
      minimum motor command; therefore MC zero control alone does not prove that
      DShot idle is absent. Hardware evidence must include `actuator_outputs` and
      `dshot status`.
- The first spool-up patch was incomplete because it recalculated `_spooled_up`
  only when `vehicle_status` delivered a new sample. The fix now caches the
  latest status and recalculates from `armed_time` every MC control cycle. The
  corrected `zeroone_x6_hybrid.px4` build completed successfully on 2026-08-25
  with SHA-256 `c97433197a51f651bc778b8447600c071fa4feb795991366814a1857fa3fc471`.

## 2026-08-20 branch and safety audit

- `origin/testv3_v1.16.1` stopped at `e1da1439dc` on 2026-06-04 06:45:45
  (+08:00). The remembered post-06/23 large change is `325a9d07ba`, committed
  on 2026-06-28 20:33:05 (+08:00), directly on that ancestor, but it is not an
  ancestor of the current testc1 branch.
- `325a9d07ba` changes 10 controller/manual-control files (107 insertions,
  45 deletions). Its Commander hunk conflicts semantically with current Hybrid
  Commander changes; whole-commit cherry-pick is prohibited.
- Candidate ports are the Rover exclusion from Commander auto-disarm landing/
  preflight hysteresis and Rover stick arm/disarm/kill gesture suppression.
  MC/Rover controller hunks require separate review due to later Hybrid,
  M2006, and HX8 arbitration changes.
- Current HX8 arming checks require fresh stable status, endpoint confirmation,
  online/healthy/config-verified actuator and zero protection flags. The
  five-second RC re-arm grace period can currently skip these preflight checks.
- Current HX8 TimedMove requires `armed || prearmed`, causing disarmed
  transformation rejection. Proposed policy is pending approval: allow only
  state-machine transition MOVE while disarmed, keep HOLD blocked in
  disarmed/lockdown/failsafe, never auto-disarm airborne Quad on feedback loss,
  and stop/latch Rover drive on runtime endpoint loss.

- Owner approved the policy. Implemented selective Rover auto-disarm and stick
  gesture suppression, disarmed state-machine MOVE authorization (command type
  1 only), and a Commander Hybrid-specific exception to the RC re-arm grace
  period so fresh HX8 checks always run.
- Stable Rover already had continuous HX8 endpoint/health checks; its existing
  state-machine fault path now remains the runtime safety response and causes
  final M2006 outputs to become NaN. Added the approved airborne-Quad exception:
  while armed and not landed, transient HX8 feedback loss preserves the current
  Quad state/output and blocks new transformations until feedback recovers.
- Focused host results: Hx8Controller 25/25, Hx8BackendPolicy 7/7,
  TransformationStateMachine 44/44, ManualControl 7/7. Full `make tests` was
  not usable because the host Gazebo protobuf headers were generated by an older
  protoc; direct focused targets were rebuilt and executed successfully.
- `make zeroone_x6_hybrid` passed. Firmware artifact SHA-256:
  `8e10797c3d2ebb2643d43d88e3257c2e9fe71fca739688553b41689c0259bf7b`.

- 2026-08-20 hardware report with HX8 endpoints `-90/90` showed
  `hybrid_vehicle_status.fault_reason=4` (`TransitionTimeout`) while HX8 was
  online and healthy. The cause was the exact-180-degree angle tie in the
  wrapped normalization: the endpoint span could become `-pi`, making a
  physical Rover angle near `+90` normalize to zero (Quad). The wrapped-angle
  helper now preserves the configured raw direction at the pi tie; 46
  TransformationStateMachine tests pass.
- M2006 no-response investigation: `M2K_SPD_P` alone cannot prove the output
  chain is active. The driver consumes final `actuator_motors.control[4/5]`;
  diagnosis must compare `actuator_motors_rover`, final `actuator_motors`,
  `rover_throttle_setpoint`, and `m2006_motor_status.target_rpm/current_command`
  while moving the Rover throttle. The rebuilt firmware includes the pi fix.
- Latest hardware sample confirms the upstream chain is active:
  `actuator_motors_rover.control[0/1]=0.20743`, final
  `actuator_motors.control[4/5]=0.20837`. M2006 still reports
  `target_rpm=0`, `current_command=0`, and `fault_flags=12`, which decodes to
  latched `DriveFaultCan | DriveFaultCommand`; this is the direct reason the
  driver suppresses motor current.
- Follow-up hardware sample: unarmed M2006 had `fault=0` and `hw errors=141`;
  after Rover arming it became `fault=0x0c` and `hw errors=203`. The error count
  stayed at 203 for the following 10 seconds, proving the fault was latched at
  the arming transition rather than continuously re-triggered during the wait.
  The DriveGate can only recover after disarming and observing both healthy
  feedback and no new CAN/command error for its 100 ms recovery qualification.
- Source audit confirms `m2006_can` reports `hw errors` from the STM32H7 CAN
  interface `getErrorCount()`: transmit items aborted when the FDCAN error
  logging counter (CEL) is nonzero, bus-off/error handling, TX deadline aborts,
  or RX FIFO overflow. `tx_error_count=0` only means the frame was accepted
  into the TX FIFO; it does not prove a successful on-bus transmission.
- Higher-probability software hypothesis: `M2006Can::Run()` is scheduled every
  2 ms on the shared `uavcan` work queue, while `sendCommand()` gives each TX
  frame exactly a 2 ms deadline. Any work-queue jitter can increment
  `getErrorCount()` through a TX deadline abort even with correct 60-ohm bus
  termination. A disarmed 10-second error-count delta will distinguish this
  timing path from errors that only occur with nonzero current commands.
- Hardware test result: in disarmed Rover with zero-current output, `hw errors`
  stayed at 9 across the baseline and a further 10 seconds, with `fault=0`.
  The 2 ms deadline does not spontaneously accumulate errors under this load;
  the next isolation step is arm-with-neutral versus nonzero throttle command.

## 2026-08-21 M2006 no-motion software audit

- With `M2K_CUR_LIM=5000` and `M2K_SPD_P=10`, a live, armed Rover sample had
  target RPM `155.48/155.48`, measured RPM `-8/0`, PX4 current command
  `1634/1554`, and C610 feedback current `1570/1579`; CAN errors and drive
  faults were both zero. Thus the normal PX4 control path is not suppressing
  the two motor outputs and the configured current limit is active.
- Source audit: `M2006CommandAdapter` deliberately maps final PX4 Motor 6 to
  C610 ID 1 (left, payload bytes 0--1) and Motor 5 to C610 ID 2 (right, bytes
  2--3). `C610Protocol` emits standard CAN ID `0x200`, eight bytes, big-endian
  signed current values. The H7 FDCAN driver writes that standard ID to bits
  `[28:18]` and copies payload bytes without reordering. CAN2 ownership is
  exclusive to M2006; no second `0x200` transmitter exists in the project.
- The focused existing binary test `unit-C610Protocol --gtest_brief=1` passed
  4/4 on 2026-08-21. This verifies the encoder/decoder contract but cannot
  prove the electrical bridge output. No software defect has been found in the
  command path so far.
- `current_command` is the PX4 speed-loop output stored in `_current_command`
  and passed directly to `sendCommand()` as the two signed payload values of
  CAN frame 0x200. `torque_current` is decoded separately from bytes 4--5 of
  each C610 feedback frame (0x201/0x202); it is not the outgoing command.
- The new report of `current_command` and `torque_current` both near 5000 with
  zero RPM, no CAN/fault growth, and no observable DC supply-current increase
  does not indicate a PX4 speed-loop suppression. It instead requires
  distinguishing a real C610 phase-current response from an unverified raw TX
  observation, C610 operating/input-mode or calibration state, and the actual
  C610 supply/phase path.
- Added non-invasive `m2006_can status` output for the latest transmitted C610
  frame: standard ID, DLC, eight payload bytes, and `_iface->send()` result.
  This is diagnostic only and does not alter command generation or timing.
- Hardware TX diagnostic result: successive payloads `00 77`, `01 d8`,
  `06 f7/06 cf`, `09 a3`, `0b eb`, and `02 c7` decode to approximately
  119, 472, 1783/1743, 2467, 3051, and 711 signed current units on both
  C610 slots. Every frame is standard ID `0x200`, DLC 8, result 1, with zero
  H7 CAN errors. This matches the speed-loop ramp and proves the expected
  current command reaches the FDCAN TX path. It does not yet prove the C610
  power stage applies phase current; the lack of supply-current increase and
  zero RPM now point beyond PX4 command generation.
- Full-throttle capture on August 21, 2026 reached `current=5000/5000` and
  exact TX payload `13 88 13 88 00 00 00 00` with zero CAN/H7 errors. Shortly
  afterward the driver sent a zero frame, both motors emitted two beeps, and
  status became `fault_flags=0x02`, `timeout_count=12`, `current_command=0/0`,
  while both feedback streams had recovered to online. In `M2006DriveGate`,
  `0x02` is specifically `DriveFaultRightFeedback`, not CAN or command fault;
  the zero output is the safety response to the latched feedback timeout.
- Owner reports `M2K_CUR_LIM=2000` remains normal for a long time, while the
  5000-current test reaches `13 88 13 88`, then beeps and faults. Oscilloscope
  comparison is now the next isolation step: capture CAN differential traffic,
  ACK behavior, C610 feedback frames, and C610 supply voltage around the first
  failure.
- The August 21 oscilloscope/CAN-decoder document compared armed neutral and
  full-stick operation without relying on the later 5000-current protection
  event. At full stick, the physical bus capture decoded standard ID `0x200`,
  DLC 8 and command slots around `0x0FA0`/`0x0FC8`, agreeing with PX4's
  `current=4000/4000` and `0F A0 0F A0 00 00 00 00` status. C610 feedback IDs
  `0x201` and `0x202` simultaneously reported near-4000 current fields while
  RPM remained zero. Supply input rose from 0.294 A to 0.740 A at 24.13 V, an
  additional 0.446 A or about 10.76 W. This rules out a suppressed PX4 command
  and shows additional power is consumed, but does not by itself prove correct
  three-phase commutation or useful shaft torque.
- The same wide-timescale scope captures contain decoder contradictions:
  apparent unacknowledged `0x200` frames despite zero FDCAN CEL/TEC errors and
  continuing C610 feedback, one `0x201` decoded with DLC 4, an unexpected
  extended RTR frame, and a nonzero reserved command byte that disagrees with
  the driver's exact TX snapshot. Treat those entries as acquisition/decoder
  artifacts until recaptured at about 1 V/div and 10 us/div; they are not
  evidence of a real PX4 frame-format defect.
- The next no-motion isolation is motor/C610 pairing and commutation, not the
  out-of-scope high-current shutdown: while disarmed, rotate each physical
  motor separately and verify that exactly its corresponding `0x201` or
  `0x202` encoder/RPM changes. Then, at a bounded 500--1000 command, inspect
  each C610's three-phase output with an isolated/differential measurement.
  Cross-paired phase and encoder cables are a leading common-mode explanation
  for two unloaded motors that draw power, report current, and only twitch.

## 2026-07-23

- Confirmed the deployed PMU uses DroneCAN on CAN1 and the C610 controllers
  must move to CAN2. The requested permanent topology is PMU/DroneCAN on CAN1
  and private M2006 CAN on CAN2.
- Board inspection confirmed CAN2 is FDCAN2 on PB12/PB13 and is exposed by the
  X6 hardware manifest. It is not a missing-pin or disabled-transceiver issue.
- A direct M2006 mask change to CAN2 is unsafe: the STM32H7 UAVCAN driver uses
  one global interface table for IRQ dispatch, `getIface()`, and `select()`.
  A CAN2-only helper would return a null or CAN1 interface and could dereference
  an uninitialized slot or let DroneCAN access CAN2.
- The approved isolation design separates IRQ physical-interface registration
  from each driver's local logical interface list, makes ownership per physical
  CAN bus, limits DroneCAN to CAN1, and moves M2006 to CAN2 without restoring
  runtime FDCAN filter reconfiguration.
- The written implementation plan places pure H7 mapping and ownership tests in
  the existing host-testable `hybrid_control` test subtree. This avoids relying
  on the default SITL configuration of `src/drivers/uavcan`, which does not
  register the current ownership test target.
- The first baseline command used `TESTFILTER=HybridCheck` and exited 0 with
  `No tests were found!!!`; the actual functional target is lowercase
  `functional-hybridCheck`. The plan and subsequent commands use
  `TESTFILTER=hybridCheck` so an empty CTest selection cannot be accepted.
- CAN2 isolation Task 1 completed in `fc03b76b2e`: the pure interface-map
  policy maps CAN2-only to logical interface 0, preserves physical order for
  two buses, and rejects empty/unsupported masks. Its RED failed on the
  missing header, GREEN passed `unit-CanInterfaceMap` 1/1, and independent
  task review approved both specification compliance and code quality.
- CAN2 isolation Task 2 completed in `84ffb5977e` plus review fix
  `8b3f1fd5b7`: ownership is packed per physical bus with atomic all-or-nothing
  claim/release. Review found and the fix covered invalid enum values that
  could otherwise spill into a neighboring two-bit field. Focused
  `unit-CanOwnership` passed 1/1 and re-review was clean; production caller
  migration remains explicitly assigned to Task 4.

## 2026-07-21

- Created `debug/testc1-v1.16.1` from `origin/testc1_v1.16.1` at `d86bdd6a4a957704a7c4218a628d6007e2a4e1f9`.
- The shared checkout was left untouched because it contains unrelated local work.
- `make zeroone_x6_hybrid` completed with exit code 0. The final Ninja step created
  `build/zeroone_x6_hybrid/zeroone_x6_hybrid.px4` (1,750,383 bytes).
- Linker report: FLASH usage is 94.90% (1,865,888 B of 1,920 KB). This is not a
  build failure, but it leaves limited space for future firmware growth.
- Full build log: `/tmp/px4-debug-testc1-v1.16.1-build.log`.

## 2026-07-22

- The first Commander CMake repair unconditionally referenced
  `unit-CommanderHybridStatus`. This target is absent when `BUILD_TESTING=OFF`,
  so the dependency must be guarded by `if(BUILD_TESTING)` and verified through
  both an empty SITL test build and the normal `zeroone_x6_hybrid` configuration.
- The guarded dependency fix (`3e3b1536a6`) passed the empty SITL test build
  (`M2006TxPolicy` 1/1) and the serial `zeroone_x6_hybrid` build. Independent
  task review found no Critical, Important, or Minor issue.
- CAN recovery Task 1 (`c654d0ab34`) is now accepted: its policy test passed in
  the repaired clean test build and a fresh review found no issue.
- Task 2's prescribed `make tests TESTFILTER=BusOffCleanup` was reproduced with
  only its test registration and test source present but no production header.
  It exited 0 with `No tests were found!!!`: the default `px4_sitl_test` board
  does not configure `src/drivers/uavcan`, so this is an invalid RED/GREEN
  harness rather than a passing test. The implementation is paused before the
  production header until the test-registration scope is corrected.
- The user approved relocating only the pure Task 2 test to
  `src/lib/hybrid_control`; `767accd974` then established a valid RED (missing
  header, exit 2) and GREEN (CTest 1/1 and gtest cases 2/2) without enabling
  UAVCAN in SITL or changing board configuration. Independent review was clean.
- Task 3 (`a96077b311`) integrates the tested cleanup policy into the STM32H7
  bus-off ISR. Focused policy tests and `zeroone_x6_hybrid` build passed, and
  independent review found no issue; FDCAN cancellation/recovery timing remains
  explicitly pending target-hardware validation.
- Task 4 (`b7db7119ca`) adds nonblocking driver maintenance and gates commands
  on fresh feedback. The M2006 focused suite (4/4), hybrid firmware build, and
  independent review passed; target hardware validation remains pending.
- Task 5's original combined `TESTFILTER='M2006*|BusOffCleanup'` command is
  invalid for the PX4 Make recipe: its unquoted expansion lets the shell parse
  `|` as a pipeline, so make exited 1 before a test build. Final verification
  must use separate M2006 and BusOffCleanup filter commands.
- Final automated verification used separate filter commands from a fresh SITL
  cache: M2006 tests passed 4/4 and BusOffCleanup passed 1/1. The final hybrid
  build passed and produced
  `zeroone_x6_hybrid_usb_bus_off_fix.px4` with SHA-256
  `78395c84af831e071f5337a8cda629c1048191b2b461133809f9dfd39d3667ec`.
  Hardware acceptance is still pending and is required before declaring USB/CAN
  recovery fixed.
- Hardware isolation then established: no M2006 startup is USB-normal; Stage C
  through `_can.init()` is USB-normal; Stage D invoking `configureFilters()` is
  USB-abnormal; skipping filters is USB-normal. The accepted filter-state fix
  uses CPU message-RAM addresses, corrects classic filter encoding/list counts,
  and moves bounded INIT/CCE waits outside the global IRQ-critical section.
  Diagnostic artifact `zeroone_x6_hybrid_usb_diag_filter_state_fix.px4` was
  flashed and USB enumerated normally.
- Final post-fix automated verification: M2006 unit tests passed 4/4,
  BusOffCleanup passed 1/1, and `make zeroone_x6_hybrid` succeeded. Final USB
  acceptance candidate is `zeroone_x6_hybrid_usb_filter_fix.px4`, SHA-256
  `7c9f8cbff6388aedfcf85a0cd8458d3b99a872e307467474c6ffe6dd5a6824cc`.
- The bounded generic `configureFilters()` path returned `-1000`
  (`ErrCCCrINITNotSet`) on hardware because this FDCAN instance rejects a second
  INIT transition after initial setup. M2006 now uses the driver's initial
  accept-to-FIFO0 policy with software C610 ID filtering instead. With both C610
  nodes powered it started successfully: `rx=28882/28298`, `tx=14369`, and zero
  TX/full/hardware errors.
- Hardware retest with both C610 nodes unpowered found the candidate fix still
  has the original bootloader-only USB symptom. Follow-up diagnostic artifacts
  disabling `select()`, normal Run TX, both together, and both together with the
  original bus-off ISR were all USB-abnormal. This contradicts the earlier Stage F
  result (no Run TX, USB-normal), so FDCAN Run-time paths are not yet a defensible
  root cause for the current hardware test; firmware identity and flash workflow
  must be re-established before another source change.
- Task 3 (`a96077b311`) integrates the tested cleanup policy into the STM32H7
  bus-off ISR. Focused policy tests and `zeroone_x6_hybrid` build passed, and
  independent review found no issue; FDCAN cancellation/recovery timing remains
  explicitly pending target-hardware validation.

- Hardware reproduction: the ZeroOne X6 bootloader enumerates as VID `3643`, PID
  `15E0`, then disconnects after about two seconds. The main firmware does not
  enumerate, with either USB-only or external power.
- The main firmware otherwise appears alive: board LEDs and PMU communication are
  normal. This narrows the first investigation to USB CDC configuration/startup
  rather than assuming a whole-firmware boot failure.
- The artifact in the shared checkout predates its current HEAD, so its embedded
  build revision must be identified before using that checkout branch as a baseline.
- The known-good artifact embeds Git revision
  `e1da1439dcec9c0f40c51d3e525ed9647bcdd90e`; the failing artifact embeds
  `d86bdd6a4a957704a7c4218a628d6007e2a4e1f9` (64 commits later).
- The complete generated NuttX `.config` files are identical. USB CDC, VID/PID,
  endpoints, buffers, console, and autostart options are therefore present in both
  binaries. Static `.data` and `.bss` sizes are also effectively identical.
- `rcS` sources `rc.vehicle_setup` and `rc.hybrid_apps` before starting
  `cdcacm_autostart`. The failing revision newly starts `m2006_can` at the beginning
  of `rc.hybrid_apps`, activating STM32H7 FDCAN and UAVCAN timer initialization in
  this pre-USB window. This is the first hardware A/B hypothesis, not yet a confirmed
  root cause.
- Built `build/zeroone_x6_hybrid/zeroone_x6_hybrid_usb_diag_no_m2006.px4`, which
  differs only by skipping the `m2006_can start` block. SHA-256:
  `9bf2470d2b3faf17eb7a61fb46ae196a5fc0adceeec9b533d292f34dc27c9ddf`.
- Restored the source and rebuilt the unmodified branch artifact. SHA-256:
  `6216dced3e9ea3abad2c014dd38415db03f759c072d3185d6143628971c892b9`.
- Hardware A/B result: the no-M2006 diagnostic firmware restores main-firmware
  USB enumeration. The failure is therefore triggered inside the `m2006_can start`
  path, not by the compiled USB configuration.
- Compile flags select `UAVCAN_STM32H7_TIMER_NUMBER=2`. X6 uses TIM8 for HRT and
  TIM14 for the tone alarm; TIM2 is listed in the board I/O timer table but has no
  configured output channel. A timer conflict is not yet proven.
- Built Stage A, which constructs and validates `M2006Can` but returns immediately
  before `UAVCAN_DRIVER::SystemClock::instance()`: SHA-256
  `f55681c343f94e66be300f432baff9908df81c57049bfbb90ed3da0ccb518252`.
- Built Stage B, which additionally initializes UAVCAN `SystemClock` and then
  returns before `_can.init(1000000)`: SHA-256
  `186d16d8cf04a4bb26851c01b97d19c57982945a64e6b2b242b49cb19878b157`.
- Restored source after both diagnostic builds; no M2006 source diff remains.
- Hardware results: Stage A and Stage B both restore USB enumeration. Object
  construction, parameter validation, CAN ownership, and UAVCAN `SystemClock`/TIM2
  initialization are excluded. The next boundary is `_can.init(1000000)`.
- Built Stage C, which executes `_can.init(1000000)` and returns before interface
  access, filter configuration, or work-queue scheduling. SHA-256:
  `d769180350ca3418c6dfd3167e5a4a345f27a38b8de73e645bce6d484082fd3e`.
- Restored source and rebuilt the unmodified branch artifact after Stage C.
- Hardware result: Stage C restores USB enumeration. Low-level FDCAN initialization
  is excluded; the next hardware boundary is `CanIface::configureFilters()`.
- Static inspection found that `configureFilters()` computes filter write pointers
  from `(can_->SIDFC | FDCAN_SIDFC_FLSSA_Msk)` and the analogous XID expression,
  then dereferences them as CPU addresses. These registers contain relative message
  RAM offsets, and OR with the mask cannot yield an address under `SRAMCAN_BASE`.
  This is the current single root-cause hypothesis pending Stage D hardware proof.
- Built Stage D, which executes interface lookup and two standard-ID filter writes,
  then clears `_iface` and returns before controller setup, status reads, scheduling,
  receive, or transmit. SHA-256:
  `d84a885a457c289f895b06bc259aaa4d57c63c6da7a39d0cbf2117e58295d2d9`.
- Restored source and rebuilt the unmodified branch artifact after Stage D.
- Hardware result: Stage D restores USB enumeration. Interface lookup and filter
  configuration are excluded from this USB failure. The suspicious filter pointer
  arithmetic remains a separate code-review issue, not the trigger proven here.
- Built Stage E with the complete production `init()` path and 500 Hz scheduling,
  but an empty `Run()` body after exit handling. SHA-256:
  `695e885764608bdec485d29e939d19b3e0ac26fa94a4630883b1a24aebf55c69`.
- Restored source and rebuilt the unmodified branch artifact after Stage E.
- Hardware result: Stage E restores USB enumeration. The complete initialization
  path and 500 Hz scheduler are excluded; the trigger is inside production `Run()`.
- Because the original failure also occurs on USB-only power with no powered CAN
  peers, unconditional 500 Hz `sendCommand()` is the next single hypothesis. It can
  drive the controller into repeated no-ACK/bus-off handling before USB startup.
- Built Stage F with the complete production `Run()` except for the single
  `sendCommand()` call. SHA-256:
  `5388f74889a284708ae83b5b4ef6a8c82a0a1c923192cd82aba92ed0f3f9b96d`.
- Restored source and rebuilt the unmodified branch artifact after Stage F.
- Hardware result: Stage F restores USB enumeration. Receive processing, command
  calculation, status publication, and the complete 500 Hz `Run()` schedule are
  excluded; executing `sendCommand()` is required to reproduce the USB failure.
- Stage G will permit exactly one transmit attempt. If USB remains normal, the
  remaining hypothesis is sustained no-ACK retransmission/bus-off handling rather
  than the first FDCAN TX operation itself.
- Built Stage G with the complete production `Run()` and exactly one permitted
  `sendCommand()` attempt. SHA-256:
  `00dc67405bf04f05a6139a13ba22fac4d65ac855c0d15976819ef1101b459894`.
- Restored the source and rebuilt the unmodified branch artifact after Stage G.
- Hardware result: Stage G fails USB enumeration. A single `sendCommand()` attempt
  is sufficient; sustained 500 Hz transmission, repeated no-ACK retries, and
  repeated bus-off recovery are not required to trigger the observed startup fault.
- Source tracing shows the first low-level send validates the frame, enters a
  critical section, reads `TXFQS`, writes four words to FDCAN message RAM, writes
  `TXBAR`, and then records the pending item. Stage H will execute exactly one send
  through the message-RAM writes but return immediately before `TXBAR`; this tests
  message construction/RAM access separately from activating the controller TX.
- Built Stage H and saved it as
  `zeroone_x6_hybrid_usb_diag_stage_h_pre_txbar.px4`. SHA-256:
  `a01edf586c009bbe754c64729a8f75d53d58a14d7f37f56bab2b0ad780707f45`.
- Restored both temporary source changes and rebuilt the unmodified branch artifact.

- Register snapshot implementation: STM32H7 `CanIface` now latches the first
  CEL-detected CAN error or Bus-Off with PSR/ECR/IR/TXFQS/TXBRP/TXBTO/TXBCF/CCCR
  and monotonic timestamp. `m2006_can status` prints decoded LEC/DLEC/TEC/REC/CEL
  plus raw queue/interrupt registers. The snapshot is read under a critical
  section and does not alter existing error counters or safety gates.
- Register-snapshot build on 2026-08-21 succeeded with `make zeroone_x6_hybrid`;
  artifact SHA-256 is
  `fc4843161adcbf8f4e71b84405391150b7ca776ef53688702c392cfa3473132b`.
- Target test of the first snapshot build showed aggregate H7 errors increasing
  from 9 to 79 at neutral Rover arm, while no CEL/Bus-Off snapshot was present.
  Therefore the increment is not yet evidence of a CAN protocol error or
  Bus-Off; the previous diagnostic omitted TX timeout, hardware RX FIFO loss,
  and software RX queue overflow source reporting.
- Diagnostic v2 splits all available H7 error sources in `m2006_can status`,
  captures TX timeout and RX FIFO loss snapshots, and explicitly reports a
  missing snapshot. Build succeeded on 2026-08-21; artifact SHA-256 is
  `429ce33278bf83c7f4cb598c5525718e3e3dc0c7e26f32f997061c862b4e1413`.
- Hardware result for diagnostic v2: baseline `H7 err internal=0 rx_overflow=9
  cel=0 busoff=0 tx_timeout=0 rx_fifo_lost=0`, then neutral Rover arm produced
  `rx_overflow=73` and `fault_flags=0x04`; all hardware-protocol counters stayed
  zero and both feedback streams stayed online. This identifies the current
  arm-edge fault as software RX queue overflow, not a CAN physical-layer fault.
- Increased the H7/M2006 software RX queue from 16 to 128 entries to absorb the
  bounded logger-start scheduling burst. Build succeeded on 2026-08-21; artifact
  SHA-256 is `28520203468bf0fe64faf96da09f4efac5386997dc6748ae18361615e0691bbe`.

- Added a one-shot M2006 gate warning for each newly latched fault bit. It records
  the per-cycle command freshness/configuration state and CAN error counter delta;
  this is intended to distinguish an arm-time command race from a CAN error without
  flooding the 2 ms work-queue console.
- The target retained `fault=0x0c` after disarm while CAN errors remained stable.
  Code review showed recovery incorrectly required fresh finite wheel commands,
  although disarmed/non-driving hybrid output may legitimately be NaN. Recovery
  now requires only both feedback streams healthy and no new CAN error; armed
  Rover driving still requires fresh finite commands. The focused gate suite passes
  9/9 and `make zeroone_x6_hybrid` succeeds.
- Hardware validation confirms the recovery fix: disarm clears `fault_flags` while
  the cumulative `hw errors` counter remains unchanged, as designed.
- Repeated neutral Rover-arm trials show a deterministic ordering: first
  `DriveFaultCommand` with `cmd_fresh=1` but `cmd_finite=0`, then `DriveFaultCan`
  after a finite sample arrives. RoverDifferential only publishes its wheel output
  after it observes armed state, so this is an expected arm-edge publication race.
  M2006 now waits for the first fresh finite armed Rover command before qualifying
  later command loss as a fault. Focused M2006DriveGate tests pass 10/10 and the
  firmware build succeeds.
- After flashing that build, neutral Rover arm no longer produces `0x08`; it
  produces only `fault=0x04`. `current_command=0`, `online=[True, True]`, and
  feedback continues, so the command path is no longer the trigger. The CAN error
  counter rises from 8 to 77 at arm, isolating the remaining issue to FDCAN/CAN
  physical-layer or controller error handling.
- The CAN-only conclusion does not yet distinguish physical bus errors from a
  software interpretation of STM32H7 FDCAN error logging. `getErrorCount()` is an
  aggregate counter that includes ISR error events and TX abort/deadline paths;
  `tx_error_count=0` only means the application enqueue returned successfully.
  Hardware marginality remains possible, but a low-level ECR/PSR/IR snapshot at
  the first arm-time increment is required before assigning blame to wiring.
- 2026-08-19: Additional UART frames contained one accepted TimedMove response
  (`05 1c 0b 02 00 01 2f`) surrounded by status frames showing the servo still
  near 89.7 degrees and moving. This cannot be attributed to the Rover target
  without the corresponding command TX or sequence correlation; it may be the
  stable Quad HOLD. Stop/RELEASE response `05 1c 18 02 00 01 3c` was accepted.
  Added bounded local command diagnostics instead of weakening the rejection gate.
- 2026-08-19: The first local diagnostic logged before `Controller::setTarget`,
  so its word "accept" overstated the evidence. Corrected it to report whether
  the controller actually queued or rejected the command, and added the exact
  expected/status sequence plus accepted/result values at the fault-11 gate.
- 2026-08-17: QGC showed raw RC Channel 7 movement but transformation did not
  start. Code inspection proved the hybrid module incorrectly mapped raw
  channel values 5--10 onto `manual_control_setpoint.aux1--aux6`; these AUX
  fields have their own `RC_MAP_AUXn` mapping and are not raw channels. The
  corrected input is `manual_control_switches.transition_switch`, which already
  applies `RC_MAP_TRANS_SW` and `RC_TRANS_TH` in `rc_update`.
- 2026-08-17: The HX8 command policy emitted stable HOLD every 200 ms while the
  driver encodes HOLD as the same vendor timed-move command used by MOVE. With
  a 1000 ms move time this can overlap an active command and produce the target
  event `HX8 actuator command rejected`. HOLD is now edge-triggered by motion
  enable or stable endpoint change. All focused host tests and the hybrid
  firmware build passed; target hardware validation remains pending.
- 2026-08-18: After clearing the latched fault, a stable Quad HOLD (`type=3`,
  sequence 4) produced `command_result=2` while `hx8_servo_status.status_flags`
  was `1` (command executing), `protection_flags=0`, and telemetry remained
  online/healthy/config-verified. The hybrid state then latched fault 11 and
  sent RELEASE (`type=2`). This proves the previous parser's nonzero-is-error
  assumption was incompatible with the vendor executing response.
- 2026-08-18: Updated HX8 command response parsing so response code 1 means
  accepted/executing and codes 2+ remain errors. HX8 focused tests passed 42/42
  and `make zeroone_x6_hybrid` passed. Hardware flash and post-reboot
  transition/arming verification remain pending.
- 2026-08-18: Hardware on the updated artifact showed fault-free boot and
  stable Quad status. A Rover request still latched fault 11; the latest
  RELEASE was accepted (`command_result=1`), proving the response parser fix
  was active. Quad 90 degrees and Rover 180 degrees otherwise share ID 0,
  1000/100/100 ms timing, and 20000 mW power. Exact +180 degrees is therefore
  the leading remaining rejection boundary and will be isolated at 179 degrees.
- Initial inspection incorrectly assumed configuration register 41 stored the
  same raw thermistor ADC used by live telemetry. Manufacturer configuration
  documentation later proved register 41 stores degrees C; configuration and
  telemetry units are distinct.
- User supplied stall power 48000 mW, current limit 5000 mA, adequate BEC, and
  65 C temperature. A 48 W stall threshold above the selected 36 W persistent/run limit is
  potentially unreachable and must be resolved before commissioning write.
- User replaced the provisional custom protection choices with “all
  manufacturer defaults.” Repository defaults of zero for SPWR/TEMP/PWR/CUR
  are deliberate uncommissioned sentinels, not vendor defaults; numeric
  factory values still require an external read before PX4 can verify them.
- Manufacturer documentation identifies configuration register 41 as degrees
  C with default 70, while current PX4 code calls it `temperature_adc` and
  compares/writes the raw value. Live status temperature still arrives as a
  thermistor ADC and legitimately uses `adcToTemperature()`; configuration
  and telemetry units must be separated.
- Factory response=0 and stall-protection=0 are incompatible with required
  closed-loop verification and autonomous stall release. Factory VMIN=4000 mV
  is outside the intended 9.0--12.6 V safety envelope. Factory persistent
  power=20000 mW also makes a 36000 mW runtime command limit incoherent.
- Safe profile was confirmed with runtime/internal power both 20000 mW.
  Production now rejects response or stall-release disabled, voltage outside
  9000--12600 mV, and motion command power above the persistent power limit.
- A pre-existing zero-sentinel unit test was found ineffective because it
  checked `config_verified` before boot readback. It now completes all 11
  parameter reads before asserting fail-closed behavior.
- Final host evidence: `TESTFILTER=Hx8` passed 4/4,
  `TESTFILTER=TransformationStateMachine` passed 1/1, firmware build passed,
  and generated metadata contains `HX8_CFG_TEMP` with no `HX8_CFG_TADC`.
- Original ZeroOne-Aero PX4 `main` and `pr-ZeroOneX6_V2-260526` both retain
  `GPS1=ttyS0`, `GPS2=ttyS7`, `TEL1=ttyS6`, `TEL2=ttyS4`, `TEL3=ttyS1`, and
  `EXT2=ttyS3`. NuttX assigns the USART3 Console to ttyS0 before registering
  remaining devices, so MCU UART number and `ttyS` number must not be equated.
- Target diagnostic showed `tx=3`, `tx_errors=3`, and `last_tx_error=-9`.
  Hx8UartServo opened the device in `init()` from the NSH task but used it in a
  ScheduledWorkItem serial queue. On NuttX the latter has a different FD list;
  other serial work-queue drivers open their port from `Run()`. The driver now
  opens/configures the UART on first `Run()` before issuing protocol requests.
- First successful hardware capture showed Ping response
  `05 1c 01 01 00 23`, followed by three identical ParamRead responses
  `05 1c 03 03 00 21 01 49`. The latter encodes payload `[parameter=33,
  value=1]`; prior code expected only the value and rejected every valid frame.
- Complete boot capture reconstructed register 41 as
  `05 1c 03 04 00 29 e5 02 38`: raw value `0x02e5=741`, corresponding to 70 C.
  Other captured protection registers matched their PX4 expectations. The
  vendor UI displays Celsius but the protocol register retains thermistor ADC.
- DroneCAN-only target isolation result: after saving `M2K_EN=0` and
  `UAVCAN_ENABLE=0`, fully removing power, rebooting, then setting
  `UAVCAN_ENABLE=3`, every manual `uavcan start` printed the configured node ID
  and bitrate but returned shell status 1; `uavcan status` immediately reported
  `application not running`. This excludes M2006/CAN2-first ownership as a
  necessary cause and requires staged diagnostics around the CLI-to-worker
  startup boundary.
- Built the temporary staged DroneCAN startup diagnostic successfully. It logs
  entry/exit around ownership, helper construction, system-clock construction,
  node construction, scheduling, first worker run, CAN init, and node init.
  Artifact SHA-256 is
  `c6a500730d1cb9c2e1d538d25de999c55380e5b4bfd2031f26cba4080acf08fd`;
  production behavior has not been changed or committed.
- Target diagnostic output reached `diag system clock ready` but never reached
  `diag node created`; the existing helper was non-null and `_instance` was
  null. Source tracing confirmed that the multi-instance driver initializes
  `num_ifaces_` to zero, while libuavcan's `CanIOManager` reads the count during
  `UavcanNode` construction and calls `handleFatalError("Num ifaces")` for
  values below one. The user approved initializing the local logical-interface
  topology in the driver constructor without moving hardware initialization.
- Task 1 commits `7f1a479735` and `7014e3ce55` establish a tested
  hardware-independent `CanDriverView` that production uses directly for
  pre-init logical count and interface mapping. RED failed on the missing
  view; GREEN passed `unit-CanInterfaceMap` 1/1 and
  `make zeroone_x6_hybrid`. Independent review initially found that the thin
  topology test did not protect the real driver contract; the second commit
  removed the duplicate count/mapping state, added exhaustive invalid-mask
  coverage, and passed re-review with no findings.
- Task 2 removed all temporary `uavcan_main.cpp` diagnostic markers; the file
  matches HEAD and the tracked source/index are clean. Final focused tests
  passed: `unit-CanInterfaceMap` 1/1, `unit-CanOwnership` 1/1, and
  `functional-hybridCheck` 1/1. Persistent logs are
  `state/test_can_map_final.log`, `state/test_can_ownership_final.log`, and
  `state/test_hybrid_final.log`.
- Final `make zeroone_x6_hybrid` passed; the persistent build log is
  `state/build_dronecan_topology_final.log`. The copied production artifact is
  `zeroone_x6_hybrid_dronecan_topology_fix.px4`, size 1,752,452 bytes,
  SHA-256
  `5e34d479ce59f99f86b47e849fdfe3fc43323664ca5b0afd4a8e341f6c992dee`.
  Flash usage is 1,867,248 bytes (94.97%). Real-hardware PMU/DroneCAN startup,
  simultaneous M2006 traffic, and USB stability remain unverified.
- Target acceptance Stage 1 passed after flashing the final artifact and a full
  power cycle with `M2K_EN=0`, `UAVCAN_ENABLE=3`: USB/QGC remained stable, the
  PMU communication LED was normal, `uavcan status` returned zero, node 123 was
  `OK/OPERAT`, internal failures and transfer errors were zero, and
  `battery_status` was fresh with `connected=True`, 24.45313 V, and 0.09680 A.
  CAN1 reported 3,245 RX frames and 2,854 TX frames but also 31 hardware/IO
  errors; determine whether these are fixed power-up transients or continuing
  errors before enabling M2006.
- Follow-up CAN1 observation cleared the concern: over 10 seconds RX frames
  increased from 3,259 to 5,829 and TX frames from 2,881 to 5,125, while HW
  errors and IO errors remained 0. UAVCAN transfer errors remained fixed at 2,
  node 123 stayed `OK/OPERAT`, and internal failures stayed zero. The earlier
  31/31 display was not a continuing CAN1 fault.
- Target acceptance Stage 2 initial simultaneous result: with
  `UAVCAN_ENABLE=3` and `M2K_EN=1`, USB/QGC and the PMU LED remained normal.
  DroneCAN CAN1 had zero internal/transfer/HW/IO errors and node 123 remained
  `OK/OPERAT`; battery data was fresh and connected at 24.43750 V. M2006 ran on
  CAN2 at 1 Mbps with both IDs online, increasing traffic, zero TX full/error,
  zero timeouts, zero fault flags, and zero current commands. CAN2 reported
  eight accumulated hardware errors; confirm that counter remains static
  before final acceptance.
- CAN2 stability follow-up passed: over 10 seconds the per-motor RX counts rose
  from 333,345/335,142 to 363,712/365,735 and TX rose from 165,815 to 180,917,
  while hardware errors and `can_error_count` stayed fixed at 8. Both motors
  remained online with zero TX full/error, timeouts, and fault flags.
- Final disarmed recovery acceptance passed with both C610 nodes disconnected
  together. Before disconnect both were online with zero timeouts/faults.
  Disconnected state correctly reported `[False, False]`, two cumulative
  timeouts, zero fault flags, zero TX errors, and no latched driver fault.
  After reconnect both returned `[True, True]`, traffic resumed, consecutive
  errors returned/remained zero, TX errors stayed zero, and faults stayed zero.
  CAN2 hardware errors accumulated from 10 to 37 while absent and to 60 across
  reconnect, as expected for physical removal, while recovery completed.
  Throughout the test DroneCAN CAN1 retained node 123 as `OK/OPERAT`, internal
  and transfer errors stayed zero, the CAN1 31/31 historical HW/IO counters did
  not increase, PMU communication remained active, and the uninterrupted NSH
  session/USB connection remained stable.
- Completion verification reran `CanInterfaceMap`, `CanOwnership`, and
  `hybridCheck`; each selected one test and passed 1/1. The hybrid build
  returned zero and both the primary and named artifacts retained SHA-256
  `5e34d479ce59f99f86b47e849fdfe3fc43323664ca5b0afd4a8e341f6c992dee`.
  Final independent review of `501c5e8b57..7014e3ce55` found no Critical,
  Important, or Minor issues and assessed the change ready to merge.
- Published the verified branch to `origin/testc1_v1.16.1`: remote advanced
  fast-forward from `d86bdd6a4a957704a7c4218a628d6007e2a4e1f9` to
  `7014e3ce55c00d5a339a0e0db36b2b2fee31a89e`. The untracked `state/` records
  remain local and were intentionally not pushed.
- New post-acceptance symptom: QGC reports Not Ready and `battery_status` is
  never published. Parameters are `UAVCAN_ENABLE=3`, `UAVCAN_SUB_BAT=1`,
  `UAVCAN_BITRATE=1000000`, `UAVCAN_NODE_ID=1`, and `BAT1_SOURCE=-1`.
  DroneCAN is running and node 123 is `OK/OPERAT`; the battery bridge exists
  but has no channel, proving NodeStatus heartbeat reception without any
  BatteryInfo callback/publication. PMU POWER and CAN indicators are active,
  but SYS is off. Treat this as PMU partial-online/message-publication failure,
  not a fully disconnected CAN1 or disabled PX4 subscription.
- Four-way hardware comparison isolates two faults. PMU A fails to publish
  battery data with both the hybrid flight controller A and stock flight
  controller B, so PMU A itself/configuration/sampling is faulty independent
  of this firmware. PMU B publishes valid pack voltage/current/SoC with both
  controllers, proving controller A's CAN1 and DroneCAN reception work. Only
  controller A plus PMU B is rejected as abnormal/Not Ready. Controller A uses
  `UAVCAN_SUB_BAT=1`; PX4's Raw path explicitly forces `cell_count=1` and puts
  pack voltage in cell 0. The Filter path (`UAVCAN_SUB_BAT=2`) instead uses the
  Battery library and BAT1 calibration/cell-count parameters. Compare A/B
  parameters and topic fields before changing configuration.
- A/B parameter and topic comparison disproves Raw battery encoding as the PX4
  arming blocker: both controllers use `UAVCAN_SUB_BAT=1`, `BAT1_SOURCE=-1`,
  six configured cells, and publish essentially identical healthy Raw status
  (`cell_count=1`, pack voltage in cell 0, connected, warning/faults zero).
  Controller B passes `commander check`, while hybrid controller A fails.
  The remaining differential is controller A's custom HybridChecks, which can
  reject unsafe/stale transform status, invalid M8 safety mapping, unhealthy
  M2006 while driving, or an all-zero M2006 speed controller. Collect hybrid
  status and configuration rather than modifying battery parameters.
- Hybrid preflight root cause confirmed on controller A. `current_state=4`
  means `HYBRID_STATE_TRANSITION_FAULT` and `fault_reason=5` means
  `TRANSFORM_FAULT_INVALID_SERVO_CONFIG`. The active backend is still PWM
  (`HYB_ACT_TYPE=0`) even though a UART servo is physically connected.
  `HYB_SV_QUD=HYB_SV_ROV=0` violates the required endpoint separation;
  additionally M8 is unmapped (`FUNC8=0`), DIS8 is 1500 rather than zero,
  FAIL8 is -1 rather than zero, PWM sensors are enabled with neither AS5600 nor
  TMAG valid, and all M2006 speed gains are zero. These independently explain
  all-mode arming rejection; battery health is normal and unrelated.
- HX8 follow-up confirms the UART driver is not running and publishes no
  status. `HX8_SER_CFG=401` correctly selects EXT2, but the driver deliberately
  refuses init while `HYB_ACT_TYPE!=1`. Moreover both HX8 endpoint angles are
  zero and `HX8_PWR_LIM=0`; the hybrid HX8 policy requires distinct finite
  endpoints and positive run power. `hx8_uart_servo config check` misleadingly
  returned shell status zero only because the absent-driver usage path returns
  success. Do not treat it as config verification. Actual servo ID, endpoints,
  power, supply, and persistent protection settings must be established before
  selecting/rebooting into HX8 mode.
- CAN2 isolation Task 1 (`fc03b76b2e`) and Task 2
  (`84ffb5977e..8b3f1fd5b7`) passed their focused host tests and independent
  reviews. Task 3 initial implementation is `b299183a29`; lifecycle hardening is
  `af0aedc203`, but Task 3 is not yet accepted.
- Task 3 review established that this pinned NuttX `pthread_once()` marks its
  control complete before running the callback, so it cannot be used as a
  completion barrier between concurrent CAN1/CAN2 initializers.
- A destroyed H7 `CanDriver` must not only remove its global IRQ pointer. It must
  first disable the corresponding physical FDCAN/NVIC interrupt sources and
  clear pending flags; otherwise a later bus-off can continuously retrigger an
  IRQ whose binding is null. The physical bus remains reserved until reboot
  because the driver has no safe hardware deinitialization path.
- Task 3 final commit `b2ba4cb877` replaces the unsuitable NuttX
  `pthread_once()` path with a host-tested mutex/condition completion barrier and
  shuts down per-physical-bus FDCAN/NVIC interrupts before detaching an instance.
  `unit-CanInterfaceMap`, H7 two-interface compilation, and a forced one-interface
  ARM object compilation passed; final independent review reported no findings.
- Task 4 commit `8657c501b0` routes DroneCAN ownership/init to CAN1 and M2006 to
  CAN2. The full hybrid firmware build passed and independent review found no
  blocking issue. Remaining minor: Kconfig help still describes the old CAN1
  mutual-exclusion topology and must be corrected before final completion.
- Task 5 commit `fee94f4c70` allows UAVCAN/DroneCAN on separated CAN1 while
  retaining a nonzero Cyphal conflict. The host test passed with nine internal
  cases, including explicit DroneCAN-allowed and Cyphal-rejected coverage; the
  independent review reported no findings.
- Final host verification on `c561c2ad6b` ran the three required test filters
  separately. `CanInterfaceMap`, `CanOwnership`, and `hybridCheck` each exited
  zero with 1/1 CTest targets passed and zero failures. Logs:
  `/tmp/m2006-can2-map-final.log`, `/tmp/m2006-can2-ownership-final.log`, and
  `/tmp/m2006-can2-hybrid-check-final.log`.
- Final `make zeroone_x6_hybrid` exited zero and created a 1,751,883-byte PX4
  artifact with FLASH usage 1,866,864 bytes (94.95%). Artifact SHA-256:
  `4e6e7788f02d180a7a3ab76df2862fb9be1170cbdf4a58e7f8bc65498bdc0369`.
  Hardware acceptance is explicitly pending; no USB/PMU/C610 target result is
  inferred from the host build.
- Final independent branch review covered all nine CAN2-isolation commits from
  `d1d35c5640` through `c561c2ad6b` and reported no Critical, Important, or Minor
  findings. It judged the code ready for hardware acceptance, with target timing
  and dual-bus behavior explicitly retained as residual validation risk.
- Target report after flashing the final artifact: USB remained stable and
  `m2006_can status` showed CAN2 1Mbps, `rx=395776/398088`, `tx=196905`,
  `tx_full=0`, `error=0`, `hw errors=0`, `fault=0`; `m2006_motor_status` showed
  both motors online, `fault_flags=0`, `can_error_count=0`, and no timeouts.
  `uavcan status` returned `ERROR [uavcan] application not running`, so PMU
  DroneCAN is not yet accepted and the direct start error is still required.
- Direct `uavcan start` printed Node ID 1/1 Mbps but returned NSH status 1;
  `uavcan status` also returned 1. Heap exhaustion is excluded by target `free`:
  566080 bytes free and a 421712-byte largest block. The failure is therefore in
  the synchronous `UavcanNode::start()` pre-scheduling path, with physical CAN1
  ownership rejection the leading hypothesis.
- Boot ordering is CAN2-first: `rc.vehicle_setup` sources `rc.hybrid_apps` and
  starts M2006 before the later generic `rcS` UAVCAN block. A clean power-cycle
  test with both autostarts disabled, followed by manual DroneCAN-only startup,
  is required to distinguish ownership interaction from a standalone CAN1
  startup failure.
- Hardware result: Stage J restores USB enumeration. Since TX-complete and RX
  interrupts remain enabled, the USB failure specifically requires the FDCAN
  bus-off interrupt/recovery path. This closes the diagnostic root-cause chain.
- The user approved the two-layer production design: M2006 transmission gating
  plus regular `CanDriver::select()` maintenance, and bounded pending-TX cleanup
  inside the shared STM32H7 bus-off handler. Design commit: `0099d099cd`.
- The approved design was expanded into a five-task TDD implementation plan and
  committed as `39f14e4d52`. Production source remains unchanged at this checkpoint.
- Task 1 commit `c654d0ab34` adds the host-tested M2006 transmit policy. RED
  failed for the expected missing header; two incremental GREEN runs passed 1/1.
  A clean SITL test build is independently blocked by the pre-existing
  Commander/uORB generated `hybrid_vehicle_status.h` dependency race, not by the
  Task 1 files. Task review approved code quality but requires a human decision
  on accepting focused GREEN versus expanding scope to repair that build race.
- Hardware result: Stage I restores USB enumeration. The controller can accept a
  real `TXBAR` request without breaking USB when `ILE=0`; CPU-side FDCAN interrupt
  delivery/handling is therefore required for the startup failure.
- `CanIOFlagAbortOnError` is only acted on by `pollErrorFlagsFromISR()`, which is
  called from TX/RX handlers or `CanDriver::select()`. The M2006 module bypasses
  `CanDriver::select()` and calls `CanIface` directly; `handleBusOff()` itself only
  clears `CCCR.INIT` and does not cancel the pending TX item. Stage J tests this
  path by disabling only `FDCAN_IE_BOE` before one real TX request.
- Built Stage J and saved it as
  `zeroone_x6_hybrid_usb_diag_stage_j_tx_no_busoff_irq.px4`. SHA-256:
  `3fc6834152dc45f668e80b0781e7eb9c795d1169b7b82f9386598d57dbd86a86`.
- Restored both temporary source changes and rebuilt the unmodified branch artifact.
- Hardware result: Stage H restores USB enumeration. Frame construction, critical
  section entry, `TXFQS` access, FIFO-index calculation, and all four message-RAM
  writes are excluded. The remaining boundary begins at the `TXBAR` write and its
  resulting controller/interrupt activity.

## 2026-08-25 Rover manual steering sign

- The physical vehicle reports reversed manual steering only: left stick turns
  right, while forward/reverse is correct. This does not justify changing CAN
  IDs or `M2K_L_REV`/`M2K_R_REV`, because those parameters define wheel direction
  and the longitudinal direction is already correct.
- The differential Rover implementation uses `manual_control_setpoint.roll`
  as the manual steering input in all four manual paths. A shared
  `RoverControl::manualSteeringInput()` conversion now negates that input before
  Manual/Acro/Stability/Position processing; Offboard and autonomous yaw inputs
  are unchanged. A focused unit test covers -1/0/+1 mapping.
- `make zeroone_x6_hybrid` completed with FLASH usage 1,872,736 / 1,966,080 B
  (95.25%); artifact SHA-256 is
  `d49c3df7ac5818b1c9a91f671b345227e84432fb5b79870c737507eab779d6b0`.
  Direct `unit-RoverControl` execution passed all 3 tests. The aggregate
  `make tests TESTFILTER=RoverControl` remains blocked by the known host
  Anaconda/Gazebo protobuf generated-header mismatch, unrelated to this patch.

## 2026-08-25 Multicopter zero-thrust root cause and fix

- Hardware telemetry showed `manual_control_setpoint.throttle=0.53585`, but
  `vehicle_attitude_setpoint.thrust_body[2]`, `vehicle_rates_setpoint`, and
  `vehicle_thrust_setpoint` were all zero; the allocator reported
  `thrust_setpoint_achieved=true` and published four zero MC outputs. This
  isolates the fault above the allocator.
- `mc_att_control` consumed `vehicle_status` once at the top of `Run()`, then
  attempted to refresh `_spooled_up` only inside a later `updated()` branch.
  That branch was usually already cleared, leaving `_spooled_up=false` and
  forcing `_manual_throttle_maximum` to zero indefinitely.
- The fix updates `_spooled_up` from the status sample consumed at the top of
  `Run()`, preserving the Rover early-return behavior. The hybrid firmware
  build passed at 95.26% FLASH with SHA-256
  `d5f657dd21fd0d0aa1a684cdd95ed7a0a23a6e4e6d55aa796ab7a16103910cb6`.
- The driver enables TC and bus-off interrupts, routes TC to FDCAN line 1 and
  bus-off to line 0, and attaches both NVIC handlers before runtime. Stage I will
  clear `ILE` immediately before one `TXBAR` request, leaving hardware TX active
  while preventing FDCAN interrupt delivery to the CPU.
- Built Stage I and saved it as
  `zeroone_x6_hybrid_usb_diag_stage_i_tx_no_irq.px4`. SHA-256:
  `9d8f1d372dc1a71da689fea1df3e24fbcfe2c9001bcd83e3852e91481a9ac2fb`.
- Restored both temporary source changes and rebuilt the unmodified branch artifact.
## 2026-08-21 M2006 speed-domain correction

- User hardware clarification supersedes the earlier output-shaft interpretation:
  the C610 feedback RPM is the motor rotor speed before the 36:1 gearbox. The
  vehicle has an additional 4:1 mechanical reduction, for 144:1 total, and a
  345 mm wheel. A 500 rpm output-shaft limit therefore corresponds to about
  18000 rpm at the C610 feedback domain.
- The previous code directly used `M2K_MAX_RPM` for the C610 feedback-domain
  target, while the parameter documentation/default still said 500 output-shaft
  rpm. This was a real unit mismatch. `M2K_MAX_RPM` and `M2K_RPM_SLEW` now use
  rotor RPM and rotor RPM/s, both defaulting to 18000.
- The user's current `health_report.arming_check_error_flags=17956864` is
  `0x01120000`, with bits 17, 20, and 24 set: local-position estimate, system,
  and global-position estimate. The simultaneous M2006 sample had
  `fault_flags=0`, fresh feedback, both motors online, and zero H7/CAN errors;
  this sample does not support an M2006 startup-fault conclusion.

## 2026-08-22 HX8 protection first-event snapshot

- `TRANSFORM_FAULT_ACTUATOR_PROTECTION` (fault reason 9) is latched when the
  HX8 status reports a nonzero protection bit during a transition. Its normal
  post-fault telemetry can already have cleared, so it cannot diagnose the
  initiating condition.
- The HX8 driver now captures only the first protection event in each driver
  lifetime and exposes it through `hx8_uart_servo status`; it records raw
  status/protection flags, command sequence/result, angle, voltage, current,
  power, and temperature. The capture does not alter motion, release, or fault
  behavior. The matching `zeroone_x6_hybrid.px4` SHA-256 is
  `627d92ccb3068dcf6f360241784561a9793e4e30c2fd90ce6f4fe6deecdbb1a9`.
- Installed-mechanism capture: sequence 3 moved from Rover `+50` toward Quad
  `-50` and reported raw `status_flags=0x41`, i.e. command-executing bit 0 plus
  protection bit `0x40` (power protection). At the initiating sample the HX8
  was at 9.4 degrees, 11.949 V, 2.083 A, 24.718 W and 25.48 C. This is above
  the configured 20 W power threshold, while voltage and temperature are normal.
  After the release command, status naturally returned to zero protection and
  14 mA idle current; that later normal sample is not evidence against the
  first-event capture.

## 2026-08-25 near-180-degree endpoint normalization

- The earlier exact-180-degree sign fix was present and still covered `-90 ->
  +90`, but it did not cover a measured angle slightly beyond the endpoint.
  With `HX8_ANG_QUD=-85`, `HX8_ANG_ROV=95`, and feedback `95.1`, shortest-angle
  wrapping produced the wrong Quad-side classification and `NoSensor`.
- `normalizeAs5600()` now chooses the equivalent revolution nearest the
  configured directed endpoint interval before normalizing and clamping. This
  keeps near-endpoint overshoot on the correct side while preserving wrapped
  spans and the prior exact-180 behavior.
- `make zeroone_x6_hybrid` succeeded. Artifact SHA-256:
  `de1fc81121d8e1d367f40d6cb4a469da507131848e4e69aa2759fc13cd4173cc`. A
  separate SITL build was blocked by the host Anaconda/Gazebo Protobuf version
  conflict, not by this source change.

## 2026-08-26 Rover control-loop and ULog-tool audit

- The current Rover chain contains four feedback levels: independent M2006
  rotor-speed PID+FF loops; rover yaw-rate PI plus geometric feedforward; yaw P
  outside the yaw-rate loop; and body-X speed PI plus throttle feedforward in
  parallel with the yaw cascade. Pure Pursuit and waypoint speed planning are
  geometric/trajectory layers rather than PID loops.
- Manual bypasses all vehicle-level PID loops but still uses each M2006 speed
  loop. Acro adds yaw-rate PI. Stabilized can add yaw P when the steering stick
  is centered while moving. Position and Auto/Mission add body-speed PI and
  path generation around the same yaw cascade.
- Current `DifferentialRateControl` applies `RO_YAW_RATE_TH` to both measured
  yaw rate and yaw-rate setpoint. This conflicts with the project rule that the
  parameter is measurement-noise-only; the documentation records the actual
  behavior so dead-zone response is not misdiagnosed as low gain.
- The old `/home/crocodile/PX4-Autopilot/rover_ulog_plot.py` remained usable for
  basic rate/attitude/speed plots but lacked source Rover outputs, mode context,
  position/Pure-Pursuit inputs, M2006 inner-loop data, bounded time selection,
  and structured error metrics. The new worktree-local script adds these while
  retaining a strict pyulog topic filter.
- `python3 -m py_compile rover_ulog_plot.py`, `--help`, and filtered `--list`
  passed. A smoke run on `log_0_2026-6-17-02-25-50.ulg` generated six PNG files
  and `summary.md`; visual inspection confirmed readable Chinese labels and
  correct final Motor 6 left/Motor 5 right traces. Missing custom topics were
  reported instead of substituted.
- On 2026-08-27, the Rover parameter table was reordered from hardware mapping
  through M2006 speed, yaw-rate, yaw, body-speed, and path/mission tuning. Each
  row now states a bounded test method and symptoms of excessive or
  insufficient values; documentation-only validation used `git diff --check`
  and a Markdown table column-count check, so no firmware build was required.
- On 2026-09-01, the owner authorized checkpointing every valid tracked and
  untracked item in the validated testc1 worktree, including documentation,
  diagnostics, the ULog tool, and `state/`. Build products and unrelated
  temporary files remain excluded. The original remote testc1/testc2 refs must
  not be rewritten; testc2 integration targets a new testc4 line.
- The testc3 checkpoint audit found only the expected Rover reference/tool and
  state evidence as untracked content; no sensitive-data pattern was found.
  `git diff --check`, `python3 -m py_compile rover_ulog_plot.py`, and
  `make zeroone_x6_hybrid` passed. Build output is recorded in
  `state/build_testc3_checkpoint.log`.

## 2026-09-02 mixed UART servo implementation

- HX8 advanced multi-turn time/acceleration command `0x0e` is implemented with
  signed 32-bit 0.1-degree targets; the existing status monitor already returns
  the matching multi-turn position domain.
- HX-65HM uses its distinct `0xff 0xff` protocol on the same shared UART. The
  single driver scheduler permits exactly one outstanding protocol transaction.
  Pair motion stages torque-enable plus target/speed to both unique IDs using
  REG_WRITE and executes both with broadcast ACTION.
- HX-65HM position does not become valid from boot/config replies; each side
  must complete the documented register-56 runtime monitor read. Undocumented
  register 65 is not interpreted as an error byte; only the status-frame error
  byte is used.
- `HybridSequenceCoordinator` separates physical shape, logical propulsion
  owner, readiness, gear command, and sequence fault. Manual gear disables only
  automatic commands; gear-down and wheel-clear interlocks stay active.
- A gear failure during an already airborne Quad-to-Rover preparation preserves
  Quad control ownership while latching a fault that blocks later arming or
  transformation. No-owner transition faults still inhibit all propulsion.
- Focused Hx8 (4 targets), Hx65 (3 targets), transformation, sequence,
  Commander status, and hybrid arming-check suites passed. Final target build
  passed with no compiler warnings in `state/build_final.log`; artifact details
  are recorded in `state/README.md`.
- Target hardware has not been tested. ACTION simultaneity, shared-bus electrical
  timing, endpoint directions, under-load skew, and protection behavior remain
  explicit acceptance items.

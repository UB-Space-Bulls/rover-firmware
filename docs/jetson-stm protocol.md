# Jetson ↔ STM32 CAN FD Protocol Spec (v0.2)

**Status:** Draft, developed decision-by-decision. Each choice below has a stated
reason — this is meant to be editable, not treated as fixed once written. Arm not yet accounted for

## 1. Overview

- **Transport:** CAN FD, two nodes (Jetson via USB-CAN/SPI-CAN adapter, STM32 onboard CAN FD peripheral)
- **Arbitration bitrate:** TBD — pick after transceiver/wiring is finalized (500 kbit/s–1 Mbit/s is safe for a short two-node link)
- **Data-phase bitrate:** TBD — same, typical range 2–8 Mbit/s
- **ID format:** 11-bit standard IDs
- **Byte order:** Little-endian (native to both STM32 and Jetson — zero conversion needed)
- **Architectural boundary:** Jetson owns path/velocity decisions; STM32 owns motor control and odometry computation. Neither side needs to know the internals of the other.

## 2. Message ID Map

IDs double as CAN arbitration priority (lower ID = wins bus contention). Ordered by
how urgent it is that a message wins if two ever collide on the bus.

| ID (hex) | Name | Direction | Priority | Rate |
|---|---|---|---|---|
| 0x000 | Estop / Fault | Jetson → STM32 | Highest — safety must never lose arbitration | Event-driven |
| 0x010 | Command (cmd_vel + mode) | Jetson → STM32 | High | ~20 Hz (tune once planning pipeline exists) |
| 0x020 | Odometry / Telemetry | STM32 → Jetson | Medium | Matches command rate, ~20 Hz |
| 0x030 | Config / Parameter Set | Jetson → STM32 | Lowest, on-demand only | On-demand |
| 0x040 | *(reserved for future use)* | — | — | — |

**Why no heartbeat:** odometry (0x020) already flows on a regular schedule. The Jetson
watching "have I received odometry recently" gives the same liveness signal a dedicated
heartbeat would, without adding a redundant message type. Config was moved down to fill
the gap left by dropping it, keeping message IDs contiguous with 0x040 open for the
next addition.

## 3. Message Definitions

### 0x000 — Estop / Fault (Jetson → STM32)
| Field | Type | Bytes | Description |
|---|---|---|---|
| estop_reason | uint8 | 1 | 0=manual, 1=comm_loss, 2=overcurrent, 3=other |
| sequence_number | uint8 | 1 | Increments per message |

**Total: 2 bytes**

**Behavior on receipt:** STM32 immediately performs a hard stop (motor output cut
entirely) — see §5 for why hard stop was chosen over a decel ramp.

### 0x010 — Command (Jetson → STM32)
| Field | Type | Bytes | Description |
|---|---|---|---|
| linear_velocity | float32 | 4 | m/s |
| angular_velocity | float32 | 4 | rad/s |
| mode | uint8 | 1 | 0=idle, 1=run, 2=estop |
| sequence_number | uint8 | 1 | Increments per message |
| reserved | uint16 | 2 | Padding — room to add a field later without changing message length |

**Total: 12 bytes**

Follows a `cmd_vel`-style convention (linear + angular velocity) rather than per-wheel
targets — the STM32, which knows the actual drivetrain geometry (wheelbase, wheel
radius), converts this to per-wheel commands. Keeps the Jetson decoupled from
drivetrain mechanics.

### 0x020 — Odometry / Telemetry (STM32 → Jetson)
| Field | Type | Bytes | Description |
|---|---|---|---|
| pose_x | float32 | 4 | m |
| pose_y | float32 | 4 | m |
| pose_heading | float32 | 4 | radians (not degrees — matches ROS/EKF conventions) |
| linear_velocity | float32 | 4 | m/s, measured (not commanded) |
| angular_velocity | float32 | 4 | rad/s, measured |
| fault_flags | uint8 | 1 | Bitfield — see §4 |
| loop_time_us | uint16 | 2 | STM32 control loop execution time — diagnostic |
| sequence_number | uint8 | 1 | Increments per message |

**Total: 24 bytes**

Reports pose + velocity together (matching the standard `nav_msgs/Odometry`-style
convention) rather than velocity alone, specifically so this can feed directly into
a future sensor fusion node (EKF combining wheel odometry with ZED visual
odometry/IMU) without the Jetson having to re-derive velocity by differentiating
noisy position data.

**Current sensing:** not yet included — TBD whether the ESC exposes current feedback
to the STM32. Add as a field once confirmed rather than reserving space for data
that may not be available.

### 0x030 — Config / Parameter Set (Jetson → STM32)
| Field | Type | Bytes | Description |
|---|---|---|---|
| param_id | uint8 | 1 | Which parameter (e.g., 0=Kp, 1=Ki, 2=Kd) |
| param_value | float32 | 4 | New value |

**Total: 5 bytes**

Runtime tuning/debugging only — not part of the main control loop path.

## 4. Fault Flags Bitfield (used in 0x020)

| Bit | Meaning |
|---|---|
| 0 | Overcurrent |
| 1 | Comm timeout (STM32 hasn't received a valid 0x010 recently) |
| 2 | Encoder fault (no counts detected while commanded to move) |
| 3 | CAN bus-off, recovered (informational) |
| 4–7 | Reserved |

Packed as a bitfield rather than a single error-code integer specifically so multiple
simultaneous faults remain visible at once (e.g., overcurrent AND encoder fault both
active) instead of one silently overwriting the other.

## 5. Timeout & Fault Handling

- **Command timeout:** ~200 ms — derived as roughly 4x the expected ~50 ms command
  period (20 Hz), following the general rule of 3–5x expected period: tight enough to
  catch real comm loss quickly, loose enough to tolerate normal single-frame jitter
  without false-triggering.
- **Behavior on command timeout:** hard stop (motor output cut entirely), not a decel
  ramp. Chosen because the robot's expected mass/speed profile makes coast-after-cutoff
  a non-issue; revisit if the platform later scales up to something faster/heavier,
  since this is pure STM32 firmware behavior and doesn't require a message format
  change to update.
- **Liveness / heartbeat:** none dedicated — Jetson treats regular 0x020 odometry
  arrival as the liveness signal (see §2).
- **Bus-off recovery:** STM32 firmware should auto-reinitialize the CAN peripheral
  after a backoff delay on bus-off — exact delay TBD during firmware implementation.
- **Malformed/unexpected frames:** dropped silently, no NACK/retransmit request — the
  next valid command arrives within one command period regardless.
- **Sequence numbers:** used for diagnostics/drop detection; a single missed frame
  should not by itself trigger a fault — only sustained loss (command timeout, above)
  should.
- **CRC/corruption:** handled by CAN FD's hardware CRC — no application-level checksum
  needed.

## 6. Extensibility

- ID gaps (0x000 / 0x010 / 0x020 / 0x030 / 0x040 free) leave room to insert new
  message types later without renumbering.
- Reserved padding bytes included in 0x010 (and worth adding to 0x020 if space
  allows) so fields can be added later without changing message length.
- No version byte — appropriate for a two-node project where both ends (Jetson code,
  STM32 firmware) are updated together by the same team. Revisit only if this
  protocol is ever deployed where the two sides can update independently.

## 7. Open TBDs

- [ ] Arbitration + data-phase bitrates, once transceiver/wiring is finalized
- [ ] Whether ESC exposes current sensing — add a current field to 0x020 if so
- [ ] cmd_vel actual rate — 20 Hz is a placeholder; tune once the planning/perception
      pipeline exists and its own update rate is known
- [ ] Bus-off recovery backoff delay

# Jetson ↔ STM32 CAN FD Protocol Spec (v0.4)

**Status:** Developed decision-by-decision, reasoning kept alongside each choice so
it's editable rather than treated as fixed. Defines the drivetrain bus protocol
and the arm's architecture; the arm is moving to its own STM32 and its
communication protocol is TBD (see Part 2).
Part 0's conventions apply to every bus defined here; message definitions are
bus-specific.

---

## Part 0 — Shared Conventions

- **Transport:** CAN FD
- **ID format:** 11-bit standard IDs (each physical bus has its own ID space — no collision risk between independent buses)
- **Byte order:** Little-endian (native to both STM32 and Jetson — zero conversion needed)
- **Framing:** Free — CAN FD is inherently message-based, no delimiter/length-prefix design needed
- **Integrity:** Free — CAN FD's hardware CRC rejects corrupted frames before application code ever sees them; no application-level checksum needed
- **ID-priority philosophy:** lower ID = higher bus arbitration priority. Order message IDs by what must win contention if two ever transmit simultaneously, not just by convenience
- **Malformed frame handling:** application-level sanity checks (length/DLC match, in-range values) catch frames that are electrically intact but nonsensical. Default behavior: drop silently, no NACK/retransmit — rely on the next valid message arriving within one normal period
- **Extensibility default:** leave numeric gaps between assigned IDs, and reserved padding bytes in messages with room, so future additions don't require renumbering or reformatting existing messages
- **Rates are tuning parameters, not structural decisions** — placeholder values below are meant to be tuned once real (or simulated) hardware and upstream software (planning/MoveIt2) exist, not treated as final

---

## Part 1 — Drivetrain Bus

### 1.1 Overview
- **Architectural boundary:** Jetson owns path/velocity decisions; STM32 owns motor control and odometry computation.
- Arbitration bitrate: TBD (500 kbit/s–1 Mbit/s typical for a short two-node link)
- Data-phase bitrate: TBD (2–8 Mbit/s typical)

### 1.2 Message ID Map

| ID (hex) | Name | Direction | Priority | Rate |
|---|---|---|---|---|
| 0x000 | Estop / Fault | Jetson → STM32 | Highest — safety must never lose arbitration | Event-driven |
| 0x010 | Command (cmd_vel + mode) | Jetson → STM32 | High | ~20 Hz (placeholder) |
| 0x020 | Odometry / Telemetry | STM32 → Jetson | Medium | Matches command rate |
| 0x030 | Config / Parameter Set | Jetson → STM32 | Lowest, on-demand | On-demand |
| 0x040 | *(reserved for future use)* | — | — | — |

No dedicated heartbeat — regular Odometry (0x020) arrival serves as the liveness signal the Jetson watches for.

### 1.3 Message Definitions

**0x000 — Estop / Fault (Jetson → STM32)**
| Field | Type | Bytes | Description |
|---|---|---|---|
| estop_reason | uint8 | 1 | 0=manual, 1=comm_loss, 2=overcurrent, 3=other |
| sequence_number | uint8 | 1 | |

Total: 2 bytes. Behavior: immediate hard stop (motor output cut entirely) — chosen over
a decel ramp because the platform's mass/speed profile makes uncontrolled coast a
non-issue; revisit if the platform scales up.

**0x010 — Command (Jetson → STM32)**
| Field | Type | Bytes | Description |
|---|---|---|---|
| linear_velocity | float32 | 4 | m/s |
| angular_velocity | float32 | 4 | rad/s |
| mode | uint8 | 1 | 0=idle, 1=run, 2=estop |
| sequence_number | uint8 | 1 | |
| reserved | uint16 | 2 | |

Total: 12 bytes. cmd_vel-style rather than per-wheel — STM32 owns wheel geometry
conversion so the Jetson stays decoupled from drivetrain mechanics.

**0x020 — Odometry / Telemetry (STM32 → Jetson)**
| Field | Type | Bytes | Description |
|---|---|---|---|
| pose_x | float32 | 4 | m |
| pose_y | float32 | 4 | m |
| pose_heading | float32 | 4 | radians |
| linear_velocity | float32 | 4 | m/s, measured |
| angular_velocity | float32 | 4 | rad/s, measured |
| fault_flags | uint8 | 1 | bitfield, see 1.4 |
| loop_time_us | uint16 | 2 | diagnostic |
| sequence_number | uint8 | 1 | |

Total: 24 bytes. Reports pose + velocity together (nav_msgs/Odometry convention) so it
can feed a future EKF (wheel odometry + ZED VIO/IMU) without the Jetson re-deriving
velocity by differentiating noisy position.

Current sensing: not yet included — TBD whether ESC exposes current feedback.

**0x030 — Config / Parameter Set (Jetson → STM32)**
| Field | Type | Bytes | Description |
|---|---|---|---|
| param_id | uint8 | 1 | e.g. 0=Kp, 1=Ki, 2=Kd |
| param_value | float32 | 4 | |

Total: 5 bytes. Runtime tuning only, not part of the real-time control path — exists so
PID gains can be adjusted live during tuning without reflashing firmware.

### 1.4 Fault Flags Bitfield (0x020)

| Bit | Meaning |
|---|---|
| 0 | Overcurrent |
| 1 | Comm timeout |
| 2 | Encoder fault |
| 3 | CAN bus-off, recovered |
| 4–7 | Reserved |

Bitfield (not a single error code) so simultaneous faults stay visible instead of one
overwriting another.

### 1.5 Timeout & Fault Handling
- **Command timeout:** ~200 ms (≈4× the ~50 ms/20 Hz command period, following the
  general 3–5× rule — tight enough to catch real loss, loose enough to tolerate normal jitter)
- **On timeout:** hard stop (see 1.3)
- **Bus-off recovery:** STM32 auto-reinitializes CAN peripheral after a backoff delay — TBD
- **Sequence numbers:** used for drop/reorder diagnostics; a single missed frame alone
  should not trigger a fault, only sustained loss (command timeout) should

### 1.6 Open TBDs
- [ ] Arbitration + data-phase bitrates, once transceiver/wiring finalized
- [ ] Whether ESC exposes current sensing
- [ ] cmd_vel actual rate — 20 Hz placeholder, tune once planning pipeline exists
- [ ] Bus-off recovery backoff delay

---

## Part 2 — Arm (communication protocol TBD)

The arm is moving to its own dedicated STM32. The arm's hardware and control
architecture below is unchanged; only the communication protocol (message
set, IDs, field layouts, rates, bitrates) is still to be decided. The
previous arm bus protocol (v0.3) is archived in
[`archive/arm/arm-bus-protocol-v0.3.md`](../archive/arm/arm-bus-protocol-v0.3.md)
for reference.

### 2.1 Overview

**Mechanical structure (5 DOF, 6 actuators):**
| Joint | Actuator(s) | Notes |
|---|---|---|
| Shoulder | 2× BLDC (w/ encoder) | Torque-combined, single DOF — not differential |
| Elbow | 1× BLDC (w/ encoder) | Single DOF, forearm rotation |
| Wrist | 2× stepper | True differential — 2 DOF (pitch, roll) from combined motor outputs |
| Gripper | 1× servo (DS3218) | Single DOF, open/close |

**Sensors:** limit switch at shoulder base + elbow (hard stops, both normal/expected
during operation, not fault conditions). IMU at elbow + hand (wrist).

**Control mode:** position control throughout — matches MoveIt2's trajectory output
(joint-space position/velocity waypoints), avoiding a translation step between planner
output and actuator command.

**Feedback fusion architecture:** arm STM32 fuses whatever each joint needs internally
and reports one clean estimated joint angle per DOF, regardless of how it was derived:
- Shoulder/elbow: BLDC encoders directly, elbow IMU as a cross-check against gearbox backlash/deflection
- Wrist: steppers are open-loop (no position feedback of their own) — the hand IMU is
  the primary source of truth for actual wrist orientation, fused with commanded step
  counts by the STM32
- This mirrors the drivetrain odometry pattern: raw sensor fusion happens once, close
  to the hardware; the Jetson only ever sees a clean per-joint state, matching the
  uniform `joint_states`-style feed MoveIt2's execution monitoring expects — it doesn't
  need to know which joints have "real" encoders vs. IMU-estimated state

### 2.2 Fault & Limit-Switch Handling

- **Limit switches are normal/expected**, not exceptional — both shoulder and elbow are
  routinely reached during normal operation, not just homing.
- **On trip:** STM32 halts output to *that joint only* — other joints continue
  executing their current trajectory uninterrupted. (Considered a whole-arm stop, but
  rejected: since these triggers are routine, aborting all coordinated motion on every
  normal limit event would be a worse outcome for no real safety benefit.)
- **Auto-resume:** no explicit "clear fault" step required — the moment a subsequent
  command's target for that joint is back within legal range, the STM32 resumes normal
  control automatically.
- **Reporting to the Jetson:** TBD with the protocol. (v0.3 used a dedicated,
  immediately-sent Fault Event message plus fault_flags bits in regular
  Feedback — see the archive.)

### 2.3 Open TBDs
- [ ] Communication protocol for the arm STM32 (message set, IDs, field layouts, rates, bitrates)
- [ ] Whether the DS3218 gripper servo provides any real position feedback, or remains
      command-echo only
- [ ] Actual joint angle ranges per DOF (depends on final mechanical design)

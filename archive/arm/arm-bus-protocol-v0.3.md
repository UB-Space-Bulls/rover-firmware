# Arm Bus Protocol (archived, v0.3)

**Archived:** the arm is moving to its own STM32, and the communication
protocol for that link is still to be decided. This is Part 2 of
`docs/jetson-stm protocol.md` as it stood at v0.3, kept for reference. The
arm's hardware and control architecture (§2.1, and the behavior in §2.5) is
still current and lives in Part 2 of the protocol doc; the message-level
protocol here (§2.2–2.4, joint_id numbering, fault reporting) is not current,
and nothing in the build uses it. The shared conventions it relied on (CAN FD, 11-bit IDs,
little-endian, malformed-frame handling) are still in Part 0 of
[`docs/jetson-stm protocol.md`](../../docs/jetson-stm%20protocol.md).

The matching encode/decode code is in this folder (`arm_encode.*`,
`arm_decode.*`) and its tests are in `test_arm.c`.

---

## Part 2 — Arm Bus

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

**joint_id numbering** (used in Fault Event, §2.3):
```
0 = shoulder
1 = elbow
2 = wrist_pitch
3 = wrist_roll
4 = gripper
```

### 2.2 Message ID Map

| ID (hex) | Name | Direction | Priority | Trigger |
|---|---|---|---|---|
| 0x000 | Estop | Jetson → STM32 | Highest | Event-driven |
| 0x005 | Fault Event | STM32 → Jetson | Very high — preempts routine traffic | Event-driven, sent immediately on detection |
| 0x010 | Command (joint targets + mode) | Jetson → STM32 | High | ~50 Hz (placeholder) |
| 0x020 | Feedback (joint state) | STM32 → Jetson | Medium | Matches command rate |
| 0x030 | Config | Jetson → STM32 | Lowest | On-demand |

### 2.3 Message Definitions

**0x000 — Estop (Jetson → STM32)**
| Field | Type | Bytes | Description |
|---|---|---|---|
| estop_reason | uint8 | 1 | 0=manual, 1=comm_loss, 2=overcurrent, 3=other |
| sequence_number | uint8 | 1 | |

Total: 2 bytes. Same layout as drivetrain Estop (§1.3) — defined identically rather
than inventing a separate format, since the semantics (immediate hard stop, reason
code) don't differ per-bus.

**0x005 — Fault Event (STM32 → Jetson)**
| Field | Type | Bytes | Description |
|---|---|---|---|
| joint_id | uint8 | 1 | See §2.1 numbering |
| fault_type | uint8 | 1 | 0=limit switch hit (room left for future types) |
| sequence_number | uint8 | 1 | |

Total: 3 bytes. Kept minimal and fast — detailed state still rides the next regular
Feedback message. STM32 reacts locally the instant its GPIO sees the limit switch
trip; this message is purely informational, not a request for permission.

**0x010 — Command (Jetson → STM32)**
| Field | Type | Bytes | Description |
|---|---|---|---|
| shoulder_angle | float32 | 4 | rad |
| elbow_angle | float32 | 4 | rad |
| wrist_pitch | float32 | 4 | rad |
| wrist_roll | float32 | 4 | rad |
| gripper_position | float32 | 4 | |
| mode | uint8 | 1 | 0=idle, 1=run, 2=estop |
| sequence_number | uint8 | 1 | |
| reserved | uint16 | 2 | |

Total: 24 bytes. No Jetson-visible homing mode — homing is handled internally by the
STM32 using the limit switches as zero-reference; a "not yet homed" condition can be
surfaced via fault_flags in Feedback rather than a dedicated mode value.

**0x020 — Feedback (STM32 → Jetson)**
| Field | Type | Bytes | Description |
|---|---|---|---|
| shoulder_angle | float32 | 4 | measured/fused |
| shoulder_velocity | float32 | 4 | |
| elbow_angle | float32 | 4 | |
| elbow_velocity | float32 | 4 | |
| wrist_pitch | float32 | 4 | IMU-fused |
| wrist_pitch_velocity | float32 | 4 | |
| wrist_roll | float32 | 4 | IMU-fused |
| wrist_roll_velocity | float32 | 4 | |
| gripper_position | float32 | 4 | echo of last commanded value — DS3218 feedback availability TBD, see §2.5 |
| fault_flags | uint8 | 1 | bitfield, see 2.4 |
| sequence_number | uint8 | 1 | |

Total: 38 bytes. Position + velocity per joint (except gripper) to match what
MoveIt2's trajectory execution monitoring expects.

**0x030 — Config (Jetson → STM32)**
| Field | Type | Bytes | Description |
|---|---|---|---|
| param_id | uint8 | 1 | |
| param_value | float32 | 4 | |

Total: 5 bytes. Same purpose as drivetrain Config — live tuning, not part of the
real-time control path.

### 2.4 Fault Flags Bitfield (0x020)

| Bit | Meaning |
|---|---|
| 0 | Shoulder limit switch active |
| 1 | Elbow limit switch active |
| 2 | Not yet homed |
| 3–7 | Reserved |

### 2.5 Fault & Limit-Switch Handling

- **Limit switches are normal/expected**, not exceptional — both shoulder and elbow are
  routinely reached during normal operation, not just homing.
- **On trip:** STM32 halts output to *that joint only* — other joints continue
  executing their current trajectory uninterrupted. (Considered a whole-arm stop, but
  rejected: since these triggers are routine, aborting all coordinated motion on every
  normal limit event would be a worse outcome for no real safety benefit.)
- **Auto-resume:** no explicit "clear fault" step required — the moment a subsequent
  Command's target for that joint is back within legal range, the STM32 resumes normal
  control automatically.
- **Reported via:** Fault Event (0x005), immediately on detection, plus the
  corresponding fault_flags bit in the next Feedback message.

### 2.6 Open TBDs
- [ ] Whether the DS3218 gripper servo provides any real position feedback, or remains
      command-echo only
- [ ] Actual joint angle ranges per DOF (depends on final mechanical design)
- [ ] Command rate — 50 Hz placeholder, tune once MoveIt2 trajectory execution is running
- [ ] Arbitration + data-phase bitrates for this bus
- [ ] Whether other fault types (e.g. encoder fault) should be added to fault_type
      beyond limit-switch events

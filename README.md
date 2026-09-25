# rover-firmware

Firmware for a rover's drivetrain, communicating with a Jetson host over a
CAN FD bus. The arm is moving to its own STM32: its hardware architecture is
described in the protocol doc, its communication protocol is TBD, and the
previous arm bus protocol is archived in [`archive/arm/`](archive/arm/).

## Layout

| Path | Contents |
|---|---|
| [`docs/jetson-stm protocol.md`](docs/jetson-stm%20protocol.md) | Drivetrain wire protocol spec (message IDs, field layouts, timing/fault behavior) and arm architecture |
| [`drivetrain/serialization/`](drivetrain/serialization/) | Drivetrain encode/decode — empty, awaiting a new implementation |
| [`drivetrain/safety/`](drivetrain/safety/) | Drivetrain safety (placeholder — only an empty README so far) |
| [`archive/drivetrain/`](archive/drivetrain/) | Previous drivetrain encode/decode and its tests — reference for the new implementation, not built |
| [`archive/arm/`](archive/arm/) | Old arm bus protocol, encode/decode, and tests — reference only, not built |

## Design

The drivetrain encode/decode converts between a plain C struct and the
fixed-length byte buffer that goes on the wire. The previous implementation
is archived in [`archive/drivetrain/`](archive/drivetrain/) so club members
can write a new one in `drivetrain/serialization/`. The archived version had
no CAN HAL dependency, so the same code compiled unmodified on both the STM32
side (paired with FDCAN HAL calls) and the Jetson side (paired with
SocketCAN), and every function returned a status enum (`DRIVETRAIN_OK` /
`DRIVETRAIN_ERR_LENGTH` / `DRIVETRAIN_ERR_RANGE`) rather than asserting, so
callers could handle a malformed frame without crashing.

Wire format follows the shared conventions in the protocol doc: little-endian
byte order, fixed message lengths, reserved padding bytes for future fields,
and application-level enum range checks (CAN FD's hardware CRC already
handles frame-level integrity).

## Building & testing

There is no active build or test setup yet. The archived test files
(`archive/drivetrain/test_protocol.c`, `archive/arm/test_arm.c`) each have a
host `gcc` command at the top for building them by hand.

## Status

The drivetrain protocol is defined; its encode/decode is being reimplemented
(previous version archived). Several protocol parameters are still placeholders pending real/simulated hardware —
see `Open TBDs` in the protocol doc (bitrates, command rate, bus-off recovery
timing, ESC current sensing). The arm communication protocol is TBD pending the
move to its own STM32.

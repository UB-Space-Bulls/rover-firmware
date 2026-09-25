# rover-firmware

Firmware for a rover's drivetrain, communicating with a Jetson host over a
CAN FD bus. The arm is moving to its own STM32: its hardware architecture is
described in the protocol doc, its communication protocol is TBD, and the
previous arm bus protocol is archived in [`archive/arm/`](archive/arm/).

## Layout

| Path | Contents |
|---|---|
| [`docs/jetson-stm protocol.md`](docs/jetson-stm%20protocol.md) | Drivetrain wire protocol spec (message IDs, field layouts, timing/fault behavior) and arm architecture |
| [`drivetrain/serialization/`](drivetrain/serialization/) | Encode/decode for the drivetrain bus (Estop, Command, Odometry, Config) |
| [`drivetrain/safety/`](drivetrain/safety/) | Drivetrain safety (placeholder — only an empty README so far) |
| [`tests/`](tests/) | Host-only roundtrip tests for the drivetrain encode/decode |
| [`archive/arm/`](archive/arm/) | Old arm bus protocol, encode/decode, and tests — reference only, not built |

## Design

The `drivetrain_encode.c`/`drivetrain_decode.c` pair converts between a plain C struct and the
fixed-length byte buffer that goes on the wire — no CAN HAL dependency, so
the same code compiles unmodified on both the STM32 side (paired with FDCAN
HAL calls at the call site) and the Jetson side (paired with SocketCAN).
Every function returns a status enum (`DRIVETRAIN_OK` / `DRIVETRAIN_ERR_LENGTH` /
`DRIVETRAIN_ERR_RANGE`) rather than asserting, so callers on either side can handle a
malformed frame without crashing.

Wire format follows the shared conventions in the protocol doc: little-endian
byte order, fixed message lengths, reserved padding bytes for future fields,
and application-level enum range checks (CAN FD's hardware CRC already
handles frame-level integrity).

## Building & testing

Tests build and run with a host compiler — no STM32 toolchain required:

```sh
./tests/run_tests.sh
```

This compiles `test_protocol.c` against the drivetrain module and runs roundtrip
tests (encode → decode → compare) plus malformed-input cases (wrong length,
out-of-range enum values).

## Status

The drivetrain bus message set is fully implemented and tested. Several
protocol parameters are still placeholders pending real/simulated hardware —
see `Open TBDs` in the protocol doc (bitrates, command rate, bus-off recovery
timing, ESC current sensing). The arm communication protocol is TBD pending the
move to its own STM32.

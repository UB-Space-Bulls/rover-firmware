# rover-firmware

Firmware for a rover's drivetrain and arm subsystems, communicating with a
Jetson host over two independent CAN FD buses.

## Layout

| Path | Contents |
|---|---|
| [`docs/jetson-stm protocol.md`](docs/jetson-stm%20protocol.md) | Wire protocol spec (message IDs, field layouts, timing/fault behavior) |
| [`drivetrain/`](drivetrain/) | Encode/decode for the drivetrain bus (Estop, Command, Odometry, Config) |
| [`arm/`](arm/) | Encode/decode for the arm bus (Estop, Fault Event, Command, Feedback, Config) |
| [`tests/`](tests/) | Host-only roundtrip tests for both modules |

## Design

Each `*_encode.c`/`*_decode.c` pair converts between a plain C struct and the
fixed-length byte buffer that goes on the wire — no CAN HAL dependency, so
the same code compiles unmodified on both the STM32 side (paired with FDCAN
HAL calls at the call site) and the Jetson side (paired with SocketCAN).
Every function returns a status enum (`*_OK` / `*_ERR_LENGTH` /
`*_ERR_RANGE`) rather than asserting, so callers on either side can handle a
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

This compiles `test_protocol.c` against both modules and runs roundtrip
tests (encode → decode → compare) plus malformed-input cases (wrong length,
out-of-range enum values).

## Status

Both buses' message sets are fully implemented and tested. Several protocol
parameters are still placeholders pending real/simulated hardware — see the
`Open TBDs` sections in the protocol doc (bitrates, command rates, bus-off
recovery timing, gripper feedback availability).

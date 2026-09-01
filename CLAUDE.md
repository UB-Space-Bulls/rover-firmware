# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

Build and run all tests (host-only, no STM32 toolchain needed):

```sh
./tests/run_tests.sh
```

This is a single `gcc -std=c99 -Wall -Wextra -Wpedantic` invocation compiling
`tests/test_protocol.c` against both `drivetrain/` and `arm/` sources, then
running the resulting binary. There is no build system beyond this script —
to run a subset of tests, comment out the unwanted `test_*()` calls in
`main()` in `tests/test_protocol.c`, or invoke the `gcc` command from the
script directly with a modified source list.

## Architecture

This repo implements the wire protocol defined in
[`docs/jetson-stm protocol.md`](docs/jetson-stm%20protocol.md): two
independent CAN FD buses (drivetrain, arm) connecting a Jetson host to an
STM32. The doc is the source of truth for message IDs, field layouts, and
timing/fault behavior — read it before changing any message format.

Each bus has a matching `*_encode.{c,h}` / `*_decode.{c,h}` pair
(`drivetrain/` and `arm/`) that converts between a plain C struct and the
fixed-length wire buffer for every message type on that bus. This code has
**no CAN HAL dependency** — it's pure struct↔bytes translation, intended to
compile unmodified on both sides of the link: the STM32 side (paired with
FDCAN HAL calls at the call site) and the Jetson side (paired with SocketCAN
at the call site). Keep it that way — don't introduce hardware-specific
includes into these files.

Conventions shared by both modules, mirroring the protocol doc:
- Little-endian byte packing via `memcpy` to a `uint32_t`/bit-shift (not
  pointer-casting a `float*`, which is UB) — see `pack_f32_le`/`unpack_f32_le`
  in each `_encode.c`/`_decode.c`.
- Every encode/decode function returns a `*_status_t` (`*_OK` /
  `*_ERR_LENGTH` / `*_ERR_RANGE`) instead of asserting; callers on either
  side must handle malformed frames without crashing.
- `*_ERR_LENGTH` on decode when `len` doesn't match the message's fixed
  `*_LEN`; `*_ERR_RANGE` on encode/decode when a closed-enum field
  (mode, estop_reason, joint_id) holds a value outside its defined range.
  Open-ended fields (`param_id`, arm's `fault_type`) are intentionally
  **not** range-checked — see the comments at each definition for why.
- Reserved bytes are zeroed on encode, ignored (not validated) on decode.

When adding a new message or field: update the protocol doc first, then add
matching struct fields / `*_LEN` constants / encode+decode logic in lockstep
across both files, then extend `tests/test_protocol.c` with a roundtrip test
(encode→decode→compare) and a malformed-input case if the message has any
range-checked or length-sensitive field.

The arm bus has one message the drivetrain bus doesn't: Fault Event (0x005),
a minimal event-driven message sent immediately on a limit-switch trip,
independent of the regular Feedback message.

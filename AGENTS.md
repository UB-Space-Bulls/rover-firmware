# AGENTS.md

Guidance for coding agents (Claude, Gemini, Codex, and others) working in this
repository.

The people working here are students learning embedded systems. The person
you are working with is **the builder**: they own the code, make every
decision, and must understand each change before it lands. You are their
tutor and pair programmer.

## Working with the builder

Follow these steps for every code change, however small:

1. **Explain before you edit.** Describe the change in plain language: what
   you would change, why, which files it touches, and what else it could
   affect (other bus, tests, protocol doc, the Jetson or STM32 side). The
   step is done when the builder has approved the change and every question
   they raised is answered.
2. **Hand off every decision.** When the work involves a choice, stop and
   give it to the builder: lay out the options, the tradeoffs of each, and
   what each would mean for the rest of the code, then give your suggested
   answer with the reasoning behind it, so the builder can weigh it and
   disagree. The builder makes the final call. Decisions include file and folder layout, new modules,
   message formats and protocol changes, function signatures and interfaces,
   error handling, dependencies, naming, and tooling. When you are unsure
   whether something counts as a decision, treat it as one and ask.
3. **Build in small steps.** Make one understandable piece at a time, then
   summarize what changed and why before moving on, so the builder can follow
   along and stop you at any point.

## Explaining

The builder will often ask questions to learn or research, not to get code
written. Teach:

- Use plain language and short sentences. Define each technical term the
  first time it appears (for example CAN FD, little-endian, undefined
  behavior, HAL).
- Keep every important technical detail exact: byte offsets, types, units,
  ranges, timing. Simplify the wording, never the facts. When you do simplify,
  say so and name what was left out.
- Ground explanations in this repo: point to the actual file, function, or
  section of the protocol doc that shows the idea.
- Say plainly when you are unsure, and point to where the answer can be
  checked (the protocol doc, a datasheet, the STM32 HAL reference).

## Code conventions

Every function, including `static` helpers and test functions, gets a
one-line comment directly above it saying what it does, in the existing
style:

```c
/* Encodes a Command message (0x010) into out. */
```

## Commands

There is no active build or test setup right now: the previous drivetrain
encode/decode and its tests are archived (see Architecture), and how the new
implementation is built and tested is a decision for the builder. Each
archived test file (`archive/drivetrain/test_protocol.c`,
`archive/arm/test_arm.c`) has a host `gcc` command at its top for building it
by hand from its own folder.

## Architecture

This repo implements the wire protocol defined in
[`docs/jetson-stm protocol.md`](docs/jetson-stm%20protocol.md): a CAN FD
drivetrain bus connecting a Jetson host to an STM32. The doc is the source of
truth for message IDs, field layouts, and timing/fault behavior — read it
before changing any message format. The drivetrain protocol (Part 1) is
current.

The arm is moving to its own STM32. Its hardware and control architecture
(joints, actuators, sensors, limit-switch behavior) is current and described
in Part 2 of the protocol doc; its communication protocol is still TBD. The
old arm bus protocol, code, and tests live in [`archive/arm/`](archive/arm/)
for reference only: nothing builds or tests them, and their message formats
do not describe the current system. The arm's new protocol and code layout
are open decisions for the builder.

The drivetrain encode/decode code (struct ↔ wire bytes for every message on
the bus) is archived in [`archive/drivetrain/`](archive/drivetrain/) so club
members can write a new implementation in
[`drivetrain/serialization/`](drivetrain/serialization/). The archived
version matches the current protocol and is a working reference, but the new
implementation's design is the builder's to decide.

The archived implementation followed these conventions. They are good
defaults for the new one; reuse or change them as the builder decides:
- No CAN HAL (hardware abstraction layer) dependency — pure struct↔bytes
  translation, so the same code compiles unmodified on the STM32 side (paired
  with FDCAN HAL calls at the call site) and the Jetson side (paired with
  SocketCAN at the call site).
- Little-endian byte packing via `memcpy` to a `uint32_t`/bit-shift (not
  pointer-casting a `float*`, which is UB) — see `pack_f32_le`/`unpack_f32_le`
  in `archive/drivetrain/drivetrain_encode.c`/`drivetrain_decode.c`.
- Every encode/decode function returns a status (`DRIVETRAIN_OK` /
  `DRIVETRAIN_ERR_LENGTH` / `DRIVETRAIN_ERR_RANGE`) instead of asserting, so
  callers on either side handle malformed frames without crashing.
- `DRIVETRAIN_ERR_LENGTH` on decode when `len` doesn't match the message's
  fixed `*_LEN`; `DRIVETRAIN_ERR_RANGE` on encode/decode when a closed-enum
  field (mode, estop_reason) holds a value outside its defined range.
  Open-ended fields (`param_id`) are intentionally **not** range-checked —
  see the comments at each definition for why.
- Reserved bytes are zeroed on encode, ignored (not validated) on decode.

When adding a new message or field: update the protocol doc first, then
change the encode and decode code in lockstep, then add a roundtrip test
(encode→decode→compare) and a malformed-input case if the message has any
range-checked or length-sensitive field.

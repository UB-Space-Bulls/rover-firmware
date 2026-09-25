# archive/arm

**Reference only — not current.** The arm is moving to its own STM32 and its
communication protocol is still TBD. The arm's hardware and control
architecture is unchanged and lives in Part 2 of
[`docs/jetson-stm protocol.md`](../../docs/jetson-stm%20protocol.md). This
folder holds the previous arm bus *protocol* and its code so they can be
consulted or reused:

- [`arm-bus-protocol-v0.3.md`](arm-bus-protocol-v0.3.md): the full v0.3 arm section of the protocol doc, including the message set removed in v0.4
- `arm_encode.{c,h}` / `arm_decode.{c,h}`: encode/decode for that spec
- `test_arm.c`: its roundtrip tests (build command at the top of the file)

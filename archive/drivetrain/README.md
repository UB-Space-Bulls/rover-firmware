# archive/drivetrain

**Reference only — not built.** The previous drivetrain encode/decode,
archived so club members can write a new implementation in
[`drivetrain/serialization/`](../../drivetrain/serialization/). It matches
the current drivetrain protocol (Part 1 of
[`docs/jetson-stm protocol.md`](../../docs/jetson-stm%20protocol.md)), so it
is a working reference:

- `drivetrain_encode.{c,h}` / `drivetrain_decode.{c,h}`: encode/decode for every drivetrain message
- `test_protocol.c`: roundtrip and malformed-input tests (build command at the top of the file)

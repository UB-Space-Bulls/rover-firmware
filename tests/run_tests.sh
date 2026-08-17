#!/usr/bin/env bash
# Builds and runs the protocol roundtrip tests with a host compiler (no
# STM32 toolchain needed). Run from anywhere: ./tests/run_tests.sh
set -euo pipefail
cd "$(dirname "$0")"

gcc -std=c99 -Wall -Wextra -Wpedantic \
    -I../drivetrain -I../arm \
    ../drivetrain/drivetrain_encode.c ../drivetrain/drivetrain_decode.c \
    ../arm/arm_encode.c ../arm/arm_decode.c \
    test_protocol.c \
    -o test_protocol

./test_protocol

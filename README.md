# Automotive CAN Telemetry Platform (In Progress)

A memory safe, fully deterministic Software-in-the-loop simulation platform written in Rust.

## Planned Architecture

This project is strictly separated into two distinct operational layers:
1. **Core Matrix Module (`src/can_matrix.rs`):** A rigoristic safety-critical library running under `#![no_std]` with zero heap allocation to guarantee hard real-time deterministic execution.
2. **Simulation Engine (`src/main.rs`):** A linux-native application (`std`) leveraging Linux SocketCAN interfaces to capture and process live network streams.

## Current Project State

- [x] Define global `CanError` handling and core validation traits under `#![no_std]`.
- [x] Implement `AbsWheelSpeeds` (ID: `0x215`) serialization & deserialization with Big-Endian alignment.
- [ ] Integrate the matrix parser into the Linux SocketCAN active receiver loop.
- [ ] Add functional boundary testing for signal validation.
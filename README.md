# Automotive CAN Telemetry Platform (In Progress)

A memory safe, fully deterministic Software-in-the-loop simulation platform written in Rust.

## Planned Architecture

This project is strictly separated into two distinct operational layers:
1. **Core Matrix Module (`src/can_matrix.rs`):** A rigoristic safety-critical library running under `#![no_std]` with zero heap allocation to guarantee hard real-time deterministic execution.
2. **Simulation Engine (`src/main.rs`):** A linux-native application (`std`) leveraging Linux SocketCAN interfaces to capture and process live network streams.

## Current Project State

- [x] Define global `CanError` handling and core validation traits under `#![no_std]`.
- [x] Implement `AbsWheelSpeeds` (ID: `0x215`) serialization & deserialization with Big-Endian alignment.
- [x] Integrate the matrix parser into the Linux SocketCAN active receiver loop.
- [x] Add functional boundary testing for signal validation.

---

## Next Milestones

### Phase 2: Multi-Frame Network Demultiplexing
- Implement `EngineData` (ID: `0x110`, DLC: 4) frame parsing (`rpm` and `pedal_position`).
- Refactor `src/main.rs` into a zero-allocation network demultiplexer supporting multi-ID decoding.

### Phase 3: Software-in-the-loop Integration
- [ ] Establish communication link between the Rust telemetry binary and Webots vehicle simulation environment.
- [ ] Broadcast real-time virtual vehicle telemetry over the Linux `vcan0` interface.

### Phase 4: Closed-Loop Control Architecture
- [ ] Implement an Autonomoous Emergency Brakiung safety algorithm within the Rust core module.
- [ ] Transmit `BrakeCommand` frames back to the simulation loop to dynamically actuate the vehicle platform based on boundary limits.
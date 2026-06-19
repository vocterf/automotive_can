//! ### CAN Matrix Module
//! This module provides a fully deterministic, `#![no_std]` implementation of CAN Frames.
//! It enforces strict compiler constraints to guarantee memory safety and zero runtime panic risks.

/// #### CAN Error Handling
/// Diagnostic errors that can occur during the serialization or deserialization of network data.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanError {
    /// Recevied payload buffer size is strictly smaller than frame's DLC.
    BufferTooSmall,
    /// One or more decoded singals violate the strictly defined physical boundaries.
    SingalOutOfRange,
}

/// #### CAN Frame Trait
/// Allows to safely create new CAN Frames, every new CAN Frame should fulfill this trait.
pub trait CanFrame {
    /// Unique identifier for the CAN Frame.
    const ID: u32;
    /// Data Length Code specifying the exact number of valid bytes in the payload.
    const DLC: usize;

    /// Serializes the internal structured fields into a raw, deterministic 8-byte network buffer.
    fn to_bytes(&self) -> [u8; 8];

    /// Deserializes a raw network byte slice into a validated, type-safe structure.
    /// ### Errors
    /// * Returns `CanError::BufferTooSmall` if the provided slice length is strictly smaller than the frame's DLC.
    /// * Returns `CanError::SignalOutOfRange` if any parsed signal violates the physical boundaries.
    fn from_bytes(raw: &[u8]) -> Result<Self, CanError>
    where
        Self: Sized;
}

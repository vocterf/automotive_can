//! ### CAN Matrix Module
//! This module provides a fully deterministic, `#![no_std]` implementation of CAN Frames.
//! It enforces strict compiler constraints to guarantee memory safety and zero runtime panic risks.


#![no_std]
#![deny(missing_docs)]
#![deny(clippy::all)]
#![deny(clippy::pedantic)]
#![deny(clippy::restriction)]


/// #### CAN Error Handling
/// Diagnostic errors that can occur during the serialization or deserialization of network data.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanError {
    /// Recevied payload buffer size is strictly smaller than frame's DLC
    BufferTooSmall,
    /// One or more decoded singals violate the strictly defined physical boundaries.
    SingalOutOfRange
}
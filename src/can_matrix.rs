//! ### CAN Matrix Module
//! This module provides a fully deterministic, `#![no_std]` implementation of CAN Frames.
//! It enforces strict compiler constraints to guarantee memory safety and zero runtime panic risks.

/// #### CAN Error Handling
/// Diagnostic errors that can occur during the serialization or deserialization of network data.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanError {
    /// Received payload buffer size is strictly smaller than frame's DLC.
    BufferTooSmall,
    /// One or more decoded signals violate the strictly defined physical boundaries.
    SignalOutOfRange,
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
    ///
    /// ### Errors
    /// * Returns `CanError::BufferTooSmall` if the provided slice length is strictly smaller than the frame's DLC.
    /// * Returns `CanError::SignalOutOfRange` if any parsed signal violates the physical boundaries.
    fn from_bytes(raw: &[u8]) -> Result<Self, CanError>
    where
        Self: Sized;
}

/// ### ID: `0x215` | `ABS_Wheel_Speeds`
/// Broadcasted by the ABS/ESP Controller every 10ms.
/// Contains high-precision, big-endian physical wheel speeds for all four corners of the vehicle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AbsWheelSpeeds {
    fl: u16,
    fr: u16,
    rl: u16,
    rr: u16,
}

impl AbsWheelSpeeds {
    const SPEED_SCALING: f32 = 100.0;
    /// Decodes and returns the Front Left wheel speed formatted as a physical value in km/h.
    #[inline]
    #[must_use]
    pub fn fl_kmh(&self) -> f32 {
        f32::from(self.fl) / Self::SPEED_SCALING
    }

    /// Decodes and returns the Front Right wheel speed formatted as a physical value in km/h.
    #[inline]
    #[must_use]
    pub fn fr_kmh(&self) -> f32 {
        f32::from(self.fr) / Self::SPEED_SCALING
    }

    /// Decodes and returns the Rear Left wheel speed formatted as a physical value in km/h.
    #[inline]
    #[must_use]
    pub fn rl_kmh(&self) -> f32 {
        f32::from(self.rl) / Self::SPEED_SCALING
    }

    /// Decodes and returns the Rear Right wheel speed formatted as a physical value in km/h.
    #[inline]
    #[must_use]
    pub fn rr_kmh(&self) -> f32 {
        f32::from(self.rr) / Self::SPEED_SCALING
    }
}

impl CanFrame for AbsWheelSpeeds {
    const ID: u32 = 0x215;
    const DLC: usize = 8;

    #[inline]
    fn from_bytes(raw: &[u8]) -> Result<Self, CanError>
    where
        Self: Sized,
    {
        const MAX_SPEED: u16 = 30000;
        if let [b0, b1, b2, b3, b4, b5, b6, b7, ..] = raw {
            let fl_val = u16::from_be_bytes([*b0, *b1]);
            let fr_val = u16::from_be_bytes([*b2, *b3]);
            let rl_val = u16::from_be_bytes([*b4, *b5]);
            let rr_val = u16::from_be_bytes([*b6, *b7]);



            if fl_val > MAX_SPEED || fr_val > MAX_SPEED || rl_val > MAX_SPEED || rr_val > MAX_SPEED
            {
                return Err(CanError::SignalOutOfRange);
            }

            Ok(Self {
                fl: fl_val,
                fr: fr_val,
                rl: rl_val,
                rr: rr_val,
            })
        } else {
            Err(CanError::BufferTooSmall)
        }
    }

    #[inline]
    fn to_bytes(&self) -> [u8; 8] {
        let fl_bytes = self.fl.to_be_bytes();
        let fr_bytes = self.fr.to_be_bytes();
        let rl_bytes = self.rl.to_be_bytes();
        let rr_bytes = self.rr.to_be_bytes();

        [
            fl_bytes[0],
            fl_bytes[1],
            fr_bytes[0],
            fr_bytes[1],
            rl_bytes[0],
            rl_bytes[1],
            rr_bytes[0],
            rr_bytes[1],
        ]
    }
}

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
///
/// #### Signal Specifications:
/// | Signal Name | Start Bit | Length | Byte Order | Value Type | Factor | Offset | Range / Unit |
/// | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
/// | `fl_speed` | bit 0 | 16 bits | Motorola (BE) | Unsigned | 0.01 | 0 | 0.00 - 300.00 km/h |
/// | `fr_speed` | bit 16 | 16 bits | Motorola (BE) | Unsigned | 0.01 | 0 | 0.00 - 300.00 km/h |
/// | `rl_speed` | bit 32 | 16 bits | Motorola (BE) | Unsigned | 0.01 | 0 | 0.00 - 300.00 km/h |
/// | `rr_speed` | bit 48 | 16 bits | Motorola (BE) | Unsigned | 0.01 | 0 | 0.00 - 300.00 km/h |
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

/// ### ID: `0x110` | `EngineData`
/// Broadcasted by the Powertrain Control Module (PCM).
/// Contains engine rotational speed (RPM) and real-time accelerator pedal position.
///
/// #### Signal Specifications:
/// | Signal Name | Start Bit | Length | Byte Order | Value Type | Factor | Offset | Range / Unit |
/// | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
/// | `rpm` | bit 0 | 16 bits | Motorola (BE) | Unsigned | 1.0 | 0 | 0 - 8000 RPM |
/// | `pedal_position` | bit 16 | 8 bits | Motorola (BE) | Unsigned | 1.0 | 0 | 0 - 100 % |
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EngineData {
    rpm: u16,
    pedal_position: u8,
}

impl EngineData {
    /// Returns the current engine speed in Revolutions Per Minute (RPM).
    #[inline]
    #[must_use]
    pub fn rpm(&self) -> u16 {
        self.rpm
    }

    /// Returns the accelerator pedal position as a percentage scaled from 0 to 100.
    #[inline]
    #[must_use]
    pub fn pedal_position(&self) -> u8 {
        self.pedal_position
    }
}

impl CanFrame for EngineData {
    const ID: u32 = 0x110;
    const DLC: usize = 3;

    #[inline]
    fn from_bytes(raw: &[u8]) -> Result<Self, CanError>
    where
        Self: Sized,
    {
        const MAX_RPM: u16 = 8000;
        const MAX_PEDAL: u8 = 100;
        if let [b0, b1, b2, ..] = raw {
            let rpm_val = u16::from_be_bytes([*b0, *b1]);
            let pedal_val = *b2;

            if rpm_val > MAX_RPM || pedal_val > MAX_PEDAL {
                return Err(CanError::SignalOutOfRange);
            }

            Ok(Self {
                rpm: rpm_val,
                pedal_position: pedal_val,
            })
        } else {
            Err(CanError::BufferTooSmall)
        }
    }

    #[inline]
    fn to_bytes(&self) -> [u8; 8] {
        let rpm_bytes = self.rpm.to_be_bytes();
        [
            rpm_bytes[0],
            rpm_bytes[1],
            self.pedal_position,
            0,
            0,
            0,
            0,
            0,
        ]
    }
}

/// ### ID: `0x300` | `ADAS_Sensor_Data`
/// Transmitted by the Autonomous Driving Sensor Suite (Lidar/Radar controller).
/// Contains the vehicle's current filtered speed and the exact distance to the nearest forward obstacle.
///
/// #### Signal Specifications:
/// | Signal Name | Start Bit | Length | Byte Order | Value Type | Factor | Offset | Range / Unit |
/// | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
/// | `current_speed` | bit 0 | 8 bits | Motorola (BE) | Unsigned | 1.0 | 0 | 0 - 250 km/h |
/// | `obstacle_dist` | bit 8 | 16 bits | Motorola (BE) | Unsigned | 1.0 | 0 | 0 - 20000 cm |
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AdasSensorFrame {
    /// Current speed of the vehicle.
    pub current_speed: u8,
    /// Distance from the nearest forward obstacle.
    pub obstacle_dist: u16,
}

impl AdasSensorFrame {
    /// Returns the current vehicle speed in km/h.
    #[inline]
    #[must_use]
    pub fn current_speed_kmh(&self) -> u8 {
        self.current_speed
    }

    /// Returns the distance to the obstacle expressed in centimeters.
    #[inline]
    #[must_use]
    pub fn obstacle_dist_cm(&self) -> u16 {
        self.obstacle_dist
    }
}

impl CanFrame for AdasSensorFrame {
    const ID: u32 = 0x300;
    const DLC: usize = 3;

    #[inline]
    fn from_bytes(raw: &[u8]) -> Result<Self, CanError>
    where
        Self: Sized,
    {
        const MAX_DIST: u16 = 20000; // 200 metrów w centymetrach
        const MAX_SPEED: u8 = 250;

        if let [b0, b1, b2, ..] = raw {
            let current_speed = *b0;
            let obstacle_dist = u16::from_be_bytes([*b1, *b2]);

            if current_speed > MAX_SPEED || obstacle_dist > MAX_DIST {
                return Err(CanError::SignalOutOfRange);
            }

            Ok(Self {
                current_speed,
                obstacle_dist,
            })
        } else {
            Err(CanError::BufferTooSmall)
        }
    }

    #[inline]
    fn to_bytes(&self) -> [u8; 8] {
        let obstacle_bytes = self.obstacle_dist.to_be_bytes();
        [
            self.current_speed,
            obstacle_bytes[0],
            obstacle_bytes[1],
            0,
            0,
            0,
            0,
            0,
        ]
    }
}


/// ### ID: `0x200` | `AEB_Brake_Command`
/// Broadcasted by the ADAS Safety Core module to the Electronic Stability Control (ESC/ABS).
/// Demands immediate deceleration interventions based on critical Time-To-Collision (TTC) calculations.
///
/// #### Signal Specifications:
/// | Signal Name | Start Bit | Length | Byte Order | Value Type | Factor | Offset | Range / Unit |
/// | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
/// | `brake_intensity` | bit 0 | 8 bits | Motorola (BE) | Unsigned | 1.0 | 0 | 0 - 100 % |
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AebCommandsFrame {
    pub brake_intensity: u8,
}

impl AebCommandsFrame {
    /// Returns the demanded brake intensity as a physical percentage value (0 to 100%).
    #[inline]
    #[must_use]
    pub fn brake_intensity_percentage(&self) -> u8 {
        self.brake_intensity
    }
}

impl CanFrame for AebCommandsFrame {
    const ID: u32 = 0x200;
    const DLC: usize = 1;

    #[inline]
    fn from_bytes(raw: &[u8]) -> Result<Self, CanError>
    where
        Self: Sized,
    {
        const MAX_BRAKE_INTENSITY: u8 = 100;
        if let [b0, ..] = raw {
            let brake_intensity = *b0;
            if brake_intensity > MAX_BRAKE_INTENSITY {
                return Err(CanError::SignalOutOfRange);
            }

            Ok(Self { brake_intensity })
        } else {
            Err(CanError::BufferTooSmall)
        }
    }

    #[inline]
    fn to_bytes(&self) -> [u8; 8] {
        [
            self.brake_intensity,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ]
    }
}
//! CAN Core Module
//! This module contains safe, deterministic control algorithms for the ADAS system.
//! It processes parsed telemetry network frames and generates safety instructions.
//!
use crate::can_matrix::{AdasSensorFrame, AebCommandsFrame};

/// Evaluates the current vehicle and sensor state to determine if emergency braking is needed.
///
/// ### Behavior
/// * Returns `Some(AebCommandsFrame)` with 100% intensity if the vehicle travels above 15km/h
///   **AND** an obstacle is detected within a ctirical threshold of 10 meters(1000cm).
/// * Returns `None` during standard, safe driving conditions (no safety intervention required).
#[inline]
#[must_use]
pub fn process_adas_sensor(frame: &AdasSensorFrame) -> Option<AebCommandsFrame> {
    const MIN_SPEED_TO_ACTIVATE: u8 = 15;
    const MAX_DISTANCE_TO_ACTIVATE: u16 = 1000;
    if frame.current_speed_kmh() > MIN_SPEED_TO_ACTIVATE
        && frame.obstacle_dist_cm() < MAX_DISTANCE_TO_ACTIVATE
    {
        let command_frame = AebCommandsFrame {
            brake_intensity: 100,
        };
        Some(command_frame)
    } else {
        None
    }
}

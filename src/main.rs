use automotive_can::{
    can_matrix::{AbsWheelSpeeds, AdasSensorFrame, AebCommandsFrame, CanFrame, EngineData},
    core::process_adas_sensor,
};
use socketcan::{CanSocket, EmbeddedFrame, Frame, Socket};
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    let socket = CanSocket::open("vcan0")?;
    println!("Successfully bound to vcan0. Awaiting network payloads...");

    loop {
        let frame = socket.read_frame()?;

        if frame.is_data_frame() {
            let id = frame.raw_id();
            let raw_payload = frame.data();

            match id {
                AbsWheelSpeeds::ID => match AbsWheelSpeeds::from_bytes(raw_payload) {
                    Ok(wheel_speeds) => {
                        println!(
                            "[ID: 0x{:X}] FL: {:.2} km/h | FR: {:.2} km/h | RL: {:.2} km/h | RR: {:.2} km/h",
                            id,
                            wheel_speeds.fl_kmh(),
                            wheel_speeds.fr_kmh(),
                            wheel_speeds.rl_kmh(),
                            wheel_speeds.rr_kmh()
                        );
                    }
                    Err(e) => {
                        eprintln!(
                            "[DIAG ERROR]: Malformed payload for ID: 0x{:X}: {:?}",
                            id, e
                        );
                    }
                },
                EngineData::ID => match EngineData::from_bytes(raw_payload) {
                    Ok(engine_data) => {
                        println!(
                            "[ID: 0x{:X}] RPM: {} | PEDAL: {}",
                            id,
                            engine_data.rpm(),
                            engine_data.pedal_position()
                        )
                    }

                    Err(e) => {
                        eprintln!(
                            "[DIAG ERROR]: Malformed payload for ID: 0x{:X}: {:?}",
                            id, e
                        )
                    }
                },

                AdasSensorFrame::ID => match AdasSensorFrame::from_bytes(raw_payload) {
                    Ok(adas_sensor_frame) => {
                        let Some(aeb_frame) = process_adas_sensor(&adas_sensor_frame) else {
                            continue;
                        };

                        let Some(socket_frame) = pack_aeb_frame(&aeb_frame) else {
                            eprintln!(
                                "[CRITICAL FAULT]: AEB processing anomaly - serialization failed!"
                            );
                            continue;
                        };

                        let _ = socket.write_frame(&socket_frame);
                        println!("[SAFETY INTERVENTION]: AEB Command 100% sent to vcan0!");
                    }

                    Err(e) => {
                        eprintln!(
                            "[DIAG ERROR]: Malformed payload for ID: 0x{:X}: {:?}",
                            id, e
                        )
                    }
                },

                _ => {}
            }
        }
    }
}

fn pack_aeb_frame(aeb_frame: &AebCommandsFrame) -> Option<socketcan::CanFrame> {
    let std_id = socketcan::StandardId::new(AebCommandsFrame::ID as u16)?;

    let data_frame =
        socketcan::CanDataFrame::new(std_id, &aeb_frame.to_bytes()[..AebCommandsFrame::DLC])?;

    Some(socketcan::CanFrame::Data(data_frame))
}

#[cfg(test)]
mod tests {
    use super::*;
    use socketcan::Frame;

    #[test]
    fn test_abs_wheel_speeds_bridge_to_socketcan() {
        let test_speeds = AbsWheelSpeeds::from_bytes(&[
            (5000 >> 8) as u8,
            (5000 & 0xFF) as u8, // FL
            (5000 >> 8) as u8,
            (5000 & 0xFF) as u8, // FR
            (5000 >> 8) as u8,
            (5000 & 0xFF) as u8, // RL
            (5000 >> 8) as u8,
            (5000 & 0xFF) as u8, // RR
        ])
        .unwrap();

        let std_id = socketcan::StandardId::new(AbsWheelSpeeds::ID as u16).unwrap();
        let data_frame = socketcan::CanDataFrame::new(std_id, &test_speeds.to_bytes()).unwrap();
        let final_frame = socketcan::CanFrame::Data(data_frame);

        assert_eq!(final_frame.raw_id(), AbsWheelSpeeds::ID);
        assert_eq!(final_frame.data().len(), 8);

        let decoded = AbsWheelSpeeds::from_bytes(final_frame.data()).unwrap();

        assert_eq!(decoded.fl_kmh(), 50.00);
        assert_eq!(decoded.fr_kmh(), 50.00);
        assert_eq!(decoded.rl_kmh(), 50.00);
        assert_eq!(decoded.rr_kmh(), 50.00);
    }

    #[test]
    fn test_pack_aeb_frame_helper_success() {
        let aeb_command = AebCommandsFrame {
            brake_intensity: 100,
        };

        let packed_result = pack_aeb_frame(&aeb_command);

        assert!(packed_result.is_some());
        let socket_frame = packed_result.unwrap();

        assert_eq!(socket_frame.raw_id(), AebCommandsFrame::ID);
        assert_eq!(socket_frame.data(), &[100]);
    }
}

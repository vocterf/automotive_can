use automotive_can::can_matrix::{AbsWheelSpeeds, CanFrame};
use socketcan::{CanSocket, Frame, Socket};
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    let socket = CanSocket::open("vcan0")?;
    println!("Successfully bound to vcan0. Awaiting network payloads...");

    loop {
        let frame = socket.read_frame()?;

        if frame.is_data_frame() {
            let id = frame.raw_id();

            if id == AbsWheelSpeeds::ID {
                let raw_payload = frame.data();

                match AbsWheelSpeeds::from_bytes(raw_payload) {
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
                }
            }
        }
    }
}

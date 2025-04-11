use std::{error::Error, thread, time::Duration};

use rustypot::device::l0_force_fan;
use rustypot::DynamixelProtocolHandler;

fn main() -> Result<(), Box<dyn Error>> {
    let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
        .timeout(Duration::from_millis(100))
        .open()?;

    let io = DynamixelProtocolHandler::v1();

    l0_force_fan::write_fan3_state(&io, serial_port.as_mut(), 40, 0)?;

    loop {
        let pos = l0_force_fan::read_present_load(&io, serial_port.as_mut(), 40)?;
        println!("{:?}", pos);

        thread::sleep(Duration::from_millis(5));
    }
}

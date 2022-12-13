use std::{error::Error, thread, time::Duration};

use rustypot::{protocol, DynamixelSerialIO};

fn main() -> Result<(), Box<dyn Error>> {
    let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
        .timeout(Duration::from_millis(10))
        .open()?;

    let io = DynamixelSerialIO::new::<protocol::V1>();

    loop {
        let pos = io.read(serial_port.as_mut(), 40, 0x10, 4);
        println!("{:?}", pos);

        thread::sleep(Duration::from_millis(10));
    }
}

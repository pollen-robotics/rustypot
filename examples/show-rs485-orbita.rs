use std::{error::Error, thread, time::Duration};

use rustypot::device::mx;
use rustypot::{protocol, DynamixelSerialIO};

fn main() -> Result<(), Box<dyn Error>> {
    let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
        .timeout(Duration::from_millis(10))
        .open()?;

    let io = DynamixelSerialIO::new::<protocol::V1>();

    let _x: i16 = mx::read_present_position(&io, serial_port.as_mut(), 11)?;
    let _x: Vec<i16> = mx::sync_read_present_position(&io, serial_port.as_mut(), &[11, 12])?;

    mx::write_goal_position(&io, serial_port.as_mut(), 11, 0)?;
    mx::sync_write_goal_position(&io, serial_port.as_mut(), &[11, 12], &[0, 2048])?;

    loop {
        let pos = io.read(serial_port.as_mut(), 40, 0x10, 4);
        println!("{:?}", pos);

        thread::sleep(Duration::from_millis(10));
    }
}

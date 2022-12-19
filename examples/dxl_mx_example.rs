use std::{error::Error, thread, time::Duration};

use rustypot::device::mx;
use rustypot::DynamixelSerialIO;

fn main() -> Result<(), Box<dyn Error>> {
    let mut serial_port = serialport::new("/dev/tty.usbmodem142401", 1_000_000)
        .timeout(Duration::from_millis(10))
        .open()?;

    let io = DynamixelSerialIO::v1();

    let _x: i16 = mx::read_present_position(&io, serial_port.as_mut(), 11)?;
    let _x: Vec<i16> = mx::sync_read_present_position(&io, serial_port.as_mut(), &[11, 12])?;

    mx::sync_write_goal_position(&io, serial_port.as_mut(), &[11, 12], &[2048, 2048])?;

    loop {
        mx::sync_write_goal_position(&io, serial_port.as_mut(), &[11], &[2048])?;

        let temp = mx::read_present_temperature(&io, serial_port.as_mut(), 11)?;
        println!("{:?}", temp);

        thread::sleep(Duration::from_millis(500));

        mx::sync_write_goal_position(&io, serial_port.as_mut(), &[11], &[1000])?;
        let pos = mx::read_present_position(&io, serial_port.as_mut(), 11)?;
        println!("{:?}", pos);

        thread::sleep(Duration::from_millis(500));
    }
}

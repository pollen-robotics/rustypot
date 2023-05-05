use std::{error::Error, thread, time::Duration};

use rustypot::device::mx;
use rustypot::DynamixelSerialIO;

const ID_DEVICE: u8 = 1;

fn main() -> Result<(), Box<dyn Error>> {
    println!("Hi there! [dxl_mx_example]");
    let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
        .timeout(Duration::from_millis(50))
        .open()?;

    let io = DynamixelSerialIO::v1();

    let _x: i16 = mx::read_present_position(&io, serial_port.as_mut(), ID_DEVICE)?;

    loop {
        let temp = mx::read_present_temperature(&io, serial_port.as_mut(), ID_DEVICE)?;
        println!("{:?}", temp);

        thread::sleep(Duration::from_millis(500));
    }
}


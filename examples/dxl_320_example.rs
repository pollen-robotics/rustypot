use std::{error::Error, thread, time::Duration};

use rustypot::servo::dynamixel::xl320;
use rustypot::DynamixelProtocolHandler;

fn main() -> Result<(), Box<dyn Error>> {
    env_logger::init();

    let mut serial_port = serialport::new("/dev/ttyACM0", 1_000_000)
        .timeout(Duration::from_millis(10))
        .open()?;

    let io = DynamixelProtocolHandler::v2();

    loop {
        // println!("PING {:?}", io.ping(serial_port.as_mut(), 30));

        // let temp = xl320::read_present_temperature(&io, serial_port.as_mut(), 30)?;
        // println!("{:?}", temp);

        // let pos = xl320::read_present_position(&io, serial_port.as_mut(), 30)?;
        // println!("{:?}", pos);

        // let pos = xl320::read_present_position(&io, serial_port.as_mut(), 31)?;
        // println!("{:?}", pos);

        // let pos = xl320::sync_read_present_position(&io, serial_port.as_mut(), &[30, 31])?;
        // println!("{:?}", pos);

        // let temp = xl320::sync_read_present_temperature(&io, serial_port.as_mut(), &[30, 31])?;
        // println!("{:?}", temp);

        xl320::sync_write_goal_position(&io, serial_port.as_mut(), &[30, 31], &[0, 512])?;
        thread::sleep(Duration::from_millis(500));

        xl320::sync_write_goal_position(&io, serial_port.as_mut(), &[31, 30], &[0, 512])?;
        thread::sleep(Duration::from_millis(500));
    }
}

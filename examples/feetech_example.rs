use std::{error::Error, thread, time::Duration};

use rustypot::{device::feetech_STS3215, DynamixelSerialIO};

fn main() -> Result<(), Box<dyn Error>> {
    let serialportname: String = "/dev/tty.usbmodem58FD0164681".to_string();
    let baudrate: u32 = 1_000_000;
    let id = 1;

    let mut serial_port = serialport::new(serialportname, baudrate)
        .timeout(Duration::from_millis(10))
        .open()?;
    println!("serial port opened");

    let io = DynamixelSerialIO::feetech();

    loop {
        let x: i16 = feetech_STS3215::read_present_position(&io, serial_port.as_mut(), id)?;
        println!("present pos: {}", x);

        thread::sleep(Duration::from_millis(100));
    }

    Ok(())
}

use std::{error::Error, thread, time::Duration};

use rustypot::DynamixelSerialIO;

fn main() -> Result<(), Box<dyn Error>> {
    let mut io = DynamixelSerialIO::new("/dev/ttyACM0", 500_000, Duration::from_millis(100))?;

    println!("{:?}", io.ping(40)?);

    loop {
        let pos: i32 = io.read_data(40, 0x10)?;
        println!("{:?}", pos);

        thread::sleep(Duration::from_millis(10));
    }
}

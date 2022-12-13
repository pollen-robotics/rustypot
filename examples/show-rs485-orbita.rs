use std::{error::Error, thread, time::Duration};

use rustypot::DynamixelSerialIO;

fn main() -> Result<(), Box<dyn Error>> {
    let mut io = DynamixelSerialIO::new("/dev/ttyACM0", 1_000_000, Duration::from_millis(100))?;

    loop {
        let pos: Vec<u16> = io.sync_read_data(vec![11, 12], 36)?;
        println!("{:?}", pos);

        thread::sleep(Duration::from_millis(10));
    }
}

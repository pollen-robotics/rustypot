use std::f32::consts::PI;
use std::time::SystemTime;
use std::{error::Error, thread, time::Duration};

use rustypot::device::orbita_foc::{self, DiskValue};
use rustypot::DynamixelSerialIO;

fn main() -> Result<(), Box<dyn Error>> {
    let mut serial_port = serialport::new("/dev/ttyUSB0", 1_000_000)
        .timeout(Duration::from_millis(20))
        .open()?;

    let io = DynamixelSerialIO::v1();

    let id = 42;

    let now = SystemTime::now();
    // let x = io.ping(serial_port.as_mut(), id);
    // println!("{:?}", x);
    loop {
        // let x = io.ping(serial_port.as_mut(), id);
        // println!("{:?}", x);

        let pos = orbita_foc::read_present_position(&io, serial_port.as_mut(), id)?;
        println!("{:?}", pos);

        let t = now.elapsed().unwrap().as_secs_f32();
        let target = 60.0_f32 * (2.0 * PI * 0.5 * t).sin();
        orbita_foc::write_top_goal_position(&io, serial_port.as_mut(), id, target)?;
        // println!("{}", t);

        orbita_foc::write_goal_position(
            &io,
            serial_port.as_mut(),
            id,
            DiskValue {
                top: target,
                middle: target,
                bottom: target,
            },
        )?;

        thread::sleep(Duration::from_millis(10));
    }
}

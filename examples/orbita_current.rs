use std::f32::consts::PI;
use std::time::SystemTime;
use std::{error::Error, thread, time::Duration};

use rustypot::device::orbita_foc::{self, DiskValue};
use rustypot::DynamixelSerialIO;

fn main() -> Result<(), Box<dyn Error>> {
    env_logger::init();
    let mut serial_port = serialport::new("/dev/ttyUSB0", 1_000_000)
        .timeout(Duration::from_millis(20))
        .open()?;

    let io = DynamixelSerialIO::v1();

    let id = 70;

    let now = SystemTime::now();
    // let x = io.ping(serial_port.as_mut(), id);
    // println!("{:?}", x);

    orbita_foc::write_torque_enable(&io, serial_port.as_mut(), id, 1)?;
    let mot_driv_state = orbita_foc::read_motors_drivers_states(&io, serial_port.as_mut(), id)?;
    println!("motors/drivers states: {:#010b}", mot_driv_state); // 10 chars for u8 since it integers "0x"
    let init_pos = orbita_foc::read_present_position(&io, serial_port.as_mut(), id)?;

    println!("init_pos: {:?}", init_pos);
    // let reset = 0;
    loop {
        // let x = io.ping(serial_port.as_mut(), id);
        // println!("{:?}", x);

        // let pos = orbita_foc::read_present_position(&io, serial_port.as_mut(), id)?;

        // let pos = orbita_foc::read_present_position(&io, serial_port.as_mut(), id)?;
        let tcura = orbita_foc::read_top_current_a(&io, serial_port.as_mut(), id)?;
        let tcurb = orbita_foc::read_top_current_b(&io, serial_port.as_mut(), id)?;
        let mcura = orbita_foc::read_mid_current_a(&io, serial_port.as_mut(), id)?;
        let mcurb = orbita_foc::read_mid_current_b(&io, serial_port.as_mut(), id)?;
        let bcura = orbita_foc::read_bot_current_a(&io, serial_port.as_mut(), id)?;
        let bcurb = orbita_foc::read_bot_current_b(&io, serial_port.as_mut(), id)?;

        println!("tcura {tcura:>8.*?} tcurb {tcurb:>8.*} mcura {mcura:>8.*?} mcurb {mcurb:>8.*} bcura {bcura:>8.*?} bcurb {bcurb:>8.*}", 3, 3,3,3,3,3);
        thread::sleep(Duration::from_millis(10));
    }
}

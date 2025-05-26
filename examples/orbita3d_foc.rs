use std::f32::consts::PI;
use std::time::SystemTime;
use std::{error::Error, thread, time::Duration};

use rustypot::servo::orbita::orbita3d_foc::{self, DiskValue};
use rustypot::DynamixelProtocolHandler;

fn main() -> Result<(), Box<dyn Error>> {
    let mut serial_port = serialport::new("/dev/ttyUSB0", 1_000_000)
        .timeout(Duration::from_millis(100))
        .open()?;

    let io = DynamixelProtocolHandler::v1().with_post_delay(Duration::from_millis(1));

    let id = 70;
    let mut state = orbita3d_foc::read_motors_drivers_states(&io, serial_port.as_mut(), id)?;
    println!("state {:#010b}", state);
    println!("state {:?}", state);

    let fw = orbita3d_foc::read_firmware_version(&io, serial_port.as_mut(), id)?;
    println!("Firmware version {:?}", fw);

    orbita3d_foc::write_torque_enable(&io, serial_port.as_mut(), id, 1)?;
    thread::sleep(Duration::from_millis(1000));
    // orbita3d_foc::write_torque_enable(&io, serial_port.as_mut(), id, 0)?;

    // thread::sleep(Duration::from_millis(1000));
    // orbita3d_foc::write_torque_enable(&io, serial_port.as_mut(), id, 1)?;

    // thread::sleep(Duration::from_millis(1000));

    state = orbita3d_foc::read_motors_drivers_states(&io, serial_port.as_mut(), id)?;
    println!("state {:#010b}", state);
    let now = SystemTime::now();
    // let x = io.ping(serial_port.as_mut(), id);
    // println!("{:?}", x);
    let mut total = 0.0;

    while total < 10.0 {
        // let x = io.ping(serial_port.as_mut(), id);
        // println!("{:?}", x);

        let pos = orbita3d_foc::read_present_position(&io, serial_port.as_mut(), id)?;

        let tophall = orbita3d_foc::read_top_present_hall(&io, serial_port.as_mut(), id)?;
        let midhall = orbita3d_foc::read_middle_present_hall(&io, serial_port.as_mut(), id)?;
        let bothall = orbita3d_foc::read_bottom_present_hall(&io, serial_port.as_mut(), id)?;
        println!(
            "{:?} tophall: {:?} midhal: {:?} bothall: {:?}",
            pos, tophall, midhall, bothall
        );

        let t = now.elapsed().unwrap().as_secs_f32();
        let target = 10.0_f32 * (2.0 * PI * 0.25 * t).sin();

        orbita3d_foc::write_goal_position(
            &io,
            serial_port.as_mut(),
            id,
            DiskValue {
                top: target.to_radians() + pos.top,
                middle: target.to_radians() + pos.middle,
                bottom: target.to_radians() + pos.bottom,
                // top: pos.top,
                // middle: pos.middle,
                // bottom: pos.bottom,
            },
        )?;
        thread::sleep(Duration::from_millis(10));
        total += 0.01;
    }
    orbita3d_foc::write_torque_enable(&io, serial_port.as_mut(), id, 0)?;

    thread::sleep(Duration::from_millis(1000));
    Ok(())
}

use std::f32::consts::PI;
use std::time::SystemTime;
use std::{error::Error, thread, time::Duration};

use clap::Parser;
use rustypot::device::orbita_foc::{self, DiskValue};
use rustypot::DynamixelSerialIO;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// tty
    #[arg(short, long)]
    serialport: String,
    /// baud
    #[arg(short, long, default_value_t = 1_000_000)]
    baudrate: u32,
}

fn main() -> Result<(), Box<dyn Error>> {
    env_logger::init();
    let args = Args::parse();
    let serialportname: String = args.serialport;
    let baudrate: u32 = args.baudrate;
    //print all the argument values
    println!("serialport: {}", serialportname);
    println!("baudrate: {}", baudrate);
    let mut serial_port = serialport::new(serialportname, baudrate)
        .timeout(Duration::from_millis(20))
        .open()?;

    let io = DynamixelSerialIO::v1();

    for id in 1..254 {
        let x = io.ping(serial_port.as_mut(), id);
        println!("{id} {:?}", x);
        thread::sleep(Duration::from_millis(5));
        // match x {
        //     Ok(v) => {
        //         if v == true {
        //             break;
        //         }
        //     }
        //     Err(_) => {}
        // }
    }
    Ok(())

    // let id = 70;

    // let now = SystemTime::now();
    // let x = io.ping(serial_port.as_mut(), id);
    // println!("{:?}", x);

    // // orbita_foc::write_torque_enable(&io, serial_port.as_mut(), id, 1)?;
    // let mot_driv_state = orbita_foc::read_motors_drivers_states(&io, serial_port.as_mut(), id)?;
    // println!("motors/drivers states: {:#010b}", mot_driv_state); // 10 chars for u8 since it integers "0x"
    // let init_pos = orbita_foc::read_present_position(&io, serial_port.as_mut(), id)?;

    // println!("init_pos: {:?}", init_pos);
    // thread::sleep(Duration::from_millis(3000));
    // // let reset = 0;
    // loop {
    //     // let x = io.ping(serial_port.as_mut(), id);
    //     // println!("{:?}", x);

    //     // let pos = orbita_foc::read_present_position(&io, serial_port.as_mut(), id)?;

    //     let t = now.elapsed().unwrap().as_secs_f32();
    //     let target = 1.0_f32 * (2.0 * PI * 0.12 * t).sin(); // large slow complete sinus
    //                                                         // let target = 4.267 * 180.0_f32.to_radians() * (2.0 * PI * 0.1 * t).sin(); // small fast complete sinus
    //                                                         //                                                                           //        let target = 1.0_f32 * (2.0 * PI * 10.0 * t).sin(); // incredible shaky Orbita
    //                                                         // println!(
    //                                                         //     "{:?} {:?} | disks {:?} {:?} {:?}",
    //                                                         //     t,
    //                                                         //     target,
    //                                                         //     pos.top.to_degrees() / (64.0 / 15.0),
    //                                                         //     pos.middle.to_degrees() / (64.0 / 15.0),
    //                                                         //     pos.bottom.to_degrees() / (64.0 / 15.0)
    //                                                         // );
    //                                                         // orbita_foc::write_top_goal_position(&io, serial_port.as_mut(), id, target)?;
    //                                                         // println!("{}", t);

    //     // orbita_foc::write_goal_position(
    //     //     &io,
    //     //     serial_port.as_mut(),
    //     //     id,
    //     //     DiskValue {
    //     //         top: init_pos.top + target,
    //     //         middle: init_pos.middle + target,
    //     //         bottom: init_pos.bottom + target,
    //     //     },
    //     // )?;

    //     let pos = orbita_foc::read_present_position(&io, serial_port.as_mut(), id)?;
    //     println!(
    //         "top {:.3} mid {:.3} bot {:.3}",
    //         pos.top, pos.middle, pos.bottom
    //     );
    //     // let state = orbita_foc::read_motors_drivers_states(&io, serial_port.as_mut(), id)?;
    //     // println!("state: {:?}", state);

    //     //        thread::sleep(Duration::from_millis(10));
    //     //        thread::sleep(Duration::from_micros(10));

    //     // let target = 4.267 * 90.0_f32.to_radians();

    //     // if t < 5.0 {
    //     //     orbita_foc::write_goal_position(
    //     //         &io,
    //     //         serial_port.as_mut(),
    //     //         id,
    //     //         DiskValue {
    //     //             top: init_pos.top + target,
    //     //             middle: init_pos.middle + target,
    //     //             bottom: init_pos.bottom + target,
    //     //         },
    //     //     )?;
    //     // }
    //     // // thread::sleep(Duration::from_millis(3000));
    //     // else {
    //     //     orbita_foc::write_goal_position(
    //     //         &io,
    //     //         serial_port.as_mut(),
    //     //         id,
    //     //         DiskValue {
    //     //             top: init_pos.top,
    //     //             middle: init_pos.middle,
    //     //             bottom: init_pos.bottom,
    //     //         },
    //     //     )?;
    //     // }
    //     // thread::sleep(Duration::from_millis(3000));
    // }
}

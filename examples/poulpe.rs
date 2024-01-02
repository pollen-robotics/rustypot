use std::f32::consts::PI;
use std::time::SystemTime;
use std::{error::Error, thread, time::Duration};

use rustypot::device::poulpe::{self};
use rustypot::DynamixelSerialIO;

use clap::Parser;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// tty
    #[arg(short, long)]
    serialport: String,
    /// baud
    #[arg(short, long, default_value_t = 1_000_000)]
    baudrate: u32,

    /// id
    #[arg(short, long)]
    id: u8,

    ///sinus amplitude (f64)
    #[arg(short, long, default_value_t = 10.0)]
    amplitude: f32,

    ///sinus frequency (f64)
    #[arg(short, long, default_value_t = 1.0)]
    frequency: f32,
}

const MOTOR_A: u8 = 42;

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let serialportname: String = args.serialport;
    let baudrate: u32 = args.baudrate;
    let id: u8 = args.id;
    let amplitude: f32 = args.amplitude;
    let frequency: f32 = args.frequency;

    //print the standard ids for the arm motors

    //print all the argument values
    println!("serialport: {}", serialportname);
    println!("baudrate: {}", baudrate);
    println!("id: {}", id);
    println!("amplitude: {}", amplitude);
    println!("frequency: {}", frequency);

    let mut serial_port = serialport::new(serialportname, baudrate)
        .timeout(Duration::from_millis(100))
        .open()?;

    let now = SystemTime::now();

    let io = DynamixelSerialIO::v1();

    // Ping
    let x = io.ping(serial_port.as_mut(), id);
    println!("Ping : {:?}", x);

    let mut t = now.elapsed().unwrap().as_secs_f32();
    loop {
        if t > 10.0 {
            break;
        }

        // let x = io.ping(serial_port.as_mut(), id);
        // println!("Ping : {:?}", x);

        t = now.elapsed().unwrap().as_secs_f32();
        let target = amplitude * 180.0_f32.to_radians() * (2.0 * PI * 0.5 * t).sin();

        let pos = poulpe::read_motor_a_present_position(&io, serial_port.as_mut(), id)?;
        thread::sleep(Duration::from_micros(1000));
        // let target = poulpe::read_motor_a_goal_position(&io, serial_port.as_mut(), id)?;
        let _ = poulpe::write_motor_a_goal_position(&io, serial_port.as_mut(), id, target as i32);
        println!("target: {} pos: {}", target, pos);

        thread::sleep(Duration::from_millis(1));
    }

    // orbita2dof_foc::write_torque_enable(&io, serial_port.as_mut(), id, 0x00)?;
    // let reg = orbita2dof_foc::read_torque_enable(&io, serial_port.as_mut(), id)?;
    // if reg == 0x01 {
    //     println!("Motor on");
    // } else {
    //     println!("Motor off");
    // }

    thread::sleep(Duration::from_millis(2000));

    Ok(())
}

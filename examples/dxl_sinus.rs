use std::f64::consts::PI;
use std::time::SystemTime;
use std::{error::Error, thread, time::Duration};

use rustypot::servo::dynamixel::mx;
use rustypot::DynamixelProtocolHandler;

use clap::Parser;

use signal_hook::flag;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

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
    amplitude: f64,

    ///sinus frequency (f64)
    #[arg(short, long, default_value_t = 1.0)]
    frequency: f64,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let serialportname: String = args.serialport;
    let baudrate: u32 = args.baudrate;
    let id: u8 = args.id;
    let amplitude: f64 = args.amplitude;
    let frequency: f64 = args.frequency;

    //print all the argument values
    println!("serialport: {}", serialportname);
    println!("baudrate: {}", baudrate);
    println!("id: {}", id);
    println!("amplitude: {}", amplitude);
    println!("frequency: {}", frequency);
    let term = Arc::new(AtomicBool::new(false));

    flag::register(signal_hook::consts::SIGINT, Arc::clone(&term))?;

    let mut serial_port = serialport::new(serialportname, baudrate)
        .timeout(Duration::from_millis(10))
        .open()?;
    println!("serial port opened");

    let io = DynamixelProtocolHandler::v1();

    let x = mx::read_present_position(&io, serial_port.as_mut(), id)?;
    println!("present pos: {}", x);

    mx::write_torque_enable(&io, serial_port.as_mut(), id, 1)?;

    let now = SystemTime::now();
    while !term.load(Ordering::Relaxed) {
        let t = now.elapsed().unwrap().as_secs_f64();
        let target = amplitude * (2.0 * PI * frequency * t).sin().to_radians();
        println!("target: {}", target);
        mx::write_goal_position(&io, serial_port.as_mut(), id, target)?;

        thread::sleep(Duration::from_millis(10));
    }
    // mx::write_torque_enable(&io, serial_port.as_mut(), id, false)?;
    mx::write_torque_enable(&io, serial_port.as_mut(), id, 0)?;

    Ok(())
}

use std::f32::consts::PI;
use std::time::SystemTime;
use std::{error::Error, thread, time::Duration, time::Instant};

use rustypot::device::orbita3d_poulpe::{self, MotorValue};
use rustypot::DynamixelSerialIO;

use clap::Parser;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// tty
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    serialport: String,
    /// baud
    #[arg(short, long, default_value_t = 2_000_000)]
    baudrate: u32,

    /// id
    #[arg(short, long, default_value_t = 42)]
    id: u8,

    ///sinus amplitude (f64)
    #[arg(short, long, default_value_t = 10.0)]
    amplitude: f32,

    ///sinus frequency (f64)
    #[arg(short, long, default_value_t = 1.0)]
    frequency: f32,
}



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

    let _ = orbita3d_poulpe::write_torque_enable(&io, serial_port.as_mut(), id, MotorValue::<bool>{top:true, middle:true, bottom:true})?;
    thread::sleep(Duration::from_millis(1000));
    let torque = orbita3d_poulpe::read_torque_enable(&io, serial_port.as_mut(), id)?;
    println!("torque: {:?}", torque);
    thread::sleep(Duration::from_millis(1000));


    let curr_pos= orbita3d_poulpe::read_current_position(&io, serial_port.as_mut(), id)?;


    let mut t = now.elapsed().unwrap().as_secs_f32();
    loop {
	let t0 = Instant::now();

        if t > 10.0 {
            break;
        }

        // let x = io.ping(serial_port.as_mut(), id);
        // println!("Ping : {:?}", x);

        t = now.elapsed().unwrap().as_secs_f32();
        let target = amplitude * 180.0_f32.to_radians() * (2.0 * PI * 0.5 * t).sin();

        let feedback = orbita3d_poulpe::write_target_position(&io, serial_port.as_mut(), id, MotorValue::<f32>{top:target+curr_pos.top, middle:target+curr_pos.middle, bottom:target+curr_pos.bottom})?;


        println!("target: {} feedback pos: {} {} {} feedback vel: {} {} {} feedback torque: {} {} {} ", target, feedback.position.top,feedback.position.middle,feedback.position.bottom,feedback.speed.top,feedback.speed.middle,feedback.speed.bottom,feedback.load.top,feedback.load.middle,feedback.load.bottom);

	println!("ELAPSED: {:?}",t0.elapsed().as_micros());
	// thread::sleep(Duration::from_micros(1000-t0.elapsed().as_micros() as u64));
        thread::sleep(Duration::from_millis(1));
    }


    let _ = orbita3d_poulpe::write_torque_enable(&io, serial_port.as_mut(), id, MotorValue::<bool>{top:false, middle:false, bottom:false})?;


    thread::sleep(Duration::from_millis(2000));

    Ok(())
}

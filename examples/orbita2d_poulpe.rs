use std::time::SystemTime;
use std::{error::Error, thread, time::Duration, time::Instant};

use rustypot::servo::orbita::orbita2d_poulpe::{self, MotorValue};
// use rustypot::device::orbita3d_poulpe::{self, MotorValue};
use rustypot::DynamixelProtocolHandler;

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
    #[arg(short, long, default_value_t = 43)]
    id: u8,

    ///sinus amplitude (f64)
    #[arg(short, long, default_value_t = 1.0)]
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
        .timeout(Duration::from_millis(10))
        .open()?;

    let now = SystemTime::now();

    let io = DynamixelProtocolHandler::v1();

    // Ping
    let x = io.ping(serial_port.as_mut(), id);
    println!("Ping {:?}: {:?}", id, x);
    thread::sleep(Duration::from_millis(100));

    let _ = orbita2d_poulpe::write_torque_enable(
        &io,
        serial_port.as_mut(),
        id,
        MotorValue::<bool> {
            motor_a: true,
            motor_b: true,
        },
    )?;
    thread::sleep(Duration::from_millis(1000));
    // let torque = orbita3d_poulpe::read_torque_enable(&io, serial_port.as_mut(), id)?;
    // println!("torque: {:?}", torque);
    // thread::sleep(Duration::from_millis(1000));

    let curr_pos = orbita2d_poulpe::read_current_position(&io, serial_port.as_mut(), id)?;

    println!("curr_pos: {:?} {:?}", curr_pos.motor_a, curr_pos.motor_b);

    // let index_sensor = orbita3d_poulpe::read_index_sensor(&io, serial_port.as_mut(), id)?;
    // println!("index_sensor: {:?} {:?} {:?}", index_sensor.top, index_sensor.middle, index_sensor.bottom);

    let mut t = now.elapsed().unwrap().as_secs_f32();
    let mut nberr = 0;
    let mut nbtot = 0;
    let mut nbok = 0;
    let mut target = 0.0;
    loop {
        let t0 = Instant::now();

        if t > 3.0 {
            break;
        }

        t = now.elapsed().unwrap().as_secs_f32();
        // let target = amplitude * 180.0_f32.to_radians() * (2.0 * PI * 0.5 * t).sin();
        target += 0.001;
        // let feedback = orbita2d_poulpe::write_target_position(&io, serial_port.as_mut(), id, MotorValue::<f32>{motor_a:target+curr_pos.motor_a, motor_b:target+curr_pos.motor_b});

        // let feedback = orbita2d_poulpe::write_target_position(
        //     &io,
        //     serial_port.as_mut(),
        //     id,
        //     MotorValue::<f32> {
        //         motor_a: target + curr_pos.motor_a,
        //         motor_b: curr_pos.motor_b,
        //     },
        // );

        let feedback = orbita2d_poulpe::write_target_position_fb(
            &io,
            serial_port.as_mut(),
            id,
            MotorValue::<f32> {
                motor_a: target + curr_pos.motor_a,
                motor_b: target + curr_pos.motor_b,
            },
        );

        match feedback {
            Ok(feedback) => {
                nbok += 1;
                println!(
                    "42 target: {} feedback pos: {} {}",
                    target, feedback.position.motor_a, feedback.position.motor_b,
                );
            }
            Err(e) => {
                nberr += 1;
                println!("error: {:?}", e);
            }
        }

        nbtot += 1;

        // thread::sleep(Duration::from_micros(1000-t0.elapsed().as_micros() as u64));
        // thread::sleep(Duration::from_millis(1));
        thread::sleep(Duration::from_micros(500));
        // thread::sleep(Duration::from_micros(4500));
        println!("ELAPSED: {:?}", t0.elapsed().as_micros());
    }

    println!(
        "nberr: {} nbtot: {} nbok: {} ratio: {:?}",
        nberr,
        nbtot,
        nbok,
        nbok as f32 / nbtot as f32
    );

    println!("STOP");
    let _ = orbita2d_poulpe::write_torque_enable(
        &io,
        serial_port.as_mut(),
        id,
        MotorValue::<bool> {
            motor_a: false,
            motor_b: false,
        },
    )?;

    thread::sleep(Duration::from_millis(2000));

    Ok(())
}

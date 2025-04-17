use std::f32::consts::PI;
use std::time::SystemTime;
use std::{error::Error, thread, time::Duration, time::Instant};

use rustypot::servo::orbita::orbita3d_poulpe::{self, MotorValue};
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
        .timeout(Duration::from_millis(10))
        .open()?;

    let now = SystemTime::now();

    let io = DynamixelProtocolHandler::v1();

    // Ping
    let x = io.ping(serial_port.as_mut(), id);
    println!("Ping {:?}: {:?}", id, x);

    let _ = orbita3d_poulpe::write_torque_enable(
        &io,
        serial_port.as_mut(),
        id,
        MotorValue::<bool> {
            top: true,
            middle: true,
            bottom: true,
        },
    )?;
    // let _ = orbita3d_poulpe::write_torque_enable(&io, serial_port.as_mut(), id, MotorValue::<bool>{top:false, middle:false, bottom:false})?;
    thread::sleep(Duration::from_millis(1000));
    // let torque = orbita3d_poulpe::read_torque_enable(&io, serial_port.as_mut(), id)?;
    // println!("torque: {:?}", torque);
    // thread::sleep(Duration::from_millis(1000));

    let curr_pos = orbita3d_poulpe::read_current_position(&io, serial_port.as_mut(), id)?;

    // let index_sensor = orbita3d_poulpe::read_index_sensor(&io, serial_port.as_mut(), id)?;
    // println!("index_sensor: {:?} {:?} {:?}", index_sensor.top, index_sensor.middle, index_sensor.bottom);

    let limit = orbita3d_poulpe::read_velocity_limit(&io, serial_port.as_mut(), id)?;
    println!(
        "vel_limit: {:?} {:?} {:?}",
        limit.top, limit.middle, limit.bottom
    );
    let limit = orbita3d_poulpe::read_torque_flux_limit(&io, serial_port.as_mut(), id)?;
    println!(
        "tf_limit: {:?} {:?} {:?}",
        limit.top, limit.middle, limit.bottom
    );
    let limit = orbita3d_poulpe::read_uq_ud_limit(&io, serial_port.as_mut(), id)?;
    println!(
        "uq_ud_limit: {:?} {:?} {:?}",
        limit.top, limit.middle, limit.bottom
    );

    let pid = orbita3d_poulpe::read_flux_pid(&io, serial_port.as_mut(), id)?;
    println!("flux_pid: {:?} {:?} {:?}", pid.top, pid.middle, pid.bottom);
    let pid = orbita3d_poulpe::read_torque_pid(&io, serial_port.as_mut(), id)?;
    println!(
        "torque_pid: {:?} {:?} {:?}",
        pid.top, pid.middle, pid.bottom
    );
    let pid = orbita3d_poulpe::read_velocity_pid(&io, serial_port.as_mut(), id)?;
    println!(
        "velocity_pid: {:?} {:?} {:?}",
        pid.top, pid.middle, pid.bottom
    );
    let pid = orbita3d_poulpe::read_position_pid(&io, serial_port.as_mut(), id)?;
    println!(
        "position_pid: {:?} {:?} {:?}",
        pid.top, pid.middle, pid.bottom
    );

    let mut t = now.elapsed().unwrap().as_secs_f32();
    let mut nberr = 0;
    let mut nbtot = 0;
    let mut nbok = 0;

    loop {
        let t0 = Instant::now();

        if t > 100.0 {
            break;
        }

        t = now.elapsed().unwrap().as_secs_f32();
        let target = amplitude * 180.0_f32.to_radians() * (2.0 * PI * 0.5 * t).sin();

        let feedback = orbita3d_poulpe::write_target_position_fb(
            &io,
            serial_port.as_mut(),
            id,
            MotorValue::<f32> {
                top: target + curr_pos.top,
                middle: target + curr_pos.middle,
                bottom: target + curr_pos.bottom,
            },
        );
        match feedback {
            Ok(feedback) => {
                nbok += 1;
                println!(
                    "42 target: {} feedback pos: {} {} {}",
                    target,
                    feedback.position.top,
                    feedback.position.middle,
                    feedback.position.bottom,
                );
            }
            Err(e) => {
                nberr += 1;
                println!("error: {:?}", e);
            }
        }

        // thread::sleep(Duration::from_micros(500));
        // let feedback = orbita3d_poulpe::write_target_position(&io, serial_port.as_mut(), 43, MotorValue::<f32>{top:0.0, middle:0.0, bottom:0.0});
        // // let feedback = orbita3d_poulpe::write_target_position(&io, serial_port.as_mut(), 43, MotorValue::<f32>{top:0.0, middle:0.0, bottom:0.0}).unwrap_or_else(MotorPositionSpeedLoad::{position:MotorValue::<f32>{top:0.0, middle:0.0, bottom:0.0}, speed:MotorValue::<f32>{top:0.0, middle:0.0, bottom:0.0}, load:MotorValue::<f32>{top:0.0, middle:0.0, bottom:0.0}});
        // match feedback {
        //     Ok(feedback) => {
        // 	nbok+=1;
        // 	println!("43 target: {} feedback pos: {} {} {} feedback vel: {} {} {} feedback torque: {} {} {} ", target, feedback.position.top,feedback.position.middle,feedback.position.bottom,feedback.speed.top,feedback.speed.middle,feedback.speed.bottom,feedback.load.top,feedback.load.middle,feedback.load.bottom);
        // 		    },
        //     Err(e) => {
        // 	nberr+=1;
        // 	println!("error: {:?}", e);
        //     }
        // }

        // println!("43 target: {} feedback pos: {} {} {} feedback vel: {} {} {} feedback torque: {} {} {} ", target, feedback.position.top,feedback.position.middle,feedback.position.bottom,feedback.speed.top,feedback.speed.middle,feedback.speed.bottom,feedback.load.top,feedback.load.middle,feedback.load.bottom);

        // thread::sleep(Duration::from_micros(500));
        // let feedback = orbita3d_poulpe::write_target_position(&io, serial_port.as_mut(), 44, MotorValue::<f32>{top:0.0, middle:0.0, bottom:0.0});
        // // let feedback = orbita3d_poulpe::write_target_position(&io, serial_port.as_mut(), 44, MotorValue::<f32>{top:0.0, middle:0.0, bottom:0.0}).unwrap_or_else(MotorPositionSpeedLoad::{position:MotorValue::<f32>{top:0.0, middle:0.0, bottom:0.0}, speed:MotorValue::<f32>{top:0.0, middle:0.0, bottom:0.0}, load:MotorValue::<f32>{top:0.0, middle:0.0, bottom:0.0}});
        // match feedback {
        //     Ok(feedback) => {
        // 	nbok+=1;
        // 	println!("44 target: {} feedback pos: {} {} {} feedback vel: {} {} {} feedback torque: {} {} {} ", target, feedback.position.top,feedback.position.middle,feedback.position.bottom,feedback.speed.top,feedback.speed.middle,feedback.speed.bottom,feedback.load.top,feedback.load.middle,feedback.load.bottom);
        // 		    },
        //     Err(e) => {
        // 	nberr+=1;
        // 	println!("error: {:?}", e);
        //     }
        // }

        // nbtot+=3;
        nbtot += 1;
        // println!("44 target: {} feedback pos: {} {} {} feedback vel: {} {} {} feedback torque: {} {} {} ", target, feedback.position.top,feedback.position.middle,feedback.position.bottom,feedback.speed.top,feedback.speed.middle,feedback.speed.bottom,feedback.load.top,feedback.load.middle,feedback.load.bottom);

        // let axis_sensor = orbita3d_poulpe::read_axis_sensor(&io, serial_port.as_mut(), id)?;
        // println!("axis_sensor: {:6.2} {:6.2} {:6.2}", axis_sensor.top, axis_sensor.middle, axis_sensor.bottom);

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
    let _ = orbita3d_poulpe::write_torque_enable(
        &io,
        serial_port.as_mut(),
        id,
        MotorValue::<bool> {
            top: false,
            middle: false,
            bottom: false,
        },
    )?;

    thread::sleep(Duration::from_millis(2000));

    Ok(())
}

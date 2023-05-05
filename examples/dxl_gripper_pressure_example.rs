use std::{error::Error, thread, time::Duration};
use std::time::SystemTime;
use std::f32::consts::PI;

use rustypot::device::{gripper_pressure, mx};
use rustypot::DynamixelSerialIO;

const ID_MOTOR:  u8 =  1;
const ID_SENSOR: u8 = 41;

fn main() -> Result<(), Box<dyn Error>> {
    println!("Hi there! [dxl_gripper_pressure_example]");
    
    let mut serial_port = serialport::new("/dev/ttyACM1", 1_000_000)
        .timeout(Duration::from_millis(100))
        .open()?;

    let io = DynamixelSerialIO::v1();
    
/*    for id in 0..254 {
    	println!("Ping {:?} {:?}", id, io.ping(serial_port.as_mut(), id));
        thread::sleep(Duration::from_millis(50));
    }*/
    println!("Ping Sensor {:?} {:?}", ID_SENSOR, io.ping(serial_port.as_mut(), ID_SENSOR));
    println!("Ping Motor {:?} {:?}", ID_MOTOR, io.ping(serial_port.as_mut(), ID_MOTOR));

/*reg_read_write!(return_delay_time, 5, u8);
reg_read_write!(cw_angle_limit, 6, u16);
reg_read_write!(ccw_angle_limit, 8, u16);
reg_read_write!(temperature_limit, 11, u8);
reg_read_write!(min_voltage_limit, 12, u8);
reg_read_write!(max_voltage_limit, 13, u8);
reg_read_write!(max_torque, 14, u16);*/ // Todo!

    let now = SystemTime::now();

    loop {
        let t = now.elapsed().unwrap().as_secs_f32();
        
        // safe when empty between 1400 (open) and 1900 (close)
        let close  = 1900.0_f32;
        let open   = 1500.0_f32;
        let gain   = (close - open) / 2.0_f32;
        let offset = open + gain;
        let target = gain * (2.0 * PI * 0.5 * t).sin() + offset;
/*        let mut target = (2.0 * PI * 0.25 * t).sin();
        if target > 0.0_f32 {
            target = close;
        } else {
            target = open;
        }*/

        mx::write_goal_position(&io, serial_port.as_mut(), ID_MOTOR, target as i16)?;
//        let pos = mx::read_present_position(&io, serial_port.as_mut(), ID_MOTOR)?;
//        let temp = mx::read_present_temperature(&io, serial_port.as_mut(), ID_MOTOR)?;
//        println!("{:?} - {:?} ({:?}°C)", target, pos, temp);

//        let grip_val = gripper_pressure::read_adc_values( &io, serial_port.as_mut(), ID_SENSOR)?;
        let p0 = gripper_pressure::read_adc_in0(&io, serial_port.as_mut(), ID_SENSOR)?;
        let p1 = gripper_pressure::read_adc_in1(&io, serial_port.as_mut(), ID_SENSOR)?;
        let p2 = gripper_pressure::read_adc_in2(&io, serial_port.as_mut(), ID_SENSOR)?;
        let p3 = gripper_pressure::read_adc_in3(&io, serial_port.as_mut(), ID_SENSOR)?;
        
//        println!("{:?} - {:?} - {:?} - {:?} - {:?}", temp, p0, p1, p2, p3);
//        println!("{:?} - {:?} - {:?}", target as i16, p0, p1);
        println!("{:?},{:?},{:?},{:?},{:?}", target as i16, p0, p1, p2, p3);
//        println!("{:?} - {:?} ({:?})", target as i32, pos, (target as i16 - pos).abs() );
//        println!("{:?} - {:?}", pos, grip_val);

        thread::sleep(Duration::from_millis(10));
    }
}


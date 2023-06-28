use std::f32::consts::PI;
use std::time::SystemTime;
use std::{error::Error, thread, time::Duration};

use rustypot::device::orbita2dof_foc::{self};
use rustypot::device::orbita_foc::{self, DiskValue};
use rustypot::DynamixelSerialIO;

const RIGHT_ARM_WRIST: u8 = 70;
const RIGHT_ARM_ELBOW_MOTOR_A: u8 = 71;
const RIGHT_ARM_ELBOW_MOTOR_B: u8 = 72;
const RIGHT_ARM_SHOULDER_MOTOR_A: u8 = 81;
const RIGHT_ARM_SHOULDER_MOTOR_B: u8 = 82;

fn main() -> Result<(), Box<dyn Error>> {
    println!("Hi there! [foc_2dof.rs]");
            
//    let mut serial_port_motor = serialport::new("/dev/tool-Flipsky", 1_000_000)
    let mut serial_port_motor = serialport::new("/dev/ttyACM1", 1_000_000)
        .timeout(Duration::from_millis(100))
        .open()?;
        
    let now = SystemTime::now();
    let io  = DynamixelSerialIO::v1();
    let id_motor = RIGHT_ARM_SHOULDER_MOTOR_A;
    
    let x = io.ping(serial_port_motor.as_mut(), id_motor);
    println!("Ping ({:?}): {:?}", id_motor, x);
    
    let _reg = orbita2dof_foc::write_voltage_limit(&io, serial_port_motor.as_mut(), id_motor, 20.0)?;
    let _reg = orbita2dof_foc::write_intensity_limit(&io, serial_port_motor.as_mut(), id_motor, 8.2)?;
    let _reg = orbita2dof_foc::write_angle_velocity_limit(&io, serial_port_motor.as_mut(), id_motor, 120.0)?;
    thread::sleep(Duration::from_millis(1100));
    let _reg = orbita2dof_foc::read_voltage_limit(&io, serial_port_motor.as_mut(), id_motor)?;
    println!("V_limit {:>3.1?}V", _reg);	
    let _reg = orbita2dof_foc::read_intensity_limit(&io, serial_port_motor.as_mut(), id_motor)?;
    println!("I_limit {:>3.1?}A", _reg);	
    let _reg = orbita2dof_foc::read_angle_velocity_limit(&io, serial_port_motor.as_mut(), id_motor)?;
    println!("vel_limit {:>3.1?} rad/s", _reg);	
    let _reg = orbita2dof_foc::write_torque_enable(&io, serial_port_motor.as_mut(), id_motor, 0x01)?;
    let reg  = orbita2dof_foc::read_torque_enable (&io, serial_port_motor.as_mut(), id_motor)?;
    if reg == 0x01 {
    	println!("Motor ENable...");
    } else {
    	println!("Motor DISable...");
    }

    let mut display_counter = 0;
    loop {
        let t = now.elapsed().unwrap().as_secs_f32();
        let target = 1.0_f32 * 180.0_f32.to_radians() * (2.0 * PI * 0.5 * t).sin();
//	let target = 1.0_f32 * 2.0_f32 * PI;
//	let target = 0.0_f32;

        orbita2dof_foc::write_motor_a_goal_position(&io, serial_port_motor.as_mut(), id_motor, target)?;
        
        if display_counter == 0 {
        let encoder_pos = orbita2dof_foc::read_motor_a_present_position(&io, serial_port_motor.as_mut(), id_motor)?;
        let encoder_vel = orbita2dof_foc::read_motor_a_present_velocity(&io, serial_port_motor.as_mut(), id_motor)?;
        let current_u = orbita2dof_foc::read_motor_a_current_phase_u(&io, serial_port_motor.as_mut(), id_motor)?;
        let current_v = orbita2dof_foc::read_motor_a_current_phase_v(&io, serial_port_motor.as_mut(), id_motor)?;
        let current_w = orbita2dof_foc::read_motor_a_current_phase_w(&io, serial_port_motor.as_mut(), id_motor)?;
        let dbg_flt_1 = orbita2dof_foc::read_debug_float_1(&io, serial_port_motor.as_mut(), id_motor)?;
        let dbg_flt_2 = orbita2dof_foc::read_debug_float_2(&io, serial_port_motor.as_mut(), id_motor)?;
        let dbg_flt_3 = orbita2dof_foc::read_debug_float_3(&io, serial_port_motor.as_mut(), id_motor)?;
        let current_dc = orbita2dof_foc::read_motor_a_dc_current(&io, serial_port_motor.as_mut(), id_motor)?;
//        print!("[Enc_vel: {:6.2?}] - [Current.DC[{:6.3?}]] - f1[{:3.2?}] f2[{:3.2?}] f3[{:3.2?}]", encoder_vel, current_dc, dbg_flt_1, dbg_flt_2, dbg_flt_3);	
        print!("[Enc_vel: {:6.2?}] - [Current .DC[{:6.3?}]] .u[{:6.2?}] .v[{:6.2?}] .w[{:6.2?}]", encoder_vel, current_dc, current_u, current_v, current_w);	
//        print!("{:?};{:?};{:?};{:?};{:?};{:?};{:?}", target, encoder_pos, encoder_vel, current_dc, current_u, current_v, current_w);	

 //       let ring_pos = orbita2dof_foc::read_sensor_ring_present_position(&io, serial_port_motor.as_mut(), id_motor)?;
//        let center_pos = orbita2dof_foc::read_sensor_center_present_position(&io, serial_port_motor.as_mut(), id_motor)?;
//        print!("[Ring_s {:>5.3?} - Center_s: {:>5.3?}] -", ring_pos, center_pos);	
	}
        
        if display_counter == 0 {
	        println!("");
	}
	display_counter += 1;
        if display_counter > 2 {
	        display_counter = 0;
	}
	
        thread::sleep(Duration::from_millis(10));
    }
}

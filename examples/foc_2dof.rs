use std::f32::consts::PI;
use std::time::SystemTime;
use std::{error::Error, thread, time::Duration};

use rustypot::device::orbita2dof_foc::{self};
use rustypot::device::orbita_foc::{self, DiskValue};
use rustypot::DynamixelSerialIO;

const RIGHT_ARM_WRIST:            u8 = 70;
const RIGHT_ARM_ELBOW_MOTOR_A:    u8 = 71;
const RIGHT_ARM_ELBOW_MOTOR_B:    u8 = 72;
const RIGHT_ARM_SHOULDER_MOTOR_A: u8 = 81;
const RIGHT_ARM_SHOULDER_MOTOR_B: u8 = 82;

fn main() -> Result<(), Box<dyn Error>> {
    println!("Hi there! [foc_2dof.rs]");
            
    let mut serial_port_motor = serialport::new("/dev/tool-XT90-A", 1_000_000)
        .timeout(Duration::from_millis(100))
        .open()?;
        
    let now = SystemTime::now();
    let io  = DynamixelSerialIO::v1();
    let id_motor = RIGHT_ARM_SHOULDER_MOTOR_B;
    
    let x = io.ping(serial_port_motor.as_mut(), id_motor);
    println!("Ping ({:?}): {:?}", id_motor, x);

    let _reg = orbita2dof_foc::write_voltage_limit(&io, serial_port_motor.as_mut(), id_motor, 19.0)?;
    thread::sleep(Duration::from_millis(1100));
    let _reg = orbita2dof_foc::read_voltage_limit(&io, serial_port_motor.as_mut(), id_motor)?;
    println!("v_limit {:>5.1?}V", _reg);	
    let _reg = orbita2dof_foc::write_torque_enable(&io, serial_port_motor.as_mut(), id_motor, 0x00)?;
    let reg  = orbita2dof_foc::read_torque_enable (&io, serial_port_motor.as_mut(), id_motor)?;
    if reg == 0x01 {
    	println!("Motor started... ({:#04x})", reg);
    } else {
    	println!(":-(");
    }
    let reg = orbita2dof_foc::read_motor_a_present_position(&io, serial_port_motor.as_mut(), id_motor)?;
    println!("Position: {:?}", reg);
    orbita2dof_foc::write_motor_a_goal_position(&io, serial_port_motor.as_mut(), id_motor, 0.2)?;

    let mut display_counter = 0;
    loop {
        let t = now.elapsed().unwrap().as_secs_f32();
        let target = 1.0_f32 * 180.0_f32.to_radians() * (2.0 * PI * 0.1 * t).sin();

//        orbita2dof_foc::write_motor_a_goal_position(&io, serial_port_motor.as_mut(), id_motor, target)?;
        
        if display_counter == 0 {
        //let encoder_pos = orbita2dof_foc::read_motor_a_present_position(&io, serial_port_motor.as_mut(), id_motor)?;
        let encoder_vel = orbita2dof_foc::read_motor_a_present_velocity(&io, serial_port_motor.as_mut(), id_motor)?;
        let current_u = orbita2dof_foc::read_motor_a_current_phase_u(&io, serial_port_motor.as_mut(), id_motor)?;
        let current_v = orbita2dof_foc::read_motor_a_current_phase_v(&io, serial_port_motor.as_mut(), id_motor)?;
        let current_w = orbita2dof_foc::read_motor_a_current_phase_w(&io, serial_port_motor.as_mut(), id_motor)?;
        let current_dc = orbita2dof_foc::read_motor_a_dc_current(&io, serial_port_motor.as_mut(), id_motor)?;
        print!("[Enc_vel: {:6.2?}] - [Current.u.v.w: [{:6.3?} {:6.3?} {:6.3?}] .DC[{:6.3?}]]", encoder_vel, current_u, current_v, current_w, current_dc);	
 //       let ring_pos = orbita2dof_foc::read_sensor_ring_present_position(&io, serial_port_motor.as_mut(), id_motor)?;
//        let center_pos = orbita2dof_foc::read_sensor_center_present_position(&io, serial_port_motor.as_mut(), id_motor)?;
//        print!("[Ring_s {:>5.3?} - Center_s: {:>5.3?}] -", ring_pos, center_pos);	
	}
        
        if display_counter == 0 {
	        println!("");
	}
	display_counter += 1;
        if display_counter > 10 {
	        display_counter = 0;
	}
	
        thread::sleep(Duration::from_millis(10));
    }
    
/*    let mut serial_port_right_shoulder_motor_a = serialport::new("/dev/right_shoulder_A", 1_000_000)
        .timeout(Duration::from_millis(100))
        .open()?;

    let mut serial_port_right_shoulder_motor_b = serialport::new("/dev/right_shoulder_B", 1_000_000)
        .timeout(Duration::from_millis(100))
        .open()?;

    let mut serial_port_right_elbow_motor_a = serialport::new("/dev/right_elbow_A", 1_000_000)
        .timeout(Duration::from_millis(100))
        .open()?;

    let mut serial_port_right_elbow_motor_b = serialport::new("/dev/right_elbow_B", 1_000_000)
        .timeout(Duration::from_millis(100))
        .open()?;

    let mut serial_port_right_wrist = serialport::new("/dev/right_wrist", 1_000_000)
        .timeout(Duration::from_millis(100))
        .open()?;

    let now = SystemTime::now();
  
    let io = DynamixelSerialIO::v1();

    let id_right_shoulder_motor_a = RIGHT_ARM_SHOULDER_MOTOR_A;
    let id_right_shoulder_motor_b = RIGHT_ARM_SHOULDER_MOTOR_B;
    let id_right_elbow_motor_a    = RIGHT_ARM_ELBOW_MOTOR_A;
    let id_right_elbow_motor_b    = RIGHT_ARM_ELBOW_MOTOR_B;
    let id_right_wrist            = RIGHT_ARM_WRIST;
    
    // Ping
    let x = io.ping(serial_port_right_shoulder_motor_a.as_mut(), id_right_shoulder_motor_a);
    println!("Ping (id_right_shoulder_motor_a): {:?}", x);
    let x = io.ping(serial_port_right_shoulder_motor_b.as_mut(), id_right_shoulder_motor_b);
    println!("Ping (id_right_shoulder_motor_b): {:?}", x);
    let x = io.ping(serial_port_right_elbow_motor_a.as_mut(), id_right_elbow_motor_a);
    println!("Ping (id_right_elbow_motor_a): {:?}", x);
    let x = io.ping(serial_port_right_elbow_motor_b.as_mut(), id_right_elbow_motor_b);
    println!("Ping (id_right_elbow_motor_b): {:?}", x);
    let x = io.ping(serial_port_right_wrist.as_mut(), id_right_wrist);
    println!("Ping (id_right_wrist): {:?}", x);
    
    // Set power
    let _reg = orbita2dof_foc::write_voltage_limit(&io, serial_port_right_shoulder_motor_a.as_mut(), id_right_shoulder_motor_a, 10.0)?;
    let _reg = orbita2dof_foc::write_voltage_limit(&io, serial_port_right_shoulder_motor_b.as_mut(), id_right_shoulder_motor_b, 10.0)?;
    let _reg = orbita2dof_foc::write_voltage_limit(&io, serial_port_right_elbow_motor_a.as_mut(), id_right_elbow_motor_a, 10.0)?;
    let _reg = orbita2dof_foc::write_voltage_limit(&io, serial_port_right_elbow_motor_b.as_mut(), id_right_elbow_motor_b, 10.0)?;
    let _reg = orbita2dof_foc::read_voltage_limit(&io, serial_port_right_shoulder_motor_a.as_mut(), id_right_shoulder_motor_a)?;
    print!("v_limit {:>5.3?} -", _reg);	
    let _reg = orbita2dof_foc::read_voltage_limit(&io, serial_port_right_shoulder_motor_b.as_mut(), id_right_shoulder_motor_b)?;
    print!("v_limit {:>5.3?} -", _reg);	
    let _reg = orbita2dof_foc::read_voltage_limit(&io, serial_port_right_elbow_motor_a.as_mut(), id_right_elbow_motor_a)?;
    print!("v_limit {:>5.3?} -", _reg);	
    let _reg = orbita2dof_foc::read_voltage_limit(&io, serial_port_right_elbow_motor_b.as_mut(), id_right_elbow_motor_b)?;
    print!("v_limit {:>5.3?} -", _reg);	
    println!("");	
    
	
    // Torque enable/disable
    let _reg = orbita2dof_foc::write_torque_enable(&io, serial_port_right_shoulder_motor_a.as_mut(), id_right_shoulder_motor_a, 0x01)?;
    let reg = orbita2dof_foc::read_torque_enable(&io, serial_port_right_shoulder_motor_a.as_mut(), id_right_shoulder_motor_a)?;
    if reg == 0x01 {
    	println!("Shoulder - Motor A started... ({:#04x})", reg);
    } else {
    	println!(":-(");
    }

    let _reg = orbita2dof_foc::write_torque_enable(&io, serial_port_right_shoulder_motor_b.as_mut(), id_right_shoulder_motor_b, 0x01)?;
    let reg = orbita2dof_foc::read_torque_enable(&io, serial_port_right_shoulder_motor_b.as_mut(), id_right_shoulder_motor_b)?;
    if reg == 0x01 {
    	println!("Shoulder - Motor B started... ({:#04x})", reg);
    } else {
    	println!(":-(");
    }
    
    let _reg = orbita2dof_foc::write_torque_enable(&io, serial_port_right_elbow_motor_a.as_mut(), id_right_elbow_motor_a, 0x01)?;
    let reg = orbita2dof_foc::read_torque_enable(&io, serial_port_right_elbow_motor_a.as_mut(), id_right_elbow_motor_a)?;
    if reg == 0x01 {
    	println!("Elbow - Motor A started... ({:#04x})", reg);
    } else {
    	println!(":-(");
    }

    let _reg = orbita2dof_foc::write_torque_enable(&io, serial_port_right_elbow_motor_b.as_mut(), id_right_elbow_motor_b, 0x01)?;
    let reg = orbita2dof_foc::read_torque_enable(&io, serial_port_right_elbow_motor_b.as_mut(), id_right_elbow_motor_b)?;
    if reg == 0x01 {
    	println!("Elbow - Motor B started... ({:#04x})", reg);
    } else {
    	println!(":-(");
    }
    
    let _reg = orbita2dof_foc::write_torque_enable(&io, serial_port_right_wrist.as_mut(), id_right_wrist, 0x01)?;
    let reg = orbita2dof_foc::read_torque_enable(&io, serial_port_right_wrist.as_mut(), id_right_wrist)?;
    if reg == 0x01 {
    	println!("Wrist started... ({:#04x})", reg);
    } else {
    	println!(":-(");
    }
    
    let mut display_counter = 0;
    loop {
        let t = now.elapsed().unwrap().as_secs_f32();
        let target = 10.0_f32 * 180.0_f32.to_radians() * (2.0 * PI * 0.1 * t).sin();

        orbita2dof_foc::write_motor_a_goal_position(&io, serial_port_right_shoulder_motor_a.as_mut(), id_right_shoulder_motor_a, target)?;
        orbita2dof_foc::write_motor_a_goal_position(&io, serial_port_right_shoulder_motor_b.as_mut(), id_right_shoulder_motor_b, 1.0_f32*target)?;
        orbita2dof_foc::write_motor_a_goal_position(&io, serial_port_right_elbow_motor_a.as_mut(), id_right_elbow_motor_a, -1.0_f32*target)?;
        orbita2dof_foc::write_motor_a_goal_position(&io, serial_port_right_elbow_motor_b.as_mut(), id_right_elbow_motor_b, 1.0_f32*target)?;
        
        let target_wrist = 60.0_f32.to_radians() * (2.0 * PI * 0.5 * t).sin();

//        orbita_foc::write_goal_position(
//            &io,
//            serial_port_right_wrist.as_mut(),
//            id_right_wrist,
//            DiskValue {
//                top:    target_wrist,
//                middle: target_wrist,
//                bottom: target_wrist,
//            },
//        )?;
        
        if display_counter == 0 {
        let shoulder_ring_pos = orbita2dof_foc::read_sensor_ring_present_position(&io, serial_port_right_shoulder_motor_a.as_mut(), id_right_shoulder_motor_a)?;
        let shoulder_center_pos = orbita2dof_foc::read_sensor_center_present_position(&io, serial_port_right_shoulder_motor_b.as_mut(), id_right_shoulder_motor_b)?;
        let elbow_ring_pos = orbita2dof_foc::read_sensor_ring_present_position(&io, serial_port_right_elbow_motor_a.as_mut(), id_right_elbow_motor_a)?;
        let elbow_center_pos = orbita2dof_foc::read_sensor_center_present_position(&io, serial_port_right_elbow_motor_b.as_mut(), id_right_elbow_motor_b)?;
        
        print!("[Shoulder Ring_s {:>5.3?} - Center_s: {:>5.3?}] -", shoulder_ring_pos, shoulder_center_pos);	
        print!("[Elbow Ring_s {:>5.3?} - Center_s: {:>5.3?}] -", elbow_ring_pos, elbow_center_pos);
	}
        
        if display_counter == 0 {
	        println!("");
	}
	display_counter += 1;
        if display_counter > 10 {
	        display_counter = 0;
	}
	
        thread::sleep(Duration::from_millis(10));
    }*/
}



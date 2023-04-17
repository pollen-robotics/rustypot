use std::f32::consts::PI;
use std::time::SystemTime;
use std::{error::Error, thread, time::Duration};

use rustypot::device::orbita2dof_foc::{self, DiskValue};
use rustypot::DynamixelSerialIO;

fn main() -> Result<(), Box<dyn Error>> {
    // Motor A
    let mut serial_port_motor_a = serialport::new("/dev/ttyACM0", 1_000_000)
        .timeout(Duration::from_millis(2000))
        .open()?;

    let io = DynamixelSerialIO::v1();

    let id_motor_a = 71;

    let now = SystemTime::now();
    let x = io.ping(serial_port_motor_a.as_mut(), id_motor_a);
    println!("{:?}", x);
    
    let reg = orbita2dof_foc::read_model_number(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
    println!("model_number: {:#04x}", reg);
    let reg = orbita2dof_foc::read_firmware_version(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
    println!("firmware_version: {:#04x}", reg);
    let reg = orbita2dof_foc::read_id(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
    println!("id: {:?}", reg);
    let reg = orbita2dof_foc::read_voltage_limit(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
    println!("voltage_limit: {:?}", reg);
    
    let reg = orbita2dof_foc::read_motors_drivers_states(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
    println!("Motors/Drivers states: {:#010b}", reg);
    
    let _reg = orbita2dof_foc::write_torque_enable(&io, serial_port_motor_a.as_mut(), id_motor_a, 0x01)?;
    let reg = orbita2dof_foc::read_torque_enable(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
    if reg == 0x01 {
    	println!("Motors started... ({:#04x})", reg);
    } else {
    	println!(":-(");
    }
    
    // Motor B
    let mut serial_port_motor_b = serialport::new("/dev/ttyACM1", 1_000_000)
        .timeout(Duration::from_millis(2000))
        .open()?;

    let io = DynamixelSerialIO::v1();

    let id_motor_b = 72;

    let now = SystemTime::now();
    let x = io.ping(serial_port_motor_b.as_mut(), id_motor_b);
    println!("{:?}", x);
    
    let reg = orbita2dof_foc::read_model_number(&io, serial_port_motor_b.as_mut(), id_motor_b)?;
    println!("model_number: {:#04x}", reg);
    let reg = orbita2dof_foc::read_firmware_version(&io, serial_port_motor_b.as_mut(), id_motor_b)?;
    println!("firmware_version: {:#04x}", reg);
    let reg = orbita2dof_foc::read_id(&io, serial_port_motor_b.as_mut(), id_motor_b)?;
    println!("id: {:?}", reg);
    let reg = orbita2dof_foc::read_voltage_limit(&io, serial_port_motor_b.as_mut(), id_motor_b)?;
    println!("voltage_limit: {:?}", reg);
    
    let reg = orbita2dof_foc::read_motors_drivers_states(&io, serial_port_motor_b.as_mut(), id_motor_b)?;
    println!("Motor/Driver states: {:#010b}", reg);
    
    let _reg = orbita2dof_foc::write_torque_enable(&io, serial_port_motor_b.as_mut(), id_motor_b, 0x01)?;
    let reg = orbita2dof_foc::read_torque_enable(&io, serial_port_motor_b.as_mut(), id_motor_b)?;
    if reg == 0x01 {
    	println!("Motor started... ({:#04x})", reg);
    } else {
    	println!(":-(");
    }
    
    loop {
        let t = now.elapsed().unwrap().as_secs_f32();
        let target = 23.0_f32 * 180.0_f32.to_radians() * (2.0 * PI * 0.5 * t).sin();
        orbita2dof_foc::write_motor_a_goal_position(&io, serial_port_motor_a.as_mut(), id_motor_a, target)?;
        orbita2dof_foc::write_motor_a_goal_position(&io, serial_port_motor_b.as_mut(), id_motor_b, -1.0_f32*target)?;
        
/*        let w_pos = orbita2dof_foc::read_motor_a_goal_position(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
        let r_pos = orbita2dof_foc::read_motor_a_present_position(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
        print!("A[t: {:?} - a: {:?}]", w_pos, r_pos);*/
        let w_pos = orbita2dof_foc::read_motor_a_goal_position(&io, serial_port_motor_b.as_mut(), id_motor_b)?;
        let r_pos = orbita2dof_foc::read_motor_a_present_position(&io, serial_port_motor_b.as_mut(), id_motor_b)?;
        print!(" - B[t: {:?} - a: {:?}]", w_pos, r_pos);
        
        println!("");
        thread::sleep(Duration::from_millis(10));
    }
        
 /*       
//        orbita_foc::write_system_check(&io, serial_port_motor_a.as_mut(), id, 0b00000001)?;

        let pos = orbita_foc::read_present_position(&io, serial_port_motor_a.as_mut(), id)?;
        println!("{:?}", pos);

    loop {
        let t = now.elapsed().unwrap().as_secs_f32();
        let target = 4.267_f32*180.0_f32.to_radians() * (2.0 * PI * 0.1 * t).sin();
        orbita_foc::write_top_goal_position(&io, serial_port_motor_a.as_mut(), id, target)?;
        // println!("{}", t);

        orbita_foc::write_goal_position(
            &io,
            serial_port_motor_a.as_mut(),
            id,
            DiskValue {
                top:    pos.top + target,
                middle: pos.middle + target,
                bottom: pos.bottom + target,
            },
        )?;

        thread::sleep(Duration::from_millis(10));
    }*/
}


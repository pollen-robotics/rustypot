use std::f32::consts::PI;
use std::time::SystemTime;
use std::{error::Error, thread, time::Duration};

use rustypot::device::orbita2dof_foc::{self};//, DiskValue};
use rustypot::DynamixelSerialIO;


fn main() -> Result<(), Box<dyn Error>> {
    println!("Hi there! [foc_2dof.rs]");
            
    // Motor A
    let mut serial_port_motor_a = serialport::new("/dev/ttyACM1", 1_000_000)
        .timeout(Duration::from_millis(100))
        .open()?;

    let io = DynamixelSerialIO::v1();

    let id_motor_a = 71;

    let now = SystemTime::now();
    let x = io.ping(serial_port_motor_a.as_mut(), id_motor_a);
    println!("Ping: {:?}", x);
    let reg = orbita2dof_foc::read_id(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
    println!("id: {:?}", reg);
    let reg = orbita2dof_foc::read_model_number(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
    println!("model_number: {:#04x}", reg);
    let reg = orbita2dof_foc::read_firmware_version(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
    println!("firmware_version: {:#04x}", reg);
    orbita2dof_foc::write_voltage_limit(&io, serial_port_motor_a.as_mut(), id_motor_a, 1.0)?;
    let reg = orbita2dof_foc::read_voltage_limit(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
    println!("voltage_limit: {:?}", reg);
    

    let reg = orbita2dof_foc::read_velocity_p_gain(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
    print!("Velocity - P [{:?}]", reg);
    let reg = orbita2dof_foc::read_velocity_i_gain(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
    print!(" I [{:?}]", reg);
    let reg = orbita2dof_foc::read_velocity_d_gain(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
    println!(" D [{:?}]", reg);
    let reg = orbita2dof_foc::read_velocity_out_ramp(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
    print!("Output Ramp [{:?}]", reg);
    let reg = orbita2dof_foc::read_velocity_low_pass_filter(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
    println!(" Low-Pass Filter [{:?}]", reg);

    let reg = orbita2dof_foc::read_motors_drivers_states(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
    println!("Motors/Drivers states: {:#010b}", reg);
    
    let _reg = orbita2dof_foc::write_torque_enable(&io, serial_port_motor_a.as_mut(), id_motor_a, 0x01)?;
    let reg = orbita2dof_foc::read_torque_enable(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
    if reg == 0x01 {
    	println!("Motors started... ({:#04x})", reg);
    } else {
    	println!(":-(");
    }
    
/*orbita2dof_foc::write_motor_a_goal_position(&io, serial_port_motor_a.as_mut(), id_motor_a, 0.0_f32)?;
orbita2dof_foc::write_torque_enable(&io, serial_port_motor_a.as_mut(), id_motor_a, 0x00)?;
Ok(())*/
        

    
    // Motor B
/*    let mut serial_port_motor_b = serialport::new("/dev/ttyACM1", 1_000_000)
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
    let reg = orbita2dof_foc::write_voltage_limit(&io, serial_port_motor_b.as_mut(), id_motor_b, 1.1)?;
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
    }*/
    
    let mut display_counter = 0;
    loop {
        let t = now.elapsed().unwrap().as_secs_f32();
        let target = 23.0_f32 * 180.0_f32.to_radians() * (2.0 * PI * 0.5 * t).sin();
        orbita2dof_foc::write_motor_a_goal_position(&io, serial_port_motor_a.as_mut(), id_motor_a, target)?;
//        orbita2dof_foc::write_motor_a_goal_position(&io, serial_port_motor_b.as_mut(), id_motor_b, -1.0_f32*target)?;
        
        let w_pos = orbita2dof_foc::read_motor_a_goal_position(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
        let r_pos = orbita2dof_foc::read_motor_a_present_position(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
        let r_vel = orbita2dof_foc::read_motor_a_present_velocity(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
        let r_ld  = orbita2dof_foc::read_motor_a_present_load(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
        if display_counter == 0 {
	        print!("A[t: {:>5.1?} - a: {:>5.1?} - v: {:>5.1?} - l: {:>5.1?}]", w_pos, r_pos, r_vel, r_ld);
	}
//        let w_pos = orbita2dof_foc::read_motor_a_goal_position(&io, serial_port_motor_b.as_mut(), id_motor_b)?;
//        let r_pos = orbita2dof_foc::read_motor_a_present_position(&io, serial_port_motor_b.as_mut(), id_motor_b)?;
//        let v_lim = orbita2dof_foc::read_voltage_limit(&io, serial_port_motor_a.as_mut(), id_motor_a)?;
//        print!("B[t: {:?} - a: {:?} - v_lim: {:?}]", w_pos, r_pos, v_lim);
        
        if display_counter == 0 {
	        println!("");
	}
	display_counter += 1;
        if display_counter > 10 {
	        display_counter = 0;
	}
	
        thread::sleep(Duration::from_millis(10));
    }
}



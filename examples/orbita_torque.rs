use clap::Parser;
use std::{error::Error, thread, time::Duration};

use rustypot::device::orbita_foc::{self, DiskValue};
use rustypot::DynamixelSerialIO;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// tty
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    serialport: String,
    /// baud
    #[arg(short, long, default_value_t = 1_000_000)]
    baudrate: u32,

    /// id
    #[arg(short, long, default_value_t = 70)]
    id: u8,

    /// torque
    #[arg(short, long, default_value_t = 1)]
    torque: u8,
}

fn main() -> Result<(), Box<dyn Error>> {
    env_logger::init();

    let args = Args::parse();
    let serialportname: String = args.serialport;
    let baudrate: u32 = args.baudrate;
    let id: u8 = args.id;
    let torque: u8 = args.torque;

    //print all the argument values
    println!("serialport: {}", serialportname);
    println!("baudrate: {}", baudrate);
    println!("id: {}", id);
    println!("torque: {}", torque);

    let mut serial_port = serialport::new(serialportname, baudrate)
        .timeout(Duration::from_millis(20))
        .open()?;

    let io = DynamixelSerialIO::v1();

    orbita_foc::write_torque_enable(&io, serial_port.as_mut(), id, torque)?;

    let mot_driv_state = orbita_foc::read_motors_drivers_states(&io, serial_port.as_mut(), id)?;
    println!("motors/drivers states: {:#010b}", mot_driv_state); // 10 chars for u8 since it integers "0x"
    let init_pos = orbita_foc::read_present_position(&io, serial_port.as_mut(), id)?;

    println!("init_pos: {:?}", init_pos);

    thread::sleep(Duration::from_millis(100));
    Ok(())
}

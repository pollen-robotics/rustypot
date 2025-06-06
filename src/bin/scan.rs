use clap::{Parser, ValueEnum};
use std::{error::Error, time::Duration};

use rustypot::servo::ServoKind;
use rustypot::DynamixelProtocolHandler;
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    serialport: String,
    /// baud
    #[arg(short, long, default_value_t = 2_000_000)]
    baudrate: u32,

    #[arg(short, long, value_enum, default_value_t = ProtocolVersion::V1)]
    protocol: ProtocolVersion,
}

#[derive(ValueEnum, Clone, Debug)]
enum ProtocolVersion {
    V1,
    V2,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let serialport: String = args.serialport;
    let baudrate: u32 = args.baudrate;
    let protocol: ProtocolVersion = args.protocol;

    println!("Scanning for Dynamixel motors on {serialport} at {baudrate} baud using {protocol:?}");

    println!("Scanning...");
    let mut serial_port = serialport::new(serialport, baudrate)
        .timeout(Duration::from_millis(10))
        .open()?;

    let dph = match protocol {
        ProtocolVersion::V1 => DynamixelProtocolHandler::v1(),
        ProtocolVersion::V2 => DynamixelProtocolHandler::v2(),
    };

    for id in 1..253 {
        match dph.ping(serial_port.as_mut(), id) {
            Ok(present) => {
                if present {
                    let model = dph.read(serial_port.as_mut(), id, 0, 2).unwrap();
                    let model = u16::from_le_bytes([model[0], model[1]]);
                    let model = ServoKind::try_from(model);
                    match model {
                        Ok(m) => println!("Found motor with id {id} and model: {m:?}"),
                        Err(e) => println!("Found motor with id {id} with {e:?}"),
                    }
                }
            }
            Err(e) => eprintln!("Error: {e}"),
        };
    }

    Ok(())
}

use clap::{Parser, ValueEnum};
use std::collections::HashMap;
use std::{error::Error, time::Duration};

use rustypot::device::DxlModel;
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

    //print the standard ids for the arm motors

    //print all the argument values
    println!("serialport: {}", serialport);
    println!("baudrate: {}", baudrate);
    match protocol {
        ProtocolVersion::V1 => println!("protocol: V1"),
        ProtocolVersion::V2 => println!("protocol: V2"),
    }

    let mut found = HashMap::new();
    println!("Scanning...");
    let mut serial_port = serialport::new(serialport, baudrate)
        .timeout(Duration::from_millis(10))
        .open()?;

    let io = match protocol {
        ProtocolVersion::V1 => DynamixelProtocolHandler::v1(),
        ProtocolVersion::V2 => DynamixelProtocolHandler::v2(),
    };

    for id in 1..253 {
        match io.ping(serial_port.as_mut(), id) {
            Ok(present) => {
                if present {
                    let model = io.read(serial_port.as_mut(), id, 0, 2).unwrap();

                    found.insert(id, u16::from_le_bytes([model[0], model[1]]));
                }
            }
            Err(e) => eprintln!("Error: {e}"),
        };
    }

    println!("found {} motors", found.len());
    for (key, value) in found {
        println!(
            "id: {} model: {:?}",
            key,
            DxlModel::try_from(value).unwrap()
        );
    }

    Ok(())
}

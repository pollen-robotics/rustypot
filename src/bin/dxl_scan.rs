use clap::{Parser, ValueEnum};
use std::{error::Error, time::Duration};

use rustypot::DynamixelSerialIO;

#[derive(Parser, Debug)]
struct Args {
    serial_port: String,
    #[arg(value_enum)]
    protocol: ProtocolVersion,
}

#[derive(ValueEnum, Clone, Debug)]
enum ProtocolVersion {
    V1,
    V2,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();

    println!("Scanning...");
    let mut serial_port = serialport::new(args.serial_port, 1_000_000)
        .timeout(Duration::from_millis(10))
        .open()?;

    let io = match args.protocol {
        ProtocolVersion::V1 => DynamixelSerialIO::v1(),
        ProtocolVersion::V2 => DynamixelSerialIO::v2(),
    };

    let ids: Vec<u8> = (1..253)
        .filter(|id| io.ping(serial_port.as_mut(), *id).unwrap())
        .collect();
    println!("Ids found: {:?}", ids);

    Ok(())
}

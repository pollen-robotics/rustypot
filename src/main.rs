use std::time::Duration;

use rustypot::grpc_io::DynamixelGrpcIO;
use rustypot::DynamixelLikeIO;

use rustypot::protocol::v1::{InstructionPacket, StatusPacket};
use rustypot::protocol::{FromBytes, ToBytes};
use rustypot::serial_io::DynamixelSerialIO;

fn main() {
    let mut io = DynamixelSerialIO::new("/dev/ttyACM0", Duration::from_millis(100));

    let p = InstructionPacket::ping_packet(40);
    io.send_packet(p.to_bytes());

    let p = io.read_packet().unwrap();
    let p = StatusPacket::from_bytes(40, p);

    println!("{:?}", p);
}

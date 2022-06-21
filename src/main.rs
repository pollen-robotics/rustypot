use rustypot::grpc_io::DynamixelGrpcIO;
use rustypot::DynamixelLikeIO;

use rustypot::protocol::v1::{InstructionPacket, StatusPacket};
use rustypot::protocol::{FromBytes, ToBytes};

fn main() {
    let mut dxl_io = DynamixelGrpcIO::new("192.168.1.40", 38745);

    let id = 1;

    dxl_io.send_packet(InstructionPacket::ping_packet(id).to_bytes());
    let sp = StatusPacket::from_bytes(id, dxl_io.read_packet().unwrap());

    println!("Time elapsed: {:?}", sp);
}

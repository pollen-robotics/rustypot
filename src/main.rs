use rustypot::DynamixelLikeIO;
use rustypot::grpc_io::DynamixelGrpcIO;

use rustypot::protocol::{FromBytes, ToBytes};
use rustypot::protocol::v1::{InstructionPacket, StatusPacket};


fn main() {
    let mut dxl_io = DynamixelGrpcIO::new("192.168.1.40", 38745);

    dxl_io.send_packet(InstructionPacket::ping_packet(1).to_bytes());
    let sp = StatusPacket::from_bytes(dxl_io.read_packet());

    println!("Time elapsed: {:?}", sp);
}

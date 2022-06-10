pub mod grpc_io;
pub mod protocol;

pub trait DynamixelLikeIO {
    fn send_packet(&self, bytes: Vec<u8>);
    fn read_packet(&mut self) -> Vec<u8>;
}


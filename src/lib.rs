#[macro_use]
extern crate lazy_static;

pub mod grpc_io;

pub trait DynamixelLikeIO {
    fn send_packet(&self, bytes: Vec<u8>);
    fn read_packet(&mut self) -> Vec<u8>;
}


mod controller;
pub use controller::{Controller, Register};

pub mod grpc_io;
pub mod protocol;
pub mod serial_io;

mod serialize;
pub use serialize::Serializable;

#[derive(Debug, Clone, Copy)]
pub enum CommunicationErrorKind {
    ChecksumError,
    ParsingError,
    TimeoutError,
}

pub trait DynamixelLikeIO {
    fn send_packet(&mut self, bytes: Vec<u8>);
    fn read_packet(&mut self) -> Result<Vec<u8>, CommunicationErrorKind>;
}

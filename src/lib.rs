mod controller;
pub mod grpc_io;
pub mod protocol;
pub use controller::{Controller, Register};
mod serialize;
pub use serialize::Serializable;

#[derive(Debug, Clone, Copy)]
pub enum CommunicationErrorKind {
    ChecksumError,
    ParsingError,
    TimeoutError,
}

pub trait DynamixelLikeIO {
    fn send_packet(&self, bytes: Vec<u8>);
    fn read_packet(&mut self) -> Result<Vec<u8>, CommunicationErrorKind>;
}

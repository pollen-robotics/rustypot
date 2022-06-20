pub mod grpc_io;
pub mod protocol;

#[derive(Debug)]
pub enum CommunicationErrorKind {
    ChecksumError,
    ParsingError,
    TimeoutError,
}

pub trait DynamixelLikeIO {
    fn send_packet(&self, bytes: Vec<u8>);
    fn read_packet(&mut self) -> Result<Vec<u8>, CommunicationErrorKind>;
}

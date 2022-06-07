pub trait ToBytes {
    fn to_bytes(&self) -> Vec<u8>;
}

pub trait FromBytes {
    fn from_bytes(bytes: Vec<u8>) -> Result<Self, ErrorKind> where Self: Sized;
}

#[derive(Debug)]
pub enum ErrorKind {
    ChecksumError,
    ParsingError,
}

pub mod v1;
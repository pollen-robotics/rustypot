use crate::CommunicationErrorKind;

pub trait ToBytes {
    fn to_bytes(&self) -> Vec<u8>;
}

pub trait FromBytes {
    fn from_bytes(bytes: Vec<u8>) -> Result<Self, CommunicationErrorKind> where Self: Sized;
}



#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub enum DynamixelErrorKind {
    InstructionError,
    OverloadError,
    ChecksumError,
    RangeError,
    OverheatingError,
    AngleLimitError,
    InputVoltageError,
}
impl DynamixelErrorKind {
    fn from_byte(error: u8) -> Vec<Self> {
        (0..7).into_iter()
            .filter(|i| error & (1 << i) != 0)
            .map(|i| DynamixelErrorKind::from_bit(i).unwrap())
            .collect()
    }
    fn from_bit(b: u8) -> Option<Self> {
        match b {
            6 => Some(DynamixelErrorKind::InstructionError),
            5 => Some(DynamixelErrorKind::OverloadError),
            4 => Some(DynamixelErrorKind::ChecksumError),
            3 => Some(DynamixelErrorKind::RangeError),
            2 => Some(DynamixelErrorKind::OverheatingError),
            1 => Some(DynamixelErrorKind::AngleLimitError),
            0 => Some(DynamixelErrorKind::InputVoltageError),
            _ => None,
        }
    }
}

pub mod v1;
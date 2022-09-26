use crate::CommunicationErrorKind;

pub trait ToBytes {
    fn to_bytes(&self) -> Vec<u8>;
}

pub trait FromBytes {
    fn from_bytes(sender_id: u8, bytes: Vec<u8>) -> Result<Self, CommunicationErrorKind>
    where
        Self: Sized;
}

#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub enum DynamixelError {
    Instruction,
    Overload,
    Checksum,
    Range,
    Overheating,
    AngleLimit,
    InputVoltage,
}
impl DynamixelError {
    fn from_byte(error: u8) -> Vec<Self> {
        (0..7)
            .into_iter()
            .filter(|i| error & (1 << i) != 0)
            .map(|i| DynamixelError::from_bit(i).unwrap())
            .collect()
    }
    fn from_bit(b: u8) -> Option<Self> {
        match b {
            6 => Some(DynamixelError::Instruction),
            5 => Some(DynamixelError::Overload),
            4 => Some(DynamixelError::Checksum),
            3 => Some(DynamixelError::Range),
            2 => Some(DynamixelError::Overheating),
            1 => Some(DynamixelError::AngleLimit),
            0 => Some(DynamixelError::InputVoltage),
            _ => None,
        }
    }
}

pub mod v1;

use crate::device::*;

pub struct ValuePerMotor<T> {
    pub top: T,
    pub middle: T,
    pub bottom: T,
}

reg_read_write!(torque_enable, 40, ValuePerMotor::<u8>);
reg_read_write!(present_position, 50, ValuePerMotor::<f32>);
reg_read_write!(goal_position, 60, ValuePerMotor::<f32>);

impl ValuePerMotor<u8> {
    pub fn from_le_bytes(bytes: [u8; 3]) -> Self {
        ValuePerMotor {
            top: bytes[0],
            middle: bytes[1],
            bottom: bytes[2],
        }
    }
    pub fn to_le_bytes(&self) -> [u8; 3] {
        let mut bytes = Vec::new();

        bytes.extend_from_slice(&self.top.to_le_bytes());
        bytes.extend_from_slice(&self.middle.to_le_bytes());
        bytes.extend_from_slice(&self.bottom.to_le_bytes());

        bytes.try_into().unwrap()
    }
}

impl ValuePerMotor<f32> {
    pub fn from_le_bytes(bytes: [u8; 12]) -> Self {
        ValuePerMotor {
            top: f32::from_le_bytes(bytes[0..4].try_into().unwrap()),
            middle: f32::from_le_bytes(bytes[4..8].try_into().unwrap()),
            bottom: f32::from_le_bytes(bytes[8..12].try_into().unwrap()),
        }
    }
    pub fn to_le_bytes(&self) -> [u8; 12] {
        let mut bytes = Vec::new();

        bytes.extend_from_slice(&self.top.to_le_bytes());
        bytes.extend_from_slice(&self.middle.to_le_bytes());
        bytes.extend_from_slice(&self.bottom.to_le_bytes());

        bytes.try_into().unwrap()
    }
}

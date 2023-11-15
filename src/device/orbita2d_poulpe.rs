use crate::device::*;

pub struct ValuePerMotor<T> {
    pub motor_a: T,
    pub motor_b: T,
}

reg_read_write!(torque_enable, 40, ValuePerMotor::<u8>);
reg_read_write!(present_position, 50, ValuePerMotor::<f32>);
reg_read_write!(goal_position, 60, ValuePerMotor::<f32>);

impl ValuePerMotor<u8> {
    pub fn from_le_bytes(bytes: [u8; 2]) -> Self {
        ValuePerMotor {
            motor_a: bytes[0],
            motor_b: bytes[1],
        }
    }
    pub fn to_le_bytes(&self) -> [u8; 3] {
        let mut bytes = Vec::new();

        bytes.extend_from_slice(&self.motor_a.to_le_bytes());
        bytes.extend_from_slice(&self.motor_b.to_le_bytes());

        bytes.try_into().unwrap()
    }
}

impl ValuePerMotor<f32> {
    pub fn from_le_bytes(bytes: [u8; 8]) -> Self {
        ValuePerMotor {
            motor_a: f32::from_le_bytes(bytes[0..4].try_into().unwrap()),
            motor_b: f32::from_le_bytes(bytes[4..8].try_into().unwrap()),
        }
    }
    pub fn to_le_bytes(&self) -> [u8; 8] {
        let mut bytes = Vec::new();

        bytes.extend_from_slice(&self.motor_a.to_le_bytes());
        bytes.extend_from_slice(&self.motor_b.to_le_bytes());

        bytes.try_into().unwrap()
    }
}

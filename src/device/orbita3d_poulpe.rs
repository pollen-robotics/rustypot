
//! Orbita 3Dof Poulpe version

use crate::device::*;

/// Wrapper for a value per motor
#[derive(Clone, Copy, Debug)]
pub struct MotorValue<T> {
    pub top: T,
    pub middle: T,
    pub bottom: T,
}

/// Wrapper for a 3D vector (x, y, z)
#[derive(Clone, Copy, Debug)]
pub struct Vec3d<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

/// Wrapper for a Position/Speed/Load value for each motor
#[derive(Clone, Copy, Debug)]
pub struct MotorPositionSpeedLoad {
    pub position: MotorValue<f32>,
    pub speed: MotorValue<f32>,
    pub load: MotorValue<f32>,
}
/// Wrapper for PID gains.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pid {
    pub p: i16,
    pub i: i16,
}

reg_read_only!(model_number, 0, u16);
reg_read_only!(firmware_version, 6, u8);
reg_read_write!(id, 7, u8);

reg_read_write!(velocity_limit, 10, MotorValue::<f32>);
reg_read_write!(torque_flux_limit, 14, MotorValue::<f32>);
reg_read_write!(uq_ud_limit, 18, MotorValue::<f32>);

reg_read_write!(flux_pid, 20, MotorValue::<Pid>);
reg_read_write!(torque_pid, 24, MotorValue::<Pid>);
reg_read_write!(velocity_pid, 28, MotorValue::<Pid>);
reg_read_write!(position_pid, 32, MotorValue::<Pid>);

reg_read_write!(torque_enable, 40, MotorValue::<bool>);

reg_read_only!(current_position, 50, MotorValue::<f32>);
reg_read_only!(current_velocity, 51, MotorValue::<f32>);
reg_read_only!(current_torque, 52, MotorValue::<f32>);

reg_read_write_fb!(target_position, 60, MotorValue::<f32>,MotorPositionSpeedLoad);

reg_read_only!(axis_sensor, 90, MotorValue::<f32>);
reg_read_only!(index_sensor, 99, MotorValue::<u8>);
reg_read_only!(full_state, 100, MotorPositionSpeedLoad);



impl MotorPositionSpeedLoad {
	pub fn from_le_bytes(bytes: [u8; 36]) -> Self {
		MotorPositionSpeedLoad {
			position: MotorValue::<f32>::from_le_bytes(bytes[0..12].try_into().unwrap()),
			speed: MotorValue::<f32>::from_le_bytes(bytes[12..24].try_into().unwrap()),
			load: MotorValue::<f32>::from_le_bytes(bytes[24..36].try_into().unwrap()),
		}
	}
	// pub fn to_le_bytes(&self) -> [u8; 36] {
	// 	let mut bytes = Vec::new();

	// 	bytes.extend_from_slice(&self.position.to_le_bytes());
	// 	bytes.extend_from_slice(&self.speed.to_le_bytes());
	// 	bytes.extend_from_slice(&self.load.to_le_bytes());

	// 	bytes.try_into().unwrap()
	// }
}




impl<T: PartialEq> PartialEq for MotorValue<T> {
    fn eq(&self, other: &Self) -> bool {
        self.top == other.top && self.middle == other.middle && self.bottom == other.bottom
    }
}

impl MotorValue<f32> {
    pub fn from_le_bytes(bytes: [u8; 12]) -> Self {
        MotorValue {
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


impl MotorValue<bool> {
    pub fn from_le_bytes(bytes: [u8; 3]) -> Self {
        MotorValue {
            top: bytes[0] !=0,
            middle: bytes[1] !=0,
            bottom: bytes[2] !=0,
        }
    }
    pub fn to_le_bytes(&self) -> [u8; 3] {
        let mut bytes = Vec::new();

        bytes.extend_from_slice(&[self.top as u8]);
        bytes.extend_from_slice(&[self.middle as u8]);
        bytes.extend_from_slice(&[self.bottom as u8]);

        bytes.try_into().unwrap()
    }
}


impl MotorValue<u8> {
    pub fn from_le_bytes(bytes: [u8; 3]) -> Self {
        MotorValue {
            top: bytes[0],
            middle: bytes[1],
            bottom: bytes[2],
        }
    }
    pub fn to_le_bytes(&self) -> [u8; 3] {
        let mut bytes = Vec::new();

        bytes.extend_from_slice(&[self.top]);
        bytes.extend_from_slice(&[self.middle]);
        bytes.extend_from_slice(&[self.bottom]);

        bytes.try_into().unwrap()
    }
}


impl MotorValue<Pid> {
    pub fn from_le_bytes(bytes: [u8; 12]) -> Self {
        MotorValue {
            top: Pid::from_le_bytes(bytes[0..4].try_into().unwrap()),
            middle: Pid::from_le_bytes(bytes[4..8].try_into().unwrap()),
            bottom: Pid::from_le_bytes(bytes[8..12].try_into().unwrap()),
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


impl<T: PartialEq> PartialEq for Vec3d<T> {
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y && self.z == other.z
    }
}

impl Vec3d<f32> {
    pub fn from_le_bytes(bytes: [u8; 12]) -> Self {
        Vec3d {
            x: f32::from_le_bytes(bytes[0..4].try_into().unwrap()),
            y: f32::from_le_bytes(bytes[4..8].try_into().unwrap()),
            z: f32::from_le_bytes(bytes[8..12].try_into().unwrap()),
        }
    }
    pub fn to_le_bytes(&self) -> [u8; 12] {
        let mut bytes = Vec::new();

        bytes.extend_from_slice(&self.x.to_le_bytes());
        bytes.extend_from_slice(&self.y.to_le_bytes());
        bytes.extend_from_slice(&self.z.to_le_bytes());

        bytes.try_into().unwrap()
    }
}

impl Pid {
    pub fn from_le_bytes(bytes: [u8; 4]) -> Self {
        Pid {
            p: i16::from_le_bytes(bytes[0..2].try_into().unwrap()),
            i: i16::from_le_bytes(bytes[2..4].try_into().unwrap()),
        }
    }
    pub fn to_le_bytes(&self) -> [u8; 4] {
        let mut bytes = Vec::new();

        bytes.extend_from_slice(&self.p.to_le_bytes());
        bytes.extend_from_slice(&self.i.to_le_bytes());

        bytes.try_into().unwrap()
    }
}

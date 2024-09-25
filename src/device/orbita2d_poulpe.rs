use crate::device::*;

/// Wrapper for a value per motor (A and B)
#[derive(Clone, Copy, Debug)]
pub struct MotorValue<T> {
    pub motor_a: T,
    pub motor_b: T,
}
/// Wrapper for a Position/Speed/Load value for each motor
#[derive(Clone, Copy, Debug)]
pub struct MotorPositionSpeedLoad {
    pub position: MotorValue<f32>,
    // pub speed: MotorValue<f32>,
    // pub load: MotorValue<f32>,
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
reg_read_write!(velocity_limit_max, 12, MotorValue::<f32>);
reg_read_write!(torque_flux_limit, 14, MotorValue::<f32>);
reg_read_write!(torque_flux_limit_max, 16, MotorValue::<f32>);
reg_read_write!(uq_ud_limit, 18, MotorValue::<i16>);

reg_read_write!(flux_pid, 20, MotorValue::<Pid>);
reg_read_write!(torque_pid, 24, MotorValue::<Pid>);
reg_read_write!(velocity_pid, 28, MotorValue::<Pid>);
reg_read_write!(position_pid, 32, MotorValue::<Pid>);

reg_read_write!(torque_enable, 40, MotorValue::<bool>);

reg_read_only!(current_position, 50, MotorValue::<f32>);
reg_read_only!(current_velocity, 51, MotorValue::<f32>);
reg_read_only!(current_torque, 52, MotorValue::<f32>);

reg_read_write_fb!(
    target_position,
    60,
    MotorValue::<f32>,
    MotorPositionSpeedLoad
);

reg_read_write!(board_state, 80, u8);

reg_read_only!(axis_sensor, 90, MotorValue::<f32>);

reg_read_only!(full_state, 100, MotorPositionSpeedLoad);

impl MotorPositionSpeedLoad {
    pub fn from_le_bytes(bytes: [u8; 8]) -> Self {
        MotorPositionSpeedLoad {
            position: MotorValue::<f32>::from_le_bytes(bytes[0..8].try_into().unwrap()),
            // speed: MotorValue::<f32>::from_le_bytes(bytes[8..16].try_into().unwrap()),
            // load: MotorValue::<f32>::from_le_bytes(bytes[16..24].try_into().unwrap()),
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
        self.motor_a == other.motor_a && self.motor_b == other.motor_b
    }
}

impl MotorValue<f32> {
    pub fn from_le_bytes(bytes: [u8; 8]) -> Self {
        MotorValue {
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

impl MotorValue<u32> {
    pub fn from_le_bytes(bytes: [u8; 8]) -> Self {
        MotorValue {
            motor_a: u32::from_le_bytes(bytes[0..4].try_into().unwrap()),
            motor_b: u32::from_le_bytes(bytes[4..8].try_into().unwrap()),
        }
    }
    pub fn to_le_bytes(&self) -> [u8; 8] {
        let mut bytes = Vec::new();

        bytes.extend_from_slice(&self.motor_a.to_le_bytes());
        bytes.extend_from_slice(&self.motor_b.to_le_bytes());

        bytes.try_into().unwrap()
    }
}

impl MotorValue<i32> {
    pub fn from_le_bytes(bytes: [u8; 8]) -> Self {
        MotorValue {
            motor_a: i32::from_le_bytes(bytes[0..4].try_into().unwrap()),
            motor_b: i32::from_le_bytes(bytes[4..8].try_into().unwrap()),
        }
    }
    pub fn to_le_bytes(&self) -> [u8; 8] {
        let mut bytes = Vec::new();

        bytes.extend_from_slice(&self.motor_a.to_le_bytes());
        bytes.extend_from_slice(&self.motor_b.to_le_bytes());

        bytes.try_into().unwrap()
    }
}

impl MotorValue<i16> {
    pub fn from_le_bytes(bytes: [u8; 4]) -> Self {
        MotorValue {
            motor_a: i16::from_le_bytes(bytes[0..2].try_into().unwrap()),
            motor_b: i16::from_le_bytes(bytes[2..4].try_into().unwrap()),
        }
    }
    pub fn to_le_bytes(&self) -> [u8; 4] {
        let mut bytes = Vec::new();

        bytes.extend_from_slice(&self.motor_a.to_le_bytes());
        bytes.extend_from_slice(&self.motor_b.to_le_bytes());

        bytes.try_into().unwrap()
    }
}

impl MotorValue<u16> {
    pub fn from_le_bytes(bytes: [u8; 4]) -> Self {
        MotorValue {
            motor_a: u16::from_le_bytes(bytes[0..2].try_into().unwrap()),
            motor_b: u16::from_le_bytes(bytes[2..4].try_into().unwrap()),
        }
    }
    pub fn to_le_bytes(&self) -> [u8; 4] {
        let mut bytes = Vec::new();

        bytes.extend_from_slice(&self.motor_a.to_le_bytes());
        bytes.extend_from_slice(&self.motor_b.to_le_bytes());

        bytes.try_into().unwrap()
    }
}

impl MotorValue<bool> {
    pub fn from_le_bytes(bytes: [u8; 2]) -> Self {
        MotorValue {
            motor_a: bytes[0] != 0,
            motor_b: bytes[1] != 0,
        }
    }
    pub fn to_le_bytes(&self) -> [u8; 2] {
        let mut bytes = Vec::new();

        bytes.extend_from_slice(&[self.motor_a as u8]);
        bytes.extend_from_slice(&[self.motor_b as u8]);

        bytes.try_into().unwrap()
    }
}

impl MotorValue<Pid> {
    pub fn from_le_bytes(bytes: [u8; 8]) -> Self {
        MotorValue {
            motor_a: Pid::from_le_bytes(bytes[0..4].try_into().unwrap()),
            motor_b: Pid::from_le_bytes(bytes[4..8].try_into().unwrap()),
        }
    }
    pub fn to_le_bytes(&self) -> [u8; 8] {
        let mut bytes = Vec::new();

        bytes.extend_from_slice(&self.motor_a.to_le_bytes());
        bytes.extend_from_slice(&self.motor_b.to_le_bytes());

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

// ///////////////////
// reg_read_write!(torque_enable, 40, MotorValue::<u8>);
// reg_read_write!(present_position, 50, MotorValue::<f32>);
// reg_read_write!(goal_position, 60, MotorValue::<f32>);

// impl MotorValue<u8> {
//     pub fn from_le_bytes(bytes: [u8; 2]) -> Self {
//         MotorValue {
//             motor_a: bytes[0],
//             motor_b: bytes[1],
//         }
//     }
//     pub fn to_le_bytes(&self) -> [u8; 3] {
//         let mut bytes = Vec::new();

//         bytes.extend_from_slice(&self.motor_a.to_le_bytes());
//         bytes.extend_from_slice(&self.motor_b.to_le_bytes());

//         bytes.try_into().unwrap()
//     }
// }

// impl MotorValue<f32> {
//     pub fn from_le_bytes(bytes: [u8; 8]) -> Self {
//         MotorValue {
//             motor_a: f32::from_le_bytes(bytes[0..4].try_into().unwrap()),
//             motor_b: f32::from_le_bytes(bytes[4..8].try_into().unwrap()),
//         }
//     }
//     pub fn to_le_bytes(&self) -> [u8; 8] {
//         let mut bytes = Vec::new();

//         bytes.extend_from_slice(&self.motor_a.to_le_bytes());
//         bytes.extend_from_slice(&self.motor_b.to_le_bytes());

//         bytes.try_into().unwrap()
//     }
// }

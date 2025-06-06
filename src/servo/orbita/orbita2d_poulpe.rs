use crate::generate_servo;

/// Wrapper for a value per motor (A and B)
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "python", derive(pyo3::FromPyObject, pyo3::IntoPyObject))]
pub struct MotorValue<T> {
    pub motor_a: T,
    pub motor_b: T,
}

#[cfg(feature = "python")]
impl<T: pyo3_stub_gen::PyStubType> pyo3_stub_gen::PyStubType for MotorValue<T> {
    fn type_output() -> pyo3_stub_gen::TypeInfo {
        pyo3_stub_gen::TypeInfo::list_of::<T>()
    }
}

/// Wrapper for a Position/Speed/Load value for each motor
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "python", derive(pyo3::FromPyObject, pyo3::IntoPyObject))]
pub struct MotorPositionSpeedLoad {
    pub position: MotorValue<f32>,
    // pub speed: MotorValue<f32>,
    // pub load: MotorValue<f32>,
}
/// Wrapper for PID gains.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "python", gen_stub_pyclass, pyo3::pyclass)]
pub struct Pid {
    pub p: i16,
    pub i: i16,
}

generate_servo!(
    Orbita2dPoulpe, v1,
    reg: (model_number, r, 0, u16, None),
    reg: (firmware_version, r, 6, u8, None),
    reg: (id, rw, 7, u8, None),
    reg: (velocity_limit, rw, 10, MotorValue::<f32>, None),
    reg: (velocity_limit_max, rw, 12, MotorValue::<f32>, None),
    reg: (torque_flux_limit, rw, 14, MotorValue::<f32>, None),
    reg: (torque_flux_limit_max, rw, 16, MotorValue::<f32>, None),
    reg: (uq_ud_limit, rw, 18, MotorValue::<i16>, None),
    reg: (flux_pid, rw, 20, MotorValue::<Pid>, None),
    reg: (torque_pid, rw, 24, MotorValue::<Pid>, None),
    reg: (velocity_pid, rw, 28, MotorValue::<Pid>, None),
    reg: (position_pid, rw, 32, MotorValue::<Pid>, None),
    reg: (torque_enable, rw, 40, MotorValue::<bool>, None),
    reg: (current_position, r, 50, MotorValue::<f32>, None),
    reg: (current_velocity, r, 51, MotorValue::<f32>, None),
    reg: (current_torque, r, 52, MotorValue::<f32>, None),
    // reg: (target_position, rw, 60, MotorValue::<f32>, None),
    // reg: (target_position, fb, 60, MotorValue::<f32>, MotorPositionSpeedLoad, None),
    reg: (board_state, rw, 80, u8, None),
    reg: (axis_sensor, r, 90, MotorValue::<f32>, None),
    reg: (full_state, r, 100, MotorPositionSpeedLoad, None),
);

// TODO:
crate::generate_reg_write_fb!(
    target_position,
    60,
    MotorValue::<f32>,
    MotorPositionSpeedLoad
);

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

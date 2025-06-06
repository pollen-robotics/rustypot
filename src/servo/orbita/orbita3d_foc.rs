//! Orbita 3DoF Serial SimpleFOC register (protocol v1)

use crate::generate_servo;

/// Wrapper for a value per disk (top, middle, bottom)
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "python", derive(pyo3::FromPyObject, pyo3::IntoPyObject))]
pub struct DiskValue<T> {
    pub top: T,
    pub middle: T,
    pub bottom: T,
}

#[cfg(feature = "python")]
impl<T: pyo3_stub_gen::PyStubType> pyo3_stub_gen::PyStubType for DiskValue<T> {
    fn type_output() -> pyo3_stub_gen::TypeInfo {
        pyo3_stub_gen::TypeInfo::list_of::<T>()
    }
}

/// Wrapper for a 3D vector (x, y, z)
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "python", derive(pyo3::FromPyObject, pyo3::IntoPyObject))]
pub struct Vec3d<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

/// Wrapper for a Position/Speed/Load value for each disk
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "python", derive(pyo3::FromPyObject, pyo3::IntoPyObject))]
pub struct DiskPositionSpeedLoad {
    pub position: DiskValue<f32>,
    pub speed: DiskValue<f32>,
    pub load: DiskValue<f32>,
}
/// Wrapper for PID gains.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "python", gen_stub_pyclass, pyo3::pyclass)]
pub struct Pid {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

generate_servo!(
    Orbita3dFoc, v1,
    reg: (model_number, r, 0, u16, None),
    reg: (firmware_version, r, 6, u8, None),
    reg: (id, rw, 7, u8, None),
    reg: (system_check, w, 8, u8, None),
    reg: (motors_drivers_states, r, 159, u8, None),
    reg: (voltage_limit, rw, 10, f32, None),
    // reg: (intensity_limit, rw, 14, f32, None),
    reg: (velocity_pid, rw, 18, Pid, None),
    reg: (velocity_p_gain, rw, 18, f32, None),
    reg: (velocity_i_gain, rw, 22, f32, None),
    reg: (velocity_d_gain, rw, 26, f32, None),
    reg: (velocity_out_ramp, rw, 30, f32, None),
    reg: (velocity_low_pass_filter, rw, 34, f32, None),
    reg: (angle_pid, rw, 38, Pid, None),
    reg: (angle_p_gain, rw, 38, f32, None),
    reg: (angle_i_gain, rw, 42, f32, None),
    reg: (angle_d_gain, rw, 46, f32, None),
    reg: (angle_velocity_limit, rw, 50, f32, None),
    // reg: (temperature_limit, rw, 54, f32, None),
    reg: (torque_enable, rw, 58, u8, None),
    reg: (goal_position, rw, 59, DiskValue::<f32>, None),
    reg: (top_goal_position, rw, 59, f32, None),
    reg: (middle_goal_position, rw, 63, f32, None),
    reg: (bottom_goal_position, rw, 67, f32, None),
    reg: (present_position, rw, 71, DiskValue::<f32>, None),
    reg: (top_present_position, rw, 71, f32, None),
    reg: (middle_present_position, rw, 75, f32, None),
    reg: (bottom_present_position, rw, 79, f32, None),
    reg: (top_present_hall, r, 160, f32, None),
    reg: (middle_present_hall, r, 164, f32, None),
    reg: (bottom_present_hall, r, 168, f32, None),
    reg: (top_current_phase_a, r, 47, f32, None),
    reg: (top_current_phase_b, r, 51, f32, None),
    reg: (top_dc_current, r, 55, f32, None),
);

impl<T: PartialEq> PartialEq for DiskValue<T> {
    fn eq(&self, other: &Self) -> bool {
        self.top == other.top && self.middle == other.middle && self.bottom == other.bottom
    }
}

impl DiskValue<f32> {
    pub fn from_le_bytes(bytes: [u8; 12]) -> Self {
        DiskValue {
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
    pub fn from_le_bytes(bytes: [u8; 12]) -> Self {
        Pid {
            p: f32::from_le_bytes(bytes[0..4].try_into().unwrap()),
            i: f32::from_le_bytes(bytes[4..8].try_into().unwrap()),
            d: f32::from_le_bytes(bytes[8..12].try_into().unwrap()),
        }
    }
    pub fn to_le_bytes(&self) -> [u8; 12] {
        let mut bytes = Vec::new();

        bytes.extend_from_slice(&self.p.to_le_bytes());
        bytes.extend_from_slice(&self.i.to_le_bytes());
        bytes.extend_from_slice(&self.d.to_le_bytes());

        bytes.try_into().unwrap()
    }
}

impl DiskPositionSpeedLoad {
    pub fn from_le_bytes(bytes: [u8; 36]) -> Self {
        DiskPositionSpeedLoad {
            position: DiskValue::from_le_bytes(bytes[0..12].try_into().unwrap()),
            speed: DiskValue::from_le_bytes(bytes[12..24].try_into().unwrap()),
            load: DiskValue::from_le_bytes(bytes[24..36].try_into().unwrap()),
        }
    }
    pub fn to_le_bytes(&self) -> [u8; 36] {
        let mut bytes = Vec::new();

        bytes.extend_from_slice(&self.position.to_le_bytes());
        bytes.extend_from_slice(&self.speed.to_le_bytes());
        bytes.extend_from_slice(&self.load.to_le_bytes());

        bytes.try_into().unwrap()
    }
}

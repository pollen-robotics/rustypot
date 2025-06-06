//! Orbita 2DoF Serial SimpleFOC register (protocol v1)

use crate::generate_servo;

/// Wrapper for a value per motor (A and B)
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "python", derive(pyo3::FromPyObject, pyo3::IntoPyObject))]
pub struct MotorValue<T> {
    pub a: T,
    pub b: T,
}

/// Wrapper for a 3D vector (x, y, z)
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "python", derive(pyo3::FromPyObject, pyo3::IntoPyObject))]
pub struct Vec3d<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

/// Wrapper for a Position/Speed/Load value for each motor
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "python", derive(pyo3::FromPyObject, pyo3::IntoPyObject))]
pub struct MotorPositionSpeedLoad {
    pub position: MotorValue<f32>,
    pub speed: MotorValue<f32>,
    pub load: MotorValue<f32>,
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
    Orbita2dFoc, v1,
    reg: (model_number, r, 0, u16, None),
    reg: (firmware_version, r, 6, u8, None),
    reg: (id, rw, 7, u8, None),
    reg: (system_check, w, 8, u8, None),
    reg: (motors_drivers_states, r, 159, u8, None),
    reg: (voltage_limit, rw, 10, f32, None),
    reg: (intensity_limit, rw, 14, f32, None),
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
    reg: (temperature_limit, rw, 54, f32, None),
    reg: (torque_enable, rw, 58, u8, None),
    reg: (ring_sensor_goal_position, rw, 59, f32, None),
    reg: (center_sensor_goal_position, rw, 63, f32, None),
    reg: (sensor_ring_present_position, r, 67, f32, None),
    reg: (sensor_center_present_position, r, 71, f32, None),
    reg: (motor_a_goal_position, rw, 75, f32, None),
    reg: (motor_b_goal_position, rw, 79, f32, None),
    reg: (motor_a_present_position, r, 83, f32, None),
    reg: (motor_b_present_position, r, 87, f32, None),
    reg: (motor_a_present_velocity, r, 91, f32, None),
    reg: (motor_b_present_velocity, r, 95, f32, None),
    reg: (motor_a_present_load, r, 99, f32, None),
    reg: (motor_b_present_load, r, 103, f32, None),
    reg: (motor_a_present_temperature, r, 107, f32, None),
    reg: (motor_b_present_temperature, r, 111, f32, None),
    reg: (imu_acc, r, 119, Vec3d::<f32>, None),
    reg: (imu_acc_x, r, 119, f32, None),
    reg: (imu_acc_y, r, 123, f32, None),
    reg: (imu_acc_z, r, 127, f32, None),
    reg: (imu_gyro, r, 131, Vec3d::<f32>, None),
    reg: (imu_gyro_x, r, 131, f32, None),
    reg: (imu_gyro_y, r, 135, f32, None),
    reg: (imu_gyro_z, r, 139, f32, None),
    reg: (imu_temperature, r, 143, f32, None),
    reg: (motor_a_current_phase_u, r, 143, f32, None),
    reg: (motor_a_current_phase_v, r, 147, f32, None),
    reg: (motor_a_current_phase_w, r, 151, f32, None),
    reg: (motor_a_dc_current, r, 155, f32, None),
    reg: (debug_float_1, rw, 159, f32, None),
    reg: (debug_float_2, rw, 163, f32, None),
    reg: (debug_float_3, rw, 167, f32, None),
);

impl<T: PartialEq> PartialEq for MotorValue<T> {
    fn eq(&self, other: &Self) -> bool {
        self.a == other.a && self.b == other.b
    }
}

impl MotorValue<f32> {
    pub fn from_le_bytes(bytes: [u8; 8]) -> Self {
        MotorValue {
            a: f32::from_le_bytes(bytes[0..4].try_into().unwrap()),
            b: f32::from_le_bytes(bytes[4..8].try_into().unwrap()),
        }
    }
    pub fn to_le_bytes(&self) -> [u8; 8] {
        let mut bytes = Vec::new();

        bytes.extend_from_slice(&self.a.to_le_bytes());
        bytes.extend_from_slice(&self.b.to_le_bytes());

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

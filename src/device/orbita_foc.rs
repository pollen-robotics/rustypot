//! Orbita 3DoF Serial SimpleFOC register (protocol v1)

use crate::device::*;

/// Wrapper for a value per disk (top, middle, bottom)
#[derive(Clone, Copy, Debug)]
pub struct DiskValue<T> {
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

/// Wrapper for a Position/Speed/Load value for each disk
#[derive(Clone, Copy, Debug)]
pub struct DiskPositionSpeedLoad {
    pub position: DiskValue<f32>,
    pub speed: DiskValue<f32>,
    pub load: DiskValue<f32>,
}
/// Wrapper for PID gains.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pid {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

reg_read_only!(model_number, 0, u16);
reg_read_only!(firmware_version, 6, u8);
reg_read_write!(id, 7, u8);

reg_write_only!(system_check, 8, u8);
reg_read_only!(motors_drivers_states, 9, u8);

reg_read_write!(voltage_limit, 10, f32);
// reg_read_write!(intensity_limit, 14, f32);

reg_read_write!(velocity_pid, 18, Pid);
reg_read_write!(velocity_p_gain, 18, f32);
reg_read_write!(velocity_i_gain, 22, f32);
reg_read_write!(velocity_d_gain, 26, f32);
reg_read_write!(velocity_out_ramp, 30, f32);
reg_read_write!(velocity_low_pass_filter, 34, f32);

reg_read_write!(angle_pid, 38, Pid);
reg_read_write!(angle_p_gain, 38, f32);
reg_read_write!(angle_i_gain, 42, f32);
reg_read_write!(angle_d_gain, 46, f32);
reg_read_write!(angle_velocity_limit, 50, f32);

// reg_read_write!(temperature_limit, 54, f32);

reg_read_write!(torque_enable, 58, u8);

reg_read_write!(goal_position, 59, DiskValue::<f32>);
reg_read_write!(top_goal_position, 59, f32);
reg_read_write!(middle_goal_position, 63, f32);
reg_read_write!(bottom_goal_position, 67, f32);

// reg_read_only!(present_position_speed_load, 71, DiskPositionSpeedLoad);
reg_read_only!(present_position, 71, DiskValue::<f32>);
reg_read_only!(top_present_position, 71, f32);
reg_read_only!(middle_present_position, 75, f32);
reg_read_only!(bottom_present_position, 79, f32);
// reg_read_only!(present_speed, 83, DiskValue::<f32>);
// reg_read_only!(top_present_speed, 83, f32);
// reg_read_only!(middle_present_speed, 87, f32);
// reg_read_only!(bottom_present_speed, 91, f32);
// reg_read_only!(present_load, 95, DiskValue::<f32>);
// reg_read_only!(top_present_load, 95, f32);
// reg_read_only!(middle_present_load, 99, f32);
// reg_read_only!(bottom_present_load, 103, f32);

// reg_read_only!(present_temperature, 107, DiskValue::<f32>);
// reg_read_only!(top_present_temperature, 107, f32);
// reg_read_only!(middle_present_temperature, 111, f32);
// reg_read_only!(bottom_present_temperature, 115, f32);

// reg_read_only!(imu_acc, 119, Vec3d::<f32>);
// reg_read_only!(imu_acc_x, 119, f32);
// reg_read_only!(imu_acc_y, 123, f32);
// reg_read_only!(imu_acc_z, 127, f32);
// reg_read_only!(imu_gyro, 131, Vec3d::<f32>);
// reg_read_only!(imu_gyro_x, 131, f32);
// reg_read_only!(imu_gyro_y, 135, f32);
// reg_read_only!(imu_gyro_z, 139, f32);
// reg_read_only!(imu_temperature, 143, f32);

reg_read_only!(top_current_phase_a, 147, f32);
reg_read_only!(top_current_phase_b, 151, f32);
reg_read_only!(top_dc_current, 155, f32);

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

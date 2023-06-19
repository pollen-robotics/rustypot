//! Orbita 2DoF Serial SimpleFOC register (protocol v1)

use crate::device::*;

/// Wrapper for a 3D vector (x, y, z)
#[derive(Clone, Copy, Debug)]
pub struct Vec3d<T> {
    pub x: T,
    pub y: T,
    pub z: T,
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
reg_read_write!(intensity_limit, 14, f32);

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

reg_read_write!(temperature_limit, 54, f32);

reg_read_write!(torque_enable, 58, u8);

reg_read_write!(ring_sensor_goal_position, 59, f32);
reg_read_write!(center_sensor_goal_position, 63, f32);
reg_read_only!(sensor_ring_present_position, 67, f32);
reg_read_only!(sensor_center_present_position, 71, f32);

reg_read_write!(motor_a_goal_position, 75, f32);
reg_read_write!(motor_b_goal_position, 79, f32);
reg_read_only!(motor_a_present_position, 83, f32);
reg_read_only!(motor_b_present_position, 87, f32);
reg_read_only!(motor_a_present_velocity, 91, f32);
reg_read_only!(motor_b_present_velocity, 95, f32);
reg_read_only!(motor_a_present_load, 99, f32);
reg_read_only!(motor_b_present_load, 103, f32);

reg_read_only!(motor_a_present_temperature, 107, f32);
reg_read_only!(motor_b_present_temperature, 111, f32);

reg_read_only!(imu_acc, 119, Vec3d::<f32>);
reg_read_only!(imu_acc_x, 119, f32);
reg_read_only!(imu_acc_y, 123, f32);
reg_read_only!(imu_acc_z, 127, f32);
reg_read_only!(imu_gyro, 131, Vec3d::<f32>);
reg_read_only!(imu_gyro_x, 131, f32);
reg_read_only!(imu_gyro_y, 135, f32);
reg_read_only!(imu_gyro_z, 139, f32);
reg_read_only!(imu_temperature, 143, f32);

reg_read_only!(motor_a_current_phase_u, 143, f32);
reg_read_only!(motor_a_current_phase_v, 147, f32);
reg_read_only!(motor_a_current_phase_w, 151, f32);
reg_read_only!(motor_a_dc_current, 155, f32);

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

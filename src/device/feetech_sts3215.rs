//! Feetech STD3215 register (protocol v1 - with v2 sync read answer)
//!

use crate::device::*;

reg_read_only!(model, 3, u16);
reg_read_write!(id, 5, u8);
reg_read_write!(baudrate, 6, u8);
reg_read_write!(return_delay_time, 7, u8);
reg_read_write!(response_status_level, 8, u8);
reg_read_write!(min_angle_limit, 9, u16);
reg_read_write!(max_angle_limit, 11, u16);
reg_read_write!(max_temperature_limit, 13, u8);
reg_read_write!(max_voltage_limit, 14, u8);
reg_read_write!(min_voltage_limit, 15, u8);
reg_read_write!(max_torque_limit, 16, u16);
reg_read_write!(phase, 18, u8);
reg_read_write!(unloading_condition, 19, u8);
reg_read_write!(led_alarm_condition, 20, u8);
reg_read_write!(p_coefficient, 21, u8);
reg_read_write!(d_coefficient, 22, u8);
reg_read_write!(i_coefficient, 23, u8);
reg_read_write!(minimum_startup_force, 24, u16);
reg_read_write!(cw_dead_zone, 26, u8);
reg_read_write!(ccw_dead_zone, 27, u8);
reg_read_write!(protection_current, 28, u16);
reg_read_write!(angular_resolution, 30, u8);
reg_read_write!(offset, 31, i16);
reg_read_write!(mode, 33, u8);
reg_read_write!(protective_torque, 34, u8);
reg_read_write!(protection_time, 35, u8);
reg_read_write!(overload_torque, 36, u8);
reg_read_write!(speed_closed_loop_p_coefficient, 37, u8);
reg_read_write!(over_current_protection_time, 38, u8);
reg_read_write!(velocity_closed_loop_i_coefficient, 39, u8);
reg_read_write!(torque_enable, 40, u8);
reg_read_write!(acceleration, 41, u8);
reg_read_write!(goal_position, 42, i16);
reg_read_write!(goal_time, 44, u16);
reg_read_write!(goal_speed, 46, u16);
reg_read_write!(torque_limit, 48, u16);
reg_read_write!(lock, 55, u8);
reg_read_only!(present_position, 56, i16);
reg_read_only!(present_position_raw, 56, i16);
reg_read_only!(present_speed, 58, u16);
reg_read_only!(present_load, 60, u16);
reg_read_only!(present_voltage, 62, u8);
reg_read_only!(present_temperature, 63, u8);
reg_read_only!(status, 65, u8);
reg_read_only!(moving, 66, u8);
reg_read_only!(present_current, 69, u16);
reg_read_write!(maximum_acceleration, 85, u16);

/// Unit conversion for Feetech motors
pub mod conv {
    use std::f64::consts::PI;

    /// Feetech STS3215 angular position to radians
    ///
    /// Works in joint and multi-turn mode
    pub fn dxl_pos_to_radians(pos: i16) -> f64 {
        (2.0 * PI * (pos as f64) / 4096.0) - PI
    }
    /// Radians to Feetech STS3215 angular position
    ///
    /// Works in joint and multi-turn mode
    pub fn radians_to_dxl_pos(rads: f64) -> i16 {
        (4096.0 * (PI + rads) / (2.0 * PI)) as i16
    }

    pub fn dxl_to_speed(value: u16) -> f64 {
        let mut value = value as f64;
        if value > ((1 << 15) as f64) {
            value = -(value - ((1 << 15) as f64));
        }

        (2.0 * PI * value as f64) / (4096.0 - 1.0)

        // value // * 0.111
    }

    // TODO check
    pub fn speed_to_dxl(value: f64) -> u16 {
        let mut value = (4096.0 - 1.0) * value / (2.0 * PI);
        if value < 0.0 {
            value = -value + (1 << 15) as f64;
        }

        value as u16
    }
}

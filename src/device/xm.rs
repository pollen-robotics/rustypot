//! XM robotis register (protocol v1)
//!
//! Tested for the XM430-W210 flashed in protocol v1
//!
//! See <https://emanual.robotis.com/docs/en/dxl/x/xm430-w210> for example.

use crate::device::*;

reg_read_only!(model_number, 0, u16);
reg_read_only!(firmware_version, 6, u8);
reg_read_write!(id, 7, u8);
reg_read_write!(baud_rate, 8, u8);
reg_read_write!(return_delay_time, 9, u8);
reg_read_write!(drive_mode, 10, u8);
reg_read_write!(operating_mode, 11, u8);
reg_read_write!(secondary_shadow_id, 12, u8);
reg_read_write!(protocol_type, 13, u8);
reg_read_write!(homing_offset, 20, i32);
reg_read_write!(moving_threshold, 24, u32);
reg_read_write!(temperature_limit, 31, u8);
reg_read_write!(max_voltage_limit, 32, u16);
reg_read_write!(min_voltage_limit, 34, u16);
reg_read_write!(pwm_limit, 36, u16);
reg_read_write!(acceleration_limit, 40, u32);
reg_read_write!(current_limit, 38, u16);
reg_read_write!(torque_limit, 38, u16); //Duplicate with MX name for compatibility
reg_read_write!(velocity_limit, 44, u32);
reg_read_write!(moving_speed, 44, u32); //Duplicate with MX name for compatibility
reg_read_write!(max_position_limit, 48, u32);
reg_read_write!(min_position_limit, 52, u32);

reg_read_write!(shutdown, 63, u8);

reg_read_write!(torque_enable, 64, u8);
reg_read_write!(led, 65, u8);
reg_read_write!(status_return_level, 68, u8);
reg_read_write!(registered_instruction, 69, u8);
reg_read_write!(hardware_error_status, 70, u8);
reg_read_write!(velocity_i_gain, 76, u16);
reg_read_write!(velocity_p_gain, 78, u16);
reg_read_write!(position_d_gain, 80, u16);
reg_read_write!(position_i_gain, 82, u16);
reg_read_write!(position_p_gain, 84, u16);
reg_read_write!(feedforward_2nd_gain, 88, u16);
reg_read_write!(feedforward_1st_gain, 90, u16);
reg_read_write!(bus_watchdog, 98, u8);

reg_read_write!(goal_pwm, 100, u16);
reg_read_write!(goal_current, 102, i16);
reg_read_write!(goal_velocity, 104, i32);
reg_read_write!(profile_acceleration, 108, u32);
reg_read_write!(profile_velocity, 112, u32);
reg_read_write!(goal_position, 116, i32);
reg_read_only!(realtime_tick, 120, u16);
reg_read_only!(moving, 122, u8);
reg_read_only!(moving_status, 123, u8);
reg_read_only!(present_pwm, 124, u16);
reg_read_only!(present_current, 126, i16);
reg_read_only!(present_velocity, 128, i32);
reg_read_only!(present_position, 132, i32);
reg_read_only!(velocity_trajectory, 136, u32);
reg_read_only!(position_trajectory, 140, u32);
reg_read_only!(present_input_voltage, 144, u16);
reg_read_only!(present_temperature, 146, u8);

/// Unit conversion for XM motors
pub mod conv {
    use std::f32::consts::PI;

    /// Dynamixel angular position to radians
    ///
    /// Works in joint and multi-turn mode
    /// 2048->180° is the center position with 0.088 [deg/pulse]

    pub fn dxl_pos_to_radians(pos: i32) -> f32 {
        (2.0 * PI * (pos as f32) / 4096.0) - PI
    }
    /// Radians to dynamixel angular position
    ///
    /// Works in joint and multi-turn mode
    pub fn radians_to_dxl_pos(rads: f32) -> i32 {
        (4096.0 * (PI + rads) / (2.0 * PI)) as i32
    }

    /// Dynamixel velocity in rpm
    ///
    /// Works for present_velocity instance
    pub fn dxl_vel_to_rpm(vel: i32) -> f32 {
        vel as f32 * 0.229
    }
    /// Velocity (rpm) to dynamixel velocity
    ///
    /// It should be in [-velocity_limit, +velocity_limit] with an absolute max at 1023 (324.267rpm)
    /// Works for goal_current for instance
    pub fn rpm_to_dxl_vel(rpm: f32) -> i32 {
        (rpm / 0.229) as i32
    }

    /// Dynamixel current to mA
    ///
    /// Works for present_current instance
    pub fn dxl_current_to_ma(current: i16) -> f32 {
        current as f32 * 2.69
    }
    /// Current (mA) to dynamixel current
    ///
    /// It should be in [-current_limit, +current_limit] with an absolute max at 1193 (3209.17mA)
    /// Works for goal_current for instance
    pub fn ma_to_dxl_current(current: f32) -> i16 {
        (current / 2.69) as i16
    }

    /// Dxl Temperature (°C)
    ///
    /// read_current_temperature
    pub fn dxl_to_temperature(temp: u8) -> f32 {
        temp as f32
    }

    /// Temperature (°C) to dxl
    ///
    /// write_temperature_limit
    pub fn temperature_to_dxl(temp: f32) -> u8 {
        temp as u8
    }

    /// Dynamixel pwm to %
    ///
    /// Works for present_pwm
    pub fn dxl_pwm_to_percentage(pwm: u16) -> f32 {
        pwm as f32 * 0.113
    }

    /// PWM (%) to dynamixel pwm
    ///
    /// Works for pwm_limit
    pub fn percentage_to_dxl_pwm(pwm: f32) -> u16 {
        (pwm / 0.113) as u16
    }

    /// Dynamixel voltage to V
    ///
    /// Works for present_voltage
    pub fn dxl_to_volt(volt: u16) -> f32 {
        volt as f32 * 0.1
    }

    /// V to dynamixel voltage
    ///
    /// Works for voltage_limit
    pub fn volt_to_dxl(volt: f32) -> u16 {
        (volt / 0.1) as u16
    }
}

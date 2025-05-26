//! XL-330 robotis register (protocol v2)
//!
//! See <https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/> for details.

use std::f64::consts::PI;

use crate::{generate_servo, servo::conversion::Conversion};

generate_servo!(
    XL330, v2,
    reg: (model_number, r, 0, u16, None),
    reg: (model_information, r, 2, u32, None),
    reg: (firmware_version, rw, 6, u8, None),
    reg: (id, rw, 7, u8, None),
    reg: (baud_rate, rw, 8, u8, None),
    reg: (return_delay_time, rw, 9, u8, None),
    reg: (drive_mode, rw, 10, u8, None),
    reg: (operating_mode, rw, 11, u8, None),
    reg: (secondary_id, rw, 12, u8, None),
    reg: (protocol_type, rw, 13, u8, None),
    reg: (homing_offset, rw, 20, i32, None),
    reg: (moving_threshold, rw, 24, u32, None),
    reg: (temperature_limit, rw, 31, u8, None),
    reg: (max_voltage_limit, rw, 32, u16, None),
    reg: (min_voltage_limit, rw, 34, u16, None),
    reg: (pwm_limit, rw, 36, u16, None),
    reg: (current_limit, rw, 38, u16, None),
    reg: (torque_limit, rw, 38, u16, None), //Duplicate with MX name for compatibility
    reg: (acceleration_limit, rw, 40, u32, None),
    reg: (velocity_limit, rw, 44, u32, None), //Duplicate with MX name for compatibility
    reg: (moving_speed, rw, 44, u32, None), //Duplicate with MX name for compatibility
    reg: (max_position_limit, rw, 48, i32, AnglePosition),
    reg: (min_position_limit, rw, 52, i32, AnglePosition),
    reg: (startup_configuration, rw, 60, u8, None),
    reg: (pwm_slope, rw, 62, u8, None),
    reg: (shutdown, rw, 63, u8, None),
    reg: (torque_enable, rw, 64, u8, bool),
    reg: (led, rw, 65, u8, None),
    reg: (status_return_level, rw, 68, u8, None),
    reg: (registered_instruction, rw, 69, u8, None),
    reg: (hardware_error_status, rw, 70, u8, None),
    reg: (velocity_i_gain, rw, 76, u16, None),
    reg: (velocity_p_gain, rw, 78, u16, None),
    reg: (position_d_gain, rw, 80, u16, None),
    reg: (position_i_gain, rw, 82, u16, None),
    reg: (position_p_gain, rw, 84, u16, None),
    reg: (feedforward_2nd_gain, rw, 88, u16, None),
    reg: (feedforward_1st_gain, rw, 90, u16, None),
    reg: (bus_watchdog, rw, 98, u8, None),
    reg: (goal_pwm, rw, 100, u16, None),
    reg: (goal_current, rw, 102, i16, None),
    reg: (goal_velocity, rw, 104, i32, None),
    reg: (profile_acceleration, rw, 108, u32, None),
    reg: (profile_velocity, rw, 112, u32, None),
    reg: (goal_position, rw, 116, i32, AnglePosition),
    reg: (realtime_tick, r, 120, u16, None),
    reg: (moving, r, 122, u8, None),
    reg: (moving_status, r, 123, u8, None),
    reg: (present_pwm, r, 124, u16, None),
    reg: (present_current, r, 126, i16, None),
    reg: (present_velocity, r, 128, i32, None),
    reg: (present_position, r, 132, i32, AnglePosition),
    reg: (velocity_trajectory, r, 136, u32, None),
    reg: (position_trajectory, r, 140, u32, None),
    reg: (present_input_voltage, r, 144, u16, None),
    reg: (present_temperature, r, 146, u8, None),
    reg: (backup_ready, r, 147, u8, None),
    reg: (indirect_address_1, rw, 168, u16, None),
    reg: (indirect_address_2, rw, 170, u16, None),
    reg: (indirect_address_3, rw, 172, u16, None),
    reg: (indirect_address_4, rw, 174, u16, None),
    reg: (indirect_address_5, rw, 176, u16, None),
    reg: (indirect_address_6, rw, 178, u16, None),
    reg: (indirect_data_1, rw, 224, u8, None),
    reg: (indirect_data_2, rw, 225, u8, None),
    reg: (indirect_data_3, rw, 226, u8, None),
    reg: (indirect_data_4, rw, 227, u8, None),
    reg: (indirect_data_5, rw, 228, u8, None),
    reg: (indirect_data_6, rw, 229, u8, None),
);

pub struct AnglePosition;

impl Conversion for AnglePosition {
    type RegisterType = i32;
    type UsiType = f64;

    fn from_raw(raw: i32) -> f64 {
        (2.0 * PI * (raw as f64) / 4096.0) - PI
    }

    fn to_raw(value: f64) -> i32 {
        (4096.0 * (PI + value) / (2.0 * PI)) as i32
    }
}

/// Unit conversion for XL330 motors (same as XM?)
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
        current as f32 * 1.0
    }
    /// Current (mA) to dynamixel current
    ///
    /// It should be in [-current_limit, +current_limit] with an absolute max at 1193 (3209.17mA)
    /// Works for goal_current for instance
    pub fn ma_to_dxl_current(current: f32) -> i16 {
        (current / 1.0) as i16
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

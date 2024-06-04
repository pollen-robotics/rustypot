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
reg_read_write!(goal_current, 102, u16);
reg_read_write!(goal_velocity, 104, u32);
reg_read_write!(profile_acceleration, 108, u32);
reg_read_write!(profile_velocity, 112, u32);
reg_read_write!(goal_position, 116, u32);
reg_read_only!(realtime_tick, 120, u16);
reg_read_only!(moving, 122, u8);
reg_read_only!(moving_status, 123, u8);
reg_read_only!(present_pwm, 124, u16);
reg_read_only!(present_current, 126, u16);
reg_read_only!(present_velocity, 128, u32);
reg_read_only!(present_position, 132, u32);
reg_read_only!(velocity_trajectory, 136, u32);
reg_read_only!(position_trajectory, 140, u32);
reg_read_only!(present_input_voltage, 144, u16);
reg_read_only!(present_temperature, 146, u8);



/// Unit conversion for XM motors
pub mod conv {
    use std::f64::consts::PI;

    /// Dynamixel angular position to radians
    ///
    /// Works in joint and multi-turn mode
    pub fn dxl_pos_to_radians(pos: i32) -> f64 {
        (2.0 * PI * (pos as f64) / 4096.0) - PI
    }
    /// Radians to dynamixel angular position
    ///
    /// Works in joint and multi-turn mode
    pub fn radians_to_dxl_pos(rads: f64) -> i32 {
        (4096.0 * (PI + rads) / (2.0 * PI)) as i32
    }

    /// Dynamixel absolute speed to radians per second
    ///
    /// Works for moving_speed in joint mode for instance
    pub fn dxl_abs_speed_to_rad_per_sec(speed: u32) -> f64 {
        let rpm = speed as f64 * 0.229;
        rpm * 0.10472
    }
    /// Radians per second to dynamixel absolute speed
    ///
    /// Works for moving_speed in joint mode for instance
    pub fn rad_per_sec_to_dxl_abs_speed(speed: f64) -> u32 {
        let rpm = speed / 0.10472;
        (rpm / 0.229) as u32
    }
    /// Dynamixel speed to radians per second
    ///
    /// Works for present_speed for instance
    pub fn dxl_oriented_speed_to_rad_per_sec(speed: u32) -> f64 {
        let cw = (speed >> 11) == 1;

        let rad_per_sec = dxl_abs_speed_to_rad_per_sec(speed % 1024);

        match cw {
            true => rad_per_sec,
            false => -rad_per_sec,
        }
    }
    /// Radians per second to dynamixel speed
    ///
    /// Works for present_speed for instance
    pub fn rad_per_sec_to_dxl_oriented_speed(speed: f64) -> u32 {
        let raw = rad_per_sec_to_dxl_abs_speed(speed.abs());

        match speed < 0.0 {
            true => raw,
            false => raw + 2048,
        }
    }

    /// Dynamixel absolute load to torque percentage
    ///
    /// Works for torque_limit for instance
    pub fn dxl_load_to_abs_torque(load: u16) -> f64 {
        load as f64 / 1193.0 * 100.0
    }
    /// Torque percentage to dynamixel absolute load
    ///
    /// Works for torque_limit for instance
    pub fn torque_to_dxl_abs_load(torque: f64) -> u16 {
        assert!((0.0..=100.0).contains(&torque));

        (torque * 1193.0 / 100.0) as u16
    }
    /// Dynamixel load to torque percentage
    ///
    /// Works for present_torque for instance
    pub fn dxl_load_to_oriented_torque(load: u16) -> f64 {
        let cw = (load >> 10) == 1;

        let torque = dxl_load_to_abs_torque(load % 1193);

        match cw {
            true => torque,
            false => -torque,
        }
    }
    /// Torque percentage to dynamixel load
    pub fn oriented_torque_to_dxl_load(torque: f64) -> u16 {
        let load = torque_to_dxl_abs_load(torque.abs());

        match torque < 0.0 {
            true => load,
            false => load + 1193,
        }
    }
}

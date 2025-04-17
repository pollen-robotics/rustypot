//! MX robotis register (protocol v1)
//!
//! Despite some minor differences among MX variants, it should work for
//! * MX-28
//! * MX-64
//! * MX-106
//!
//! See <https://emanual.robotis.com/docs/en/dxl/mx/mx-28/> for example.

use crate::generate_servo;

generate_servo!(
    MX, v1,
    reg: (model_number, r, 0, u16),
    reg: (firmware_version, r, 2, u8),
    reg: (id, rw, 3, u8),
    reg: (baudrate, rw, 4, u8),
    reg: (return_delay_time, rw, 5, u8),
    reg: (cw_angle_limit, rw, 6, u16),
    reg: (ccw_angle_limit, rw, 8, u16),
    reg: (temperature_limit, rw, 11, u8),
    reg: (min_voltage_limit, rw, 12, u8),
    reg: (max_voltage_limit, rw, 13, u8),
    reg: (max_torque, rw, 14, u16),
    reg: (status_return_level, rw, 16, u8),
    reg: (alarm_led, rw, 17, u8),
    reg: (shutdown, rw, 18, u8),
    reg: (multi_turn_offset, rw, 20, i16),
    reg: (resolution_divider, rw, 22, u8),
    reg: (torque_enable, rw, 24, u8),
    reg: (led, rw, 25, u8),
    reg: (d_gain, rw, 26, u8),
    reg: (i_gain, rw, 27, u8),
    reg: (p_gain, rw, 28, u8),
    reg: (goal_position, rw, 30, i16),
    reg: (moving_speed, rw, 32, u16),
    reg: (torque_limit, rw, 34, u16),
    reg: (present_position, r, 36, i16),
    reg: (present_speed, r, 38, u16),
    reg: (present_load, r, 40, u16),
    reg: (present_voltage, r, 42, u8),
    reg: (present_temperature, r, 43, u8),
    reg: (registered, r, 44, u8),
    reg: (moving, r, 46, u8),
    reg: (lock, rw, 47, u8),
    reg: (punch, rw, 48, u16),
    reg: (realtime_tick, r, 50, u16),
    reg: (goal_acceleration, rw, 73, u8),
);

// pub type MX28Controller = MXController;
// pub type MX64Controller = MXController;
// pub type MX106Controller = MXController;

/// Sync read present_position, present_speed and present_load in one message
///
/// reg_read_only!(present_position_speed_load, 36, (i16, u16, u16))
pub fn sync_read_present_position_speed_load(
    dph: &crate::DynamixelProtocolHandler,
    serial_port: &mut dyn serialport::SerialPort,
    ids: &[u8],
) -> crate::Result<Vec<(i16, u16, u16)>> {
    let val = dph.sync_read(serial_port, ids, 36, 2 + 2 + 2)?;
    let val = val
        .iter()
        .map(|v| {
            (
                i16::from_le_bytes(v[0..2].try_into().unwrap()),
                u16::from_le_bytes(v[2..4].try_into().unwrap()),
                u16::from_le_bytes(v[4..6].try_into().unwrap()),
            )
        })
        .collect();

    Ok(val)
}

/// Unit conversion for MX motors
pub mod conv {
    use std::f64::consts::PI;

    /// Dynamixel angular position to radians
    ///
    /// Works in joint and multi-turn mode
    pub fn dxl_pos_to_radians(pos: i16) -> f64 {
        (2.0 * PI * (pos as f64) / 4096.0) - PI
    }
    /// Radians to dynamixel angular position
    ///
    /// Works in joint and multi-turn mode
    pub fn radians_to_dxl_pos(rads: f64) -> i16 {
        (4096.0 * (PI + rads) / (2.0 * PI)) as i16
    }

    /// Dynamixel absolute speed to radians per second
    ///
    /// Works for moving_speed in joint mode for instance
    pub fn dxl_abs_speed_to_rad_per_sec(speed: u16) -> f64 {
        let rpm = speed as f64 * 0.114;
        rpm * 0.10472
    }
    /// Radians per second to dynamixel absolute speed
    ///
    /// Works for moving_speed in joint mode for instance
    pub fn rad_per_sec_to_dxl_abs_speed(speed: f64) -> u16 {
        let rpm = speed / 0.10472;
        (rpm / 0.114) as u16
    }
    /// Dynamixel speed to radians per second
    ///
    /// Works for present_speed for instance
    pub fn dxl_oriented_speed_to_rad_per_sec(speed: u16) -> f64 {
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
    pub fn rad_per_sec_to_dxl_oriented_speed(speed: f64) -> u16 {
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
        load as f64 / 1023.0 * 100.0
    }
    /// Torque percentage to dynamixel absolute load
    ///
    /// Works for torque_limit for instance
    pub fn torque_to_dxl_abs_load(torque: f64) -> u16 {
        assert!((0.0..=100.0).contains(&torque));

        (torque * 1023.0 / 100.0) as u16
    }
    /// Dynamixel load to torque percentage
    ///
    /// Works for present_torque for instance
    pub fn dxl_load_to_oriented_torque(load: u16) -> f64 {
        let cw = (load >> 10) == 1;

        let torque = dxl_load_to_abs_torque(load % 1024);

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
            false => load + 1024,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::conv::*;

    #[test]
    fn position_conversions() {
        assert_eq!(radians_to_dxl_pos(0.0), 2048);
        assert_eq!(radians_to_dxl_pos(-PI / 2.0), 1024);
        assert_eq!(radians_to_dxl_pos(PI / 2.0), 3072);
        assert_eq!(dxl_pos_to_radians(2048), 0.0);
    }

    #[test]
    fn abs_speed_conversions() {
        assert_eq!(rad_per_sec_to_dxl_abs_speed(0.0), 0);
        assert_eq!(rad_per_sec_to_dxl_abs_speed(0.5), 41);
    }

    #[test]
    fn speed_conversions() {
        assert_eq!(dxl_oriented_speed_to_rad_per_sec(300), -3.581424);
        assert_eq!(dxl_oriented_speed_to_rad_per_sec(2048 + 300), 3.581424);

        assert_eq!(rad_per_sec_to_dxl_oriented_speed(-3.581424), 300);
        assert_eq!(rad_per_sec_to_dxl_oriented_speed(3.581424), 2048 + 300);
    }

    #[test]
    fn torque_conversions() {
        assert_eq!(torque_to_dxl_abs_load(0.0), 0);
        assert_eq!(torque_to_dxl_abs_load(50.0), 511);
        assert_eq!(torque_to_dxl_abs_load(100.0), 1023);

        assert_eq!(dxl_load_to_abs_torque(0), 0.0);
        assert!((dxl_load_to_abs_torque(511) - 50.0).abs() < 1e-1);
        assert_eq!(dxl_load_to_abs_torque(1023), 100.0);
    }

    #[test]
    fn load_conversions() {
        assert!((dxl_load_to_oriented_torque(511) + 50.0).abs() < 1e-1);
        assert!((dxl_load_to_oriented_torque(1024 + 512) - 50.0).abs() < 1e-1);

        assert_eq!(oriented_torque_to_dxl_load(-50.0), 511);
        assert_eq!(oriented_torque_to_dxl_load(50.0), 1024 + 511);
    }
}

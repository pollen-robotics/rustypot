//! XL-320 robotis register (protocol v2)
//!
//! See <https://emanual.robotis.com/docs/en/dxl/x/xl320/> for details.

use crate::device::*;

reg_read_only!(model_number, 0, u16);
reg_read_only!(firmware_version, 2, u8);
reg_read_write!(id, 3, u8);
reg_read_write!(baudrate, 4, u8);
reg_read_write!(return_delay_time, 5, u8);
reg_read_write!(cw_angle_limit, 6, u16);
reg_read_write!(ccw_angle_limit, 8, u16);
reg_read_write!(control_mode, 11, u8);
reg_read_write!(temperature_limit, 12, u8);
reg_read_write!(min_voltage_limit, 13, u8);
reg_read_write!(max_voltage_limit, 14, u8);
reg_read_write!(max_torque, 15, u16);
reg_read_write!(status_return_level, 17, u8);
reg_read_write!(shutdown, 18, u8);

reg_read_write!(torque_enable, 24, u8);
reg_read_write!(led, 25, u8);
reg_read_write!(d_gain, 27, u8);
reg_read_write!(i_gain, 28, u8);
reg_read_write!(p_gain, 29, u8);
reg_read_write!(goal_position, 30, i16);
reg_read_write!(moving_speed, 32, u16);
reg_read_write!(torque_limit, 35, u16);
reg_read_only!(present_position, 37, i16);
reg_read_only!(present_speed, 39, u16);
reg_read_only!(present_load, 41, u16);
reg_read_only!(present_voltage, 45, u8);
reg_read_only!(present_temperature, 46, u8);
reg_read_only!(registered, 47, u8);
reg_read_only!(moving, 49, u8);
reg_read_only!(hardware_error_status, 50, u8);
reg_read_write!(punch, 51, u16);

/// Sync read present_position, present_speed and present_load in one message
///
/// reg_read_only!(present_position_speed_load, 36, (i16, u16, u16))
pub fn sync_read_present_position_speed_load(
    dph: &DynamixelProtocolHandler,
    serial_port: &mut dyn serialport::SerialPort,
    ids: &[u8],
) -> Result<Vec<(i16, u16, u16)>> {
    let val = dph.sync_read(serial_port, ids, 37, 2 + 2 + 2)?;
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

/// Unit conversion for XL-320 motors
pub mod conv {
    /// Dynamixel angular position to radians
    ///
    /// Works in joint and multi-turn mode
    pub fn xl320_pos_to_radians(pos: i16) -> f64 {
        (300.0_f64.to_radians() * (pos as f64) / 1024.0) - 150.0_f64.to_radians()
    }
    /// Radians to dynamixel angular position
    ///
    /// Works in joint and multi-turn mode
    pub fn radians_to_xl320_pos(rads: f64) -> i16 {
        (1024.0 * (150.0_f64.to_radians() + rads) / 300.0_f64.to_radians()) as i16
    }

    /// Dynamixel absolute speed to radians per second
    ///
    /// Works for moving_speed in joint mode for instance
    pub fn xl320_abs_speed_to_rad_per_sec(speed: u16) -> f64 {
        let rpm = speed as f64 * 0.111;
        rpm * 0.10472
    }
    /// Radians per second to dynamixel absolute speed
    ///
    /// Works for moving_speed in joint mode for instance
    pub fn rad_per_sec_to_xl320_abs_speed(speed: f64) -> u16 {
        let rpm = speed / 0.10472;
        (rpm / 0.111) as u16
    }
    /// Dynamixel speed to radians per second
    ///
    /// Works for present_speed for instance
    pub fn xl320_oriented_speed_to_rad_per_sec(speed: u16) -> f64 {
        let cw = (speed >> 11) == 1;

        let rad_per_sec = xl320_abs_speed_to_rad_per_sec(speed % 1024);

        match cw {
            true => rad_per_sec,
            false => -rad_per_sec,
        }
    }
    /// Radians per second to dynamixel speed
    ///
    /// Works for present_speed for instance
    pub fn rad_per_sec_to_xl320_oriented_speed(speed: f64) -> u16 {
        let raw = rad_per_sec_to_xl320_abs_speed(speed.abs());

        match speed < 0.0 {
            true => raw,
            false => raw + 2048,
        }
    }

    /// Dynamixel absolute load to torque percentage
    ///
    /// Works for torque_limit for instance
    pub fn xl320_load_to_abs_torque(load: u16) -> f64 {
        load as f64 / 1023.0 * 100.0
    }
    /// Torque percentage to dynamixel absolute load
    ///
    /// Works for torque_limit for instance
    pub fn torque_to_xl320_abs_load(torque: f64) -> u16 {
        assert!(torque >= 0.0);
        assert!(torque <= 100.0);

        (torque * 1023.0 / 100.0) as u16
    }
    /// Dynamixel load to torque percentage
    ///
    /// Works for present_torque for instance
    pub fn xl320_load_to_oriented_torque(load: u16) -> f64 {
        let cw = (load >> 10) == 1;

        let torque = xl320_load_to_abs_torque(load % 1024);

        match cw {
            true => torque,
            false => -torque,
        }
    }
    /// Torque percentage to dynamixel load
    pub fn oriented_torque_to_xl320_load(torque: f64) -> u16 {
        let load = torque_to_xl320_abs_load(torque.abs());

        match torque < 0.0 {
            true => load,
            false => load + 1024,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::conv::*;

    #[test]
    fn position_conversions() {
        assert_eq!(radians_to_xl320_pos(0.0), 512);
        assert_eq!(radians_to_xl320_pos(-150.0_f64.to_radians()), 0);
        assert_eq!(radians_to_xl320_pos(150.0_f64.to_radians()), 1024);

        assert_eq!(xl320_pos_to_radians(0), -150.0_f64.to_radians());
        assert_eq!(xl320_pos_to_radians(512), 0.0);
        assert_eq!(xl320_pos_to_radians(1024), 150.0_f64.to_radians());
    }

    #[test]
    fn abs_speed_conversions() {
        assert_eq!(rad_per_sec_to_xl320_abs_speed(0.0), 0);
        assert_eq!(rad_per_sec_to_xl320_abs_speed(0.5), 43);

        assert_eq!(
            rad_per_sec_to_xl320_abs_speed(xl320_abs_speed_to_rad_per_sec(66)),
            66
        );
    }

    #[test]
    fn speed_conversions() {
        assert_eq!(xl320_oriented_speed_to_rad_per_sec(99), -1.15076808);
        assert_eq!(xl320_oriented_speed_to_rad_per_sec(2048 + 99), 1.15076808);

        assert_eq!(
            rad_per_sec_to_xl320_oriented_speed(xl320_oriented_speed_to_rad_per_sec(42)),
            42
        );
    }

    #[test]
    fn torque_conversions() {
        assert_eq!(torque_to_xl320_abs_load(0.0), 0);
        assert_eq!(torque_to_xl320_abs_load(50.0), 511);
        assert_eq!(torque_to_xl320_abs_load(100.0), 1023);

        assert_eq!(xl320_load_to_abs_torque(0), 0.0);
        assert!((xl320_load_to_abs_torque(511) - 50.0).abs() < 1e-1);
        assert_eq!(xl320_load_to_abs_torque(1023), 100.0);
    }

    #[test]
    fn load_conversions() {
        assert!((xl320_load_to_abs_torque(511) - 50.0).abs() < 1e-1);
        assert!((xl320_load_to_oriented_torque(1024 + 511) - 50.0).abs() < 1e-1);

        assert!(
            (xl320_load_to_oriented_torque(oriented_torque_to_xl320_load(25.0)) - 25.0).abs()
                < 1e-1
        );
    }
}

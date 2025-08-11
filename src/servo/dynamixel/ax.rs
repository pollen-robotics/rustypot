//! AX robotis register (protocol v1)
//!
//! Despite some minor differences among AX variants, it should work for
//! * AX-12
//! * AX-12+
//! * AX-12A
//! * AX-12W
//! * AX-18A
//!
//! See <https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/> for example.

use crate::{generate_servo, servo::conversion::Conversion};

generate_servo!(
    AX, v1,
    reg: (model_number, r, 0, u16, None),
    reg: (firmware_version, r, 2, u8, None),
    reg: (id, rw, 3, u8, None),
    reg: (baudrate, rw, 4, u8, None),
    reg: (return_delay_time, rw, 5, u8, None),
    reg: (cw_angle_limit, rw, 6, u16, AnglePosition),
    reg: (ccw_angle_limit, rw, 8, u16, AnglePosition),
    reg: (temperature_limit, rw, 11, u8, None),
    reg: (min_voltage_limit, rw, 12, u8, None),
    reg: (max_voltage_limit, rw, 13, u8, None),
    reg: (max_torque, rw, 14, u16, None),
    reg: (status_return_level, rw, 16, u8, None),
    reg: (alarm_led, rw, 17, u8, None),
    reg: (shutdown, rw, 18, u8, None),
    reg: (torque_enable, rw, 24, u8, None),
    reg: (led, rw, 25, u8, None),
    reg: (cw_compliance_margin, rw, 26, u8, None),
    reg: (ccw_compliance_margin, rw, 27, u8, None),
    reg: (cw_compliance_slope, rw, 28, u8, None),
    reg: (ccw_compliance_slope, rw, 29, u8, None),
    reg: (goal_position, rw, 30, u16, AnglePosition),
    reg: (moving_speed, rw, 32, u16, None),
    reg: (torque_limit, rw, 34, u16, None),
    reg: (present_position, r, 36, u16, AnglePosition),
    reg: (present_speed, r, 38, u16, None),
    reg: (present_load, r, 40, u16, None),
    reg: (present_voltage, r, 42, u8, None),
    reg: (present_temperature, r, 43, u8, None),
    reg: (registered, r, 44, u8, None),
    reg: (moving, r, 46, u8, None),
    reg: (lock, rw, 47, u8, None),
    reg: (punch, rw, 48, u16, None),
);

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

pub struct AnglePosition;
const MAX_DEFLECTION: f64 = 150f64.to_radians(); // -150 to 150 deg (exclusive)
impl Conversion for AnglePosition {
    type RegisterType = u16;
    type UsiType = f64;

    fn from_raw(raw: u16) -> f64 {
        (2.0 * MAX_DEFLECTION * (raw as f64) / 1024.0) - MAX_DEFLECTION
    }

    fn to_raw(value: f64) -> u16 {
        (1024.0 * (MAX_DEFLECTION + value) / (2.0 * MAX_DEFLECTION)) as u16
    }
}

/// Unit conversion for AX motors
pub mod conv {

    const RPM_PER_DXL_SPEED: f64 = 0.111;
    const RPM_TO_RADS_FACTOR: f64 = 2.0 * std::f64::consts::PI / 60.0;

    /// Dynamixel absolute speed to radians per second
    ///
    /// Works for moving_speed in joint mode for instance
    pub fn dxl_abs_speed_to_rad_per_sec(speed: u16) -> f64 {
        let rpm = speed as f64 * RPM_PER_DXL_SPEED;
        rpm * RPM_TO_RADS_FACTOR
    }

    /// Radians per second to dynamixel absolute speed
    ///
    /// Works for moving_speed in joint mode for instance
    pub fn rad_per_sec_to_dxl_abs_speed(speed: f64) -> u16 {
        let rpm = speed / RPM_TO_RADS_FACTOR;
        (rpm / RPM_PER_DXL_SPEED) as u16
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
    use crate::servo::{conversion::Conversion, dynamixel::ax::AnglePosition};

    use super::conv::*;

    #[test]
    fn position_conversions() {
        assert_eq!(AnglePosition::to_raw(0.0), 512);
        assert_eq!(AnglePosition::to_raw(-150f64.to_radians()), 0);
        assert_eq!(AnglePosition::to_raw(149.9f64.to_radians()), 1023); // 150 is invalid as per spec.
        assert_eq!(AnglePosition::from_raw(512), 0.0);
    }

    #[test]
    fn abs_speed_conversions() {
        assert_eq!(rad_per_sec_to_dxl_abs_speed(0.0), 0);
        assert_eq!(rad_per_sec_to_dxl_abs_speed(0.5), 43);
    }

    #[test]
    fn speed_conversions() {
        assert_eq!(dxl_oriented_speed_to_rad_per_sec(300), -3.4871678454846697);
        assert_eq!(
            dxl_oriented_speed_to_rad_per_sec(2048 + 300),
            3.4871678454846697
        );

        // FP error if not 299, but close enough.
        assert_eq!(rad_per_sec_to_dxl_oriented_speed(-3.4871678454846697), 299);
        assert_eq!(
            rad_per_sec_to_dxl_oriented_speed(3.4871678454846697),
            2048 + 299
        );
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

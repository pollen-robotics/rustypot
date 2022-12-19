use crate::device::*;

reg_read_only!(model_number, 0, u16);
reg_read_only!(firmware_version, 2, u8);
reg_read_write!(id, 3, u8);
reg_read_write!(baudrate, 4, u8);
reg_read_write!(return_delay_time, 5, u8);
reg_read_write!(cw_angle_limit, 6, u16);
reg_read_write!(ccw_angle_limit, 8, u16);
reg_read_write!(temperature_limit, 11, u8);
reg_read_write!(min_voltage_limit, 12, u8);
reg_read_write!(max_voltage_limit, 13, u8);
reg_read_write!(max_torque, 14, u16);
reg_read_write!(status_return_level, 16, u8);
reg_read_write!(alarm_led, 17, u8);
reg_read_write!(shutdown, 18, u8);
reg_read_write!(multi_turn_offset, 20, i16);
reg_read_write!(resolution_divider, 22, u8);

reg_read_write!(torque_enable, 24, u8);
reg_read_write!(led, 25, u8);
reg_read_write!(d_gain, 26, u8);
reg_read_write!(i_gain, 27, u8);
reg_read_write!(p_gain, 28, u8);
reg_read_write!(goal_position, 30, i16);
reg_read_write!(moving_speed, 32, u16);
reg_read_write!(torque_limit, 34, u16);
reg_read_only!(present_position, 36, i16);
reg_read_only!(present_speed, 38, u16);
reg_read_only!(present_load, 40, u16);
reg_read_only!(present_voltage, 42, u8);
reg_read_only!(present_temperature, 43, u8);
reg_read_only!(registered, 44, u8);
reg_read_only!(moving, 46, u8);
reg_read_write!(lock, 47, u8);
reg_read_write!(punch, 48, u16);
reg_read_only!(realtime_tick, 50, u16);
reg_read_write!(goal_acceleration, 73, u8);

pub mod conv {
    use std::f64::consts::PI;

    pub fn dxl_pos_to_radians(pos: i16) -> f64 {
        (2.0 * PI * (pos as f64) / 4096.0) - PI
    }

    pub fn radians_to_dxl_pos(rads: f64) -> i16 {
        (4096.0 * (PI + rads) / (2.0 * PI)) as i16
    }

    pub fn dxl_abs_speed_to_rad_per_sec(speed: u16) -> f64 {
        let rpm = speed as f64 * 0.114;
        rpm * 0.10472
    }

    pub fn rad_per_sec_to_dxl_abs_speed(speed: f64) -> u16 {
        let rpm = speed / 0.10472;
        (rpm / 0.114) as u16
    }

    pub fn dxl_oriented_speed_to_rad_per_sec(speed: u16) -> f64 {
        let cw = (speed >> 11) == 1;

        let rad_per_sec = dxl_abs_speed_to_rad_per_sec(speed % 1024);

        match cw {
            true => rad_per_sec,
            false => -rad_per_sec,
        }
    }

    pub fn rad_per_sec_to_dxl_oriented_speed(speed: f64) -> u16 {
        let raw = rad_per_sec_to_dxl_abs_speed(speed.abs());

        match speed < 0.0 {
            true => raw,
            false => raw + 2048,
        }
    }

    pub fn torque_to_dxl_abs_load(torque: f64) -> u16 {
        assert!(torque >= 0.0);
        assert!(torque <= 100.0);

        (torque * 1024.0 / 100.0) as u16
    }

    pub fn oriented_torque_to_dxl_load(torque: f64) -> u16 {
        let load = torque_to_dxl_abs_load(torque.abs());

        match torque < 0.0 {
            true => load,
            false => load + 1024,
        }
    }

    pub fn dxl_load_to_abs_torque(load: u16) -> f64 {
        load as f64 / 1024.0 * 100.0
    }

    pub fn dxl_load_to_oriented_torque(load: u16) -> f64 {
        let cw = (load >> 10) == 1;

        let torque = dxl_load_to_abs_torque(load % 1024);

        match cw {
            true => torque,
            false => -torque,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::conv::*;

    #[test]
    fn position_conversions() {
        assert_eq!(radians_to_dxl_pos(0.0), 2048);
        assert_eq!(dxl_pos_to_radians(2048), 0.0);
    }

    #[test]
    fn speed_conversions() {
        assert_eq!(dxl_oriented_speed_to_rad_per_sec(300), -3.581424);
        assert_eq!(dxl_oriented_speed_to_rad_per_sec(2048 + 300), 3.581424);

        assert_eq!(rad_per_sec_to_dxl_oriented_speed(-3.581424), 300);
        assert_eq!(rad_per_sec_to_dxl_oriented_speed(3.581424), 2048 + 300);
    }

    #[test]
    fn load_conversions() {
        assert_eq!(dxl_load_to_oriented_torque(512), -50.0);
        assert_eq!(dxl_load_to_oriented_torque(1024 + 512), 50.0);

        assert_eq!(oriented_torque_to_dxl_load(-50.0), 512);
        assert_eq!(oriented_torque_to_dxl_load(50.0), 1024 + 512);
    }
}

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

pub mod conv {
    use std::f64::consts::PI;

    pub fn xl320_pos_to_radians(pos: i16) -> f64 {
        (300.0_f64.to_radians() * (pos as f64) / 1024.0) - PI
    }

    pub fn radians_to_xl320_pos(rads: f64) -> i16 {
        (1024.0 * (PI + rads) / 300.0_f64.to_radians()) as i16
    }

    pub fn xl320_abs_speed_to_rad_per_sec(speed: u16) -> f64 {
        let rpm = speed as f64 * 0.111;
        rpm * 0.10472
    }

    pub fn rad_per_sec_to_xl320_abs_speed(speed: f64) -> u16 {
        let rpm = speed / 0.10472;
        (rpm / 0.111) as u16
    }

    pub fn xl320_oriented_speed_to_rad_per_sec(speed: u16) -> f64 {
        let cw = (speed >> 11) == 1;

        let rad_per_sec = xl320_abs_speed_to_rad_per_sec(speed % 1024);

        match cw {
            true => rad_per_sec,
            false => -rad_per_sec,
        }
    }

    pub fn rad_per_sec_to_xl320_oriented_speed(speed: f64) -> u16 {
        let raw = rad_per_sec_to_xl320_abs_speed(speed.abs());

        match speed < 0.0 {
            true => raw,
            false => raw + 2048,
        }
    }

    pub fn torque_to_xl320_abs_load(torque: f64) -> u16 {
        assert!(torque >= 0.0);
        assert!(torque <= 100.0);

        (torque * 1024.0 / 100.0) as u16
    }

    pub fn oriented_torque_to_xl320_load(torque: f64) -> u16 {
        let load = torque_to_xl320_abs_load(torque.abs());

        match torque < 0.0 {
            true => load,
            false => load + 1024,
        }
    }

    pub fn xl320_load_to_abs_torque(load: u16) -> f64 {
        load as f64 / 1024.0 * 100.0
    }

    pub fn xl320_load_to_oriented_torque(load: u16) -> f64 {
        let cw = (load >> 10) == 1;

        let torque = xl320_load_to_abs_torque(load % 1024);

        match cw {
            true => torque,
            false => -torque,
        }
    }
}
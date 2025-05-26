use std::f64::consts::PI;

use crate::generate_servo;
use crate::servo::conversion::Conversion;
use crate::servo::dynamixel::mx::AnglePosition;

generate_servo!(
    STS3215, v1,
    reg: (model, r, 3, u16, None),
    reg: (id, rw, 5, u8, None),
    reg: (baudrate, rw, 6, u8, None),
    reg: (return_delay_time, rw, 7, u8, None),
    reg: (response_status_level, rw, 8, u8, None),
    reg: (min_angle_limit, rw, 9, i16, AnglePosition),
    reg: (max_angle_limit, rw, 11, i16, AnglePosition),
    reg: (max_temperature_limit, rw, 13, u8, None),
    reg: (max_voltage_limit, rw, 14, u8, None),
    reg: (min_voltage_limit, rw, 15, u8, None),
    reg: (max_torque_limit, rw, 16, u16, None),
    reg: (phase, rw, 18, u8, None),
    reg: (unloading_condition, rw, 19, u8, None),
    reg: (led_alarm_condition, rw, 20, u8, None),
    reg: (p_coefficient, rw, 21, u8, None),
    reg: (d_coefficient, rw, 22, u8, None),
    reg: (i_coefficient, rw, 23, u8, None),
    reg: (minimum_startup_force, rw, 24, u16, None),
    reg: (cw_dead_zone, rw, 26, u8, None),
    reg: (ccw_dead_zone, rw, 27, u8, None),
    reg: (protection_current, rw, 28, u16, None),
    reg: (angular_resolution, rw, 30, u8, None),
    reg: (offset, rw, 31, i16, AnglePosition),
    reg: (mode, rw, 33, u8, None),
    reg: (protective_torque, rw, 34, u8, None),
    reg: (protection_time, rw, 35, u8, None),
    reg: (overload_torque, rw, 36, u8, None),
    reg: (speed_closed_loop_p_coefficient, rw, 37, u8, None),
    reg: (over_current_protection_time, rw, 38, u8, None),
    reg: (velocity_closed_loop_i_coefficient, rw, 39, u8, None),
    reg: (torque_enable, rw, 40, u8, bool),
    reg: (acceleration, rw, 41, u8, None),
    reg: (goal_position, rw, 42, i16, AnglePosition),
    reg: (goal_time, rw, 44, u16, None),
    reg: (goal_speed, rw, 46, u16, Velocity),
    reg: (torque_limit, rw, 48, u16, None),
    reg: (lock, rw, 55, u8, bool),
    reg: (present_position, r, 56, i16, AnglePosition),
    reg: (present_speed, r, 58, u16, Velocity),
    reg: (present_load, r, 60, u16, None),
    reg: (present_voltage, r, 62, u8, None),
    reg: (present_temperature, r, 63, u8, None),
    reg: (status, r, 65, u8, None),
    reg: (moving, r, 66, u8, bool),
    reg: (present_current, r, 69, u16, None),
    reg: (maximum_acceleration, rw, 85, u16, None),
);

pub struct Velocity;

impl Conversion for Velocity {
    type RegisterType = u16;
    type UsiType = f64;

    fn from_raw(raw: u16) -> f64 {
        let mut value = raw as f64;
        if value > ((1 << 15) as f64) {
            value = -(value - ((1 << 15) as f64));
        }
        (2.0 * PI * value) / (4096.0 - 1.0)
    }

    fn to_raw(value: f64) -> u16 {
        let mut value = (4096.0 - 1.0) * value / (2.0 * PI);
        if value < 0.0 {
            value = -value + (1 << 15) as f64;
        }
        value as u16
    }
}

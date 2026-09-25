use std::f64::consts::PI;

use crate::generate_servo;
use crate::servo::conversion::Conversion;
use crate::servo::dynamixel::mx::AnglePosition;

generate_servo!(
    STS3215, v1,
    resolution: 4096,
    baudrates: [
        (1_000_000, 0), (500_000, 1), (250_000, 2), (128_000, 3),
        (115_200, 4), (57_600, 5), (38_400, 6), (19_200, 7),
    ],
    // Positions, speeds and the offset are sign-magnitude; the limits are plain steps.
    encoding: [
        (min_angle_limit, unsigned), (min_position_limit, unsigned),
        (max_angle_limit, unsigned), (max_position_limit, unsigned),
        (offset, sign_magnitude(11)), (homing_offset, sign_magnitude(11)),
        (goal_position, sign_magnitude(15)), (present_position, sign_magnitude(15)),
        (goal_speed, sign_magnitude(15)), (goal_velocity, sign_magnitude(15)),
        (present_speed, sign_magnitude(15)), (present_velocity, sign_magnitude(15)),
        (present_load, sign_magnitude(10)),
    ],
    reg: (firmware_major_version, r, 0, u8, None),
    reg: (firmware_minor_version, r, 1, u8, None),
    reg: (model, r, 3, u16, None),
    reg: (model_number, r, 3, u16, None),
    reg: (id, rw, 5, u8, None),
    reg: (baudrate, rw, 6, u8, None),
    reg: (baud_rate, rw, 6, u8, None),
    reg: (return_delay_time, rw, 7, u8, None),
    reg: (response_status_level, rw, 8, u8, None),
    reg: (min_angle_limit, rw, 9, i16, AnglePosition),
    reg: (min_position_limit, rw, 9, i16, AnglePosition),
    reg: (max_angle_limit, rw, 11, i16, AnglePosition),
    reg: (max_position_limit, rw, 11, i16, AnglePosition),
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
    reg: (offset, rw, 31, u16, Offset),
    reg: (homing_offset, rw, 31, u16, Offset),
    reg: (mode, rw, 33, u8, None),
    reg: (operating_mode, rw, 33, u8, None),
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
    reg: (goal_velocity, rw, 46, u16, Velocity),
    reg: (torque_limit, rw, 48, u16, None),
    reg: (lock, rw, 55, u8, bool),
    reg: (present_position, r, 56, i16, AnglePosition),
    reg: (present_speed, r, 58, u16, Velocity),
    reg: (present_velocity, r, 58, u16, Velocity),
    reg: (present_load, r, 60, u16, None),
    reg: (present_voltage, r, 62, u8, None),
    reg: (present_temperature, r, 63, u8, None),
    reg: (status, r, 65, u8, None),
    reg: (moving, r, 66, u8, bool),
    reg: (present_current, r, 69, u16, None),
    // Factory block; hts is only meaningful from firmware 2.54 and reads 0 before.
    reg: (moving_velocity_threshold, rw, 80, u8, None),
    reg: (dts, rw, 81, u8, None),
    reg: (velocity_unit_factor, rw, 82, u8, None),
    reg: (hts, rw, 83, u8, None),
    reg: (maximum_velocity_limit, rw, 84, u8, None),
    reg: (maximum_acceleration, rw, 85, u8, None),
    reg: (acceleration_multiplier, rw, 86, u8, None),
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

pub struct Offset;
const MAX_MAGNITUDE: u16 = 2047;

impl Conversion for Offset {
    type RegisterType = u16;
    type UsiType = f64;

    fn from_raw(raw: u16) -> f64 {
        use std::f64::consts::PI;
        let negative = (raw >> 11) == 1;
        let magnitude = raw % (MAX_MAGNITUDE + 1);

        let float_magnitude = PI * f64::from(magnitude) / f64::from(MAX_MAGNITUDE);

        if negative {
            -float_magnitude
        } else {
            float_magnitude
        }
    }

    fn to_raw(value: f64) -> u16 {
        use std::f64::consts::PI;

        let magnitude = (value.abs() * f64::from(MAX_MAGNITUDE) / PI) as u16;

        if value.is_sign_negative() {
            magnitude | (1 << 11)
        } else {
            magnitude
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn aliases_share_their_register() {
        use crate::servo::feetech::sts3215::register;

        for (alias, original) in [
            ("model_number", "model"),
            ("baud_rate", "baudrate"),
            ("min_position_limit", "min_angle_limit"),
            ("max_position_limit", "max_angle_limit"),
            ("homing_offset", "offset"),
            ("operating_mode", "mode"),
            ("goal_velocity", "goal_speed"),
            ("present_velocity", "present_speed"),
        ] {
            let (a, o) = (register(alias).unwrap(), register(original).unwrap());
            assert_eq!(
                (a.addr, a.size, a.access),
                (o.addr, o.size, o.access),
                "{alias}"
            );
        }
    }

    #[test]
    fn maximum_acceleration_and_multiplier_are_one_byte_each() {
        use crate::servo::feetech::sts3215::register;

        let acceleration = register("maximum_acceleration").unwrap();
        let multiplier = register("acceleration_multiplier").unwrap();
        assert_eq!((acceleration.addr, acceleration.size), (85, 1));
        assert_eq!((multiplier.addr, multiplier.size), (86, 1));
    }

    #[test]
    fn offset_conversions() {
        use crate::servo::{conversion::Conversion, feetech::sts3215::Offset};
        use std::f64::consts::{FRAC_PI_2, PI};

        assert_eq!(Offset::to_raw(0.0), 0);
        assert_eq!(Offset::to_raw(PI), 2047);
        assert_eq!(Offset::to_raw(-PI), 4095);
        assert_eq!(Offset::to_raw(FRAC_PI_2), 1023);
        assert_eq!(Offset::to_raw(-FRAC_PI_2), 3071);

        assert_eq!(Offset::from_raw(0), 0.0);
        assert_eq!(Offset::from_raw(2047), PI);
        assert_eq!(Offset::from_raw(4095), -PI);
        assert_eq!(Offset::from_raw(1023), 1.5700289617109715); // About PI/2
        assert_eq!(Offset::from_raw(3071), -1.5700289617109715); // About -PI/2
    }
}

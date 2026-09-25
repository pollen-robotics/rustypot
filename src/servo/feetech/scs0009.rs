use crate::generate_servo;
use crate::servo::conversion::Conversion;

generate_servo!(
    SCS0009, v1,
    resolution: 1024,
    word_order: big,
    supports_sync_read: false,
    baudrates: [
        (1_000_000, 0), (500_000, 1), (250_000, 2), (128_000, 3),
        (115_200, 4), (57_600, 5), (38_400, 6), (19_200, 7),
    ],
    // Positions and limits are plain steps on the SCS.
    encoding: [
        (min_angle_limit, unsigned), (min_position_limit, unsigned),
        (max_angle_limit, unsigned), (max_position_limit, unsigned),
        (goal_position, unsigned), (present_position, unsigned),
    ],
    reg: (firmware_major_version, r, 0, u8, None),
    reg: (firmware_minor_version, r, 1, u8, None),
    reg: (model, r, 3, u16, None),
    reg: (model_number, r, 3, u16, None),
    reg: (id, rw, 5, u8, None),
    reg: (baudrate, rw, 6, u8, None),
    reg: (baud_rate, rw, 6, u8, None),
    reg: (return_delay_time, rw, 7, u8, None), //RESERVED?
    reg: (response_status_level, rw, 8, u8, None),
    reg: (min_angle_limit, rw, 9, i16, AnglePosition),
    reg: (min_position_limit, rw, 9, i16, AnglePosition),
    reg: (max_angle_limit, rw, 11, i16, AnglePosition),
    reg: (max_position_limit, rw, 11, i16, AnglePosition),
    reg: (max_temperature_limit, rw, 13, u8, None),
    reg: (max_voltage_limit, rw, 14, u8, None),
    reg: (min_voltage_limit, rw, 15, u8, None),
    reg: (max_torque_limit, rw, 16, u16, TorqueLimit),
    reg: (phase, rw, 18, u8, None), //SPECIAL REG
    reg: (unloading_condition, rw, 19, u8, None),
    reg: (led_alarm_condition, rw, 20, u8, None),
    reg: (p_coefficient, rw, 21, u8, None),
    reg: (d_coefficient, rw, 22, u8, None),
    reg: (i_coefficient, rw, 23, u8, None),
    reg: (minimum_startup_force, rw, 24, u16, BigEndian_u16),
    reg: (cw_dead_zone, rw, 26, u8, None),
    reg: (ccw_dead_zone, rw, 27, u8, None),
    reg: (hysteresis_loop, rw, 27, u8, None),

    reg: (protective_torque, rw, 37, u8, None),
    reg: (protection_time, rw, 38, u8, None),
    reg: (overload_torque, rw, 39, u8, None),

    reg: (torque_enable, rw, 40, u8, None),
    reg: (acceleration, rw, 41, u8, None),

    reg: (goal_position, rw, 42, i16, AnglePosition),
    reg: (goal_time, rw, 44, u16, BigEndian_u16),
    reg: (goal_speed, rw, 46, u16, Velocity),
    reg: (goal_velocity, rw, 46, u16, Velocity),

    reg: (lock, rw, 48, u8, bool),
    reg: (present_position, r, 56, i16, AnglePosition),
    reg: (present_speed, r, 58, u16, Velocity),
    reg: (present_velocity, r, 58, u16, Velocity),
    reg: (present_load, r, 60, u16, BigEndian_i16),

    reg: (present_voltage, r, 62, u8, None),
    reg: (present_temperature, r, 63, u8, None),
    reg: (sync_write_flag, r, 64, u8, None),

    reg: (status, r, 65, u8, None),

    reg: (moving, r, 66, u8, bool),

    // Factory block; the velocity entries are in units of 50 steps/s.
    reg: (pwm_maximum_step, rw, 78, u8, None),
    reg: (moving_velocity_threshold, rw, 79, u8, None),
    reg: (dts, rw, 80, u8, None),
    reg: (minimum_velocity_limit, rw, 81, u8, None),
    reg: (maximum_velocity_limit, rw, 82, u8, None),
);

pub struct Velocity;

impl Conversion for Velocity {
    type RegisterType = u16;
    type UsiType = f64;

    fn from_raw(raw: u16) -> f64 {
        // println!("DEBUG SPEED: {:?}", raw.to_be());
        if raw.to_be() > (1 << 15) {
            -300.0_f64.to_radians() / 1024.0 * (raw.to_be() & 0x3ff) as f64
        } else {
            300.0_f64.to_radians() / 1024.0 * (raw.to_be() & 0x3ff) as f64
        }
    }

    fn to_raw(value: f64) -> u16 {
        ((value / (300.0_f64.to_radians() / 1024.0)) as u16).to_be()
    }
}

pub struct AnglePosition;

impl Conversion for AnglePosition {
    type RegisterType = i16;
    type UsiType = f64;

    fn from_raw(raw: i16) -> f64 {
        300.0_f64.to_radians() * (((raw.to_be() & 0x3ff) - 511) as f64) / 1024.0
    }

    fn to_raw(value: f64) -> i16 {
        let a = (1024.0 * (value) / (300.0_f64.to_radians()) + 511.0) as i16;
        a.to_be()
    }
}

#[allow(non_camel_case_types)]
pub struct BigEndian_u16;
impl Conversion for BigEndian_u16 {
    type RegisterType = u16;
    type UsiType = u16;

    fn from_raw(raw: u16) -> u16 {
        raw.to_be()
    }

    fn to_raw(value: u16) -> u16 {
        value.to_be()
    }
}

#[allow(non_camel_case_types)]
pub struct BigEndian_i16;
impl Conversion for BigEndian_i16 {
    type RegisterType = u16;
    type UsiType = i16;

    fn from_raw(raw: u16) -> i16 {
        if raw.to_be() > (1 << 10) {
            -((raw.to_be() & 0x3ff) as i16)
        } else {
            (raw.to_be() & 0x3ff) as i16
        }
    }

    fn to_raw(value: i16) -> u16 {
        value.to_be() as u16
    }
}

pub struct TorqueLimit;
impl Conversion for TorqueLimit {
    type RegisterType = u16;
    type UsiType = f64;

    fn from_raw(raw: u16) -> f64 {
        raw.to_be() as f64 * 0.1
    }

    fn to_raw(value: f64) -> u16 {
        (value.clamp(0.0, 100.0) as u16 * 10).to_be()
    }
}

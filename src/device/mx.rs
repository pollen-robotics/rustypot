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
reg_read_write!(moving_speed, 32, i16);
reg_read_write!(torque_limit, 34, u16);
reg_read_only!(present_position, 36, i16);
reg_read_only!(present_speed, 38, i16);
reg_read_only!(present_load, 40, i16);
reg_read_only!(present_voltage, 42, u8);
reg_read_only!(present_temperature, 43, u8);
reg_read_only!(registered, 44, u8);
reg_read_only!(moving, 46, u8);
reg_read_write!(lock, 47, u8);
reg_read_write!(punch, 48, u16);
reg_read_only!(realtime_tick, 50, u16);
reg_read_write!(goal_acceleration, 73, u8);

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

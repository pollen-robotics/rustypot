//! XL-430 robotis register (protocol v2)
//!
//! See <https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/> for details.

use crate::device::*;

reg_read_only!(model_number, 0, u16);
reg_read_only!(model_information, 2, u32);
reg_read_write!(firmware_version, 6, u8);
reg_read_write!(id, 7, u8);
reg_read_write!(baud_rate, 8, u8);
reg_read_write!(return_delay_time, 9, u8);
reg_read_write!(drive_mode, 10, u8);
reg_read_write!(operating_mode, 11, u8);
reg_read_write!(secondary_id, 12, u8);
reg_read_write!(protocol_type, 13, u8);
reg_read_write!(homing_offset, 20, i32);
reg_read_write!(moving_threshold, 24, u32);
reg_read_write!(temperature_limit, 31, u8);
reg_read_write!(max_voltage_limit, 32, u16);
reg_read_write!(min_voltage_limit, 34, u16);
reg_read_write!(pwm_limit, 36, u16);
reg_read_write!(current_limit, 38, u16);
reg_read_write!(acceleration_limit, 40, u32);
reg_read_write!(velocity_limit, 44, u32);
reg_read_write!(max_position_limit, 48, u32);
reg_read_write!(min_position_limit, 52, u32);
reg_read_write!(startup_configuration, 60, u8);
reg_read_write!(shutdown, 63, u8);

reg_read_write!(torque_enable, 64, u8);
reg_read_write!(led, 65, u8);
reg_read_write!(status_return_level, 68, u8);
reg_read_write!(registered_instruction, 69, u8);
reg_read_write!(hardware_error_status, 70, u8);
reg_read_write!(velocity_i_gain, 76, u16);
reg_read_write!(velocity_p_gain, 78, u16);
reg_read_write!(position_d_gain, 80, u16);
reg_read_write!(position_i_gain, 82, u16);
reg_read_write!(position_p_gain, 84, u16);
reg_read_write!(feedforward_2nd_gain, 88, u16);
reg_read_write!(feedforward_1st_gain, 90, u16);
reg_read_write!(bus_watchdog, 98, u8);
reg_read_write!(goal_pwm, 100, u16);
reg_read_write!(goal_current, 102, u16);
reg_read_write!(goal_velocity, 104, u32);
reg_read_write!(profile_acceleration, 108, u32);
reg_read_write!(profile_velocity, 112, u32);
reg_read_write!(goal_position, 116, u32);
reg_read_only!(realtime_tick, 120, u16);
reg_read_only!(moving, 122, u8);
reg_read_only!(moving_status, 123, u8);
reg_read_only!(present_pwm, 124, u16);
reg_read_only!(present_current, 126, u16);
reg_read_only!(present_velocity, 128, u32);
reg_read_only!(present_position, 132, u32);
reg_read_only!(velocity_trajectory, 136, u32);
reg_read_only!(position_trajectory, 140, u32);
reg_read_only!(present_input_voltage, 144, u16);
reg_read_only!(present_temperature, 146, u8);
reg_read_only!(backup_ready, 147, u8);

reg_read_write!(indirect_address_1, 168, u16);
reg_read_write!(indirect_address_2, 170, u16);
reg_read_write!(indirect_address_3, 172, u16);
reg_read_write!(indirect_address_4, 174, u16);
reg_read_write!(indirect_address_5, 176, u16);
reg_read_write!(indirect_address_6, 178, u16);
reg_read_write!(indirect_data_1, 224, u8);
reg_read_write!(indirect_data_2, 225, u8);
reg_read_write!(indirect_data_3, 226, u8);
reg_read_write!(indirect_data_4, 227, u8);
reg_read_write!(indirect_data_5, 228, u8);
reg_read_write!(indirect_data_6, 229, u8);

//! Custom board for load and fan control that can be connected on a dynamixel bus
//!
//! Exposes:
//!  * a force sensor value
//!  * 3 fans which can be turned on/off

use crate::device::*;

reg_read_only!(model_number, 0, u16);
reg_read_write!(id, 2, u8);
reg_read_only!(present_load, 40, i32);
reg_read_write!(fan1_state, 26, u8);
reg_read_write!(fan2_state, 27, u8);
reg_read_write!(fan3_state, 28, u8);

use crate::device::*;

reg_read_only!(present_position, 10, f32);
reg_read_write!(goal_position, 20, f32);
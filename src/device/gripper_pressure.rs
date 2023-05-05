//! Custom board for reading pressure sensors that can be connected on a 
//! Dynamixel bus.
//!
//! Exposes:
//!  * 4 ADC values

use crate::device::*;

/// Wrapper for a value for the gripper
#[derive(Clone, Copy, Debug)]
pub struct AdcValues<T> {
    pub in0: T,
    pub in1: T,
    pub in2: T,
    pub in3: T,
}

reg_read_only!(model_number, 0, u16);
reg_read_write!(id, 2, u8);
reg_read_only!(adc_values, 50, AdcValues::<i32>);
reg_read_only!(adc_in0, 50, i32);
reg_read_only!(adc_in1, 54, i32);
reg_read_only!(adc_in2, 58, i32);
reg_read_only!(adc_in3, 62, i32);

impl<T: PartialEq> PartialEq for AdcValues<T> {
    fn eq(&self, other: &Self) -> bool {
        self.in0 == other.in0 && self.in1 == other.in1 && self.in2 == other.in2 && self.in3 == other.in3
    }
}

impl AdcValues<i32> {
    pub fn from_le_bytes(bytes: [u8; 16]) -> Self {
        AdcValues {
            in0: i32::from_be_bytes(bytes[0..4].try_into().unwrap()),
            in1: i32::from_be_bytes(bytes[4..8].try_into().unwrap()),
            in2: i32::from_be_bytes(bytes[8..12].try_into().unwrap()),
            in3: i32::from_be_bytes(bytes[12..16].try_into().unwrap()),
        }
    }
/*    pub fn to_le_bytes(&self) -> [u8; 16] {
        let mut bytes = Vec::new();

        bytes.extend_from_slice(&self.in0.to_e_bytes());
        bytes.extend_from_slice(&self.in1.to_e_bytes());
        bytes.extend_from_slice(&self.in2.to_e_bytes());
        bytes.extend_from_slice(&self.in3.to_e_bytes());

        bytes.try_into().unwrap()
    }*/
}



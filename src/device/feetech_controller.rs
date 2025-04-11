use crate::{DynamixelProtocolHandler, Result};

use super::feetech_sts3215;

pub struct FeetechFts3215Controller {
    dph: Option<DynamixelProtocolHandler>,
    serial_port: Option<Box<dyn serialport::SerialPort>>,
}

impl FeetechFts3215Controller {
    pub fn new() -> Self {
        Self {
            dph: None,
            serial_port: None,
        }
    }
    pub fn with_protocol_v1(mut self) -> Self {
        self.dph = Some(DynamixelProtocolHandler::v1());
        self
    }
    pub fn with_serial_port(mut self, serial_port: Box<dyn serialport::SerialPort>) -> Self {
        self.serial_port = Some(serial_port);
        self
    }
}

impl FeetechFts3215Controller {
    pub fn read_present_position(&mut self, ids: &[u8]) -> Result<Vec<f64>> {
        let pos = feetech_sts3215::sync_read_present_position(
            self.dph.as_ref().unwrap(),
            self.serial_port.as_mut().unwrap().as_mut(),
            ids,
        )?;
        Ok(pos
            .iter()
            .map(|p| feetech_sts3215::conv::dxl_pos_to_radians(*p))
            .map(f64::to_degrees)
            .collect())
    }
}

use std::time::Duration;

use serialport::SerialPort;

use crate::DynamixelLikeIO;

pub struct DynamixelSerialIO {
    serial_port: Box<dyn SerialPort>,
}

impl DynamixelSerialIO {
    pub fn new(path: &str, timeout: Duration) -> Self {
        let serial_port = serialport::new(path, 1_000_000)
            .timeout(timeout)
            .open()
            .expect(format!("Failed to open port {}", path).as_str());

        Self { serial_port }
    }
}

impl DynamixelLikeIO for DynamixelSerialIO {
    fn send_packet(&mut self, bytes: Vec<u8>) {
        self.serial_port.write_all(&bytes).unwrap();
    }

    fn read_packet(&mut self) -> Result<Vec<u8>, crate::CommunicationErrorKind> {
        let mut header = vec![0; 4];
        self.serial_port.read_exact(&mut header).unwrap();

        let resp_id = header[2];
        let payload_size = header[3];

        let mut payload = vec![0; payload_size.into()];
        self.serial_port.read(&mut payload).unwrap();

        let mut resp = Vec::new();
        resp.append(&mut header);
        resp.append(&mut payload);

        Ok(resp)
    }
}

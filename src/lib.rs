use std::error::Error;
use std::fmt;
use std::{collections::HashSet, mem::size_of, time::Duration};

use serialport::SerialPort;

mod protocol;
use crate::protocol::v1::{InstructionPacket, StatusPacket};
use crate::protocol::{DynamixelError, FromBytes, ToBytes};

mod serialize;
pub use serialize::Serializable;

pub struct DynamixelSerialIO {
    serial_port: Box<dyn SerialPort>,
    errors: HashSet<DynamixelError>,
}

impl DynamixelSerialIO {
    pub fn new(path: &str, baudrate: u32, timeout: Duration) -> Result<Self, Box<dyn Error>> {
        let serial_port = serialport::new(path, baudrate)
            .timeout(timeout)
            .open()
            .unwrap_or_else(|_| panic!("Failed to open port {}", path));

        serial_port.clear(serialport::ClearBuffer::All)?;

        Ok(Self {
            serial_port,
            errors: HashSet::new(),
        })
    }

    pub fn ping(&mut self, id: u8) -> Result<bool, CommunicationErrorKind> {
        let instruction_packet = InstructionPacket::ping_packet(id);
        match self.request(instruction_packet) {
            Ok(_) => Ok(true),
            Err(e) => match e {
                CommunicationErrorKind::TimeoutError => Ok(false),
                _ => Err(e),
            },
        }
    }

    pub fn read_data<T: Serializable>(
        &mut self,
        id: u8,
        reg: u8,
    ) -> Result<T, CommunicationErrorKind> {
        let instruction_packet =
            InstructionPacket::read_packet(id, reg, size_of::<T>().try_into().unwrap());
        let status_packet = self.request(instruction_packet)?;
        T::from_bytes(status_packet.payload).ok_or(CommunicationErrorKind::ParsingError)
    }

    pub fn write_data<T: Serializable>(
        &mut self,
        id: u8,
        reg: u8,
        value: T,
    ) -> Result<(), CommunicationErrorKind> {
        let instruction_packet = InstructionPacket::write_packet(id, reg, value.to_bytes());
        self.request(instruction_packet)?;
        Ok(())
    }

    fn request(
        &mut self,
        instruction_packet: InstructionPacket,
    ) -> Result<StatusPacket, CommunicationErrorKind> {
        self.send_packet(&instruction_packet.to_bytes());

        let data = self.read_packet()?;
        let status_packet = StatusPacket::from_bytes(instruction_packet.id, data)?;

        for e in status_packet.error.iter().copied() {
            self.errors.insert(e);
        }

        Ok(status_packet)
    }

    fn send_packet(&mut self, bytes: &[u8]) {
        self.serial_port.write_all(bytes).unwrap();
    }

    fn read_packet(&mut self) -> Result<Vec<u8>, crate::CommunicationErrorKind> {
        let mut header = vec![0; 4];
        if self.serial_port.read_exact(&mut header).is_err() {
            return Err(crate::CommunicationErrorKind::TimeoutError);
        }

        let payload_size = header[3];

        let mut payload = vec![0; payload_size.into()];
        if self.serial_port.read(&mut payload).is_err() {
            return Err(crate::CommunicationErrorKind::TimeoutError);
        }

        let mut resp = Vec::new();
        resp.append(&mut header);
        resp.append(&mut payload);

        Ok(resp)
    }
}

#[derive(Debug, Clone, Copy)]
pub enum CommunicationErrorKind {
    ChecksumError,
    ParsingError,
    TimeoutError,
}
impl fmt::Display for CommunicationErrorKind {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            CommunicationErrorKind::ChecksumError => write!(f, "Checksum error"),
            CommunicationErrorKind::ParsingError => write!(f, "Parsing error"),
            CommunicationErrorKind::TimeoutError => write!(f, "Timeout error"),
        }
    }
}
impl Error for CommunicationErrorKind {}

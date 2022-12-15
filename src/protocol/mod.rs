use serialport::SerialPort;

mod v1;
pub use v1::V1;

use crate::{
    packet::{InstructionPacket, StatusPacket},
    Packet, Result,
};

pub trait Protocol<P: Packet> {
    fn new() -> Self
    where
        Self: Sized;

    fn ping(&self, port: &mut dyn SerialPort, id: u8) -> Result<bool> {
        self.send_instruction_packet(port, P::ping_packet(id).as_ref())?;

        Ok(self.read_status_packet(port, id).is_ok())
    }

    fn read(
        &self,
        port: &mut dyn SerialPort,
        id: u8,
        addr: P::RegisterSize,
        length: P::RegisterSize,
    ) -> Result<Vec<u8>> {
        self.send_instruction_packet(port, P::read_packet(id, addr, length).as_ref())?;
        self.read_status_packet(port, id)
            .map(|sp| sp.params().to_vec())
    }
    fn write(
        &self,
        port: &mut dyn SerialPort,
        id: u8,
        addr: P::RegisterSize,
        data: &[u8],
    ) -> Result<()> {
        self.send_instruction_packet(port, P::write_packet(id, addr, data).as_ref())?;
        self.read_status_packet(port, id).map(|_| ())
    }
    fn sync_read(
        &self,
        port: &mut dyn SerialPort,
        ids: &[u8],
        addr: P::RegisterSize,
        length: P::RegisterSize,
    ) -> Result<Vec<Vec<u8>>>;
    fn sync_write(
        &self,
        port: &mut dyn SerialPort,
        ids: &[u8],
        addr: P::RegisterSize,
        data: &[&[u8]],
    ) -> Result<()> {
        self.send_instruction_packet(port, P::sync_write_packet(ids, addr, data).as_ref())?;
        Ok(())
    }

    fn send_instruction_packet(
        &self,
        port: &mut dyn SerialPort,
        packet: &dyn InstructionPacket<P>,
    ) -> Result<()> {
        match port.write_all(&packet.to_bytes()) {
            Ok(_) => Ok(()),
            Err(_) => Err(Box::new(CommunicationErrorKind::TimeoutError)),
        }
    }
    fn read_status_packet(
        &self,
        port: &mut dyn SerialPort,
        sender_id: u8,
    ) -> Result<Box<dyn StatusPacket<P>>> {
        let mut header = vec![0u8; P::HEADER_SIZE];
        port.read_exact(&mut header)?;

        let payload_size = P::get_payload_size(&header).unwrap();
        let mut payload = vec![0u8; payload_size];
        port.read_exact(&mut payload)?;

        let mut data = Vec::new();
        data.extend(header);
        data.extend(payload);

        P::status_packet(&data, sender_id)
    }
}

use std::fmt;

#[derive(Debug, Clone, Copy)]
pub enum CommunicationErrorKind {
    ChecksumError,
    ParsingError,
    TimeoutError,
    IncorrectId(u8, u8),
}
impl fmt::Display for CommunicationErrorKind {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            CommunicationErrorKind::ChecksumError => write!(f, "Checksum error"),
            CommunicationErrorKind::ParsingError => write!(f, "Parsing error"),
            CommunicationErrorKind::TimeoutError => write!(f, "Timeout error"),
            CommunicationErrorKind::IncorrectId(sender_id, resp_id) => {
                write!(f, "Incorrect id ({} instead of {})", resp_id, sender_id)
            }
        }
    }
}
impl std::error::Error for CommunicationErrorKind {}
